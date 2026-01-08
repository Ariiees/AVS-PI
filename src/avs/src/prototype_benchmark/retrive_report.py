#!/usr/bin/env python3
import mmap
import os
import sqlite3
import struct
import sys
import time
import yaml
import resource
import operator
from dataclasses import dataclass
from pathlib import Path


SSD_ROOT = Path("/home/avs/DATA/SSD")

def avs_pi_path():
    return Path("/home/avs") / ("AVS" + chr(45) + "PI")

TOPIC_MAP_PATH = avs_pi_path() / "src/avs/config/topics.yaml"


@dataclass(frozen=True)
class TopicInfo:
    sensor_type: str
    folder_name: str


@dataclass(frozen=True)
class GlobalTripRow:
    topic_folder: str
    day: str
    trip_id: int
    start_ts_ns: int
    end_ts_ns: int


@dataclass(frozen=True)
class TripIndexEntry:
    start_ts_ns: int
    end_ts_ns: int
    file_offset: int
    chunk_size_bytes: int
    record_count: int


@dataclass(frozen=True)
class RecordMeta:
    ts_ns: int
    payload_size: int
    locator: str


def load_topic_map(path):
    if not path.exists():
        raise FileNotFoundError(f"topics yaml not found: {path}")
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    out = {}
    for topic, meta in raw.items():
        out[topic] = TopicInfo(
            sensor_type=str(meta["sensor_type"]),
            folder_name=str(meta["folder_name"]),
        )
    return out


def open_global_db():
    db_path = SSD_ROOT / "global.sqlite3"
    if not db_path.exists():
        raise FileNotFoundError(f"global sqlite3 not found: {db_path}")
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    return conn


def load_closed_trips_overlapping(conn, sensor_topic, t_start_ns, t_end_ns):
    cur = conn.cursor()
    cur.execute(
        """
        SELECT topic_folder, day, trip_id, start_ts_ns, end_ts_ns
        FROM global
        WHERE sensor_topic = ?
          AND end_ts_ns != '0'
          AND CAST(start_ts_ns AS INTEGER) <= ?
          AND CAST(end_ts_ns AS INTEGER) >= ?
        ORDER BY day ASC, trip_id ASC
        """,
        (sensor_topic, t_end_ns, t_start_ns),
    )
    rows = []
    for r in cur.fetchall():
        rows.append(
            GlobalTripRow(
                topic_folder=str(r["topic_folder"]),
                day=str(r["day"]),
                trip_id=int(r["trip_id"]),
                start_ts_ns=int(r["start_ts_ns"]),
                end_ts_ns=int(r["end_ts_ns"]),
            )
        )
    return rows


def trip_paths(topic_folder, day, trip_id):
    day_dir = SSD_ROOT / topic_folder / day
    log_p = day_dir / f"trip_{trip_id:02d}.log"
    idx_p = day_dir / f"trip_{trip_id:02d}.idx"
    return log_p, idx_p


def iter_index_entries_stream(idx_f):
    fmt = "<qqQII"
    sz = struct.calcsize(fmt)
    while True:
        b = idx_f.read(sz)
        if not b:
            return
        if len(b) != sz:
            return
        s, e, fo, cb, rc = struct.unpack(fmt, b)
        yield TripIndexEntry(int(s), int(e), int(fo), int(cb), int(rc))


def iter_record_sizes_for_trip_mmap(log_path, idx_path, t_start_ns, t_end_ns, locator_prefix):
    """
    Streaming iterator backed by mmap, without creating memoryview objects.
    Yields RecordMeta plus payload size.
    This prevents BufferError on mmap close.
    """
    chunk_header_fmt = "<qqII"
    chunk_header_sz = struct.calcsize(chunk_header_fmt)
    record_header_fmt = "<qI"
    record_header_sz = struct.calcsize(record_header_fmt)

    with log_path.open("rb") as log_f, idx_path.open("rb") as idx_f:
        try:
            mm = mmap.mmap(log_f.fileno(), 0, access=mmap.ACCESS_READ)
        except ValueError:
            return

        try:
            mm_len = mm.size()
            chunk_i = 0
            started = False

            for ent in iter_index_entries_stream(idx_f):
                if not started:
                    if ent.end_ts_ns < t_start_ns:
                        chunk_i += 1
                        continue
                    started = True

                if ent.start_ts_ns > t_end_ns:
                    return

                off = int(ent.file_offset)
                sz = int(ent.chunk_size_bytes)

                if off < 0 or sz <= 0:
                    chunk_i += 1
                    continue

                end = off + sz
                if end > mm_len:
                    chunk_i += 1
                    continue

                if sz < chunk_header_sz:
                    chunk_i += 1
                    continue

                try:
                    _s, _e, record_count, _r = struct.unpack_from(chunk_header_fmt, mm, off)
                except struct.error:
                    chunk_i += 1
                    continue

                p = off + chunk_header_sz
                rec_i = 0

                while rec_i < record_count and (p + record_header_sz) <= end:
                    try:
                        ts_ns, payload_sz = struct.unpack_from(record_header_fmt, mm, p)
                    except struct.error:
                        break

                    p = p + record_header_sz

                    if payload_sz < 0:
                        break
                    if (p + payload_sz) > end:
                        break

                    p = p + payload_sz

                    if t_start_ns <= ts_ns <= t_end_ns:
                        loc = f"{locator_prefix}:off{ent.file_offset}:chunk{chunk_i}:rec{rec_i}"
                        meta = RecordMeta(int(ts_ns), int(payload_sz), loc)
                        yield meta, int(payload_sz)

                    rec_i += 1

                chunk_i += 1
        finally:
            mm.close()


def percentile(vals, p):
    if not vals:
        return 0
    v = sorted(vals)
    n = len(v)
    if n == 1:
        return int(v[0])

    if p <= 0.0:
        return int(v[0])
    if p >= 100.0:
        return int(v[operator.sub(n, 1)])

    rank = (p / 100.0) * float(operator.sub(n, 1))
    lo = int(rank)
    hi = lo + 1
    if hi >= n:
        hi = operator.sub(n, 1)
    frac = rank - float(lo)
    est = float(v[lo]) + (float(v[hi]) - float(v[lo])) * frac
    return int(est)


def mode_list(sensor, info, t_start_ns, t_end_ns):
    conn = open_global_db()
    try:
        trips = load_closed_trips_overlapping(conn, sensor, t_start_ns, t_end_ns)
    finally:
        conn.close()

    for tr in trips:
        log_p, idx_p = trip_paths(tr.topic_folder, tr.day, tr.trip_id)
        if not log_p.exists() or not idx_p.exists():
            continue

        prefix = f"SSD:{sensor}:{tr.day}:trip{tr.trip_id:02d}"
        for meta, _psz in iter_record_sizes_for_trip_mmap(log_p, idx_p, t_start_ns, t_end_ns, prefix):
            sys.stdout.write(f"{sensor}|{info.sensor_type}|{meta.ts_ns}|{meta.locator}\n")


def mode_bench(sensor, info, t_start_ns, t_end_ns, max_frames):
    t0 = time.perf_counter_ns()

    first = True
    ttfb_ns = 0

    n = 0
    total_bytes = 0
    lats = []

    conn = open_global_db()
    try:
        trips = load_closed_trips_overlapping(conn, sensor, t_start_ns, t_end_ns)
    finally:
        conn.close()

    for tr in trips:
        log_p, idx_p = trip_paths(tr.topic_folder, tr.day, tr.trip_id)
        if not log_p.exists() or not idx_p.exists():
            continue

        prefix = f"SSD:{sensor}:{tr.day}:trip{tr.trip_id:02d}"
        it = iter_record_sizes_for_trip_mmap(log_p, idx_p, t_start_ns, t_end_ns, prefix)

        while True:
            if max_frames > 0 and n >= max_frames:
                break

            d0 = time.perf_counter_ns()
            try:
                _meta, payload_sz = next(it)
            except StopIteration:
                break
            d1 = time.perf_counter_ns()

            lat = operator.sub(d1, d0)

            if first:
                ttfb_ns = operator.sub(d1, t0)
                first = False

            n += 1
            total_bytes += int(payload_sz)
            lats.append(int(lat))

        if max_frames > 0 and n >= max_frames:
            break

    if first:
        ttfb_ns = operator.sub(time.perf_counter_ns(), t0)

    elapsed_ns = operator.sub(time.perf_counter_ns(), t0)
    elapsed_s = float(elapsed_ns) / 1e9 if elapsed_ns > 0 else 0.0

    total_lat = sum(lats) if lats else 0
    avg_lat = int(total_lat / n) if n > 0 else 0

    p50_v = percentile(lats, 50.0)
    p95_v = percentile(lats, 95.0)
    p99_v = percentile(lats, 99.0)

    rps = (float(n) / elapsed_s) if elapsed_s > 0 else 0.0
    bps = (float(total_bytes) / elapsed_s) if elapsed_s > 0 else 0.0

    ru1 = resource.getrusage(resource.RUSAGE_SELF)
    max_rss_kb = int(ru1.ru_maxrss)

    sys.stdout.write(f"SUMMARY\trecords\t{n}\n")
    sys.stdout.write(f"SUMMARY\tbytes\t{total_bytes}\n")
    sys.stdout.write(f"SUMMARY\tttfb_ns\t{ttfb_ns}\n")
    sys.stdout.write(f"SUMMARY\tavg_record_latency_ns\t{avg_lat}\n")
    sys.stdout.write(f"SUMMARY\tp50_record_latency_ns\t{p50_v}\n")
    sys.stdout.write(f"SUMMARY\tp95_record_latency_ns\t{p95_v}\n")
    sys.stdout.write(f"SUMMARY\tp99_record_latency_ns\t{p99_v}\n")
    sys.stdout.write(f"SUMMARY\tthroughput_records_per_s\t{rps}\n")
    sys.stdout.write(f"SUMMARY\tthroughput_bytes_per_s\t{bps}\n")
    sys.stdout.write(f"SUMMARY\tmax_rss_kb\t{max_rss_kb}\n")


def main():
    if len(sys.argv) < 5:
        sys.stderr.write(
            "Usage\n"
            "  python3 retrieve_report.py sensor_topic t_start_ns t_end_ns mode [max_frames]\n"
            "mode is list or bench\n"
        )
        return 2

    sensor = sys.argv[1]
    t_start_ns = int(sys.argv[2])
    t_end_ns = int(sys.argv[3])
    mode = sys.argv[4]
    max_frames = int(sys.argv[5]) if len(sys.argv) >= 6 else 0

    if t_end_ns < t_start_ns:
        raise ValueError("t_end_ns must be >= t_start_ns")

    topic_map = load_topic_map(TOPIC_MAP_PATH)
    if sensor not in topic_map:
        raise RuntimeError(f"sensor not found in topics yaml: {sensor}")

    info = topic_map[sensor]

    if mode == "list":
        mode_list(sensor, info, t_start_ns, t_end_ns)
        return 0

    if mode == "bench":
        mode_bench(sensor, info, t_start_ns, t_end_ns, max_frames)
        return 0

    raise ValueError("mode must be list or bench")


if __name__ == "__main__":
    raise SystemExit(main())
