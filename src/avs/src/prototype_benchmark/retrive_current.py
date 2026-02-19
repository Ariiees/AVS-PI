#!/usr/bin/env python3
import argparse
import mmap
import operator
import resource
import sqlite3
import struct
import sys
import time
import yaml
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


def load_topic_map(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"topics yaml not found: {path}")
    with path.open("r") as f:
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


def table_columns(conn: sqlite3.Connection, table_name: str):
    cur = conn.cursor()
    cur.execute(f"PRAGMA table_info({table_name})")
    cols = [str(r[1]) for r in cur.fetchall()]
    return set(cols)


def pick_open_record_count_column(conn: sqlite3.Connection):
    cols = table_columns(conn, "global")
    candidates = [
        "record_num",
        "record_count",
        "records",
        "num_records",
        "n_records",
        "total_records",
    ]
    for c in candidates:
        if c in cols:
            return c
    return None


def trip_paths(topic_folder: str, day: str, trip_id: int):
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


def iter_record_sizes_open_trip_mmap(log_path: Path, idx_path: Path):
    """
    For current open trip, scan all indexed chunks and yield per record payload size.
    No timestamp filtering.
    Uses mmap snapshot of current log file size.
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

            for ent in iter_index_entries_stream(idx_f):
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
                        _ts_ns, payload_sz = struct.unpack_from(record_header_fmt, mm, p)
                    except struct.error:
                        break

                    p = p + record_header_sz

                    if payload_sz < 0:
                        break
                    if (p + payload_sz) > end:
                        break

                    p = p + payload_sz

                    yield int(payload_sz)

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


def load_open_trips(conn: sqlite3.Connection, sensor_topic: str):
    rc_col = pick_open_record_count_column(conn)
    if rc_col is not None:
        where_open = f"CAST({rc_col} AS INTEGER) = 0"
    else:
        where_open = "CAST(end_ts_ns AS INTEGER) = 0"

    cur = conn.cursor()
    cur.execute(
        f"""
        SELECT topic_folder, day, trip_id, start_ts_ns, end_ts_ns
        FROM global
        WHERE sensor_topic = ?
          AND ({where_open})
        ORDER BY day ASC, trip_id ASC
        """,
        (sensor_topic,),
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


def bench_open_current(sensor: str, info: TopicInfo, max_frames: int):
    t0 = time.perf_counter_ns()
    first = True
    ttfb_ns = 0

    n = 0
    total_bytes = 0
    lats = []

    conn = open_global_db()
    try:
        trips = load_open_trips(conn, sensor)
    finally:
        conn.close()

    for tr in trips:
        log_p, idx_p = trip_paths(tr.topic_folder, tr.day, tr.trip_id)
        if not log_p.exists() or not idx_p.exists():
            continue

        it = iter_record_sizes_open_trip_mmap(log_p, idx_p)

        while True:
            if max_frames > 0 and n >= max_frames:
                break

            d0 = time.perf_counter_ns()
            try:
                payload_sz = next(it)
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

    max_rss_kb = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)

    sys.stdout.write(f"SUMMARY\ttopic\t{sensor}\n")
    sys.stdout.write(f"SUMMARY\tsensor_type\t{info.sensor_type}\n")
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


def build_arg_parser():
    p = argparse.ArgumentParser(prog="retrieve_current.py")
    p.add_argument("sensor_topic", help="sensor topic key from topics yaml")
    p.add_argument(
        "max_frames",
        nargs="?",
        type=int,
        default=0,
        help="optional cap on records, 0 means no cap",
    )
    return p


def main():
    args = build_arg_parser().parse_args()
    sensor = str(args.sensor_topic)
    max_frames = int(args.max_frames)

    topic_map = load_topic_map(TOPIC_MAP_PATH)
    if sensor not in topic_map:
        raise RuntimeError(f"sensor not found in topics yaml: {sensor}")
    info = topic_map[sensor]

    bench_open_current(sensor, info, max_frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())