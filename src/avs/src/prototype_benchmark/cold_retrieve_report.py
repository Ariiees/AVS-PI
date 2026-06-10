#!/usr/bin/env python3
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
from typing import Dict, Optional, Tuple, List


HDD_ROOT = Path("/home/avs/DATA/HDD")

def avs_pi_path():
    return Path("/home/avs") / ("AVS" + chr(45) + "PI")

TOPIC_MAP_PATH = avs_pi_path() / "src/avs/config/topics.yaml"

TAR_BLOCK = 512


@dataclass(frozen=True)
class TopicInfo:
    sensor_type: str
    folder_name: str


@dataclass(frozen=True)
class GlobalTripRow:
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


def normalize_day_to_dash(day: str) -> str:
    if len(day) != 10:
        raise ValueError(f"bad day format: {day}")
    if not (day[4] in "-_" and day[7] in "-_"):
        raise ValueError(f"bad day separators: {day}")
    out = list(day)
    out[4] = "-"
    out[7] = "-"
    return "".join(out)


def year_month_from_day_dash(day_dash: str) -> Tuple[str, str]:
    if len(day_dash) != 10 or day_dash[4] != "-" or day_dash[7] != "-":
        raise ValueError(f"bad day string: {day_dash}")
    return day_dash[0:4], day_dash[5:7]


def load_topic_map(path: Path) -> Dict[str, TopicInfo]:
    if not path.exists():
        raise FileNotFoundError(f"topics yaml not found: {path}")
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    out: Dict[str, TopicInfo] = {}
    for topic, meta in raw.items():
        out[str(topic)] = TopicInfo(
            sensor_type=str(meta["sensor_type"]),
            folder_name=str(meta["folder_name"]),
        )
    return out


def open_global_db_hdd() -> sqlite3.Connection:
    db_path = HDD_ROOT / "global.sqlite3"
    if not db_path.exists():
        raise FileNotFoundError(f"HDD global sqlite3 not found: {db_path}")
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    return conn


def load_closed_trips_overlapping(conn: sqlite3.Connection,
                                 sensor_topic: str,
                                 t_start_ns: int,
                                 t_end_ns: int) -> List[GlobalTripRow]:
    """
    HDD global schema (from archive.cpp):
      global(sensor_topic, day, trip_id, start_ts_ns, end_ts_ns)
    start_ts_ns and end_ts_ns stored as TEXT, so cast to INTEGER.
    """
    cur = conn.cursor()
    cur.execute(
        """
        SELECT day, trip_id, start_ts_ns, end_ts_ns
        FROM global
        WHERE sensor_topic = ?
          AND CAST(start_ts_ns AS INTEGER) <= ?
          AND CAST(end_ts_ns   AS INTEGER) >= ?
        ORDER BY day ASC, trip_id ASC
        """,
        (sensor_topic, t_end_ns, t_start_ns),
    )

    rows: List[GlobalTripRow] = []
    for r in cur.fetchall():
        rows.append(
            GlobalTripRow(
                day=str(r["day"]),
                trip_id=int(r["trip_id"]),
                start_ts_ns=int(r["start_ts_ns"]),
                end_ts_ns=int(r["end_ts_ns"]),
            )
        )
    return rows


def tar_paths_for_topic_day(topic_folder: str, day_any: str) -> Tuple[Path, Path, str]:
    day_dash = normalize_day_to_dash(day_any)
    yyyy, mm = year_month_from_day_dash(day_dash)
    tar_path = HDD_ROOT / topic_folder / yyyy / mm / f"{day_dash}.tar"
    idx_path = HDD_ROOT / topic_folder / yyyy / mm / f"{day_dash}.tar.idx"
    return tar_path, idx_path, day_dash


def load_tar_index(idx_path: Path) -> Dict[str, Tuple[int, int]]:
    """
    idx line: name \t data_offset \t size
    """
    if not idx_path.exists():
        raise FileNotFoundError(f"tar index not found: {idx_path}")
    out: Dict[str, Tuple[int, int]] = {}
    with open(idx_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip("\n")
            if not line:
                continue
            parts = line.split("\t")
            if len(parts) != 3:
                continue
            name = parts[0]
            try:
                off = int(parts[1])
                sz = int(parts[2])
            except ValueError:
                continue
            out[name] = (off, sz)
    return out


def read_all_from_pread(fd: int, offset: int, size: int) -> bytes:
    """
    Robustly read exactly size bytes via os.pread.
    """
    out = bytearray()
    left = size
    off = offset
    while left > 0:
        chunk = os.pread(fd, left, off)
        if not chunk:
            break
        out.extend(chunk)
        n = len(chunk)
        off += n
        left -= n
    if len(out) != size:
        raise IOError(f"pread short read: want={size} got={len(out)} off={offset}")
    return bytes(out)


def iter_index_entries_from_tar(tar_fd: int,
                               idx_member_off: int,
                               idx_member_sz: int):
    fmt = "<qqQII"
    rec_sz = struct.calcsize(fmt)
    if idx_member_sz <= 0:
        return
    data = read_all_from_pread(tar_fd, idx_member_off, idx_member_sz)
    nrec = len(data) // rec_sz
    for i in range(nrec):
        b = data[i * rec_sz:(i + 1) * rec_sz]
        if len(b) != rec_sz:
            return
        s, e, fo, cb, rc = struct.unpack(fmt, b)
        yield TripIndexEntry(int(s), int(e), int(fo), int(cb), int(rc))


def iter_record_sizes_for_trip_in_tar(tar_fd: int,
                                     log_member_off: int,
                                     log_member_sz: int,
                                     idx_entries,
                                     t_start_ns: int,
                                     t_end_ns: int,
                                     locator_prefix: str):
    """
    Streaming yield of (RecordMeta, payload_sz).
    Uses os.pread on tar_fd for each selected chunk region.
    log_member_off is the start of trip_XX.log data payload in tar.
    ent.file_offset is offset inside trip_XX.log where chunk begins.
    """
    chunk_header_fmt = "<qqII"
    chunk_header_sz = struct.calcsize(chunk_header_fmt)
    record_header_fmt = "<qI"
    record_header_sz = struct.calcsize(record_header_fmt)

    chunk_i = 0
    started = False

    for ent in idx_entries:
        if not started:
            if ent.end_ts_ns < t_start_ns:
                chunk_i += 1
                continue
            started = True

        if ent.start_ts_ns > t_end_ns:
            return

        off_in_log = int(ent.file_offset)
        sz = int(ent.chunk_size_bytes)

        if off_in_log < 0 or sz <= 0:
            chunk_i += 1
            continue

        # Validate within this log member
        if off_in_log + sz > log_member_sz:
            chunk_i += 1
            continue

        # Read the whole chunk region (header plus records)
        abs_off = log_member_off + off_in_log
        try:
            buf = read_all_from_pread(tar_fd, abs_off, sz)
        except Exception:
            chunk_i += 1
            continue

        if len(buf) < chunk_header_sz:
            chunk_i += 1
            continue

        try:
            _s, _e, record_count, _chunk_sz_bytes = struct.unpack_from(chunk_header_fmt, buf, 0)
        except struct.error:
            chunk_i += 1
            continue

        p = chunk_header_sz
        rec_i = 0
        end = len(buf)

        while rec_i < int(record_count) and (p + record_header_sz) <= end:
            try:
                ts_ns, payload_sz = struct.unpack_from(record_header_fmt, buf, p)
            except struct.error:
                break
            p += record_header_sz

            if payload_sz < 0:
                break
            if (p + int(payload_sz)) > end:
                break

            # Skip payload bytes without copying
            p += int(payload_sz)

            if t_start_ns <= int(ts_ns) <= t_end_ns:
                loc = f"{locator_prefix}:off{ent.file_offset}:chunk{chunk_i}:rec{rec_i}"
                meta = RecordMeta(int(ts_ns), int(payload_sz), loc)
                yield meta, int(payload_sz)

            rec_i += 1

        chunk_i += 1


def percentile(vals: List[int], p: float) -> int:
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


def mode_list(sensor: str, info: TopicInfo, t_start_ns: int, t_end_ns: int) -> None:
    conn = open_global_db_hdd()
    try:
        trips = load_closed_trips_overlapping(conn, sensor, t_start_ns, t_end_ns)
    finally:
        conn.close()

    # Each trip resolves to tar and members via tar idx
    for tr in trips:
        tar_path, tar_idx_path, day_dash = tar_paths_for_topic_day(info.folder_name, tr.day)
        if not tar_path.exists() or not tar_idx_path.exists():
            continue

        tar_index = load_tar_index(tar_idx_path)
        log_name = f"{day_dash}/trip_{tr.trip_id:02d}.log"
        idx_name = f"{day_dash}/trip_{tr.trip_id:02d}.idx"
        if log_name not in tar_index or idx_name not in tar_index:
            continue

        log_off, log_sz = tar_index[log_name]
        idx_off, idx_sz = tar_index[idx_name]

        try:
            tar_fd = os.open(str(tar_path), os.O_RDONLY | getattr(os, "O_CLOEXEC", 0))
        except OSError:
            continue

        try:
            idx_entries = iter_index_entries_from_tar(tar_fd, idx_off, idx_sz)
            prefix = f"HDD_TAR:{sensor}:{day_dash}:trip{tr.trip_id:02d}"
            for meta, _psz in iter_record_sizes_for_trip_in_tar(
                tar_fd, log_off, log_sz, idx_entries, t_start_ns, t_end_ns, prefix
            ):
                sys.stdout.write(f"{sensor}|{info.sensor_type}|{meta.ts_ns}|{meta.locator}\n")
        finally:
            os.close(tar_fd)


def mode_bench(sensor: str, info: TopicInfo, t_start_ns: int, t_end_ns: int, max_frames: int) -> None:
    t0 = time.perf_counter_ns()
    first = True
    ttfb_ns = 0

    n = 0
    total_bytes = 0
    lats: List[int] = []

    conn = open_global_db_hdd()
    try:
        trips = load_closed_trips_overlapping(conn, sensor, t_start_ns, t_end_ns)
    finally:
        conn.close()

    for tr in trips:
        if max_frames > 0 and n >= max_frames:
            break

        tar_path, tar_idx_path, day_dash = tar_paths_for_topic_day(info.folder_name, tr.day)
        if not tar_path.exists() or not tar_idx_path.exists():
            continue

        tar_index = load_tar_index(tar_idx_path)
        log_name = f"{day_dash}/trip_{tr.trip_id:02d}.log"
        idx_name = f"{day_dash}/trip_{tr.trip_id:02d}.idx"
        if log_name not in tar_index or idx_name not in tar_index:
            continue

        log_off, log_sz = tar_index[log_name]
        idx_off, idx_sz = tar_index[idx_name]

        try:
            tar_fd = os.open(str(tar_path), os.O_RDONLY | getattr(os, "O_CLOEXEC", 0))
        except OSError:
            continue

        try:
            idx_entries = list(iter_index_entries_from_tar(tar_fd, idx_off, idx_sz))
            prefix = f"HDD_TAR:{sensor}:{day_dash}:trip{tr.trip_id:02d}"
            it = iter_record_sizes_for_trip_in_tar(
                tar_fd, log_off, log_sz, idx_entries, t_start_ns, t_end_ns, prefix
            )

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
        finally:
            os.close(tar_fd)

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


def main() -> int:
    if len(sys.argv) < 5:
        sys.stderr.write(
            "Usage\n"
            "  python3 cold_retrieve_report.py sensor_topic t_start_ns t_end_ns mode [max_frames]\n"
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
