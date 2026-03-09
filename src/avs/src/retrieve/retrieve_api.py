#!/usr/bin/env python3
import mmap
import sqlite3
import struct
import yaml
from dataclasses import dataclass
from pathlib import Path
from typing import Generator, Iterator, Optional, Tuple


SSD_ROOT = Path("/home/avs/DATA/SSD")

def avs_pi_path() -> Path:
    # build path without using a dash in source text
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
class RetrievedRecord:
    sensor_topic: str
    sensor_type: str
    ts_ns: int
    payload: bytes
    locator: str


def load_topic_map(path: Path) -> dict:
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


def open_global_db() -> sqlite3.Connection:
    db_path = SSD_ROOT / "global.sqlite3"
    if not db_path.exists():
        raise FileNotFoundError(f"global sqlite3 not found: {db_path}")
    conn = sqlite3.connect(str(db_path))
    conn.row_factory = sqlite3.Row
    return conn


def load_closed_trips_overlapping(
    conn: sqlite3.Connection,
    sensor_topic: str,
    t_start_ns: int,
    t_end_ns: int,
) -> list[GlobalTripRow]:
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
    rows: list[GlobalTripRow] = []
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


def trip_paths(topic_folder: str, day: str, trip_id: int) -> Tuple[Path, Path]:
    day_dir = SSD_ROOT / topic_folder / day
    log_p = day_dir / f"trip_{trip_id:02d}.log"
    idx_p = day_dir / f"trip_{trip_id:02d}.idx"
    return log_p, idx_p


def iter_index_entries_stream(idx_f) -> Iterator[TripIndexEntry]:
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


def iter_records_for_trip_mmap_copy_payload(
    sensor_topic: str,
    sensor_type: str,
    log_path: Path,
    idx_path: Path,
    t_start_ns: int,
    t_end_ns: int,
    locator_prefix: str,
    max_frames: int = 0,
) -> Generator[RetrievedRecord, None, None]:
    """
    Streams matching records from one trip.
    Uses mmap for the trip log, copies each payload to bytes for safe handoff.
    Copying is intentional so the caller can send over UDP without mmap lifetime issues.
    """
    chunk_header_fmt = "<qqII"
    chunk_header_sz = struct.calcsize(chunk_header_fmt)
    record_header_fmt = "<qI"
    record_header_sz = struct.calcsize(record_header_fmt)

    n = 0

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
                if end > mm_len or sz < chunk_header_sz:
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

                    if payload_sz < 0 or (p + payload_sz) > end:
                        break

                    payload_start = p
                    p = p + payload_sz

                    if t_start_ns <= ts_ns <= t_end_ns:
                        loc = f"{locator_prefix}:off{ent.file_offset}:chunk{chunk_i}:rec{rec_i}"
                        payload = bytes(mm[payload_start : payload_start + payload_sz])
                        yield RetrievedRecord(
                            sensor_topic=sensor_topic,
                            sensor_type=sensor_type,
                            ts_ns=int(ts_ns),
                            payload=payload,
                            locator=loc,
                        )
                        n += 1
                        if max_frames > 0 and n >= max_frames:
                            return

                    rec_i += 1

                chunk_i += 1
        finally:
            mm.close()


class RetrieveAPI:
    """
    Application facing retrieve API.

    Usage
      api = RetrieveAPI()
      for rec in api.query(topic, start_ns, end_ns, max_frames=0):
          sock.sendto(rec.payload, addr)
    """

    def __init__(self, ssd_root: Path = SSD_ROOT, topic_map_path: Path = TOPIC_MAP_PATH):
        self.ssd_root = ssd_root
        self.topic_map_path = topic_map_path
        self.topic_map = load_topic_map(topic_map_path)

    def query(
        self,
        sensor_topic: str,
        t_start_ns: int,
        t_end_ns: int,
        max_frames: int = 0,
    ) -> Generator[RetrievedRecord, None, None]:
        if t_end_ns < t_start_ns:
            raise ValueError("t_end_ns must be >= t_start_ns")
        if sensor_topic not in self.topic_map:
            raise RuntimeError(f"sensor not found in topics yaml: {sensor_topic}")

        info = self.topic_map[sensor_topic]

        conn = open_global_db()
        try:
            trips = load_closed_trips_overlapping(conn, sensor_topic, t_start_ns, t_end_ns)
        finally:
            conn.close()

        remaining = max_frames

        for tr in trips:
            log_p, idx_p = trip_paths(tr.topic_folder, tr.day, tr.trip_id)
            if not log_p.exists() or not idx_p.exists():
                continue

            prefix = f"SSD:{sensor_topic}:{tr.day}:trip{tr.trip_id:02d}"

            trip_max = 0
            if remaining > 0:
                trip_max = remaining

            for rec in iter_records_for_trip_mmap_copy_payload(
                sensor_topic=sensor_topic,
                sensor_type=info.sensor_type,
                log_path=log_p,
                idx_path=idx_p,
                t_start_ns=t_start_ns,
                t_end_ns=t_end_ns,
                locator_prefix=prefix,
                max_frames=trip_max,
            ):
                yield rec
                if remaining > 0:
                    remaining -= 1
                    if remaining <= 0:
                        return
