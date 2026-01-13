#!/usr/bin/env python3
import socket
import struct
import time
import os
import threading
import zlib
from dataclasses import dataclass
from typing import List, Tuple

from retrieve_api import RetrieveAPI


MAGIC = b"AVS1"
VERSION = 1

FLAG_START = 1
FLAG_END = 2
FLAG_DATA = 4

HDR_FMT = "<4sHHQQQHH"
HDR_SZ = struct.calcsize(HDR_FMT)

END_FMT = "<QQQQQ"
END_SZ = struct.calcsize(END_FMT)


def monotonic_ns() -> int:
    return time.monotonic_ns()


@dataclass
class ProcSample:
    t_ns: int
    cpu_s: float
    rss_bytes: int


class ProcSampler:
    def __init__(self, interval_s: float = 0.2):
        self.interval_s = interval_s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.samples: list[ProcSample] = []

    @staticmethod
    def _read_proc_times() -> float:
        with open("/proc/self/stat", "r") as f:
            parts = f.read().split()
        utime_ticks = int(parts[13])
        stime_ticks = int(parts[14])
        clk = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        return float(utime_ticks + stime_ticks) / float(clk)

    @staticmethod
    def _read_rss_bytes() -> int:
        rss_kb = 0
        with open("/proc/self/status", "r") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    rss_kb = int(line.split()[1])
                    break
        return rss_kb * 1024

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            t_ns = monotonic_ns()
            cpu_s = self._read_proc_times()
            rss_b = self._read_rss_bytes()
            self.samples.append(ProcSample(t_ns=t_ns, cpu_s=cpu_s, rss_bytes=rss_b))
            time.sleep(self.interval_s)

    def cpu_stats(self) -> Tuple[float, float]:
        if len(self.samples) < 2:
            return 0.0, 0.0
        cpu_percents = []
        for a, b in zip(self.samples[:-1], self.samples[1:]):
            dt = (b.t_ns - a.t_ns) / 1e9
            dc = b.cpu_s - a.cpu_s
            if dt > 0:
                cpu_percents.append(100.0 * dc / dt)
        if not cpu_percents:
            return 0.0, 0.0
        avg = sum(cpu_percents) / float(len(cpu_percents))
        mx = max(cpu_percents)
        return avg, mx

    def rss_stats(self) -> Tuple[float, float]:
        if not self.samples:
            return 0.0, 0.0
        rss_vals = [s.rss_bytes for s in self.samples]
        avg = sum(rss_vals) / float(len(rss_vals))
        mx = max(rss_vals)
        return avg, mx


def pack_header(flags: int, request_id: int, pkt_seq: int, sender_t_ns: int, recs: int) -> bytes:
    return struct.pack(HDR_FMT, MAGIC, VERSION, flags, request_id, pkt_seq, sender_t_ns, recs, 0)


def main() -> int:
    import sys
    if len(sys.argv) < 6:
        print("usage: udp_offload_send.py recv_ip port topic start_ts_ns end_ts_ns [max_frames] [mtu_payload_bytes]")
        return 2

    recv_ip = sys.argv[1]
    port = int(sys.argv[2])
    topic = sys.argv[3]
    start_ns = int(sys.argv[4])
    end_ns = int(sys.argv[5])

    max_frames = int(sys.argv[6]) if len(sys.argv) >= 7 else 0
    mtu_payload = int(sys.argv[7]) if len(sys.argv) >= 8 else 1200

    api = RetrieveAPI()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (recv_ip, port)

    request_id = monotonic_ns() & 0xFFFFFFFFFFFFFFFF
    pkt_seq = 0

    sampler = ProcSampler(interval_s=0.2)
    sampler.start()

    total_records = 0
    total_payload_bytes = 0
    total_packets = 0

    first_send_t_ns = 0
    last_send_t_ns = 0

    try:
        start_t_ns = monotonic_ns()
        sock.sendto(pack_header(FLAG_START, request_id, pkt_seq, start_t_ns, 0), addr)
        pkt_seq += 1

        buf = bytearray()
        rec_entries: List[bytes] = []
        buf_budget = mtu_payload

        def flush_data_packet() -> None:
            nonlocal pkt_seq, total_packets, first_send_t_ns, last_send_t_ns, buf, rec_entries
            if not rec_entries:
                return
            now_ns = monotonic_ns()
            hdr = pack_header(FLAG_DATA, request_id, pkt_seq, now_ns, len(rec_entries))
            payload = b"".join(rec_entries)
            packet = hdr + payload
            sock.sendto(packet, addr)
            if first_send_t_ns == 0:
                first_send_t_ns = now_ns
            last_send_t_ns = now_ns
            total_packets += 1
            pkt_seq += 1
            buf = bytearray()
            rec_entries = []

        for rec in api.query(topic, start_ns, end_ns, max_frames=max_frames):
            payload = rec.payload
            crc = zlib.crc32(payload) & 0xFFFFFFFF
            entry_hdr = struct.pack("<QII", int(rec.ts_ns), int(len(payload)), int(crc))
            entry = entry_hdr + payload

            if len(entry) > buf_budget:
                flush_data_packet()
                if len(entry) > buf_budget:
                    seg_off = 0
                    seg_max = max(1, buf_budget - 16)
                    while seg_off < len(payload):
                        seg = payload[seg_off : seg_off + seg_max]
                        seg_crc = zlib.crc32(seg) & 0xFFFFFFFF
                        seg_hdr = struct.pack("<QII", int(rec.ts_ns), int(len(seg)), int(seg_crc))
                        rec_entries.append(seg_hdr + seg)
                        total_records += 1
                        total_payload_bytes += len(seg)
                        flush_data_packet()
                        seg_off += len(seg)
                    continue

            rec_entries.append(entry)
            total_records += 1
            total_payload_bytes += len(payload)

            used = sum(len(x) for x in rec_entries)
            if used >= buf_budget:
                flush_data_packet()

        flush_data_packet()

        end_t_ns = monotonic_ns()
        end_hdr = pack_header(FLAG_END, request_id, pkt_seq, end_t_ns, 0)
        end_footer = struct.pack(
            END_FMT,
            int(total_records),
            int(total_payload_bytes),
            int(total_packets),
            int(first_send_t_ns),
            int(last_send_t_ns),
        )
        sock.sendto(end_hdr + end_footer, addr)

        sampler.stop()
        cpu_avg, cpu_max = sampler.cpu_stats()
        rss_avg, rss_max = sampler.rss_stats()

        print("")
        print("UDP_OFFLOAD_SENDER_SUMMARY")
        print(f"request_id {request_id}")
        print(f"topic {topic}")
        print(f"start_ts_ns {start_ns}")
        print(f"end_ts_ns {end_ns}")
        print(f"mtu_payload_bytes {mtu_payload}")
        print(f"records_sent {total_records}")
        print(f"bytes_payload_sent {total_payload_bytes}")
        print(f"data_packets_sent {total_packets}")
        print(f"cpu_avg_percent {cpu_avg:.3f}")
        print(f"cpu_max_percent {cpu_max:.3f}")
        print(f"rss_avg_bytes {int(rss_avg)}")
        print(f"rss_max_bytes {int(rss_max)}")
        print("")
    finally:
        try:
            sampler.stop()
        except Exception:
            pass
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
