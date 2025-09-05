#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AVS retrieval benchmark — via retrieve_cli --bench (Table 1 only)
Now supports modalities: image, lidar, gps, both (image+lidar), all (image+lidar+gps).
"""

import argparse
import datetime as dt
import re
import shlex
import subprocess
import sys
import time
import random
import bisect
from collections import namedtuple, defaultdict
from typing import List, Tuple, Dict

AvsRec = namedtuple("AvsRec", "sensor_id data_type ts_ms path")

# -------- time utils --------
def parse_wall(s: str) -> int:
    fmts = ("%Y-%m-%d_%H-%M-%S", "%Y-%m-%d_%H-%M")
    for fmt in fmts:
        try:
            t = dt.datetime.strptime(s, fmt)
            return int(t.timestamp() * 1e9)
        except ValueError:
            pass
    date, clock = s.split("_", 1)
    y, m, d = [int(x) for x in date.split("-")]
    parts = [int(x) for x in clock.split("-")]
    H, M, S = (parts + [0, 0, 0])[:3]
    t = dt.datetime(y, m, d, H, M, S)
    return int(t.timestamp() * 1e9)

def floor_minute_str_from_ns(ns: int) -> str:
    t = dt.datetime.fromtimestamp(ns / 1e9).replace(second=0, microsecond=0)
    return t.strftime("%Y-%m-%d_%H-%M")

def ceil_minute_str_from_ns(ns: int) -> str:
    t = dt.datetime.fromtimestamp(ns / 1e9)
    if t.second or t.microsecond:
        t = (t + dt.timedelta(minutes=1)).replace(second=0, microsecond=0)
    else:
        t = t.replace(second=0, microsecond=0)
    return t.strftime("%Y-%m-%d_%H-%M")

def ensure_minute_span(t0_ns: int, t1_ns: int) -> Tuple[str, str]:
    t0 = floor_minute_str_from_ns(t0_ns)
    t1 = ceil_minute_str_from_ns(t1_ns)
    if t1 == t0:
        t1_dt = dt.datetime.strptime(t1, "%Y-%m-%d_%H-%M") + dt.timedelta(minutes=1)
        t1 = t1_dt.strftime("%Y-%m-%d_%H-%M")
    return t0, t1

def percentile(vals: List[float], p: float) -> float:
    if not vals: return float("nan")
    v = sorted(vals)
    if p <= 0: return v[0]
    if p >= 100: return v[-1]
    rank = (p / 100.0) * (len(v) - 1)
    lo = int(rank); hi = min(lo + 1, len(v) - 1)
    frac = rank - lo
    return v[lo] + (v[hi] - v[lo]) * frac

# -------- retrieve_cli interop --------
_LINE_RE = re.compile(r"^\s*([^|]+)\|\s*([^|]+)\|\s*(\d+)\s*\|\s*(.+?)\s*$")
_BENCH_TTFB = re.compile(r"^TTFB\t([\d.]+)\s*$")
_BENCH_DEC  = re.compile(r"^DECODE\t(\d+)\t([\d.]+)\t(\d+)\s*$")

def run(cmd: str) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

def run_retrieve_cli_list(cli: str, data: str, start_wall: str, end_wall: str,
                          sensor: str = "", image_db: str = "", lidar_db: str = "",
                          gps_root: str = "", debug: bool = False) -> List[AvsRec]:
    args = [cli, "--data", data, "--start", start_wall, "--end", end_wall, "--list"]
    if sensor: args += ["--sensor", sensor]
    if data == "image" and image_db: args += ["--image-db", image_db]
    if data == "lidar" and lidar_db: args += ["--lidar-db", lidar_db]
    if data == "gps" and gps_root:   args += ["--gps-root", gps_root]
    cmd = " ".join(shlex.quote(x) for x in args)
    proc = run(cmd)
    if proc.returncode != 0:
        if debug:
            print(f"[retrieve_cli ERR] rc={proc.returncode}\n{proc.stderr}", file=sys.stderr)
        return []
    recs: List[AvsRec] = []
    for raw in proc.stdout.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"): continue
        m = _LINE_RE.match(line)
        if not m:
            if debug: print(f"[DEBUG] skip unparsable: {line}")
            continue
        sid, dtype, ts, path = (m.group(1).strip(), m.group(2).strip(), m.group(3).strip(), m.group(4).strip())
        try:
            ts_ms = int(ts)
        except:
            if debug: print(f"[DEBUG] bad ts: {line}")
            continue
        recs.append(AvsRec(sid, dtype, ts_ms, path))
    recs.sort(key=lambda r: (r.ts_ms, r.path))
    if debug: print(f"[DEBUG] {data}: parsed {len(recs)} rows {start_wall}..{end_wall}")
    return recs

def run_retrieve_cli_bench(cli: str, data: str, t0: str, t1: str, sensor: str,
                           image_db: str = "", lidar_db: str = "", gps_root: str = "",
                           max_items: int = 0, debug: bool = False) -> Tuple[float, List[float]]:
    """
    Returns: (ttfb_ms, steady_lat_ms list)
    """
    args = [cli, "--data", data, "--start", t0, "--end", t1, "--bench"]
    if sensor: args += ["--sensor", sensor]
    if data == "image" and image_db: args += ["--image-db", image_db]
    if data == "lidar" and lidar_db: args += ["--lidar-db", lidar_db]
    if data == "gps"   and gps_root: args += ["--gps-root", gps_root]
    if max_items > 0: args += ["--max", str(max_items)]
    cmd = " ".join(shlex.quote(x) for x in args)

    proc = run(cmd)

    ttfb_ms = float("nan")
    lats_ms: List[float] = []

    for line in proc.stdout.splitlines():
        if not line: continue
        m1 = _BENCH_TTFB.match(line)
        if m1:
            try: ttfb_ms = float(m1.group(1))
            except: pass
            continue
        m2 = _BENCH_DEC.match(line)
        if m2:
            try:
                lat = float(m2.group(2))
                lats_ms.append(lat)
            except: pass

    if debug and (ttfb_ms != ttfb_ms) and not lats_ms:
        sys.stderr.write(f"[DEBUG] bench produced no samples for {data} {t0}..{t1}\n")
        if proc.stderr:
            sys.stderr.write(proc.stderr[:1000] + "\n")

    return ttfb_ms, lats_ms

# -------- windowing --------
def choose_windows_from_recs(recs: List[AvsRec], n: int, dur_s: int, min_items: int = 2, seed: int = 42) -> List[Tuple[int,int]]:
    if not recs: return []
    rnd = random.Random(seed)
    ts_ms = [r.ts_ms for r in recs]
    tmin, tmax = min(ts_ms), max(ts_ms)
    dur_ns = int(max(1, dur_s)*1e9)
    windows: List[Tuple[int,int]] = []
    attempts = 0
    while len(windows) < n and attempts < n*30:
        attempts += 1
        anchor_ms = rnd.randrange(tmin, tmax+1)
        t0_ns = anchor_ms * 1_000_000
        t1_ns = t0_ns + dur_ns
        lo = bisect.bisect_left(ts_ms, t0_ns // 1_000_000)
        hi = bisect.bisect_right(ts_ms, t1_ns // 1_000_000)
        if (hi - lo) >= min_items:
            windows.append((t0_ns, t1_ns))
    # de-dup coarse
    uniq, seen = [], set()
    for t0, t1 in windows:
        key = (t0 // 1_000_000, t1 // 1_000_000)
        if key not in seen: seen.add(key); uniq.append((t0, t1))
    return uniq[:n]

def filter_recs_window(recs: List[AvsRec], t0_ns: int, t1_ns: int) -> List[AvsRec]:
    lo_ms, hi_ms = t0_ns // 1_000_000, t1_ns // 1_000_000
    return [r for r in recs if lo_ms <= r.ts_ms <= hi_ms]

def fmt(x, prec=2):
    return "n/a" if (x is None or x != x) else f"{x:.{prec}f}"

def main():
    ap = argparse.ArgumentParser(description="AVS retrieval benchmark via retrieve_cli --bench (Table 1 only)")
    ap.add_argument("--avs-image-db")
    ap.add_argument("--avs-lidar-db")
    ap.add_argument("--gps-root", default="/home/avs/DATA/SSD/gps")
    ap.add_argument("--retrieve-cli", default="retrieve_cli")

    ap.add_argument("--image-topic")
    ap.add_argument("--lidar-topic")

    ap.add_argument("--start", required=True)   
    ap.add_argument("--end",   required=True)

    ap.add_argument("--windows", type=int, default=6)
    ap.add_argument("--dur", type=int, default=75, help="window duration in seconds (auto-bumped to ≥65 if <60)")
    ap.add_argument("--modality", choices=["image","lidar","gps","both","all"], default="both")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--max-per-window", type=int, default=0, help="max steady items per window (0=all)")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    # enforce sane duration
    if args.dur < 60:
        if args.debug:
            print(f"[DEBUG] --dur {args.dur}s < 60s; bumping to 65s to ensure ≥1-minute span", file=sys.stderr)
        args.dur = 65

    start_ns = parse_wall(args.start)
    end_ns   = parse_wall(args.end)
    if end_ns <= start_ns:
        print("[FATAL] end <= start", file=sys.stderr); sys.exit(2)

    # Build modality list
    if args.modality == "both":
        modalities: List[str] = ["image", "lidar"]
    elif args.modality == "all":
        modalities = ["image", "lidar", "gps"]
    else:
        modalities = [args.modality]

    # Validate required args per modality
    if "image" in modalities and not (args.avs_image_db and args.image_topic):
        print("[FATAL] image modality requires --avs-image-db and --image-topic", file=sys.stderr); sys.exit(2)
    if "lidar" in modalities and not (args.avs_lidar_db and args.lidar_topic):
        print("[FATAL] lidar modality requires --avs-lidar-db and --lidar-topic", file=sys.stderr); sys.exit(2)
    if "gps" in modalities and not args.gps_root:
        print("[FATAL] gps modality requires --gps-root", file=sys.stderr); sys.exit(2)

    agg_ttfb: Dict[str, List[float]] = defaultdict(list)
    agg_lat:  Dict[str, List[float]] = defaultdict(list)

    for modality in modalities:
        if modality == "image":
            all_recs = run_retrieve_cli_list(
                args.retrieve_cli, "image", args.start, args.end,
                sensor=args.image_topic, image_db=args.avs_image_db, debug=args.debug
            )
            if not all_recs:  # fallback
                all_recs = run_retrieve_cli_list(args.retrieve_cli, "image", args.start, args.end,
                                                 sensor="", image_db=args.avs_image_db, debug=args.debug)
        elif modality == "lidar":
            all_recs = run_retrieve_cli_list(
                args.retrieve_cli, "lidar", args.start, args.end,
                sensor=args.lidar_topic, lidar_db=args.avs_lidar_db, debug=args.debug
            )
            if not all_recs:
                all_recs = run_retrieve_cli_list(args.retrieve_cli, "lidar", args.start, args.end,
                                                 sensor="", lidar_db=args.avs_lidar_db, debug=args.debug)
        else:  # gps
            all_recs = run_retrieve_cli_list(
                args.retrieve_cli, "gps", args.start, args.end,
                sensor="", gps_root=args.gps_root, debug=args.debug
            )

        if args.debug:
            print(f"[DEBUG] {modality}: total rows={len(all_recs)} in full range")

        if not all_recs:
            continue

        windows = choose_windows_from_recs(all_recs, args.windows, args.dur, min_items=2, seed=args.seed)
        if args.debug:
            print(f"[DEBUG] {modality}: picked {len(windows)} windows (dur={args.dur}s)")

        for (t0_ns, t1_ns) in windows:
            win_recs = filter_recs_window(all_recs, t0_ns, t1_ns)
            if not win_recs: continue

            t0, t1 = ensure_minute_span(t0_ns, t1_ns)

            if modality == "image":
                ttfb_ms, lats_ms = run_retrieve_cli_bench(
                    args.retrieve_cli, "image", t0, t1, args.image_topic,
                    image_db=args.avs_image_db, max_items=args.max_per_window, debug=args.debug
                )
            elif modality == "lidar":
                ttfb_ms, lats_ms = run_retrieve_cli_bench(
                    args.retrieve_cli, "lidar", t0, t1, args.lidar_topic,
                    lidar_db=args.avs_lidar_db, max_items=args.max_per_window, debug=args.debug
                )
            else:  # gps
                ttfb_ms, lats_ms = run_retrieve_cli_bench(
                    args.retrieve_cli, "gps", t0, t1, "",
                    gps_root=args.gps_root, max_items=args.max_per_window, debug=args.debug
                )

            if ttfb_ms == ttfb_ms: agg_ttfb[modality].append(ttfb_ms)
            agg_lat[modality].extend([x for x in lats_ms if x == x])

    print("\n=== Table: TTFB and Latency Percentiles (ms) ===")
    print("{:<12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}".format(
        "Modality", "TTFB_p50", "TTFB_p95", "TTFB_p99", "Lat_p50", "Lat_p95", "Lat_p99"
    ))
    for modality in modalities:
        ttfb = agg_ttfb.get(modality, [])
        lat  = agg_lat.get(modality, [])
        row = [
            modality,
            fmt(percentile(ttfb, 50), 4),
            fmt(percentile(ttfb, 95), 4),
            fmt(percentile(ttfb, 99), 4),
            fmt(percentile(lat, 50), 4),
            fmt(percentile(lat, 95), 4),
            fmt(percentile(lat, 99), 4),
        ]
        print("{:<12} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}".format(*row))
    print("\nDone.\n")

if __name__ == "__main__":
    main()
