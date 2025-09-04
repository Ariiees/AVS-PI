#!/usr/bin/env python3
import argparse
import os
import shlex
import subprocess
import sys
import threading
import time
from typing import Dict, Set

import psutil
from pathlib import Path

# --------- Helpers ---------
def tee_stream(src, dst):
    for line in src:
        dst.write(line)
        dst.flush()

def find_default_binary() -> str:
    # Common AVS install/build locations
    candidates = [
        "/home/avs/AVS-PI/install/avs/lib/avs/archive_move",
        "/home/avs/AVS-PI/install/avs/lib/avs/move_manager",
        "/home/avs/AVS-PI/build/avs/archive_move",
        "/home/avs/AVS-PI/build/avs/move_manager",
    ]
    for p in candidates:
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return ""

def alive(p: psutil.Process) -> bool:
    try:
        return p.is_running() and p.status() != psutil.STATUS_ZOMBIE
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return False

def proc_tree(root: psutil.Process) -> Set[psutil.Process]:
    out = set()
    if alive(root):
        out.add(root)
        try:
            for c in root.children(recursive=True):
                if alive(c):
                    out.add(c)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return out

def cpu_time_sum(p: psutil.Process) -> float:
    # sum user+system seconds for one process
    try:
        ct = p.cpu_times()
        return float(ct.user + ct.system)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return 0.0

# --------- Main ---------
def main():
    ap = argparse.ArgumentParser(description="Benchmark archive_move: avg CPU% and memory over entire run.")
    ap.add_argument("--before", required=True, help="Cutoff date: YYYY-MM-DD (passed to the binary).")
    ap.add_argument("--binary", help="Path to archive binary (default: auto-detect).")
    ap.add_argument("--interval", type=float, default=0.1, help="RSS sampling interval seconds (default 0.1)")
    args = ap.parse_args()

    binary = args.binary or find_default_binary()
    if not binary:
        print("[ERR] Could not auto-locate the archive binary. Pass --binary PATH.", file=sys.stderr)
        sys.exit(2)
    if not (os.path.isfile(binary) and os.access(binary, os.X_OK)):
        print(f"[ERR] Binary not executable: {binary}", file=sys.stderr)
        sys.exit(2)

    cmd = [binary, "--before", args.before]
    print(f"[INFO] Running: {' '.join(shlex.quote(c) for c in cmd)}")

    # Launch in text mode to avoid buffering warnings; line-buffered
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=1,
        text=True,
        close_fds=True,
    )

    t_out = threading.Thread(target=tee_stream, args=(proc.stdout, sys.stdout), daemon=True)
    t_err = threading.Thread(target=tee_stream, args=(proc.stderr, sys.stderr), daemon=True)
    t_out.start(); t_err.start()

    root = psutil.Process(proc.pid)

    # Prime CPU% (not strictly needed since we derive CPU from cpu_times, but harmless)
    try:
        root.cpu_percent(None)
    except psutil.Error:
        pass

    # Track CPU times per PID to compute accurate average CPU%
    # We collect any new children as they appear and store their starting cpu_times.
    start_wall = time.time()
    seen: Dict[int, float] = {}   # pid -> start_cpu_time
    final: Dict[int, float] = {}  # pid -> end_cpu_time

    # Memory samples (aggregate RSS across tree)
    rss_samples = []
    rss_max = 0.0

    # Small warmup to let children spawn (if any)
    time.sleep(0.02)

    # Initialize CPU start times for current tree
    for p in proc_tree(root):
        try:
            seen[p.pid] = cpu_time_sum(p)
        except Exception:
            pass

    # Sample loop until process exits and no descendants remain
    while True:
        ret = proc.poll()
        # Refresh process tree
        tree = proc_tree(root)

        # Record start cpu for any new PIDs
        for p in tree:
            if p.pid not in seen:
                seen[p.pid] = cpu_time_sum(p)

        # Sample RSS aggregate
        rss_total = 0.0
        for p in tree:
            try:
                rss_total += p.memory_info().rss
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        if tree:
            rss_mb = rss_total / (1024 * 1024)
            rss_samples.append(rss_mb)
            if rss_mb > rss_max:
                rss_max = rss_mb

        # Stop when launcher exited AND no more descendants are alive
        if ret is not None and not tree:
            break

        time.sleep(max(0.02, args.interval))

    end_wall = time.time()

    # Gather final cpu times for all seen PIDs (even if they died we try/catch)
    for pid, start_ct in seen.items():
        try:
            p = psutil.Process(pid)
            end_ct = cpu_time_sum(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            # If it already died, we can’t query; assume last known end time equals start (best effort)
            # psutil doesn’t expose historical cpu_times; to avoid undercounting too much, we ignore
            # here and rely on most PIDs being sampled while alive.
            end_ct = start_ct
        final[pid] = end_ct

    # Compute totals
    wall = max(1e-6, end_wall - start_wall)
    total_cpu_sec = 0.0
    for pid in seen:
        total_cpu_sec += max(0.0, final.get(pid, seen[pid]) - seen[pid])

    # psutil’s per-process % is relative to a single CPU (can exceed 100 on multicore).
    avg_cpu_percent = 100.0 * (total_cpu_sec / wall)

    rss_avg = (sum(rss_samples) / len(rss_samples)) if rss_samples else 0.0

    status = "SUCCESS" if proc.returncode == 0 else f"EXIT({proc.returncode})"

    print("\n========== Archive Benchmark ==========")
    print(f"Command:   {' '.join(shlex.quote(c) for c in cmd)}")
    print(f"Status:    {status}")
    print(f"Duration:  {wall:.2f} s")
    print("---------------------------------------")
    print("CPU% (sum over proc tree, avg over wall time):")
    print(f"  avg {avg_cpu_percent:.2f}%")
    print("---------------------------------------")
    print("Memory RSS (MB, sum over proc tree):")
    print(f"  avg {rss_avg:.2f}    max {rss_max:.2f}    samples {len(rss_samples)}")
    print("=======================================\n")

if __name__ == "__main__":
    main()
