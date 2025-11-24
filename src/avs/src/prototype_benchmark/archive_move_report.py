#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
import time
import threading
from pathlib import Path
from typing import List, Tuple, Set, Optional

import psutil

# -------- Fixed roots (as requested) --------
SSD_ROOT_DEFAULT = "/home/avs/DATA/SSD"   # nvme0n1p3
HDD_ROOT_DEFAULT = "/home/avs/DATA/HDD"   # sda1

WRAPPERS = {"ros2", "python", "python3", "bash", "sh", "zsh"}

# -------- Helpers --------
def bytes_to_mb(b: int) -> float:
    """Decimal MB: 1 MB = 1,000,000 bytes."""
    return b / 1_000_000.0

def dir_size_bytes(root: Path) -> int:
    """Sum sizes of all regular files under root (recursive)."""
    total = 0
    try:
        for base, dirs, files in os.walk(root, followlinks=False):
            for name in files:
                try:
                    total += (Path(base) / name).stat().st_size
                except Exception:
                    pass
    except Exception:
        pass
    return total

def human_mb(n_bytes: int) -> float:
    return round(bytes_to_mb(n_bytes), 2)

# ---- Process utilities (modeled after your subscriber benchmark) ----
def proc_tree(p: psutil.Process) -> List[psutil.Process]:
    """Return list of p + all recursive children, alive and non-zombie only (deduped)."""
    out = []
    try:
        if p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
            out.append(p)
            for c in p.children(recursive=True):
                try:
                    if c.is_running() and c.status() != psutil.STATUS_ZOMBIE:
                        out.append(c)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        pass
    return list({x.pid: x for x in out}.values())

def prime_cpu_for(procs: List[psutil.Process]) -> None:
    for p in procs:
        try:
            p.cpu_percent(None)
        except Exception:
            pass

def is_wrapper_proc(p: psutil.Process) -> bool:
    try:
        base = os.path.basename(p.exe()) if p.exe() else (p.name() or "")
        return base in WRAPPERS
    except Exception:
        return False

def leaf_execs_of_launcher(launcher_pid: int) -> List[psutil.Process]:
    """Return non-wrapper leaf processes spawned under the launcher (e.g., actual archive_move binary)."""
    try:
        root = psutil.Process(launcher_pid)
        kids = root.children(recursive=True)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return []
    leaves = []
    for k in kids:
        try:
            if not k.children(recursive=True) and not is_wrapper_proc(k):
                leaves.append(k)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return list({p.pid: p for p in leaves}.values())

def sample_all_trees(roots: List[psutil.Process]) -> Tuple[float, float, Set[int]]:
    """
    Sum CPU% and RSS over the union of all trees rooted at `roots`.
    Returns (cpu_total_percent, rss_total_mb, pid_set).
    """
    pid_seen: Set[int] = set()
    cpu_total = 0.0
    rss_total = 0
    for r in roots:
        for q in proc_tree(r):
            if q.pid in pid_seen:
                continue
            pid_seen.add(q.pid)
            try:
                cpu_total += q.cpu_percent(None)
            except Exception:
                pass
            try:
                rss_total += q.memory_info().rss
            except Exception:
                pass
    return cpu_total, bytes_to_mb(rss_total), pid_seen

# -------- Streaming tee of child output --------
def tee_child_output(proc: subprocess.Popen, report_holder: dict) -> None:
    """
    Read child's merged stdout/stderr line-by-line, print through, and
    remember the last '[REPORT] Total archive latency:' line.
    """
    try:
        assert proc.stdout is not None
        for raw in iter(proc.stdout.readline, ''):
            # Print through immediately to preserve child logs
            print(raw, end='')
            if raw.startswith("[REPORT] Total archive latency:"):
                report_holder["line"] = raw.strip()
    except Exception:
        # Swallow read errors; main loop handles process lifecycle
        pass

# -------- Main --------
def parse_args():
    ap = argparse.ArgumentParser(description="Benchmark 'ros2 run avs archive_move' end-to-end (top-level SSD/HDD deltas), while also echoing the child's [REPORT] line.")
    ap.add_argument("--before", required=True, help="Cutoff day YYYY-MM-DD to pass to archive_move.")
    ap.add_argument("--interval", type=float, default=1.0, help="Sampling interval seconds (default: 1.0).")
    ap.add_argument("--ros2-pkg",   type=str, default="avs", help="ROS 2 package name (default: avs).")
    ap.add_argument("--ros2-exe",   type=str, default="archive_move", help="ROS 2 executable name (default: archive_move).")
    return ap.parse_args()

def main():
    args = parse_args()

    ssd_root = Path(SSD_ROOT_DEFAULT)
    hdd_root = Path(HDD_ROOT_DEFAULT)

    # Pre sizes
    ssd_before = dir_size_bytes(ssd_root)
    hdd_before = dir_size_bytes(hdd_root)

    # Launch (capture merged stdout/stderr to tee)
    cmd = ["ros2", "run", args.ros2_pkg, args.ros2_exe, "--before", args.before]
    print(f"[INFO] Launching: {' '.join(cmd)}")

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,  # line-buffered
    )

    # Tee thread to forward child output and capture [REPORT] line
    report_holder: dict = {"line": None}  # will store the last matching line
    tee_thread = threading.Thread(target=tee_child_output, args=(proc, report_holder), daemon=True)
    tee_thread.start()

    # Determine measurement roots: prefer leaf executables; fallback to launcher itself
    time.sleep(1.5)  # let children spawn
    roots = leaf_execs_of_launcher(proc.pid)
    if not roots:
        try:
            roots = [psutil.Process(proc.pid)]
        except psutil.NoSuchProcess:
            roots = []

    # Prime CPU counters for all trees
    initial_procs: List[psutil.Process] = []
    for r in roots:
        initial_procs.extend(proc_tree(r))
    prime_cpu_for(initial_procs)

    cpu_samples: List[float] = []
    rss_samples: List[float] = []
    pid_union: Set[int] = set()

    # Sample until process exits
    while True:
        if proc.poll() is not None:
            break
        time.sleep(args.interval)
        cpu, rss_mb, pid_set = sample_all_trees(roots)
        cpu_samples.append(cpu)
        rss_samples.append(rss_mb)
        pid_union.update(pid_set)

    # Ensure all child output has been consumed
    try:
        tee_thread.join(timeout=2.0)
    except Exception:
        pass

    # Post sizes
    ssd_after = dir_size_bytes(ssd_root)
    hdd_after = dir_size_bytes(hdd_root)

    # Deltas
    ssd_delta_mb = max(0.0, bytes_to_mb(ssd_before - ssd_after))
    hdd_delta_mb = max(0.0, bytes_to_mb(hdd_after - hdd_before))
    moved_mb_est = min(ssd_delta_mb, hdd_delta_mb)

    # Stats
    avg_cpu = round(sum(cpu_samples) / len(cpu_samples), 2) if cpu_samples else 0.0
    peak_cpu = round(max(cpu_samples), 2) if cpu_samples else 0.0
    avg_rss = round(sum(rss_samples) / len(rss_samples), 2) if rss_samples else 0.0
    peak_rss = round(max(rss_samples), 2) if rss_samples else 0.0

    # Report (your wrapper report)
    print("\n=== Archive Move Benchmark (Top-level) ===")
    print(f"Cutoff day:               {args.before}")
    print(f"Avg CPU (sum):            {avg_cpu:.2f} %")
    print(f"Peak CPU (sum):           {peak_cpu:.2f} %")
    print(f"Avg RSS (sum):            {avg_rss:.2f} MB")
    print(f"Peak RSS (sum):           {peak_rss:.2f} MB")
    print("---- Storage Effects ----")
    print(f"SSD decrease (nvme0n1p3): {ssd_delta_mb:.2f} MB   under {ssd_root}")
    print(f"HDD increase (sda1):      {hdd_delta_mb:.2f} MB   under {hdd_root}")
    print(f"Estimated moved:          {moved_mb_est:.2f} MB   (min(SSD decrease, HDD increase))")
    print("---- Debug ----")
    try:
        root_pids = [r.pid for r in roots]
    except Exception:
        root_pids = []
    print(f"Roots considered:         {root_pids}")
    print(f"Union PIDs seen:          {len(pid_union)} processes")

    # ALSO print the child's [REPORT] line (if present) so it's visible in your summary too
    if report_holder.get("line"):
        print(report_holder["line"])

    return proc.returncode if proc.returncode is not None else 0

if __name__ == "__main__":
    sys.exit(main())
