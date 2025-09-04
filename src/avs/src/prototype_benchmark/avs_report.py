#!/usr/bin/env python3
"""
Benchmark average CPU and memory usage for AVS subscriber nodes (image/lidar/gps),
and print a final report matching the rosbag_report layout.

- Default: launch via `ros2 run avs ...` and benchmark.
- --attach: attach to already-running node executables.
- Data size growth is measured on /home/avs/DATA/SSD.

Examples:
  ./bench_avs_procs.py --duration 45
  ./bench_avs_procs.py --attach --duration 60 --interval 0.5 --csv /tmp/subs.csv
  ROS_NAMESPACE=car1 ./bench_avs_procs.py --duration 30
  ./bench_avs_procs.py --duration 30 --namespace car1
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import psutil

TARGET_EXES = ["image_subscriber", "lidar_subscriber", "gps_subscriber"]
WRAPPERS = {"ros2", "python", "python3"}

SSD_ROOT = Path("/home/avs/DATA/SSD")

# ---------------- Utilities ----------------

def human_mb(n_bytes: int) -> float:
    return round(n_bytes / (1024.0 * 1024.0), 2)

def dir_size_bytes(root: Path) -> int:
    total = 0
    if not root.exists():
        return 0
    for p in root.rglob("*"):
        if p.is_file():
            try:
                total += p.stat().st_size
            except FileNotFoundError:
                pass
    return total

def start_proc(cmd):
    """Start a process in its own group for clean shutdown."""
    return subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

def terminate_group(popen_obj):
    """Try graceful then hard termination of a process group."""
    try:
        os.killpg(os.getpgid(popen_obj.pid), signal.SIGINT)
        time.sleep(1.0)
    except Exception:
        pass
    try:
        os.killpg(os.getpgid(popen_obj.pid), signal.SIGTERM)
    except Exception:
        pass
    try:
        time.sleep(0.5)
        os.killpg(os.getpgid(popen_obj.pid), signal.SIGKILL)
    except Exception:
        pass

def find_procs_by_names(names):
    """
    Return only the actual node executables, not 'ros2' wrappers.
    Strategy:
      - Prefer basename of p.exe() matching one of names
      - Fallback: last token in cmdline equals one of names
      - Exclude wrappers: 'ros2', 'python', 'python3'
    """
    procs = []
    for p in psutil.process_iter(["name", "exe", "cmdline"]):
        try:
            exe_path = p.info.get("exe") or ""
            exe_base = os.path.basename(exe_path) if exe_path else ""
            cmdline  = p.info.get("cmdline") or []
            pname    = p.info.get("name") or ""

            if pname in WRAPPERS or exe_base in WRAPPERS:
                continue

            match = False
            if exe_base and exe_base in names:
                match = True
            else:
                if cmdline:
                    last = os.path.basename(cmdline[-1])
                    if last in names:
                        match = True

            if match:
                procs.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    # Dedup by PID
    return list({p.pid: p for p in procs}.values())

def leaf_children_of_launchers(popens, names):
    """When launched via `ros2`, get the leaf children that match target executables."""
    leaf = []
    for pop in popens:
        try:
            kids = psutil.Process(pop.pid).children(recursive=True)
            for k in kids:
                try:
                    base = os.path.basename(k.exe())
                    if base in names and base not in WRAPPERS:
                        leaf.append(k)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return list({p.pid: p for p in leaf}.values())

def proc_tree(p: psutil.Process):
    """Return list of p + all recursive children, alive and non-zombie only."""
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
    # Dedup by PID
    return list({x.pid: x for x in out}.values())

def prime_cpu(procs):
    for p in procs:
        try:
            _ = p.cpu_percent(None)
        except Exception:
            pass

def sample_all_trees(roots):
    """
    Sum CPU% and RSS over the union of all trees rooted at `roots`.
    Returns cpu_total, rss_total_mb, pid_set
    """
    pid_seen = set()
    cpu_total = 0.0
    rss_total = 0
    for r in roots:
        for q in proc_tree(r):
            pid_seen.add(q.pid)
            try:
                cpu_total += q.cpu_percent(None)
            except Exception:
                pass
            try:
                rss_total += q.memory_info().rss
            except Exception:
                pass
    return cpu_total, human_mb(rss_total), pid_seen

# ---------------- Main ----------------

def main():
    ap = argparse.ArgumentParser(description="Benchmark avg CPU & memory for AVS image/lidar/gps subscribers (report-matched).")
    ap.add_argument("--duration", type=float, default=30.0, help="Benchmark duration in seconds (default: 30)")
    ap.add_argument("--interval", type=float, default=1.0, help="Sampling interval in seconds (default: 1.0)")
    ap.add_argument("--attach", action="store_true", help="Attach to existing processes instead of launching them")
    ap.add_argument("--csv", type=Path, default=None, help="Optional CSV output path for per-sample totals")
    ap.add_argument("--namespace", default="", help="Optional ROS namespace to pass when launching")
    args = ap.parse_args()

    popens = []
    roots  = []  # root executables (image_subscriber, lidar_subscriber, gps_subscriber)
    try:
        # Measure SSD size before
        size_before = dir_size_bytes(SSD_ROOT)

        if args.attach:
            print("[INFO] Attaching to existing processes...")
            roots = find_procs_by_names(TARGET_EXES)
            if not roots:
                print("[ERR] No matching processes found. Are the nodes running?")
                sys.exit(1)
        else:
            print("[INFO] Launching processes via `ros2 run avs ...`")
            if args.namespace:
                os.environ["ROS_NAMESPACE"] = args.namespace

            cmds = [
                ["ros2", "run", "avs", "image_subscriber"],
                ["ros2", "run", "avs", "lidar_subscriber"],
                ["ros2", "run", "avs", "gps_subscriber"],
            ]
            for c in cmds:
                popens.append(start_proc(c))

            # Give them time to spawn leaf executables
            time.sleep(2.0)

            # Prefer leaf children of our launchers; fallback to global scan
            roots = leaf_children_of_launchers(popens, TARGET_EXES)
            if not roots:
                roots = find_procs_by_names(TARGET_EXES)

            if not roots:
                print("[ERR] Failed to find launched node executables; check your environment.")
                sys.exit(1)

        # Prime CPU counters for all trees
        all_initial = []
        for r in roots:
            all_initial.extend(proc_tree(r))
        prime_cpu(all_initial)

        if args.csv:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            with args.csv.open("w") as f:
                f.write("timestamp,cpu_total_percent,rss_total_mib,proc_count\n")

        print(f"[INFO] Sampling for {args.duration:.1f}s every {args.interval:.2f}s…")
        start_t = time.time()
        t_end = start_t + args.duration
        cpu_samples, rss_samples = [], []
        pid_union = set()
        exited_early = False

        while time.time() < t_end:
            # If any root vanished early, mark
            for r in list(roots):
                try:
                    if not r.is_running() or r.status() == psutil.STATUS_ZOMBIE:
                        exited_early = True
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    exited_early = True

            cpu, rss_mb, pid_set = sample_all_trees(roots)
            cpu_samples.append(cpu)
            rss_samples.append(rss_mb)
            pid_union.update(pid_set)

            ts = datetime.now().isoformat(timespec="seconds")
            if args.csv:
                with args.csv.open("a") as f:
                    f.write(f"{ts},{cpu:.2f},{rss_mb:.2f},{len(pid_set)}\n")

            time.sleep(max(0.0, args.interval))

        # Averages
        cpu_avg = round((sum(cpu_samples) / len(cpu_samples)) if cpu_samples else 0.0, 2)
        cpu_max = round(max(cpu_samples) if cpu_samples else 0.0, 2)
        rss_avg = round((sum(rss_samples) / len(rss_samples)) if rss_samples else 0.0, 2)
        rss_max = round(max(rss_samples) if rss_samples else 0.0, 2)

        # Size after
        size_after = dir_size_bytes(SSD_ROOT)
        size_delta_mb = human_mb(max(0, size_after - size_before))

        duration = round(time.time() - start_t, 3)
        status_str = "OK" if not exited_early else "one or more nodes exited early"
        root_pids = [r.pid for r in roots]
        try:
            cmdlines = []
            for r in roots:
                try:
                    cmdlines.append(" ".join(r.cmdline()))
                except Exception:
                    pass
            cmdline_str = " | ".join(cmdlines) if cmdlines else "<unavailable>"
        except psutil.Error:
            cmdline_str = "<unavailable>"

        # ---- Final report (matched layout) ----
        print("\n========== AVS Benchmark Report ==========")
        print(f"Mode:                  avs_subscribers")
        print(f"Status:                {status_str}")
        print(f"Root PIDs:             {root_pids if root_pids else '[]'}")
        print(f"All related PIDs:      {sorted(pid_union)}")
        print(f"Command:               {cmdline_str}")
        print(f"Measured dir:          {SSD_ROOT}")
        print(f"Run duration:          {duration} s  (target: {args.duration:.1f} s)")
        print("-----------------------------------------------")
        print("CPU% / Memory (RSS MB):  (summed over all subscriber process trees)")
        print(f"  avg {cpu_avg}% (max {cpu_max}%), RSS avg {rss_avg} MB (max {rss_max} MB)")
        print("-----------------------------------------------")
        print("Data size growth (MB):")
        print(f"  +{size_delta_mb} MB (total {human_mb(size_after)} MB in {SSD_ROOT})")
        print("================================================\n")
        if args.csv:
            print(f"[INFO] CSV written to: {args.csv}")
        print("[INFO] AVS subscriber benchmark complete and report generated.")

    finally:
        # Only terminate if we launched them
        if popens:
            for p in popens:
                try:
                    terminate_group(p)
                except Exception:
                    pass

if __name__ == "__main__":
    main()
