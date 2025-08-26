#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from collections import deque

import psutil

# ROS 2 (Jazzy-friendly)
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from std_msgs.msg import Int64

# --------- Identify processes and launch commands per mode ----------
IMAGE_PATTERNS = ["image_subscriber", "img_dedup_node", "image_subscribe"]
LIDAR_PATTERNS = ["lidar_subscriber", "lidar_dedup_node", "lidar_process"]
GPS_PATTERNS   = ["gps_subscriber"]

# Prefer exact executable names for the real node binaries
EXEC_NAMES = {
    "image": ["image_subscriber", "img_dedup_node", "image_subscribe"],
    "lidar": ["lidar_subscriber", "lidar_dedup_node", "lidar_process"],
    "gps":   ["gps_subscriber"],
}

LAUNCH_CMDS = {
    "image": "ros2 run avs image_subscriber",
    "lidar": "ros2 run avs lidar_subscriber",
    "gps":   "ros2 run avs gps_subscriber",
}

# Root data directories (adjust if your paths differ)
IMG_ROOT   = Path("/home/avs/DATA/SSD/images")
LIDAR_ROOT = Path("/home/avs/DATA/SSD/lidar")
GPS_ROOT   = Path("/home/avs/DATA/SSD/gps")  # gps_subscriber writes <YYYY-MM-DD>.sqlite here

# Latency topics published by your AVS nodes
AVS_LAT_TOPIC = "/avs/record_latency_us"


# ----------------------- helpers -----------------------
def find_processes_by_cmd_contains(substrs):
    """Return a list of psutil.Process where cmdline contains ANY of substrs."""
    matches = []
    for p in psutil.process_iter(attrs=['pid', 'cmdline', 'name']):
        try:
            cmd = ' '.join(p.info.get('cmdline') or [])
            if any(s in cmd for s in substrs):
                matches.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return matches

def du_bytes(path: Path) -> int:
    """Disk usage (bytes) for file or recursively for a directory."""
    total = 0
    if not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    for p in path.rglob('*'):
        if p.is_file():
            try:
                total += p.stat().st_size
            except FileNotFoundError:
                pass
    return total

def human_mb(nbytes: int) -> float:
    return round(nbytes / (1024.0 * 1024.0), 2)

def mean(xs):
    return (sum(xs) / len(xs)) if xs else 0.0

def percentiles(xs, ps):
    if not xs:
        return {p: 0 for p in ps}
    s = sorted(xs)
    n = len(s)
    out = {}
    for p in ps:
        if n == 1:
            out[p] = s[0]
            continue
        k = max(0, min(n - 1, int(round((p / 100.0) * (n - 1)))))
        out[p] = s[k]
    return out

def list_children_recursive(pid):
    try:
        parent = psutil.Process(pid)
        return parent.children(recursive=True)
    except psutil.Error:
        return []

def prefer_real_node_processes(candidates, prefer_names):
    """Pick processes whose name/cmdline matches actual node executables."""
    real = []
    for p in candidates:
        try:
            name = (p.name() or "")
            cmd  = " ".join(p.cmdline() or [])
            if any(name == n for n in prefer_names):
                real.append(p); continue
            if any((" " + n + " ") in (" " + cmd + " ") for n in prefer_names):
                real.append(p); continue
        except psutil.Error:
            continue
    return real

def exclude_wrappers(candidates):
    """Drop obvious wrapper/launcher processes like ros2/python/sh."""
    out = []
    for p in candidates:
        try:
            name = (p.name() or "").lower()
            cmd  = " ".join(p.cmdline() or "").lower()
            if (" ros2 " in cmd) or ("python" in name) or name in ("bash","sh"):
                continue
            out.append(p)
        except psutil.Error:
            continue
    return out

def resolve_target_processes(mode, launched, spawned_proc, patterns):
    prefer_names = EXEC_NAMES[mode]
    targets = []

    if launched and spawned_proc is not None:
        # Give the child process a moment to spawn
        for _ in range(20):  # up to ~2s
            kids = list_children_recursive(spawned_proc.pid)
            # Best: exact executable(s)
            real = prefer_real_node_processes(kids, prefer_names)
            if real:
                targets = real
                break
            # Next: any non-wrapper child
            nonwrap = exclude_wrappers(kids)
            if nonwrap:
                targets = nonwrap
                break
            time.sleep(0.1)

        if not targets:
            # Fallback: measure the launcher itself (worst case)
            try:
                targets = [psutil.Process(spawned_proc.pid)]
            except psutil.Error:
                targets = []
    else:
        # Already running: scan whole system
        prelim = []
        for p in psutil.process_iter(attrs=['pid','cmdline','name']):
            try:
                cmd  = ' '.join(p.info.get('cmdline') or "")
                name = p.info.get('name') or ""
                if any(s in cmd for s in patterns) or any(name == n for n in prefer_names):
                    prelim.append(p)
            except psutil.Error:
                continue

        # Prefer exact executables, otherwise drop wrappers, otherwise keep prelim
        exact = prefer_real_node_processes(prelim, prefer_names)
        if exact:
            targets = exact
        else:
            nonwrap = exclude_wrappers(prelim)
            targets = nonwrap if nonwrap else prelim

    # Deduplicate by PID and drop zombies
    uniq = {}
    for p in targets:
        try:
            if p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
                uniq[p.pid] = p
        except psutil.Error:
            continue
    return list(uniq.values())

def prime_cpu_counters(procs):
    for p in procs:
        try:
            p.cpu_percent(None)
        except psutil.Error:
            pass

def sample_group_cpu_rss(procs):
    cpu = 0.0
    rss = 0
    alive = []
    for p in procs:
        try:
            cpu += p.cpu_percent(None)
            rss += p.memory_info().rss
            alive.append(p)
        except psutil.Error:
            continue
    return cpu, round(rss / (1024*1024), 2), alive


# ---------------- ROS Latency Collector (single-threaded) ---------------
class AvsLatencyCollector(Node):
    def __init__(self, topic):
        super().__init__('avs_latency_collector')
        self.latencies = deque(maxlen=200000)
        self.sub = self.create_subscription(Int64, topic, self.cb, 100)

    def cb(self, msg: Int64):
        self.latencies.append(int(msg.data))


# ------------------------------ main -----------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Ensure AVS subscriber is running, run benchmark for a fixed duration, then report and clean up if we launched it."
    )
    parser.add_argument("--mode", choices=["image", "lidar", "gps"], required=True,
                        help="Which AVS subscriber to check/launch")
    parser.add_argument("--duration-sec", type=int, required=True,
                        help="Benchmark duration in seconds (fixed timer)")
    args = parser.parse_args()

    # Mode setup
    today = datetime.now().strftime("%Y-%m-%d")
    if args.mode == "image":
        patterns = IMAGE_PATTERNS
        data_dir = (IMG_ROOT / today)
    elif args.mode == "lidar":
        patterns = LIDAR_PATTERNS
        data_dir = (LIDAR_ROOT / today)
    else:  # gps
        patterns = GPS_PATTERNS
        data_dir = GPS_ROOT

    launch_cmd = LAUNCH_CMDS[args.mode]
    avs_lat_topic = AVS_LAT_TOPIC

    # 1) Ensure AVS subscriber is running (auto-launch if not)
    procs = find_processes_by_cmd_contains(patterns)
    launched = False
    spawned_proc = None
    spawned_pgid = None
    launch_t = time.time()

    if not procs:
        print(f"[INFO] AVS {args.mode} subscriber not found. Launching:")
        print(f"{launch_cmd}")
        try:
            spawned_proc = subprocess.Popen(launch_cmd, shell=True, preexec_fn=os.setsid)
            spawned_pgid = os.getpgid(spawned_proc.pid)
        except Exception as e:
            print(f"[ERROR] Failed to launch: {e}")
            sys.exit(1)

        time.sleep(2.0)
        procs = find_processes_by_cmd_contains(patterns)
        if not procs:
            try:
                if spawned_proc and spawned_proc.poll() is None:
                    os.killpg(spawned_pgid, signal.SIGTERM)
            except Exception:
                pass
            print("[ERROR] Launch executed, but subscriber not detected as running.")
            sys.exit(2)
        launched = True
        print(f"[OK] AVS {args.mode} subscriber launched (pid={spawned_proc.pid}).")
    else:
        print(f"[OK] AVS {args.mode} subscriber already running.")

    # 2) Resolve the *real* node processes to measure (not the ros2/python wrapper)
    target_procs = resolve_target_processes(args.mode, launched, spawned_proc, patterns)
    if not target_procs:
        print("[ERROR] Could not identify target process(es) to measure.")
        sys.exit(3)

    # Build a display string for PIDs & command(s)
    try:
        cmdlines = []
        for p in target_procs:
            try:
                cmdlines.append(f"{p.pid}:{' '.join(p.cmdline() or [])}")
            except psutil.Error:
                cmdlines.append(f"{p.pid}:<unavailable>")
        cmdline_display = " | ".join(cmdlines)
    except Exception:
        cmdline_display = "<unavailable>"

    # 3) Capture initial metrics
    start_time = time.time()
    end_time = start_time + max(1, int(args.duration_sec))
    data_dir.mkdir(parents=True, exist_ok=True)  # ensure path exists for du
    start_size_bytes = du_bytes(data_dir)

    # Prime CPU counters for all targets
    prime_cpu_counters(target_procs)

    cpu_samples = []
    rss_samples = []

    # 4) ROS init & latency collector
    rclpy.init()
    lat_node = AvsLatencyCollector(avs_lat_topic)
    exec_ = SingleThreadedExecutor()
    exec_.add_node(lat_node)

    print(f"[INFO] Latency collector subscribed to {avs_lat_topic}")
    print(f"[INFO] Measuring PIDs: {[p.pid for p in target_procs]}")
    print(f"[INFO] Beginning benchmark for {args.duration_sec} seconds...")

    # 5) Run until the timer hits
    try:
        while time.time() < end_time:
            if rclpy.ok():
                try:
                    exec_.spin_once(timeout_sec=0.05)
                except ExternalShutdownException:
                    pass

            # Sample CPU/RSS roughly once per second
            if not cpu_samples or (time.time() - start_time) >= len(cpu_samples) + 1:
                cpu, rss_mb, alive = sample_group_cpu_rss(target_procs)
                target_procs = alive  # drop any that died
                cpu_samples.append(round(cpu, 2))
                rss_samples.append(round(rss_mb, 2))

            time.sleep(0.05)
    finally:
        # Clean up ROS
        print("[INFO] Benchmark finished, shutting down ROS...")
        try:
            exec_.remove_node(lat_node)
            latencies = list(lat_node.latencies)
            lat_node.destroy_node()
        except Exception:
            latencies = []
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    # 6) If we launched the subscriber, clean it up (and related PIDs)
    if launched and spawned_proc:
        print("[INFO] Cleaning up launched AVS subscriber...")
        try:
            if spawned_proc.poll() is None and spawned_pgid is not None:
                os.killpg(spawned_pgid, signal.SIGINT)
                time.sleep(0.8)
        except Exception:
            pass
        try:
            if spawned_proc.poll() is None and spawned_pgid is not None:
                os.killpg(spawned_pgid, signal.SIGTERM)
                time.sleep(0.5)
        except Exception:
            pass
        try:
            for p in find_processes_by_cmd_contains(patterns):
                try:
                    if p.create_time() >= launch_t - 1.0:
                        p.kill()
                except Exception:
                    pass
        except Exception:
            pass

    # 7) Final metrics & report
    duration = round(time.time() - start_time, 2)
    final_size_bytes = du_bytes(data_dir)
    size_delta_mb = human_mb(max(0, final_size_bytes - start_size_bytes))

    cpu_avg = round(mean(cpu_samples), 2) if cpu_samples else 0.0
    cpu_max = round(max(cpu_samples), 2) if cpu_samples else 0.0
    rss_avg = round(mean(rss_samples), 2) if rss_samples else 0.0
    rss_max = round(max(rss_samples), 2) if rss_samples else 0.0

    lat_count = len(latencies)
    lat_stats = percentiles(latencies, [50, 95])
    lat_avg = round(mean(latencies), 2) if latencies else 0.0

    print("\n========== AVS Benchmark Report ==========")
    print(f"Mode:                  {args.mode}")
    print(f"Status:                {'launched now' if launched else 'already running'}")
    print(f"PIDs measured:         {[p.pid for p in target_procs]}")
    print(f"Commands:              {cmdline_display}")
    print(f"Data dir:              {data_dir}")
    print(f"Run duration:          {duration} s  (timer target: {args.duration_sec} s)")
    print("------------------------------------------")
    print("CPU% (aggregate) / Memory (RSS MB, aggregate):")
    print(f"  AVS:      avg {cpu_avg}% (max {cpu_max}%), "
          f"RSS avg {rss_avg} MB (max {rss_max} MB)")
    print("------------------------------------------")
    print("Record latency (microseconds):")
    if lat_count > 0:
        print(f"  count={lat_count}, avg={lat_avg}, p50={lat_stats[50]}, p95={lat_stats[95]}")
    else:
        print("  (No AVS latency messages received on the expected topic.)")
    print("------------------------------------------")
    print("Data size growth (MB):")
    print(f"  +{size_delta_mb} MB (total {human_mb(final_size_bytes)} MB in {data_dir})")
    print("==========================================\n")


if __name__ == "__main__":
    main()
