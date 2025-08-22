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
IMAGE_PATTERNS = ["img_dedup_node", "image_subscriber", "image_subscribe"]
LIDAR_PATTERNS = ["lidar_subscriber", "lidar_dedup_node", "lidar_process"]

LAUNCH_CMDS = {
    "image": "ros2 run avs image_subscriber",
    "lidar": "ros2 run avs lidar_subscriber",
}

# Root data directories (adjust if your paths differ)
IMG_ROOT = Path("/home/avs/DATA/SSD/images")
LIDAR_ROOT = Path("/home/avs/DATA/SSD/lidar")

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

def pick_heaviest(procs):
    if not procs:
        return None
    return max(procs, key=lambda x: (x.is_running() and x.memory_info().rss) or 0)

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
    parser.add_argument("--mode", choices=["image", "lidar"], required=True,
                        help="Which AVS subscriber to check/launch")
    parser.add_argument("--duration-sec", type=int, required=True,
                        help="Benchmark duration in seconds (fixed timer)")
    args = parser.parse_args()

    # Mode setup
    patterns = IMAGE_PATTERNS if args.mode == "image" else LIDAR_PATTERNS
    launch_cmd = LAUNCH_CMDS[args.mode]
    avs_lat_topic = AVS_LAT_TOPIC
    today = datetime.now().strftime("%Y-%m-%d")
    data_dir = (IMG_ROOT if args.mode == "image" else LIDAR_ROOT) / today

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
            # new session/process group so we can clean it (and its children) later
            spawned_proc = subprocess.Popen(launch_cmd, shell=True, preexec_fn=os.setsid)
            spawned_pgid = os.getpgid(spawned_proc.pid)
        except Exception as e:
            print(f"[ERROR] Failed to launch: {e}")
            sys.exit(1)

        time.sleep(2.0)
        procs = find_processes_by_cmd_contains(patterns)
        if not procs:
            # best-effort cleanup of just-started PG on failure
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


    proc = pick_heaviest(procs)
    try:
        cmdline = " ".join(proc.cmdline())
    except psutil.Error:
        cmdline = "<unavailable>"

    # 2) Capture initial metrics
    start_time = time.time()
    end_time = start_time + max(1, int(args.duration_sec))
    data_dir.mkdir(parents=True, exist_ok=True)  # avoid scanning a missing tree
    start_size_bytes = du_bytes(data_dir)

    # Prime CPU measurement
    try:
        proc.cpu_percent(None)
    except psutil.Error:
        pass

    cpu_samples = []
    rss_samples = []

    # 3) ROS init (default global context) & latency collector
    rclpy.init()
    lat_node = AvsLatencyCollector(avs_lat_topic)
    exec_ = SingleThreadedExecutor()
    exec_.add_node(lat_node)

    print(f"[INFO] Latency collector subscribed to {avs_lat_topic}")
    print(f"[INFO] Beginning benchmark for {args.duration_sec} seconds...")

    # 4) Run until the timer hits
    try:
        while time.time() < end_time:
            # Spin ROS briefly to collect latencies
            if rclpy.ok():
                try:
                    exec_.spin_once(timeout_sec=0.05)
                except ExternalShutdownException:
                    pass

            # Sample CPU/RSS roughly once per second
            if not cpu_samples or (time.time() - start_time) >= len(cpu_samples) + 1:
                try:
                    cpu = proc.cpu_percent(None)
                    rss_mb = round(proc.memory_info().rss / (1024 * 1024), 2)
                except psutil.Error:
                    cpu = 0.0
                    rss_mb = 0.0
                cpu_samples.append(cpu)
                rss_samples.append(rss_mb)

            time.sleep(0.05)  # gentle loop
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

    # 5) If we launched the subscriber, clean it up (and related PIDs)
    if launched and spawned_proc:
        # Prefer graceful shutdown first, then force
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
        # Best-effort extra cleanup: kill any lingering processes matching patterns,
        # but only those started after we launched (avoid killing user's pre-existing ones).
        try:
            for p in find_processes_by_cmd_contains(patterns):
                try:
                    if p.create_time() >= launch_t - 1.0:
                        p.kill()
                except Exception:
                    pass
        except Exception:
            pass

    # 6) Final metrics & report
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
    print(f"PID:                   {proc.pid}")
    print(f"Command:               {cmdline}")
    print(f"Data dir:              {data_dir}")
    print(f"Run duration:          {duration} s  (timer target: {args.duration_sec} s)")
    print("------------------------------------------")
    print("CPU% / Memory (RSS MB):")
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
