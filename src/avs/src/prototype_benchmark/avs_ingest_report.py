#!/usr/bin/env python3
# avs_ingest_profiler.py
#
# Auto-launches AVS ingest nodes via `ros2 launch avs avs_ingest_nodes.launch.py`,
# collects tail-latency and bounded-queue metrics, then terminates the launch.
#
# Usage examples:
#   python3 avs_ingest_profiler.py --duration 60 --label "1x nominal"
#   python3 avs_ingest_profiler.py --duration 60 --label "2x burst" -r 2.0
#   python3 avs_ingest_profiler.py --duration 60 --csv /tmp/ingest.csv \
#       --config-path /home/avs/AVS-PI/src/avs/config/avs_config.yaml \
#       --max-queue-image 8 --max-queue-lidar 8 --max-queue-gps 64
#
# Notes:
# - Assumes your package is named `avs` and you installed the launch file:
#     share/avs/launch/avs_ingest_nodes.launch.py
# - Make sure to `source install/setup.bash` before running.

import argparse
import math
import os
import signal
import subprocess as sp
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float64

MODS = ["image", "lidar", "gps"]

def percentile(vals, p):
    if not vals:
        return None
    vals = sorted(vals)
    k = (len(vals) - 1) * (p / 100.0)
    f = math.floor(k); c = math.ceil(k)
    if f == c:
        return float(vals[int(k)])
    d0 = vals[int(f)] * (c - k)
    d1 = vals[int(c)] * (k - f)
    return float(d0 + d1)

class LaunchManager:
    """Start/stop `ros2 launch` in its own process group."""
    def __init__(self, cmd, log_path: Optional[str] = None):
        self.cmd = cmd
        self.log_path = log_path
        self.proc: Optional[sp.Popen] = None

    def start(self):
        # Inherit parent stdout/stderr so logs stream to your terminal.
        stdout = None
        stderr = None
        print("[LAUNCH]", " ".join(self.cmd))
        self.proc = sp.Popen(
            self.cmd,
            stdout=stdout,
            stderr=stderr,
            preexec_fn=os.setsid,  # new process group (Linux)
        )

    def terminate(self, grace_int=1.0, grace_term=1.0):
        if not self.proc:
            return
        try:
            pgid = os.getpgid(self.proc.pid)
        except Exception:
            # Fallback: just try to kill the child
            try:
                self.proc.terminate()
            except Exception:
                pass
            return

        def _send(sig):
            try:
                os.killpg(pgid, sig)
            except Exception:
                pass

        # Try SIGINT -> wait -> SIGTERM -> wait -> SIGKILL
        _send(signal.SIGINT);  time.sleep(grace_int)
        if self.proc.poll() is None:
            _send(signal.SIGTERM); time.sleep(grace_term)
        if self.proc.poll() is None:
            _send(signal.SIGKILL)

        try:
            self.proc.wait(timeout=5)
        except Exception:
            pass

class Prof(Node):
    def __init__(self, duration_s, label, warmup_s=0.0, csv_path=None):
        super().__init__("avs_ingest_profiler")
        now = time.time()
        self.collect_start = now + max(0.0, warmup_s)
        self.end_time = self.collect_start + duration_s

        self.label    = label
        self.csv_path = csv_path

        self.lat = {m: [] for m in MODS}
        self.max_q = {m: 0 for m in MODS}
        self.drop_last = {m: 0 for m in MODS}
        self.rss_max = {m: 0.0 for m in MODS}
        self.processed = {m: 0 for m in MODS}

        # Subscriptions
        for m in MODS:
            self.create_subscription(Int64, f"/avs/latency_us/{m}",
                                     lambda msg, mm=m: self._on_lat(mm, msg), 1000)
            self.create_subscription(Int64, f"/avs/queue_depth/{m}",
                                     lambda msg, mm=m: self._on_q(mm, msg), 10)
            self.create_subscription(Int64, f"/avs/dropped/{m}",
                                     lambda msg, mm=m: self._on_drop(mm, msg), 10)
            self.create_subscription(Float64, f"/avs/rss_mb/{m}",
                                     lambda msg, mm=m: self._on_rss(mm, msg), 10)

        # Main timer
        self.create_timer(0.25, self._tick)

        if self.csv_path:
            try:
                with open(self.csv_path, "w") as f:
                    f.write("ts,phase,modality,latency_us,queue_depth,dropped,rss_mb\n")
            except Exception as e:
                self.get_logger().warn(f"CSV open failed: {e}")
                self.csv_path = None

        print(f"[INFO] Warmup: {warmup_s:.2f}s | Collecting for {duration_s:.2f}s | Label: '{label}'")

    def _phase(self):
        t = time.time()
        if t < self.collect_start:
            return "warmup"
        elif t < self.end_time:
            return "collect"
        else:
            return "done"

    def _log_csv(self, phase, mod, lat=None, q=None, d=None, rss=None):
        if not self.csv_path:
            return
        try:
            with open(self.csv_path, "a") as f:
                f.write(f"{time.time():.3f},{phase},{mod},"
                        f"{'' if lat is None else int(lat)},"
                        f"{'' if q   is None else int(q)},"
                        f"{'' if d   is None else int(d)},"
                        f"{'' if rss is None else f'{rss:.2f}'}\n")
        except Exception:
            pass

    def _on_lat(self, mod, msg):
        phase = self._phase()
        if phase != "collect":
            self._log_csv(phase, mod, lat=msg.data)
            return
        self.lat[mod].append(int(msg.data))
        self.processed[mod] += 1
        self._log_csv(phase, mod, lat=msg.data)

    def _on_q(self, mod, msg):
        phase = self._phase()
        self.max_q[mod] = max(self.max_q[mod], int(msg.data))
        self._log_csv(phase, mod, q=msg.data)

    def _on_drop(self, mod, msg):
        phase = self._phase()
        self.drop_last[mod] = int(msg.data)
        self._log_csv(phase, mod, d=msg.data)

    def _on_rss(self, mod, msg):
        phase = self._phase()
        self.rss_max[mod] = max(self.rss_max[mod], float(msg.data))
        self._log_csv(phase, mod, rss=float(msg.data))

    def _tick(self):
        if time.time() >= self.end_time:
            self._report()
            rclpy.shutdown()

    def _report(self):
        print("\n========== AVS Ingest Tail-Latency Report ==========")
        print(f"Label: {self.label}")
        print("Units: latency=milliseconds (computed from microseconds)")

        for m in MODS:
            vals_ms = [v/1000.0 for v in self.lat[m]]
            p50 = percentile(vals_ms, 50) if vals_ms else None
            p95 = percentile(vals_ms, 95) if vals_ms else None
            p99 = percentile(vals_ms, 99) if vals_ms else None
            drops = self.drop_last[m]
            processed = self.processed[m]
            total = processed + drops
            drate = (drops / total) if total > 0 else 0.0

            print(f"\n[{m}] samples={len(vals_ms)} processed={processed} drops={drops} (rate {drate:.2%})")
            if p50 is not None:
                print(f"    p50={p50:.2f} ms   p95={p95:.2f} ms   p99={p99:.2f} ms")
            else:
                print("    (no samples)")
            print(f"    max queue depth observed: {self.max_q[m]}")
            print(f"    max RSS observed: {self.rss_max[m]:.2f} MB")
        print("====================================================\n")

def build_launch_cmd(args):
    # ros2 launch avs avs_ingest_nodes.launch.py config_path:=... max_queue_image:=... etc.
    cmd = [
        "ros2", "launch", args.launch_pkg, args.launch_file,
        f"config_path:={args.config_path}",
        f"max_queue_image:={args.max_queue_image}",
        f"max_queue_lidar:={args.max_queue_lidar}",
        f"max_queue_gps:={args.max_queue_gps}",
    ]
    if args.namespace:
        cmd.append(f"namespace:={args.namespace}")
    return cmd

def main():
    ap = argparse.ArgumentParser(description="Auto-launch AVS ingest nodes, collect tail-latency metrics, then stop.")
    ap.add_argument("--duration", type=float, default=60.0, help="Seconds to COLLECT metrics (excludes warmup).")
    ap.add_argument("--warmup", type=float, default=3.0, help="Warm-up seconds before collecting (nodes spin-up).")
    ap.add_argument("--label", type=str, default="", help="Label to print in the report.")
    ap.add_argument("--csv", type=str, default=None, help="Optional CSV path for raw samples.")
    ap.add_argument("--no-auto", action="store_true", help="Do NOT auto-launch; just attach to running nodes.")

    # Launch params
    ap.add_argument("--launch-pkg", default="avs", help="ROS 2 package of the launch file.")
    ap.add_argument("--launch-file", default="avs_bench.launch.py", help="Launch file name.")
    ap.add_argument("--config-path", default="/home/avs/AVS-PI/src/avs/config/avs_config.yaml", help="Path to AVS YAML config.")
    ap.add_argument("--namespace", default="", help="Optional ROS namespace for all nodes.")
    ap.add_argument("--max-queue-image", type=int, default=10, help="Image queue bound.")
    ap.add_argument("--max-queue-lidar", type=int, default=10, help="LiDAR queue bound.")
    ap.add_argument("--max-queue-gps",   type=int, default=10, help="GPS queue bound.")
    ap.add_argument("--launch-log", default=None, help="Write ros2 launch stdout to this file.")
    args = ap.parse_args()

    lm = None
    try:
        if not args.no_auto:
            # Start `ros2 launch ...`
            cmd = build_launch_cmd(args)
            lm = LaunchManager(cmd, log_path=args.launch_log)
            lm.start()
            # Let launch bring up nodes a bit before we init ROS here
            time.sleep(max(0.5, args.warmup * 0.5))  # small head start

        rclpy.init()
        node = Prof(duration_s=args.duration, label=args.label, warmup_s=args.warmup, csv_path=args.csv)
        rclpy.spin(node)
    finally:
        # Always try to stop the launch once profiling completes
        if lm:
            print("[LAUNCH] Stopping launch...")
            lm.terminate(grace_int=1.0, grace_term=1.0)
        # Ensure rclpy is down (idempotent)
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
