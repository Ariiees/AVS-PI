#!/usr/bin/env python3
import argparse
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from collections import deque
import uuid

import psutil

# ROS 2 (Jazzy-friendly)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from std_msgs.msg import Int64

# rosbag2 (Jazzy)
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions, TopicMetadata

# -------------------- constants / defaults --------------------
BAG_ROOT = Path("/home/avs/DATA/ros2bag")
IMAGE_TOPIC = "/camera/image"
LIDAR_TOPIC = "/lidar/points"

# patterns to detect our own child writer
IMAGE_PATTERNS = ["--writer-child", "rosbag_writer_child", "sensor_msgs/msg/Image"]
LIDAR_PATTERNS = ["--writer-child", "rosbag_writer_child", "sensor_msgs/msg/PointCloud2"]

# ----------------------- helpers -----------------------
def find_processes_by_cmd_contains(substrs):
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

def parse_external_outdir_from_cmdline(cmd: str) -> Path | None:
    """Extract output dir from another writer's cmdline."""
    m = re.search(r"--writer-out\s+(\S+)", cmd)
    if m:
        return Path(m.group(1))
    m = re.search(r"(?:^|\s)-o\s+(\S+)", cmd)  # ros2 bag record -o
    if m:
        return Path(m.group(1))
    return None

def fresh_uri(base: Path) -> Path:
    """Return a non-existing path by appending a numeric suffix if needed."""
    if not base.exists():
        return base
    for i in range(1, 1000):
        candidate = base.parent / f"{base.name}_{i:03d}"
        if not candidate.exists():
            return candidate
    raise RuntimeError("Unable to find a free output directory name")


# ---------------- ROS Latency Collector (parent) ----------------
class RosbagLatencyCollector(Node):
    def __init__(self, topic):
        super().__init__('rosbag_latency_collector')
        self.latencies = deque(maxlen=200000)
        self.sub = self.create_subscription(Int64, topic, self.cb, 100)
    def cb(self, msg: Int64):
        self.latencies.append(int(msg.data))


# ---------------------- child writer entry ----------------------
def writer_child(mode: str, topic: str, out_dir: str, lat_topic: str):
    """
    Subscribes to `topic`, writes with rosbag2_py.SequentialWriter to `out_dir`,
    and publishes per-message write latency on a UNIQUE topic `lat_topic`.
    """
    def _child_sig(_s, _f):
        if rclpy.ok():
            rclpy.shutdown()
    signal.signal(signal.SIGINT, _child_sig)
    signal.signal(signal.SIGTERM, _child_sig)

    rclpy.init()

    if mode == "image":
        from sensor_msgs.msg import Image as MsgType
        ros_type = "sensor_msgs/msg/Image"
    else:
        from sensor_msgs.msg import PointCloud2 as MsgType
        ros_type = "sensor_msgs/msg/PointCloud2"

    class WriterNode(Node):
        def __init__(self, topic_name: str, out_dir: str, ros_type: str, lat_topic: str):
            super().__init__('rosbag_writer_child')
            self.topic = topic_name
            self.ros_type = ros_type
            self.lat_pub = self.create_publisher(Int64, lat_topic, 10)

            # QoS mirror if possible, else sensor profile
            qos = None
            try:
                infos = self.get_publishers_info_by_topic(topic_name)
                if infos:
                    offered = infos[0].qos_profile
                    qos = QoSProfile(depth=max(200, getattr(offered, "depth", 10)))
                    qos.history = offered.history
                    qos.reliability = offered.reliability
                    qos.durability = offered.durability
            except Exception:
                qos = None
            if qos is None:
                qos = qos_profile_sensor_data
                qos.depth = 400 if mode == "lidar" else 200

            # Ensure we don't collide with an existing bag dir
            uri = fresh_uri(Path(out_dir))

            self.writer = rosbag2_py.SequentialWriter()
            storage_opts = StorageOptions(uri=str(uri), storage_id='sqlite3')
            converter_opts = ConverterOptions(input_serialization_format='cdr',
                                              output_serialization_format='cdr')
            self.writer.open(storage_opts, converter_opts)

            meta = TopicMetadata(
                id=0,
                name=self.topic,
                type=self.ros_type,
                serialization_format='cdr',
            )
            self.writer.create_topic(meta)

            self.sub = self.create_subscription(MsgType, self.topic, self.cb, qos)
            self.get_logger().info(f"Writer ready. Topic: {self.topic}, Out: {uri}, LatTopic: {lat_topic}")

        def cb(self, msg):
            try:
                data = serialize_message(msg)
                ts = int(self.get_clock().now().nanoseconds)
                t0 = time.perf_counter()
                self.writer.write(self.topic, data, ts)
                dt_us = int((time.perf_counter() - t0) * 1e6)
                m = Int64(); m.data = dt_us
                self.lat_pub.publish(m)
            except Exception as e:
                self.get_logger().warn(f"write error: {e}")

    node = WriterNode(topic, out_dir, ros_type, lat_topic)
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)
    try:
        while rclpy.ok():
            exec_.spin_once(timeout_sec=0.05)
    except ExternalShutdownException:
        pass
    finally:
        try: exec_.remove_node(node)
        except Exception: pass
        try: node.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


# -------------------------------- main --------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Run Jazzy SequentialWriter, benchmark for a fixed duration, then report."
    )
    # NOTE: duration-sec is NOT required globally so the child can reuse this parser.
    parser.add_argument("--mode", choices=["image", "lidar"])
    parser.add_argument("--duration-sec", type=int)
    parser.add_argument("--attach-existing", action="store_true",
                        help="Attach to an already-running writer instead of launching our own")
    # hidden child mode
    parser.add_argument("--writer-child", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--writer-out", default="", help=argparse.SUPPRESS)
    parser.add_argument("--topic", default="", help=argparse.SUPPRESS)
    parser.add_argument("--lat-topic", default="", help=argparse.SUPPRESS)
    args = parser.parse_args()

    # Child entry
    if args.writer_child:
        if not args.mode:
            print("[ERROR] --mode is required in child mode")
            sys.exit(2)
        mode = args.mode
        topic = args.topic or (IMAGE_TOPIC if mode == "image" else LIDAR_TOPIC)
        lat_topic = args.lat_topic or "/rosbag/writer_latency_us/run_default"
        writer_child(mode, topic, args.writer_out, lat_topic)
        return

    # Parent validation
    if not args.mode or not args.duration_sec:
        print("usage: rosbag_report.py --mode {image,lidar} --duration-sec N [--attach-existing]")
        sys.exit(2)

    # Parent setup
    mode = args.mode
    topic = IMAGE_TOPIC if mode == "image" else LIDAR_TOPIC
    patterns = IMAGE_PATTERNS if mode == "image" else LIDAR_PATTERNS

    # Unique run id & latency topic (must not start with a number)
    run_id = "run_" + datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]
    lat_topic = f"/rosbag/writer_latency_us/{run_id}"

    # Our intended run dir (do NOT create here; let the writer create it)
    run_dir = BAG_ROOT / f"writer_{run_id}"
    # Ensure only the ROOT exists
    BAG_ROOT.mkdir(parents=True, exist_ok=True)

    # Decide: launch own writer (default) or attach existing
    procs = find_processes_by_cmd_contains(patterns)
    launched = False
    spawned_proc = None
    spawned_pgid = None
    launch_t = time.time()
    size_dir = None  # directory we will measure for size growth
    status_str = ""

    if args.attach_existing and procs:
        print(f"[OK] Attaching to existing rosbag writer for metrics.")
        proc = pick_heaviest(procs)
        try:
            cmdline = " ".join(proc.cmdline())
        except psutil.Error:
            cmdline = "<unavailable>"
        ext_dir = parse_external_outdir_from_cmdline(cmdline) if cmdline != "<unavailable>" else None
        size_dir = ext_dir if ext_dir else BAG_ROOT
        print(f"[INFO] Measuring size at: {size_dir}")
        status_str = "attached to existing"
    else:
        print(f"[INFO] No attach (or none found). Launching internal writer...")
        script_path = os.path.abspath(sys.argv[0])
        try:
            spawned_proc = subprocess.Popen(
                [sys.executable, script_path,
                 "--mode", mode,
                 "--writer-child",
                 "--writer-out", str(run_dir),
                 "--topic", topic,
                 "--lat-topic", lat_topic],
                preexec_fn=os.setsid
            )
            spawned_pgid = os.getpgid(spawned_proc.pid)
        except Exception as e:
            print(f"[ERROR] Failed to launch writer: {e}")
            sys.exit(1)

        # give child time to start and register
        time.sleep(2.0)
        procs = find_processes_by_cmd_contains(patterns)
        if not procs:
            print("[ERROR] Writer not detected after launch.")
            try:
                if spawned_proc and spawned_proc.poll() is None:
                    os.killpg(spawned_pgid, signal.SIGTERM)
            except Exception:
                pass
            sys.exit(2)
        launched = True
        proc = pick_heaviest(procs)
        try:
            cmdline = " ".join(proc.cmdline())
        except psutil.Error:
            cmdline = "<unavailable>"
        size_dir = run_dir  # measure exactly where we told child to write
        status_str = "launched now"
        print(f"[OK] Writer launched (pid={spawned_proc.pid}), topic={topic}, out={run_dir}")
        print(f"[INFO] Using unique latency topic: {lat_topic}")

    # Metrics prep
    start_time = time.time()
    end_time = start_time + max(1, int(args.duration_sec))
    Path(size_dir).parent.mkdir(parents=True, exist_ok=True)  # parent exists; child will create leaf
    size_before = du_bytes(size_dir) if size_dir.exists() else 0  # may be 0 before child creates

    # Prime CPU measurement
    try:
        proc.cpu_percent(None)
    except psutil.Error:
        pass

    cpu_samples, rss_samples = [], []

    # Parent ROS: collect ONLY our unique latency topic
    rclpy.init()
    lat_node = RosbagLatencyCollector(lat_topic)
    exec_ = SingleThreadedExecutor()
    exec_.add_node(lat_node)

    print(f"[INFO] Latency collector subscribed to {lat_topic}")
    print(f"[INFO] Beginning rosbag benchmark for {args.duration_sec} seconds...")
    print(f"[INFO] Output directory measured: {size_dir}")

    # Run until timer hits
    try:
        while time.time() < end_time:
            if rclpy.ok():
                try:
                    exec_.spin_once(timeout_sec=0.05)
                except ExternalShutdownException:
                    pass
            if not cpu_samples or (time.time() - start_time) >= len(cpu_samples) + 1:
                try:
                    cpu = proc.cpu_percent(None)
                    rss_mb = round(proc.memory_info().rss / (1024 * 1024), 2)
                except psutil.Error:
                    cpu, rss_mb = 0.0, 0.0
                cpu_samples.append(cpu)
                rss_samples.append(rss_mb)
            time.sleep(0.05)
    finally:
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

    # Clean up child we launched
    if launched and spawned_proc:
        print("[INFO] Cleaning up launched rosbag writer...")
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
        # extra best-effort for stragglers started after launch
        try:
            for p in find_processes_by_cmd_contains(IMAGE_PATTERNS if mode == "image" else LIDAR_PATTERNS):
                try:
                    if p.create_time() >= launch_t - 1.0:
                        p.kill()
                except Exception:
                    pass
        except Exception:
            pass

    # Final metrics & report (from the directory we actually measured)
    duration = round(time.time() - start_time, 2)
    size_after = du_bytes(size_dir) if size_dir.exists() else 0
    size_delta_mb = human_mb(max(0, size_after - size_before))

    cpu_avg = round(mean(cpu_samples), 2) if cpu_samples else 0.0
    cpu_max = round(max(cpu_samples), 2) if cpu_samples else 0.0
    rss_avg = round(mean(rss_samples), 2) if rss_samples else 0.0
    rss_max = round(max(rss_samples), 2) if rss_samples else 0.0

    lat_count = len(latencies)
    lat_stats = percentiles(latencies, [50, 95])
    lat_avg = round(mean(latencies), 2) if latencies else 0.0

    print("\n========== ROS2 Bag Benchmark Report ==========")
    print(f"Mode:                  {mode}")
    print(f"Status:                {status_str}")
    print(f"PID:                   {proc.pid}")
    try:
        cmdline = " ".join(proc.cmdline())
    except psutil.Error:
        cmdline = "<unavailable>"
    print(f"Command:               {cmdline}")
    print(f"Measured dir:          {size_dir}")
    if launched:
        print(f"(Run subdir we created: {run_dir.name})")
    print(f"Run duration:          {duration} s  (target: {args.duration_sec} s)")
    print("-----------------------------------------------")
    print("CPU% / Memory (RSS MB):")
    print(f"  avg {cpu_avg}% (max {cpu_max}%), "
          f"RSS avg {rss_avg} MB (max {rss_max} MB)")
    print("-----------------------------------------------")
    print("Writer latency (microseconds):")
    if lat_count > 0:
        print(f"  count={lat_count}, avg={lat_avg}, p50={lat_stats[50]}, p95={lat_stats[95]}")
    else:
        print("  (No writer latency messages received.)")
    print("-----------------------------------------------")
    print("Data size growth (MB):")
    print(f"  +{size_delta_mb} MB (total {human_mb(size_after)} MB in {size_dir})")
    print("================================================\n")
    print("[INFO] rosbag benchmark complete and report generated.")


if __name__ == "__main__":
    main()
