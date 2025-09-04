#!/usr/bin/env python3
import argparse
import os
import signal
import time
import uuid
import subprocess as sp
from pathlib import Path

import psutil

# -------------------- Defaults --------------------
DEFAULT_TOPICS = [
    "/my_camera/pylon_ros2_camera_node/image_raw",
    "/novatel/oem7/gps",
    "/sensing/lidar/top/pointcloud",
]

DEFAULT_OUT_ROOT = Path("/home/avs/DATA/rosbag_bench")

# -------------------- Helpers --------------------
def human_mb(n_bytes: int) -> float:
    return n_bytes / (1024.0 * 1024.0)

def unique_run_dir(root: Path, prefix="ros2bag_run") -> Path:
    root.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime(f"{prefix}_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:6]
    return root / stamp

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

def process_tree_of(proc: psutil.Process):
    """Return a fresh list of proc + all recursive children (ignoring dead/missing)."""
    procs = []
    try:
        if proc.is_running():
            procs.append(proc)
            procs.extend(proc.children(recursive=True))
    except Exception:
        pass
    # filter out zombies / dead
    out = []
    for p in procs:
        try:
            if p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
                out.append(p)
        except Exception:
            pass
    return out

def prime_cpu_counters(procs):
    """Prime cpu_percent() for all processes so first measurement isn't a spike."""
    for p in procs:
        try:
            _ = p.cpu_percent(None)
        except Exception:
            pass

def sample_cpu_and_rss_tree(root_proc: psutil.Process):
    """Return (cpu%, rss_bytes) summed over root + all children at this instant."""
    cpu_total = 0.0
    rss_total = 0
    procs = process_tree_of(root_proc)
    for p in procs:
        try:
            cpu_total += p.cpu_percent(None)
        except Exception:
            pass
        try:
            rss_total += p.memory_info().rss
        except Exception:
            pass
    return cpu_total, rss_total, [p.pid for p in procs]

def terminate_process_group(pg_id: int):
    """Try SIGINT -> wait -> SIGTERM -> wait -> SIGKILL."""
    try:
        os.killpg(pg_id, signal.SIGINT)
    except Exception:
        pass
    try:
        time.sleep(1.0)
        os.killpg(pg_id, signal.SIGTERM)
    except Exception:
        pass
    try:
        time.sleep(0.5)
        os.killpg(pg_id, signal.SIGKILL)
    except Exception:
        pass

# -------------------- Runner --------------------
def record_and_measure(cmd, out_dir: Path, duration_s: int, sample_dt: float = 0.25):
    log_path = out_dir.parent / (out_dir.name + "_record.log")
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    logf = open(log_path, "w", buffering=1)

    proc = sp.Popen(cmd, stdout=logf, stderr=logf, preexec_fn=os.setsid)
    psproc = psutil.Process(proc.pid)
    pgid = os.getpgid(proc.pid)

    # Prime counters on the whole tree
    prime_cpu_counters(process_tree_of(psproc))

    start_t = time.time()
    cpu_samples = []
    rss_samples = []
    pid_seen = set()
    exited_early = False

    try:
        while True:
            if proc.poll() is not None:
                exited_early = True
                break
            if time.time() - start_t >= duration_s:
                break

            cpu_pct, rss_bytes, pids = sample_cpu_and_rss_tree(psproc)
            cpu_samples.append(cpu_pct)
            rss_samples.append(rss_bytes)
            pid_seen.update(pids)

            time.sleep(sample_dt)
    finally:
        # Stop recording cleanly
        try:
            terminate_process_group(pgid)
        except Exception:
            pass
        try:
            proc.wait(timeout=10)
        except Exception:
            pass
        try:
            logf.flush(); logf.close()
        except Exception:
            pass

    end_t = time.time()
    time.sleep(0.75)  # let filesystem settle

    cpu_avg = round((sum(cpu_samples) / len(cpu_samples)) if cpu_samples else 0.0, 2)
    cpu_max = round(max(cpu_samples) if cpu_samples else 0.0, 2)
    rss_avg_mb = round(human_mb(sum(rss_samples) / len(rss_samples)) if rss_samples else 0.0, 2)
    rss_max_mb = round(human_mb(max(rss_samples)) if rss_samples else 0.0, 2)

    return {
        "proc": psproc,
        "start_t": start_t,
        "end_t": end_t,
        "exited_early": exited_early,
        "cpu_avg": cpu_avg,
        "cpu_max": cpu_max,
        "rss_avg_mb": rss_avg_mb,
        "rss_max_mb": rss_max_mb,
        "pids": sorted(pid_seen),
    }

# -------------------- Main --------------------
def main():
    ap = argparse.ArgumentParser(description="ros2bag benchmark with CPU/RSS over full process tree.")
    ap.add_argument("--duration", type=int, required=True, help="Recording duration in seconds.")
    ap.add_argument("--compression", action="store_true",
                    help="Enable ros2 message-level zstd compression.")
    ap.add_argument("--topics", nargs="*", default=None,
                    help="Override topic list. If omitted, uses default topics.")
    ap.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT,
                    help=f"Output root directory (default: {DEFAULT_OUT_ROOT})")
    ap.add_argument("--storage-id", default="sqlite3", choices=["sqlite3"],
                    help="rosbag2 storage plugin (sqlite3).")
    args = ap.parse_args()

    topics = args.topics if args.topics else DEFAULT_TOPICS
    run_dir = unique_run_dir(args.out_root)
    out_dir = run_dir

    cmd = ["ros2", "bag", "record", "-s", args.storage_id, "-o", str(out_dir)]
    cmd.extend(topics)

    if args.compression:
        # Use ros2 embedded message-level zstd compression
        cmd += ["--compression-mode", "message", "--compression-format", "zstd"]

    size_before = dir_size_bytes(out_dir)

    print("[INFO] Command:", " ".join(cmd))
    print(f"[INFO] Topics: {topics}")
    print(f"[INFO] Recording for {args.duration} s into: {out_dir}")

    stats = record_and_measure(cmd, out_dir, args.duration)

    size_after = dir_size_bytes(out_dir)
    size_delta_mb = round(max(0.0, human_mb(size_after - size_before)), 2)
    duration = round(stats["end_t"] - stats["start_t"], 3)
    proc = stats["proc"]

    # Command line of the root proc (may be shell python wrapper)
    try:
        cmdline = " ".join(proc.cmdline()) if proc else "<unavailable>"
    except psutil.Error:
        cmdline = "<unavailable>"

    status_str = "OK"
    if stats["exited_early"]:
        try:
            rc = proc.wait(timeout=0) if proc.is_running() else getattr(proc, "returncode", None)
        except Exception:
            rc = None
        status_str = f"exited early (rc={rc})"

    print("\n========== ROS2 Bag Benchmark Report ==========")
    print(f"Mode:                  topics")
    print(f"Status:                {status_str}")
    print(f"Root PID:              {proc.pid if proc else -1}")
    print(f"All related PIDs:      {stats['pids']}")
    print(f"Command:               {cmdline}")
    print(f"Measured dir:          {out_dir}")
    print(f"(Run subdir we created: {Path(out_dir).name})")
    print(f"Run duration:          {duration} s  (target: {args.duration} s)")
    print("-----------------------------------------------")
    print("CPU% / Memory (RSS MB):  (summed over entire process tree)")
    print(f"  avg {stats['cpu_avg']}% (max {stats['cpu_max']}%), "
          f"RSS avg {stats['rss_avg_mb']} MB (max {stats['rss_max_mb']} MB)")
    print("-----------------------------------------------")
    print("Config / Storage options:")
    print(f"  storage_id:          {args.storage_id}")
    print(f"  compression:         {'message/zstd' if args.compression else 'none'}")
    print("-----------------------------------------------")
    print("Data size growth (MB):")
    print(f"  +{size_delta_mb} MB (total {round(human_mb(size_after), 2)} MB in {out_dir})")
    print("================================================\n")
    print("[INFO] rosbag benchmark complete and report generated.")

if __name__ == "__main__":
    main()
