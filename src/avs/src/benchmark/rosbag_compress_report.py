#!/usr/bin/env python3
import argparse, os, signal, time, uuid, subprocess as sp, psutil
from pathlib import Path

def human_mb(n_bytes: int) -> float:
    return n_bytes / (1024.0 * 1024.0)

def unique_run_dir(root: Path, prefix="writer_run") -> Path:
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

def record_and_sample(cmd, out_dir: Path, duration_s: int, sample_dt: float = 0.25):
    log_path = out_dir.parent / (out_dir.name + "_record.log")
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    logf = open(log_path, "w", buffering=1)
    proc = sp.Popen(cmd, stdout=logf, stderr=logf, preexec_fn=os.setsid)
    psproc = psutil.Process(proc.pid)

    _ = psproc.cpu_percent(None)
    for ch in psproc.children(recursive=True):
        try: _ = ch.cpu_percent(None)
        except Exception: pass

    start_t = time.time()
    cpu_samples = []
    rss_samples = []
    exited_early = False
    try:
        while True:
            if proc.poll() is not None:
                exited_early = True
                break
            if time.time() - start_t >= duration_s:
                break

            cpu_pct = 0.0
            try:
                cpu_pct += psproc.cpu_percent(None)
                for ch in psproc.children(recursive=True):
                    try: cpu_pct += ch.cpu_percent(None)
                    except Exception: pass
            except Exception:
                pass
            cpu_samples.append(cpu_pct)

            rss = 0
            try:
                rss += psproc.memory_info().rss
                for ch in psproc.children(recursive=True):
                    try: rss += ch.memory_info().rss
                    except Exception: pass
            except Exception:
                pass
            rss_samples.append(rss)

            time.sleep(sample_dt)
    finally:
        try: os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception: pass
        try: proc.wait(timeout=15)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM); proc.wait(timeout=5)
            except Exception:
                try: os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception: pass
        try: logf.flush(); logf.close()
        except Exception: pass

    end_t = time.time()
    time.sleep(0.75)

    cpu_avg = round((sum(cpu_samples) / len(cpu_samples)) if cpu_samples else 0.0, 1)
    cpu_max = round(max(cpu_samples) if cpu_samples else 0.0, 1)
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
    }

def main():
    ap = argparse.ArgumentParser(description="SQLite3 ros2bag benchmark with report layout.")
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument("--topics", nargs="+", help="Topics to record.")
    grp.add_argument("--all", action="store_true", help="Record all topics (-a).")
    ap.add_argument("--duration-sec", type=int, required=True)
    ap.add_argument("--out-root", type=Path, default=Path.home() / "ros2bag_bench")
    ap.add_argument("--storage-id", default="sqlite3", choices=["sqlite3"])
    ap.add_argument("--compression-mode", default="none", choices=["none", "message", "file"])
    ap.add_argument("--compression-format", default="zstd", choices=["zstd", "lz4"])
    ap.add_argument("--mode-label", default=None, help="String for 'Mode:' in report (e.g., image, lidar).")
    args = ap.parse_args()

    run_dir = unique_run_dir(args.out_root)
    out_dir = run_dir
    cmd = ["ros2", "bag", "record", "-s", args.storage_id, "-o", str(out_dir)]
    if args.all:
        cmd.append("-a")
        mode = args.mode_label or "all"
    else:
        cmd.extend(args.topics)
        mode = args.mode_label or "topics"
    if args.compression_mode != "none":
        cmd += ["--compression-mode", args.compression_mode, "--compression-format", args.compression_format]

    size_before = dir_size_bytes(out_dir)

    print("[INFO] Command:", " ".join(cmd))
    print(f"[INFO] Recording for {args.duration_sec} s into: {out_dir}")

    stats = record_and_sample(cmd, out_dir, args.duration_sec)

    size_after = dir_size_bytes(out_dir)
    size_delta_mb = round(max(0.0, human_mb(size_after - size_before)), 2)

    duration = round(stats["end_t"] - stats["start_t"], 3)
    launched = True
    proc = stats["proc"]
    status_str = "OK"
    if stats["exited_early"]:
        rc = proc.wait(timeout=0) if proc.is_running() else getattr(proc, "returncode", None)
        status_str = f"exited early (rc={rc})"

    # Build pretty command line
    try:
        cmdline = " ".join(proc.cmdline()) if proc else "<unavailable>"
    except psutil.Error:
        cmdline = "<unavailable>"

    size_dir = str(out_dir)

    print("\n========== ROS2 Bag Benchmark Report ==========")
    print(f"Mode:                  {mode}")
    print(f"Status:                {status_str}")
    print(f"PID:                   {proc.pid if proc else -1}")
    print(f"Command:               {cmdline}")
    print(f"Measured dir:          {size_dir}")
    if launched:
        print(f"(Run subdir we created: {Path(size_dir).name})")
    print(f"Run duration:          {duration} s  (target: {args.duration_sec} s)")
    print("-----------------------------------------------")
    print("CPU% / Memory (RSS MB):")
    print(f"  avg {stats['cpu_avg']}% (max {stats['cpu_max']}%), RSS avg {stats['rss_avg_mb']} MB (max {stats['rss_max_mb']} MB)")
    print("-----------------------------------------------")
    print("Writer latency (microseconds):")
    print("  (No writer latency messages received.)")
    print("-----------------------------------------------")
    print("Config / Storage options:")
    print(f"  storage_id:          {args.storage_id}")
    print(f"  compression:         {args.compression_mode}/{args.compression_format}")
    print("-----------------------------------------------")
    print("Data size growth (MB):")
    print(f"  +{size_delta_mb} MB (total {round(human_mb(size_after), 2)} MB in {size_dir})")
    print("================================================\n")
    print("[INFO] rosbag benchmark complete and report generated.")

if __name__ == "__main__":
    main()
