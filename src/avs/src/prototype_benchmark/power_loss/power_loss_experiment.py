#!/usr/bin/env python3
"""
Power loss + recovery experiment runner for AVS ingestion.

Behavior:
  - Starts a ROS 2 launch file (default: avs_store.launch.py) and periodically records
    a heartbeat plus newest data file time (excluding global.sqlite3).
  - Persists state so a reboot can be detected.
  - After reboot, resumes, measures recovery time, data loss window, power cut interval,
    and validates durability via CRC + sqlite recovery.

Typical use:
  1) Start the experiment before power loss:
       ./power_loss_experiment.py
  2) Cut power during ingestion and reboot.
  3) Run the same script after reboot (or via a service):
       ./power_loss_experiment.py
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import shlex
import sqlite3
import struct
import subprocess
import zlib
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

DEFAULT_DATA_ROOT = Path("/home/avs/DATA/SSD")
LOG_ROOT = Path("/home/avs/Log")

SCRIPT_DIR = Path(__file__).resolve().parent
RUNS_DIR = LOG_ROOT / "power_loss"
ACTIVE_RUN_FILE = RUNS_DIR / "active_run.json"

STATE_FILE = "state.json"
REPORT_JSON = "power_loss_report.json"
REPORT_TXT = "power_loss_report.txt"
INGEST_LOG = "ingest.log"


def now_iso(ts: Optional[float] = None) -> str:
    if ts is None:
        ts = time.time()
    return datetime.fromtimestamp(ts).isoformat(timespec="seconds")


def read_boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except Exception:
        return "unknown"


def read_boot_time_epoch() -> Optional[float]:
    try:
        uptime = float(Path("/proc/uptime").read_text().split()[0])
    except Exception:
        return None
    return time.time() - uptime


def write_json_atomic(path: Path, data: Dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True, ensure_ascii=True)
    os.replace(tmp, path)


def load_json(path: Path) -> Optional[Dict]:
    if not path.exists():
        return None
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def latest_data_stats(root: Path) -> Tuple[Optional[float], Optional[str], Optional[int]]:
    if not root.exists():
        return None, None, None
    latest_mtime = None
    latest_path = None
    latest_size = None
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            if name == "global.sqlite3":
                continue
            path = os.path.join(dirpath, name)
            try:
                st = os.stat(path, follow_symlinks=False)
            except (FileNotFoundError, PermissionError):
                continue
            mtime = st.st_mtime
            if latest_mtime is None or mtime > latest_mtime:
                latest_mtime = mtime
                latest_path = path
                latest_size = st.st_size
    return latest_mtime, latest_path, latest_size


def crc32_file(path: Path, chunk_size: int = 1024 * 1024) -> Optional[int]:
    try:
        crc = 0
        with path.open("rb") as f:
            while True:
                chunk = f.read(chunk_size)
                if not chunk:
                    break
                crc = zlib.crc32(chunk, crc)
        return crc & 0xFFFFFFFF
    except Exception:
        return None


TRIP_INDEX_STRUCT = struct.Struct("<qqQII")


def find_trip_idx(day_dir: Path, trip_id: int) -> Optional[Path]:
    if not day_dir.exists():
        return None
    for p in day_dir.glob("trip_*.idx"):
        name = p.name
        if not (name.startswith("trip_") and name.endswith(".idx")):
            continue
        num_str = name[len("trip_") : -len(".idx")]
        if not num_str.isdigit():
            continue
        if int(num_str) == trip_id:
            return p
    return None


def read_trip_idx_summary(idx_path: Path) -> Optional[Tuple[int, int]]:
    try:
        with idx_path.open("rb") as f:
            total_records = 0
            last_end_ts = None
            while True:
                buf = f.read(TRIP_INDEX_STRUCT.size)
                if len(buf) < TRIP_INDEX_STRUCT.size:
                    break
                _, end_ts_ns, _, _, record_count = TRIP_INDEX_STRUCT.unpack(buf)
                total_records += int(record_count)
                last_end_ts = int(end_ts_ns)
    except Exception:
        return None
    if last_end_ts is None:
        return None
    return last_end_ts, total_records


def recover_global_sqlite(data_root: Path) -> Dict:
    start = time.time()
    db_path = data_root / "global.sqlite3"
    if not db_path.exists():
        return {
            "db_path": str(db_path),
            "status": "missing",
            "checked_rows": 0,
            "recovered_rows": 0,
            "missing_idx": 0,
            "duration_sec": time.time() - start,
        }

    recovered = 0
    checked = 0
    missing_idx = 0
    try:
        conn = sqlite3.connect(str(db_path), timeout=3.0)
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT sensor_topic, topic_folder, day, trip_id, end_ts_ns, number_of_records "
            "FROM global "
            "WHERE end_ts_ns IS NULL OR end_ts_ns = '' OR end_ts_ns = '0' OR number_of_records = 0;"
        )
        rows = cur.fetchall()
        for row in rows:
            checked += 1
            topic_folder = row["topic_folder"]
            day = row["day"]
            trip_id = int(row["trip_id"])
            day_dir = data_root / topic_folder / day
            idx_path = find_trip_idx(day_dir, trip_id)
            summary = read_trip_idx_summary(idx_path) if idx_path else None
            if not summary:
                missing_idx += 1
                continue
            end_ts_ns, record_count = summary
            if end_ts_ns == 0 and record_count == 0:
                continue
            cur.execute(
                "UPDATE global SET end_ts_ns = ?, number_of_records = ? "
                "WHERE sensor_topic = ? AND day = ? AND trip_id = ?;",
                (str(end_ts_ns), int(record_count), row["sensor_topic"], day, trip_id),
            )
            recovered += 1
        conn.commit()
        conn.close()
    except Exception as exc:
        return {
            "db_path": str(db_path),
            "status": f"error: {exc}",
            "checked_rows": checked,
            "recovered_rows": recovered,
            "missing_idx": missing_idx,
            "duration_sec": time.time() - start,
        }

    return {
        "db_path": str(db_path),
        "status": "ok",
        "checked_rows": checked,
        "recovered_rows": recovered,
        "missing_idx": missing_idx,
        "duration_sec": time.time() - start,
    }




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


def estimate_write_rate(samples: List[Dict]) -> Optional[float]:
    if len(samples) < 2:
        return None
    first = samples[0]
    last = samples[-1]
    dt = last["t"] - first["t"]
    if dt <= 0:
        return None
    return (last["size"] - first["size"]) / dt


def run_cmd(cmd: List[str], timeout: float = 10.0) -> Dict:
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
            text=True,
        )
    except Exception as exc:
        return {"ok": False, "error": str(exc), "cmd": cmd}
    return {
        "ok": result.returncode == 0,
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
        "cmd": cmd,
    }


def run_cmd_maybe_sudo(cmd: List[str], timeout: float = 10.0, allow_sudo: bool = True) -> Dict:
    result = run_cmd(cmd, timeout=timeout)
    if result.get("ok") or not allow_sudo:
        return result
    sudo_res = run_cmd(["sudo", "-n"] + cmd, timeout=timeout)
    sudo_res["sudo_attempted"] = True
    return sudo_res



def build_launch_command(args: argparse.Namespace) -> List[str]:
    cmd = [
        "ros2",
        "launch",
        args.launch_package,
        args.launch_file,
    ]
    if args.launch_namespace:
        cmd.append(f"namespace:={args.launch_namespace}")
    cmd.extend(args.launch_args or [])
    return cmd


def issue_reboot(command: str, timeout: float, allow_sudo: bool) -> Tuple[bool, Optional[str]]:
    if not command:
        return False, "reboot command is empty"
    cmd = shlex.split(command)
    result = run_cmd_maybe_sudo(cmd, timeout=timeout, allow_sudo=allow_sudo)
    if not result.get("ok"):
        err = (result.get("stderr") or "").strip() or (result.get("error") or "").strip()
        return False, err or f"reboot command failed with code {result.get('returncode')}"
    return True, None


def start_ingest_launch(args: argparse.Namespace, run_dir: Path) -> Optional[subprocess.Popen]:
    if args.no_launch:
        return None
    cmd = build_launch_command(args)
    log_path = run_dir / INGEST_LOG
    log_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        log_f = log_path.open("a", encoding="utf-8")
    except Exception:
        log_f = None
    try:
        return subprocess.Popen(
            cmd,
            stdout=log_f or subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    except Exception:
        if log_f:
            log_f.close()
        return None


def terminate_process(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        time.sleep(1.0)
    except Exception:
        pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        time.sleep(0.5)
    except Exception:
        pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        pass


def load_active_run() -> Optional[Path]:
    data = load_json(ACTIVE_RUN_FILE)
    if not data:
        return None
    run_dir = data.get("run_dir")
    if not run_dir:
        return None
    return Path(run_dir)


def set_active_run(run_dir: Path) -> None:
    write_json_atomic(
        ACTIVE_RUN_FILE,
        {"run_dir": str(run_dir), "updated_iso": now_iso()},
    )


def clear_active_run() -> None:
    try:
        ACTIVE_RUN_FILE.unlink()
    except FileNotFoundError:
        pass


def create_run_dir(run_id: Optional[str] = None) -> Path:
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    if not run_id:
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = RUNS_DIR / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def init_state(args: argparse.Namespace, run_dir: Path) -> Dict:
    boot_id = read_boot_id()
    boot_time = read_boot_time_epoch()
    now = time.time()
    data_mtime, data_path, data_size = latest_data_stats(args.data_root)
    crc_pre = None
    if data_path:
        p = Path(data_path)
        try:
            if p.is_file() and p.stat().st_size <= args.crc_max_mib * 1024 * 1024:
                crc_pre = crc32_file(p)
        except Exception:
            crc_pre = None
    size_now = dir_size_bytes(args.data_root)
    size_samples = [{"t": now, "size": size_now}]

    return {
        "run_id": run_dir.name,
        "status": "running",
        "start_epoch": now,
        "start_iso": now_iso(now),
        "boot_id": boot_id,
        "boot_time_epoch": boot_time,
        "boot_time_iso": now_iso(boot_time) if boot_time else None,
        "data_root": str(args.data_root),
        "last_heartbeat_epoch": now,
        "last_heartbeat_iso": now_iso(now),
        "last_data_mtime": data_mtime,
        "last_data_iso": now_iso(data_mtime) if data_mtime else None,
        "last_data_path": data_path,
        "last_data_size": data_size,
        "last_data_crc32": crc_pre,
        "size_samples": size_samples,
        "ingest_cmd": build_launch_command(args),
        "auto_reboot_after_sec": args.auto_reboot_after,
        "reboot_command": args.reboot_command,
    }


def update_state(state_path: Path, state: Dict) -> None:
    write_json_atomic(state_path, state)


def monitor_run(
    args: argparse.Namespace,
    run_dir: Path,
    state: Dict,
    ingest_proc: Optional[subprocess.Popen],
) -> None:
    state_path = run_dir / STATE_FILE
    start_epoch = state.get("start_epoch", time.time())
    next_size_epoch = time.time() + args.size_sample_interval if args.size_sample_interval > 0 else None

    while True:
        now = time.time()
        if args.max_runtime > 0 and now - start_epoch >= args.max_runtime:
            state["status"] = "stopped_max_runtime"
            break

        if args.auto_reboot_after > 0 and not state.get("reboot_requested"):
            if now - start_epoch >= args.auto_reboot_after:
                state["status"] = "reboot_requested"
                state["reboot_requested"] = True
                state["last_heartbeat_epoch"] = now
                state["last_heartbeat_iso"] = now_iso(now)
                state["reboot_requested_epoch"] = now
                state["reboot_requested_iso"] = now_iso(now)
                update_state(state_path, state)
                ok, err = issue_reboot(args.reboot_command, args.reboot_command_timeout, args.allow_reboot_sudo)
                if not ok:
                    state["status"] = "reboot_failed"
                    state["reboot_error"] = err
                    update_state(state_path, state)
                    print(f"[ERR] Reboot command failed: {err}")
                else:
                    print("[INFO] Reboot command issued; system should restart shortly.")
                break

        if ingest_proc and ingest_proc.poll() is not None:
            if args.auto_reboot_after > 0:
                state["ingest_exited"] = True
            else:
                state["status"] = "stopped_ingest_exit"
                break

        data_mtime, data_path, data_size = latest_data_stats(args.data_root)
        if data_mtime and (state.get("last_data_mtime") is None or data_mtime > state["last_data_mtime"]):
            state["last_data_mtime"] = data_mtime
            state["last_data_iso"] = now_iso(data_mtime)
            state["last_data_path"] = data_path
            state["last_data_size"] = data_size

        state["last_heartbeat_epoch"] = now
        state["last_heartbeat_iso"] = now_iso(now)

        if next_size_epoch and now >= next_size_epoch:
            size_now = dir_size_bytes(args.data_root)
            samples = state.get("size_samples", [])
            samples.append({"t": now, "size": size_now})
            if args.size_sample_max > 0 and len(samples) > args.size_sample_max:
                samples = samples[-args.size_sample_max :]
            state["size_samples"] = samples
            next_size_epoch = now + args.size_sample_interval

        update_state(state_path, state)
        time.sleep(args.scan_interval)

    update_state(state_path, state)


def wait_for_new_data(
    data_root: Path,
    last_mtime: Optional[float],
    scan_interval: float,
    timeout: float,
    min_epoch: Optional[float] = None,
) -> Tuple[Optional[float], Optional[str], Optional[int]]:
    start = time.time()
    while True:
        data_mtime, data_path, data_size = latest_data_stats(data_root)
        if data_mtime is not None and (last_mtime is None or data_mtime > last_mtime):
            if min_epoch is None or data_mtime >= min_epoch:
                return data_mtime, data_path, data_size
        if timeout > 0 and time.time() - start > timeout:
            return None, None, None
        time.sleep(scan_interval)


def write_report(run_dir: Path, report: Dict) -> None:
    write_json_atomic(run_dir / REPORT_JSON, report)
    lines = [
        "Power Loss Experiment Report",
        f"Run ID: {report.get('run_id')}",
        f"Start Time: {report.get('start_iso')}",
        f"Boot Time (post-reboot): {report.get('reboot_boot_time_iso')}",
        f"Power Cut Interval (s): {report.get('power_cut_interval_sec')}",
        f"Recovery Time (s): {report.get('recovery_time_sec')}",
        f"Data Loss Window (s): {report.get('data_loss_window_sec')}",
        f"Estimated Write Rate (B/s): {report.get('estimated_write_rate_bytes_per_sec')}",
        f"Estimated Lost Bytes: {report.get('estimated_lost_bytes')}",
        f"Last durable record before crash: {report.get('last_data_before_iso')} {report.get('last_data_before_path')}",
        f"First recovered valid record after reboot: {report.get('first_data_after_iso')} {report.get('first_data_after_path')}",
        f"CRC validation summary: {report.get('crc_validation_summary')}",
        f"SQLite recovery: {report.get('sqlite_recovery')}",
        f"Notes: {report.get('notes')}",
    ]
    (run_dir / REPORT_TXT).write_text("\n".join(lines) + "\n", encoding="utf-8")


def resume_after_reboot(args: argparse.Namespace, run_dir: Path, state: Dict) -> None:
    boot_time = read_boot_time_epoch()
    now = time.time()

    power_cut_interval = None
    if boot_time and state.get("last_heartbeat_epoch"):
        power_cut_interval = boot_time - state["last_heartbeat_epoch"]


    sqlite_recovery = recover_global_sqlite(args.data_root)

    avs_proc = start_ingest_launch(args, run_dir)
    last_before = state.get("last_data_mtime")
    last_before_path = state.get("last_data_path")
    crc_summary = None
    if last_before_path:
        p = Path(last_before_path)
        post_crc = None
        if p.exists():
            post_crc = crc32_file(p)
        crc_summary = {
            "path": last_before_path,
            "pre_crc32": state.get("last_data_crc32"),
            "post_crc32": post_crc,
            "match": state.get("last_data_crc32") is not None and post_crc == state.get("last_data_crc32"),
            "note": None if post_crc is not None else "missing or unreadable after reboot",
        }

    first_after_mtime, first_after_path, first_after_size = wait_for_new_data(
        args.data_root, last_before, args.scan_interval, args.recovery_timeout, min_epoch=boot_time
    )

    recovery_time = None
    if boot_time and first_after_mtime and first_after_mtime >= boot_time:
        recovery_time = first_after_mtime - boot_time

    data_loss_window = None
    if last_before and first_after_mtime and first_after_mtime >= last_before:
        data_loss_window = first_after_mtime - last_before

    size_samples = state.get("size_samples", [])
    write_rate = estimate_write_rate(size_samples)
    lost_bytes = None
    if write_rate is not None and data_loss_window is not None:
        lost_bytes = max(0.0, write_rate * data_loss_window)

    report = {
        "run_id": state.get("run_id"),
        "start_epoch": state.get("start_epoch"),
        "start_iso": state.get("start_iso"),
        "reboot_boot_time_epoch": boot_time,
        "reboot_boot_time_iso": now_iso(boot_time) if boot_time else None,
        "reboot_detected_epoch": now,
        "reboot_detected_iso": now_iso(now),
        "power_cut_interval_sec": power_cut_interval,
        "recovery_time_sec": recovery_time,
        "data_loss_window_sec": data_loss_window,
        "estimated_write_rate_bytes_per_sec": write_rate,
        "estimated_lost_bytes": lost_bytes,
        "last_data_before_epoch": last_before,
        "last_data_before_iso": now_iso(last_before) if last_before else None,
        "last_data_before_path": last_before_path,
        "first_data_after_epoch": first_after_mtime,
        "first_data_after_iso": now_iso(first_after_mtime) if first_after_mtime else None,
        "first_data_after_path": first_after_path,
        "crc_validation_summary": crc_summary,
        "sqlite_recovery": sqlite_recovery,
        "notes": None if first_after_mtime else "Recovery timeout: no new data detected",
    }
    if first_after_mtime and boot_time and first_after_mtime < boot_time:
        report["notes"] = (
            (report["notes"] + "; " if report["notes"] else "")
            + "First data timestamp precedes reboot; check system clock or file mtimes"
        )

    state["status"] = "completed"
    state["recovery_report"] = report
    update_state(run_dir / STATE_FILE, state)
    write_report(run_dir, report)

    if avs_proc and not args.leave_avs_running:
        terminate_process(avs_proc)

    clear_active_run()


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Power loss + recovery experiment runner.")
    ap.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    ap.add_argument("--scan-interval", type=float, default=2.0)
    ap.add_argument("--max-runtime", type=float, default=0.0, help="0 means no limit")
    ap.add_argument("--recovery-timeout", type=float, default=600.0)
    ap.add_argument("--crc-max-mib", type=int, default=64)
    ap.add_argument("--size-sample-interval", type=float, default=10.0)
    ap.add_argument("--size-sample-max", type=int, default=30)
    ap.add_argument("--start-new", action="store_true")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--leave-avs-running", action="store_true")
    ap.add_argument("--auto-reboot-after", type=float, default=0.0)
    ap.add_argument("--reboot-command", default="systemctl reboot")
    ap.add_argument("--reboot-command-timeout", type=float, default=10.0)
    ap.add_argument("--no-reboot-sudo", action="store_false", dest="allow_reboot_sudo", default=True)

    ap.add_argument("--launch-package", default="avs")
    ap.add_argument("--launch-file", default="avs_store.launch.py")
    ap.add_argument("--launch-namespace", default="")
    ap.add_argument("--launch-args", nargs="*", default=None)
    ap.add_argument("--no-launch", action="store_true")
    return ap.parse_args()


def main() -> int:
    args = parse_args()

    if args.start_new:
        run_dir = create_run_dir()
        state = init_state(args, run_dir)
        set_active_run(run_dir)
        ingest_proc = start_ingest_launch(args, run_dir)
        monitor_run(args, run_dir, state, ingest_proc)
        return 0

    active_run_dir = load_active_run()
    if args.resume and not active_run_dir:
        print("[INFO] --resume: no active run found; nothing to do.")
        return 0

    if not active_run_dir:
        run_dir = create_run_dir()
        state = init_state(args, run_dir)
        set_active_run(run_dir)
        ingest_proc = start_ingest_launch(args, run_dir)
        monitor_run(args, run_dir, state, ingest_proc)
        return 0

    state = load_json(active_run_dir / STATE_FILE)
    if not state:
        if args.resume:
            print("[INFO] --resume: active run has no state; nothing to do.")
            return 0
        print("[ERR] Active run found but state.json is missing or unreadable.")
        return 1

    resumeable = {"running", "reboot_requested", "reboot_failed"}
    if state.get("status") not in resumeable:
        if args.resume:
            print("[INFO] --resume: active run not resumable; nothing to do.")
            return 0
        run_dir = create_run_dir()
        state = init_state(args, run_dir)
        set_active_run(run_dir)
        ingest_proc = start_ingest_launch(args, run_dir)
        monitor_run(args, run_dir, state, ingest_proc)
        return 0

    current_boot_id = read_boot_id()
    if state.get("boot_id") and state["boot_id"] != current_boot_id:
        resume_after_reboot(args, active_run_dir, state)
        return 0

    if args.resume:
        print("[ERR] --resume requested but no reboot detected.")
        return 1

    ingest_proc = start_ingest_launch(args, active_run_dir)
    monitor_run(args, active_run_dir, state, ingest_proc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
