#!/usr/bin/env python3
# archive_bench.py
#
# Usage
#   chmod +x archive_bench.py
#   ./archive_bench.py <cutoff_day_YYYY-MM-DD> [topic]
#
# Behavior
#   - Runs: ros2 run avs archive <cutoff_day> [topic]
#   - Streams ALL archive.cpp stdout lines to terminal unchanged (ARCHIVE_PER, ARCHIVE_SUM, errors, etc.)
#   - Samples process CPU% and RSS during the run
#   - Prints ONE extra line at the end:
#       BENCH_SUM avg_cpu_pct=... max_cpu_pct=... avg_rss_kb=... max_rss_kb=... exit_code=...

import sys
import time
import subprocess

try:
    import psutil
except Exception as e:
    print(f"error psutil is required: {e}", file=sys.stderr)
    sys.exit(1)


def normalize_day(day: str) -> str:
    # Enforce YYYY-MM-DD (no conversion to underscores)
    if len(day) != 10:
        raise ValueError(f"bad day: {day}")
    if day[4] != "-" or day[7] != "-":
        raise ValueError(f"day must be YYYY-MM-DD: {day}")
    return day


def main() -> int:
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("usage: archive_bench.py <cutoff_day_YYYY-MM-DD> [topic]", file=sys.stderr)
        return 2

    cutoff = normalize_day(sys.argv[1])
    topic = sys.argv[2] if len(sys.argv) == 3 else None

    cmd = ["ros2", "run", "avs", "archive", cutoff]
    if topic:
        cmd.append(topic)

    p = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,          # line buffered
        universal_newlines=True,
    )

    proc = psutil.Process(p.pid)

    # Prime cpu_percent so the next call measures interval usage.
    try:
        proc.cpu_percent(interval=None)
    except Exception:
        pass

    sample_interval_s = 0.2
    cpu_samples = []
    rss_samples_kb = []

    # Stream output in real time while sampling CPU and RSS.
    assert p.stdout is not None
    last_sample_t = time.time()

    for line in p.stdout:
        # Pass through archive.cpp output unchanged
        sys.stdout.write(line)
        sys.stdout.flush()

        now = time.time()
        if now - last_sample_t >= sample_interval_s:
            last_sample_t = now
            try:
                cpu = float(proc.cpu_percent(interval=None))
            except Exception:
                cpu = 0.0
            try:
                rss_kb = int(proc.memory_info().rss // 1024)
            except Exception:
                rss_kb = 0

            cpu_samples.append(cpu)
            rss_samples_kb.append(rss_kb)

    rc = p.wait()

    # Final sample after exit (best effort)
    try:
        rss_kb = int(proc.memory_info().rss // 1024)
        rss_samples_kb.append(rss_kb)
    except Exception:
        pass

    if cpu_samples:
        avg_cpu = sum(cpu_samples) / len(cpu_samples)
        max_cpu = max(cpu_samples)
    else:
        avg_cpu = 0.0
        max_cpu = 0.0

    if rss_samples_kb:
        avg_rss = sum(rss_samples_kb) / len(rss_samples_kb)
        max_rss = max(rss_samples_kb)
    else:
        avg_rss = 0.0
        max_rss = 0

    print(
        "BENCH_SUM"
        f" avg_cpu_pct={avg_cpu:.6f}"
        f" max_cpu_pct={max_cpu:.6f}"
        f" avg_rss_kb={avg_rss:.2f}"
        f" max_rss_kb={max_rss}"
        f" exit_code={rc}"
    )

    return rc


if __name__ == "__main__":
    sys.exit(main())
