#!/usr/bin/env python3
import subprocess
import sys


def main() -> int:
    cmd = ["ros2", "run", "avs", "reduct_report", *sys.argv[1:]]
    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        print("ros2 command not found. Source your ROS 2 environment first.", file=sys.stderr)
        return 127


if __name__ == "__main__":
    raise SystemExit(main())
