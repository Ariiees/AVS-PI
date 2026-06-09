#!/usr/bin/env python3
import os
import subprocess
import sys


def main():
    cmd = ["ros2", "run", "avs", "edge_query_benchmark", *sys.argv[1:]]
    raise SystemExit(subprocess.call(cmd, env=os.environ.copy()))


if __name__ == "__main__":
    main()
