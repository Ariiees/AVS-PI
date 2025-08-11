#!/usr/bin/env bash
set -euo pipefail

# HDD tar read benchmark (cold cache, direct IO), robust for ext4 & XFS.
# NO CSV output; prints per-tar averages and final overall average.
# Usage (sudo required to drop caches/flush buffers):
#   sudo ./hdd_read_benchmark.sh --tar-dir /home/avs/DATA/HDD/xfs/archive --bs 1M --loops 3

LC_ALL=C
LANG=C

TAR_DIR=""
BS="1M"
LOOPS=3   # run a few loops by default to reduce noise

err() { echo "ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tar-dir) TAR_DIR="${2:-}"; shift 2;;
    --bs)      BS="${2:-}"; shift 2;;
    --loops)   LOOPS="${2:-}"; shift 2;;
    -h|--help)
      cat <<EOF
HDD Tar Benchmark (cold cache, direct IO)
  --tar-dir <dir>   Directory containing .tar/.tar.gz/.tar.xz/.tar.zst archives
  --bs <size>       dd block size (default: 1M)
  --loops <N>       Cold-cache repetitions per tar (default: 3)
Requires: sudo, bsdtar, filefrag, dd, stat, findmnt, blockdev
EOF
      exit 0;;
    *) err "Unknown arg: $1";;
  esac
done

[[ -z "$TAR_DIR" ]] && err "Provide --tar-dir"
[[ -d "$TAR_DIR" ]] || err "Directory not found: $TAR_DIR"

for cmd in filefrag dd stat findmnt blockdev; do
  command -v "$cmd" >/dev/null || err "Missing dependency: $cmd"
done

# Find tars (non-recursive; change -maxdepth to taste)
mapfile -t TARS < <(find "$TAR_DIR" -maxdepth 1 -type f \
  \( -iname "*.tar" -o -iname "*.tar.gz" -o -iname "*.tgz" -o -iname "*.tar.xz" -o -iname "*.tar.zst" -o -iname "*.tar.bz2" -o -iname "*.tbz2" \) \
  | sort)
[[ ${#TARS[@]} -eq 0 ]] && err "No .tar* files found in $TAR_DIR"

# Resolve device and filesystem block size
DEV_SRC=$(findmnt -n -T "$TAR_DIR" -o SOURCE || true)
FS_BSIZE=$(stat -f -c %s "$TAR_DIR" 2>/dev/null || echo 4096)

flush_caches() {
  sync
  echo 3 > /proc/sys/vm/drop_caches
  if [[ -n "$DEV_SRC" && -e "$DEV_SRC" ]]; then
    blockdev --flushbufs "$DEV_SRC" 2>/dev/null || true
  fi
}

# Robust extent parser for ext4/XFS using "filefrag -e" (extent lines are colon-separated):
# Example (ext4/XFS):
#  " 0: 0..255: 123456..123711: 256 ..."
# We count lines like "  N: ..." and take the 3rd colon field (length in blocks).
frag_stats() {
  local f="$1"
  local sizeB exts maxlen
  sizeB=$(stat -c %s "$f")

  # Count extents and capture max length (in FS blocks)
  read -r exts maxlen < <(
    filefrag -e "$f" 2>/dev/null \
    | awk -F: '
        /^[[:space:]]*[0-9]+:/ {
          # fields: [index] : [logical range] : [physical range] : [length blocks] ...
          n=split($0,a,":");
          if (n>=4) {
            # strip non-digits
            g=a[4]; gsub(/[^0-9]/,"",g);
            if (g+0>0) {
              ecount++; if (g+0>max) max=g+0;
            }
          }
        }
        END{ if (ecount=="") ecount=0; if (max=="") max=0; print ecount, max }'
  )

  # Convert largest length blocks -> bytes
  local lrgB=$(( maxlen * FS_BSIZE ))

  # Average extent bytes (avoid div-by-zero; if 0 extents, treat as 1 for avg)
  local avgB
  if [[ "$exts" -gt 0 ]]; then
    avgB=$(( sizeB / exts ))
  else
    avgB="$sizeB"
  fi

  # Normalized fragmentation index: 1 - (largest_extent / size)
  local finx="0.000000"
  if [[ "$sizeB" -gt 0 ]]; then
    finx=$(awk -v l="$lrgB" -v s="$sizeB" 'BEGIN{
      if (1 > s) l = s;
      x = (s>0 ? (1.0 - (l/s)) : 0.0);
      if (x < 0) x = 0; if (x > 1) x = 1;
      printf "%.6f", x
    }')
  fi

  echo "$exts" "$lrgB" "$avgB" "$finx"
}

# One cold-cache direct read; returns "seconds MBps"
one_read() {
  local f="$1" bs="$2"
  flush_caches
  # Use /usr/bin/time for the elapsed seconds; dd with iflag=direct to bypass page cache
  local elapsed
  elapsed=$(/usr/bin/time -f "%e" dd if="$f" of=/dev/null iflag=direct,fullblock bs="$bs" status=none 2>&1 | tail -n1)
  local sizeB
  sizeB=$(stat -c %s "$f")
  local mbps
  mbps=$(awk -v s="$sizeB" -v t="$elapsed" 'BEGIN{printf "%.2f", (t>0? (s/1048576.0)/t : 0)}')
  echo "$elapsed" "$mbps"
}

echo "=== HDD Tar Benchmark (cold-cache, direct IO) ==="
echo "Dir: $TAR_DIR"
echo "Tars: ${#TARS[@]}"
echo "FS block size: $FS_BSIZE bytes"
echo "dd bs: $BS, loops: $LOOPS"
echo

total_bytes=0
sum_weighted_time=0  # sum of per-tar avg seconds
sum_size=0

for tarf in "${TARS[@]}"; do
  sizeB=$(stat -c %s "$tarf")
  read -r exts lrgB avgB finx < <(frag_stats "$tarf")

  # Loops (cold cache each time)
  sum_secs=0
  sum_mbps=0
  for ((i=1; i<=LOOPS; i++)); do
    read -r secs mbps < <(one_read "$tarf" "$BS")
    sum_secs=$(awk -v a="$sum_secs" -v b="$secs" 'BEGIN{printf "%.6f", a+b}')
    sum_mbps=$(awk -v a="$sum_mbps" -v b="$mbps" 'BEGIN{printf "%.6f", a+b}')
  done
  avg_secs=$(awk -v s="$sum_secs" -v n="$LOOPS" 'BEGIN{printf "%.6f", s/n}')
  avg_mbps=$(awk -v s="$sum_mbps" -v n="$LOOPS" 'BEGIN{printf "%.2f", s/n}')

  printf "[FRAG] %s | size=%dB  extents=%d  largest=%dB  avg_extent=%dB  frag_index=%s\n" \
    "$tarf" "$sizeB" "$exts" "$lrgB" "$avgB" "$finx"
  printf "[READ] %s | loops=%d  avg_seconds=%s  avg_MBps=%s\n" \
    "$tarf" "$LOOPS" "$avg_secs" "$avg_mbps"

  total_bytes=$(( total_bytes + sizeB ))
  # For the final "overall" MB/s, we weight simply by total size / sum of per-tar avg seconds
  sum_weighted_time=$(awk -v a="$sum_weighted_time" -v b="$avg_secs" 'BEGIN{printf "%.6f", a+b}')
done

echo
overall_mbps=$(awk -v bytes="$total_bytes" -v secs="$sum_weighted_time" 'BEGIN{printf "%.2f", (secs>0? (bytes/1048576.0)/secs : 0)}')
echo "----- OVERALL (cold-cache) -----"
echo "Total size read: $total_bytes bytes"
echo "Total (sum of per-tar averages): $sum_weighted_time s"
echo "Overall MB/s (size / sum_avg_seconds): $overall_mbps"
