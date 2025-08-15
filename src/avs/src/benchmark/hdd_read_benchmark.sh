#!/usr/bin/env bash
set -euo pipefail

# HDD tar read benchmark (cold cache, direct IO), tuned to be fair across EXT4/XFS.
# - Normalizes device readahead (sets a fixed value and restores after)
# - Verifies bs alignment to FS block size
# - Uses ionice to reduce scheduler variance
#
# Usage (sudo required to drop caches / tune device):
#   sudo ./hdd_read_benchmark.sh --tar-dir /path/to/archive --bs 1M --loops 3 [--set-ra-kb 1024]
#
# Notes:
# - Readahead is set on both the partition and its parent disk when applicable.
# - We restore the original readahead values on exit.

LC_ALL=C
LANG=C

TAR_DIR=""
BS="1M"
LOOPS=3
SET_RA_KB=1024   # unified readahead in KiB (1 MiB); override with --set-ra-kb 2048, etc.

err() { echo "ERROR: $*" >&2; exit 1; }
warn() { echo "WARN: $*" >&2; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tar-dir) TAR_DIR="${2:-}"; shift 2;;
    --bs)      BS="${2:-}"; shift 2;;
    --loops)   LOOPS="${2:-}"; shift 2;;
    --set-ra-kb) SET_RA_KB="${2:-}"; shift 2;;
    -h|--help)
      cat <<EOF
HDD Tar Benchmark (cold cache, direct IO) — fair XFS/EXT4
  --tar-dir <dir>     Directory containing .tar/.tar.* archives
  --bs <size>         dd block size (default: 1M)
  --loops <N>         cold-cache repetitions per tar (default: 3)
  --set-ra-kb <KiB>   device readahead to set during test (default: 1024 KiB)
Requires: sudo, filefrag, dd, stat, findmnt, blockdev
EOF
      exit 0;;
    *) err "Unknown arg: $1";;
  esac
done

[[ -z "$TAR_DIR" ]] && err "Provide --tar-dir"
[[ -d "$TAR_DIR" ]] || err "Directory not found: $TAR_DIR"

for cmd in filefrag dd stat findmnt blockdev awk sed ionice; do
  command -v "$cmd" >/dev/null || err "Missing dependency: $cmd"
done

# Find tars (non-recursive)
mapfile -t TARS < <(find "$TAR_DIR" -maxdepth 1 -type f \
  \( -iname "*.tar" -o -iname "*.tar.gz" -o -iname "*.tgz" -o -iname "*.tar.xz" -o -iname "*.tar.zst" -o -iname "*.tar.bz2" -o -iname "*.tbz2" \) \
  | sort)
[[ ${#TARS[@]} -eq 0 ]] && err "No .tar* files found in $TAR_DIR"

# Resolve device backing the directory (partition or disk)
DEV_SRC=$(findmnt -n -T "$TAR_DIR" -o SOURCE || true)
[[ -z "$DEV_SRC" ]] && err "Cannot resolve block device for $TAR_DIR"

# Partition device (e.g., /dev/sda2) and its parent (e.g., /dev/sda)
get_parent_dev() {
  local dev="$1"
  local base
  base=$(basename "$dev")
  # NVMe: nvme0n1p2 -> nvme0n1 ; SCSI: sda2 -> sda
  if [[ "$base" =~ ^(nvme.+)p[0-9]+$ ]]; then
    echo "/dev/${BASH_REMATCH[1]}"
  else
    echo "/dev/${base%%[0-9]*}"
  fi
}

PARENT_DEV=$(get_parent_dev "$DEV_SRC")

# FS block size and sector info
FS_BSIZE=$(stat -f -c %s "$TAR_DIR" 2>/dev/null || echo 4096)
LOG_SEC=$(blockdev --getss "$PARENT_DEV" 2>/dev/null || echo 512)

# Convert BS to bytes (supports K/M/G suffix)
bs_to_bytes() {
  local bs="$1"
  if [[ "$bs" =~ ^([0-9]+)([KMG]?)$ ]]; then
    local n="${BASH_REMATCH[1]}"
    local s="${BASH_REMATCH[2]}"
    case "$s" in
      K) echo $(( n * 1024 ));;
      M) echo $(( n * 1024 * 1024 ));;
      G) echo $(( n * 1024 * 1024 * 1024 ));;
      "") echo "$n";;
    esac
  else
    err "Unsupported bs format: $bs (use forms like 4096, 128K, 1M, 2M)"
  fi
}

BS_BYTES=$(bs_to_bytes "$BS")
if (( BS_BYTES % FS_BSIZE != 0 )); then
  warn "bs ($BS) is not a multiple of FS block size ($FS_BSIZE). Consider using a multiple (e.g., 1M)."
fi
if (( BS_BYTES % LOG_SEC != 0 )); then
  warn "bs ($BS) is not a multiple of logical sector size ($LOG_SEC). Consider adjusting (e.g., 1M)."
fi

# Save and set readahead (in KiB)
ORIG_RA_PART=""
ORIG_RA_PARENT=""
save_ra() {
  local d="$1"
  blockdev --getra "$d" 2>/dev/null || echo ""
}
set_ra_kb() {
  local d="$1" kb="$2"
  # blockdev --setra expects sectors of 512B; convert KiB->sectors
  local sectors=$(( (kb * 1024) / 512 ))
  blockdev --setra "$sectors" "$d" 2>/dev/null || true
  # also write queue/read_ahead_kb when available
  local sysrq="/sys/block/$(basename "$d")/queue/read_ahead_kb"
  [[ -w "$sysrq" ]] && echo "$kb" > "$sysrq" || true
}

restore_ra() {
  local d="$1" orig="$2"
  if [[ -n "$orig" ]]; then
    blockdev --setra "$orig" "$d" 2>/dev/null || true
    local kb=$(( (orig * 512) / 1024 ))
    local sysrq="/sys/block/$(basename "$d")/queue/read_ahead_kb"
    [[ -w "$sysrq" ]] && echo "$kb" > "$sysrq" || true
  fi
}

# Record originals (blockdev --getra returns sectors)
ORIG_RA_PART=$(blockdev --getra "$DEV_SRC" 2>/dev/null || echo "")
ORIG_RA_PARENT=$(blockdev --getra "$PARENT_DEV" 2>/dev/null || echo "")

cleanup() {
  # restore readahead
  restore_ra "$DEV_SRC" "$ORIG_RA_PART"
  restore_ra "$PARENT_DEV" "$ORIG_RA_PARENT"
}
trap cleanup EXIT

# Apply normalized readahead
set_ra_kb "$DEV_SRC" "$SET_RA_KB"
set_ra_kb "$PARENT_DEV" "$SET_RA_KB"

flush_caches() {
  sync
  echo 3 > /proc/sys/vm/drop_caches
  blockdev --flushbufs "$DEV_SRC" 2>/dev/null || true
  blockdev --flushbufs "$PARENT_DEV" 2>/dev/null || true
}

# Parse extents
frag_stats() {
  local f="$1"
  local sizeB exts maxlen
  sizeB=$(stat -c %s "$f")
  read -r exts maxlen < <(
    filefrag -e "$f" 2>/dev/null \
    | awk -F: '
        /^[[:space:]]*[0-9]+:/ {
          n=split($0,a,":");
          if (n>=4) {
            g=a[4]; gsub(/[^0-9]/,"",g);
            if (g+0>0) { ecount++; if (g+0>max) max=g+0; }
          }
        }
        END{ if (ecount=="") ecount=0; if (max=="") max=0; print ecount, max }'
  )
  local lrgB=$(( maxlen * FS_BSIZE ))
  local avgB
  if [[ "$exts" -gt 0 ]]; then
    avgB=$(( sizeB / exts ))
  else
    avgB="$sizeB"
  fi
  local finx="0.000000"
  if [[ "$sizeB" -gt 0 ]]; then
    finx=$(awk -v l="$lrgB" -v s="$sizeB" 'BEGIN{
      if (l > s) l = s;
      x = (s>0 ? (1.0 - (l/s)) : 0.0);
      if (x < 0) x = 0; if (x > 1) x = 1;
      printf "%.6f", x
    }')
  fi
  echo "$exts" "$lrgB" "$avgB" "$finx"
}

one_read() {
  local f="$1" bs="$2"
  flush_caches
  local elapsed
  # Use ionice to reduce variance; direct IO & fullblock for stable timing
  elapsed=$(/usr/bin/time -f "%e" ionice -c2 -n0 dd if="$f" of=/dev/null iflag=direct,fullblock bs="$bs" status=none 2>&1 | tail -n1)
  local sizeB
  sizeB=$(stat -c %s "$f")
  local mbps
  mbps=$(awk -v s="$sizeB" -v t="$elapsed" 'BEGIN{printf "%.2f", (t>0? (s/1048576.0)/t : 0)}')
  echo "$elapsed" "$mbps"
}

# Header
echo "=== HDD Tar Benchmark (cold-cache, direct IO) ==="
echo "Dir: $TAR_DIR"
echo "Tars: ${#TARS[@]}"
echo "Backing device (partition): $DEV_SRC"
echo "Backing device (parent)   : $PARENT_DEV"
echo "FS block size: $FS_BSIZE bytes | logical sector: $LOG_SEC bytes"
echo "dd bs: $BS ($BS_BYTES bytes), loops: $LOOPS"
echo "Readahead (normalized): ${SET_RA_KB} KiB (originals saved & will be restored)"
echo

total_bytes=0
sum_weighted_time=0

for tarf in "${TARS[@]}"; do
  sizeB=$(stat -c %s "$tarf")
  read -r exts lrgB avgB finx < <(frag_stats "$tarf")

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
  sum_weighted_time=$(awk -v a="$sum_weighted_time" -v b="$avg_secs" 'BEGIN{printf "%.6f", a+b}')
done

echo
overall_mbps=$(awk -v bytes="$total_bytes" -v secs="$sum_weighted_time" 'BEGIN{printf "%.2f", (secs>0? (bytes/1048576.0)/secs : 0)}')
echo "----- OVERALL (cold-cache) -----"
echo "Total size read: $total_bytes bytes"
echo "Total (sum of per-tar averages): $sum_weighted_time s"
echo "Overall MB/s (size / sum_avg_seconds): $overall_mbps"
