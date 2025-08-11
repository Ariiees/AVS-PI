#!/usr/bin/env bash
set -euo pipefail

# HDD tar benchmark: cold-cache sequential read + fragmentation + storage efficiency
# Usage:
#   sudo ./hdd_tar_bench.sh --tar-dir /home/avs/DATA/HDD/archive --csv /home/avs/DATA/HDD/hdd_tar_bench.csv \
#        --bs 1M --loops 1
#
# Notes:
# - Must run with sudo to drop caches and flush device buffers.
# - Uses dd iflag=direct to bypass page cache (true disk throughput).
# - Storage efficiency computed from tar headers (no extraction).

TAR_DIR=""
CSV_OUT=""
BS="1M"
LOOPS=1

err() { echo "ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tar-dir) TAR_DIR="${2:-}"; shift 2;;
    --csv)     CSV_OUT="${2:-}"; shift 2;;
    --bs)      BS="${2:-}"; shift 2;;
    --loops)   LOOPS="${2:-}"; shift 2;;
    -h|--help)
      cat <<EOF
HDD Tar Benchmark
  --tar-dir <dir>   Directory containing .tar/.tar.gz/.tar.xz/.tar.zst archives
  --csv <path>      Optional CSV output
  --bs <size>       dd block size (default: 1M)
  --loops <N>       Repeat each tar read N times (drop caches each loop) (default: 1)
Requires: sudo, bsdtar, filefrag, dd, stat, findmnt, blockdev
EOF
      exit 0;;
    *) err "Unknown arg: $1";;
  esac
done

[[ -z "$TAR_DIR" ]] && err "Provide --tar-dir"
[[ -d "$TAR_DIR" ]] || err "Directory not found: $TAR_DIR"
command -v bsdtar >/dev/null || err "bsdtar not found (sudo apt-get install -y libarchive-tools)"
command -v filefrag >/dev/null || err "filefrag not found (sudo apt-get install -y e2fsprogs)"

# Find device backing TAR_DIR (for buffer flush)
DEV_SRC=$(findmnt -n -T "$TAR_DIR" -o SOURCE || true)
if [[ -z "$DEV_SRC" ]]; then
  echo "WARN: cannot resolve block device for $TAR_DIR; will skip blockdev flush." >&2
fi

# Gather tars
mapfile -t TARS < <(find "$TAR_DIR" -maxdepth 1 -type f \
  \( -iname "*.tar" -o -iname "*.tar.gz" -o -iname "*.tgz" -o -iname "*.tar.xz" -o -iname "*.tar.zst" -o -iname "*.tar.bz2" -o -iname "*.tbz2" \) \
  | sort)

[[ ${#TARS[@]} -eq 0 ]] && err "No .tar* files found in $TAR_DIR"

# CSV header
if [[ -n "$CSV_OUT" ]]; then
  echo "Tar,SizeBytes,Extents,LargestExtentBytes,AvgExtentBytes,FragIndex,FSBlockSize,Loops,BS,Seconds,MBps,MemberBytes,PackRatio" > "$CSV_OUT"
fi

# Resolve FS block size
FS_BSIZE=$(stat -f -c %s "$TAR_DIR" 2>/dev/null || echo 4096)

# Helper: drop caches and flush device buffers
flush_caches() {
  sync
  # drop pagecache, dentries, inodes
  echo 3 > /proc/sys/vm/drop_caches
  # Flush the underlying block device buffers if we can
  if [[ -n "$DEV_SRC" && -e "$DEV_SRC" ]]; then
    # If device is a mapper/partition, blockdev still works
    blockdev --flushbufs "$DEV_SRC" 2>/dev/null || true
  fi
}

# Helper: get fragmentation stats via filefrag -v
# Output: extents,lrg_bytes,avg_bytes,frag_index
frag_stats() {
  local f="$1"
  # filefrag -v output example lines:
  # "extents: N"
  # per-extent lines contain "... length: X blocks"
  local exts lrg len avg sizeB
  sizeB=$(stat -c %s "$f")
  # total extents
  exts=$(filefrag -v "$f" 2>/dev/null | awk -F: '/extents/ {gsub(/[^0-9]/,"",$2); print $2; exit}')
  # largest extent in blocks
  lrg=$(filefrag -v "$f" 2>/dev/null | awk '/[0-9]+:[ \t]/ && /length:/ {for(i=1;i<=NF;i++){if($i ~ /^length:/){g=$(i+1); gsub(/[^0-9]/,"",g); if(g>m)m=g}}} END{print (m>0?m:0)}')
  # avg extent length in bytes (approx): file_size / extents
  if [[ -z "$exts" || "$exts" -eq 0 ]]; then
    avg=0
  else
    avg=$(( sizeB / exts ))
  fi
  # convert largest blocks -> bytes (filefrag reports blocks of filesystem block size)
  local lrgB=$(( lrg * FS_BSIZE ))
  # frag index = 1 - (largest_extent / file_size)
  local finx="0"
  if [[ "$sizeB" -gt 0 ]]; then
    # bash float: use awk for precision
    finx=$(awk -v l="$lrgB" -v s="$sizeB" 'BEGIN{printf "%.6f", (s>0? (1.0 - (l/s)) : 0.0)}')
  fi
  echo "$exts" "$lrgB" "$avg" "$finx"
}

# Helper: member bytes from tar headers (no extraction)
member_bytes() {
  local tarf="$1"
  # bsdtar -tvf prints size in 3rd column typically; be robust: grab the first integer on the line
  bsdtar -tvf "$tarf" 2>/dev/null | awk '{for(i=1;i<=NF;i++){if($i ~ /^[0-9]+$/){sum+=$i; break}}} END{print (sum+0)}'
}

# Helper: one cold-cache sequential read (O_DIRECT). Outputs: seconds,MBps
one_read() {
  local f="$1"
  local bs="$2"

  # Use /usr/bin/time for reliable elapsed seconds; parse dd's bytes and time
  # dd with iflag=direct bypasses page cache for read path on ext4/xfs
  local out tmpfile
  tmpfile=$(mktemp)
  # dd stdout suppressed; stderr contains stats
  # LC_ALL=C to stabilize dd output format
  flush_caches
  out=$(/usr/bin/time -f "%e" bash -c "LC_ALL=C dd if='$f' of=/dev/null iflag=direct,fullblock bs='$bs' status=none" 2>&1)
  # 'out' will be like:
  # <seconds-from-time>
  # Extract seconds from the last line (the time output)
  local secs
  secs=$(echo "$out" | tail -n1)
  # Get file size
  local sizeB
  sizeB=$(stat -c %s "$f")
  # MB/s
  local mbps
  mbps=$(awk -v s="$sizeB" -v t="$secs" 'BEGIN{printf "%.2f", (t>0? (s/1048576.0)/t : 0)}')
  echo "$secs" "$mbps"
}

echo "=== HDD Tar Benchmark (cold-cache, direct IO) ==="
echo "Dir: $TAR_DIR"
echo "Tars: ${#TARS[@]}"
echo "FS block size: $FS_BSIZE bytes"
echo "dd bs: $BS, loops: $LOOPS"
echo

total_secs=0
total_bytes=0

for tarf in "${TARS[@]}"; do
  sizeB=$(stat -c %s "$tarf")
  read -r exts lrgB avgB finx < <(frag_stats "$tarf")
  mbytes=$(member_bytes "$tarf")
  # packing ratio: member_bytes / tar_size  (>=1 means no compression, >1 means sparse? for .tar it should be ~1)
  # For compressed tars (.tar.gz/.xz/.zst), ratio will be > 1.0 → more efficient packing.
  packratio=$(awk -v mb="$mbytes" -v sz="$sizeB" 'BEGIN{printf "%.4f", (sz>0? (mb/sz) : 0)}')

  # loops of cold-cache direct reads
  sum_secs=0
  sum_mbps=0
  for ((i=1; i<=LOOPS; i++)); do
    read -r secs mbps < <(one_read "$tarf" "$BS")
    sum_secs=$(awk -v a="$sum_secs" -v b="$secs" 'BEGIN{printf "%.6f", a+b}')
    sum_mbps=$(awk -v a="$sum_mbps" -v b="$mbps" 'BEGIN{printf "%.6f", a+b}')
  done
  avg_secs=$(awk -v s="$sum_secs" -v n="$LOOPS" 'BEGIN{printf "%.6f", s/n}')
  avg_mbps=$(awk -v s="$sum_mbps" -v n="$LOOPS" 'BEGIN{printf "%.2f", s/n}')

  total_secs=$(awk -v a="$total_secs" -v b="$avg_secs" 'BEGIN{printf "%.6f", a+b}')
  total_bytes=$(( total_bytes + sizeB ))

  printf "[FRAG] %s | size=%dB  extents=%s  largest=%dB  avg_extent=%dB  frag_index=%s  pack_ratio=%s\n" \
    "$tarf" "$sizeB" "$exts" "$lrgB" "$avgB" "$finx" "$packratio"
  printf "[READ] %s | loops=%d  avg_seconds=%s  avg_MBps=%s\n" \
    "$tarf" "$LOOPS" "$avg_secs" "$avg_mbps"

  if [[ -n "$CSV_OUT" ]]; then
    echo "\"$tarf\",$sizeB,$exts,$lrgB,$avgB,$finx,$FS_BSIZE,$LOOPS,$BS,$avg_secs,$avg_mbps,$mbytes,$packratio" >> "$CSV_OUT"
  fi
done

echo
overall_mbps=$(awk -v bytes="$total_bytes" -v secs="$total_secs" 'BEGIN{printf "%.2f", (secs>0? (bytes/1048576.0)/secs : 0)}')
echo "----- OVERALL (avg across tars, weighted by time) -----"
echo "Total size: $total_bytes bytes"
echo "Total seconds (sum of per-tar averages): $total_secs"
echo "Overall MB/s (size/sum_seconds): $overall_mbps"
[[ -n "$CSV_OUT" ]] && echo "CSV written: $CSV_OUT"
