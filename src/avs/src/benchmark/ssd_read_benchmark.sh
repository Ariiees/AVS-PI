#!/usr/bin/env bash
set -euo pipefail

# SSD filesystem read-side microbench:
#   - Random 4KiB read latency on existing files (images/*.jpg, lidar_laz/*.laz)
#   - Metadata/search: time find/ls/stat on the tree
#
# Prefers fio; falls back to dd micro-reads. No CSV; prints summaries.

IMAGES=""
LIDAR=""
ROOT=""
LOOPS=3
RAND_READS=3000
DROP=false
FIO_RUNTIME=20         # seconds per dir (randread)
RAND_BS=$((4*1024))    # 4 KiB

usage() {
  cat <<EOF
Usage:
  $0 --root /SSD/ROOT [--loops N] [--rand-reads N] [--drop-caches]
  or
  $0 --images /path/images --lidar /path/lidar_laz [--loops N] [--rand-reads N] [--drop-caches]

Options:
  --root PATH         Root containing 'images' and 'lidar_laz'
  --images PATH       Images directory (jpg)
  --lidar PATH        LiDAR LAZ directory
  --loops N           Loops for averaging (default $LOOPS)
  --rand-reads N      Total random 4K reads across all loops per dir (default $RAND_READS)
  --drop-caches       Drop Linux page cache between runs (needs sudo)
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --root) ROOT="${2:-}"; shift 2;;
    --images) IMAGES="${2:-}"; shift 2;;
    --lidar)  LIDAR="${2:-}"; shift 2;;
    --loops) LOOPS="${2:-}"; shift 2;;
    --rand-reads) RAND_READS="${2:-}"; shift 2;;
    --drop-caches) DROP=true; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1"; usage; exit 1;;
  esac
done

if [[ -n "$ROOT" ]]; then
  IMAGES="${IMAGES:-$ROOT/images}"
  LIDAR="${LIDAR:-$ROOT/lidar_laz}"
fi
[[ -d "$IMAGES" && -d "$LIDAR" ]] || { echo "Dirs not found: $IMAGES or $LIDAR"; exit 1; }

have() { command -v "$1" >/dev/null 2>&1; }

drop_caches() {
  $DROP || return 0
  sync
  if [[ $EUID -ne 0 ]]; then
    echo 3 | sudo tee /proc/sys/vm/drop_caches >/dev/null
  else
    echo 3 > /proc/sys/vm/drop_caches
  fi
}

ns() { date +%s%N; }

time_cmd() { # prints seconds (floating) to stdout
  { /usr/bin/time -f "%e" -- "$@" >/dev/null; } 2>&1;
}

# ---------- Metadata / search ----------
meta_walk() {
  local dir="$1" label="$2"
  echo "META [$label]: directory = $dir"

  # Count files fast (no content read)
  local t_find_all t_find_jpg t_find_laz t_ls t_stat
  t_find_all=$(time_cmd find "$dir" -type f)
  # Specific patterns (simulate real-world queries)
  t_find_jpg=$(time_cmd find "$dir" -type f -name '*.jpg')
  t_find_laz=$(time_cmd find "$dir" -type f -name '*.laz')

  # Recursive ls with metadata (triggers directory traversal + stat-like work)
  t_ls=$(time_cmd bash -c "ls -lR \"$dir\" >/dev/null")

  # Sample stat on up to 500 files (metadata access without reading payloads)
  mapfile -t sample < <(find "$dir" -type f -printf '%p\n' | head -n 500)
  if (( ${#sample[@]} )); then
    t_stat=$(time_cmd bash -c 'for f in "$@"; do stat -c "%n %s %y" "$f" >/dev/null; done' _ "${sample[@]}")
  else
    t_stat="NA"
  fi

  echo "  find -type f             : ${t_find_all}s"
  echo "  find -name '*.jpg'       : ${t_find_jpg}s"
  echo "  find -name '*.laz'       : ${t_find_laz}s"
  echo "  ls -lR                   : ${t_ls}s"
  echo "  stat sample (<=500 files): ${t_stat}s"
}

# ---------- Random latency (fio) ----------
rand_latency_fio() {
  local dir="$1" label="$2"
  echo "RAND [$label] (fio): 4KiB randread, iodepth=1, direct=1, ${FIO_RUNTIME}s"
  fio --name="randread_${label}" \
      --directory="$dir" \
      --rw=randread --bs=4k --iodepth=1 --ioengine=psync \
      --direct=1 --time_based=1 --runtime="$FIO_RUNTIME" \
      --numjobs=1 --group_reporting=1 2>&1 | awk 'NF{print}'
}

# ---------- Random latency (dd fallback) ----------
rand_latency_dd() {
  local dir="$1" label="$2"
  echo "RAND [$label] (dd): 4KiB random reads with O_DIRECT; total=$RAND_READS over $LOOPS loops"
  mapfile -t files < <(find "$dir" -maxdepth 1 -type f -printf "%p\n")
  (( ${#files[@]} )) || { echo "  no files found"; return; }

  local per_loop=$(( RAND_READS / LOOPS )); (( per_loop<1 )) && per_loop=1

  for ((i=1;i<=LOOPS;i++)); do
    drop_caches
    lat=()
    for ((r=0;r<per_loop;r++)); do
      f="${files[RANDOM % ${#files[@]}]}"
      size=$(stat -c '%s' "$f" || echo 0)
      (( size < RAND_BS )) && continue
      max_skip=$(( (size - RAND_BS) / RAND_BS ))
      (( max_skip < 1 )) && continue
      skip=$(( RANDOM % max_skip ))
      t0=$(ns)
      dd if="$f" of=/dev/null bs=$RAND_BS count=1 skip=$skip iflag=direct status=none 2>/dev/null || true
      t1=$(ns)
      lat_ms=$(awk -v n=$((t1 - t0)) 'BEGIN{printf("%.3f", n/1e6)}')
      lat+=("$lat_ms")
    done

    if (( ${#lat[@]} == 0 )); then
      echo "  loop $i: insufficient samples"
      continue
    fi

    readarray -t sorted < <(printf "%s\n" "${lat[@]}" | sort -n)
    n=${#sorted[@]}
    p50=${sorted[$(( n*50/100 ))]}
    p95=${sorted[$(( n*95/100 ))]}
    p99=${sorted[$(( n*99/100 < n ? n*99/100 : n-1 ))]}
    avg=$(printf "%s\n" "${sorted[@]}" | awk '{s+=$1} END{printf("%.3f", s/NR)}')
    echo "  loop $i: N=$n  avg=${avg} ms  p50=${p50} ms  p95=${p95} ms  p99=${p99} ms"
  done
}

# ---------- Run ----------
echo "========== SSD FS RAND + META =========="
echo "images: $IMAGES"
echo "lidar : $LIDAR"
echo "loops : $LOOPS | rand_reads: $RAND_READS | drop_caches: $DROP"
echo

# Metadata/search (cold-ish)
for d in "$IMAGES" "$LIDAR"; do
  drop_caches
  lbl=$(basename "$d")
  meta_walk "$d" "$lbl"
  echo
done

# Random latency
for d in "$IMAGES" "$LIDAR"; do
  lbl=$(basename "$d")
  drop_caches
  if have fio; then
    rand_latency_fio "$d" "$lbl"
  else
    rand_latency_dd  "$d" "$lbl"
  fi
  echo
done

echo "Done."
