#!/usr/bin/env bash
set -euo pipefail

SSD_IMG=""; SSD_LAZ=""; SSD_PART=""
HDD_ROOT=""; HDD_PART=""
DO_DROP=0; SAFE_MODE=0

die(){ echo "[ERR] $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ssd-images) SSD_IMG="${2:?}"; shift 2 ;;
    --ssd-lidar)  SSD_LAZ="${2:?}"; shift 2 ;;
    --ssd-part)   SSD_PART="${2:?}"; shift 2 ;;
    --hdd-root)   HDD_ROOT="${2:?}"; shift 2 ;;
    --hdd-part)   HDD_PART="${2:?}"; shift 2 ;;
    --drop-caches) DO_DROP=1; shift ;;
    --safe-mode)   SAFE_MODE=1; shift ;;
    -h|--help)
      cat <<USAGE
Usage: $0 --ssd-images DIR --ssd-lidar DIR --ssd-part DEV --hdd-root DIR --hdd-part DEV [--drop-caches] [--safe-mode]
  DEV examples: nvme0n1p3, sda1
  Streams tar directly from SSD to HDD; no temp files on SSD.
USAGE
      exit 0;;
    *) die "Unknown arg: $1" ;;
  esac
done

[[ -n "$SSD_IMG" && -n "$SSD_LAZ" && -n "$SSD_PART" && -n "$HDD_ROOT" && -n "$HDD_PART" ]] \
  || die "Missing required args. Use -h."

mkdir -p "$HDD_ROOT"

sector_bytes(){
  local dev="$1"
  local path="/sys/class/block/$dev/queue/hw_sector_size"
  if [[ -f "$path" ]]; then cat "$path"; return; fi
  local base; base=$(basename "$(readlink -f "/sys/class/block/$dev/..")")
  cat "/sys/class/block/$base/queue/hw_sector_size"
}

# r_ios r_merges r_sectors r_ms  w_ios w_merges w_sectors w_ms
read_stats(){ awk -v d="$1" '$3==d {print $4,$5,$6,$7,$8,$9,$10,$11}' /proc/diskstats; }

delta_bytes_MB(){
  local s0="$1" s1="$2" sec_bytes="$3" field="$4"
  awk -v a="$s0" -v b="$s1" -v f="$field" -v sb="$sec_bytes" '
    BEGIN{split(a,A," "); split(b,B," "); d=(B[f]-A[f])*sb; printf "%.2f", d/1048576}'
}

drop_caches(){
  (( DO_DROP )) || return 0
  if [[ $EUID -ne 0 ]]; then
    echo "[WARN] --drop-caches requested but not root; skipping."
    return 0
  fi
  sync
  echo 3 > /proc/sys/vm/drop_caches || true
}

bytes_or_zero(){ [[ -d "$1" ]] && du -sb "$1" 2>/dev/null | awk '{print $1}' || echo 0; }
count_files(){ [[ -d "$1" ]] && find "$1" -type f -printf '.' 2>/dev/null | wc -c || echo 0; }

# ---- estimate preallocation (best-effort) ----
IMG_BYTES=$(bytes_or_zero "$SSD_IMG")
LAZ_BYTES=$(bytes_or_zero "$SSD_LAZ")
IMG_FILES=$(count_files "$SSD_IMG")
LAZ_FILES=$(count_files "$SSD_LAZ")
DATA_BYTES=$(( IMG_BYTES + LAZ_BYTES ))
HEADER_BYTES=$(( (IMG_FILES + LAZ_FILES) * 1024 ))   # ~1 KiB per file avg
CUSHION=$(( 10 * 1024 * 1024 ))                       # +10 MiB
PREALLOC=$(( DATA_BYTES + HEADER_BYTES + CUSHION ))

STAMP=$(date -u +"%Y%m%d_%H%M%S")
OUT_TAR="$HDD_ROOT/archive_${STAMP}.tar"

SSD_SEC_BYTES=$(sector_bytes "$SSD_PART")
HDD_SEC_BYTES=$(sector_bytes "$HDD_PART")

# ---- cold cache before benchmark (optional) ----
drop_caches

SSD_S0="$(read_stats "$SSD_PART")"
HDD_S0="$(read_stats "$HDD_PART")"
t0_ns=$(date +%s%N)

# ---- preallocate on HDD for contiguous extents (best-effort) ----
if command -v fallocate >/dev/null 2>&1; then
  : > "$OUT_TAR"
  fallocate -l "$PREALLOC" "$OUT_TAR" || true
fi

# ---- stream SSD -> HDD; capture exact bytes written WITHOUT any temp files ----
export SSD_IMG SSD_LAZ OUT_TAR
PIPE_CMD='set -euo pipefail
{
  if [[ -d "$SSD_IMG" ]]; then
    tar -cf - --hard-dereference --numeric-owner \
      --transform "s|^|images/|" -C "$SSD_IMG" .
  fi
  if [[ -d "$SSD_LAZ" ]]; then
    tar -cf - --hard-dereference --numeric-owner \
      --transform "s|^|lidar/|"  -C "$SSD_LAZ" .
  fi
} | tee >(dd of="$OUT_TAR" bs=4M conv=notrunc status=none) | wc -c
'

if (( SAFE_MODE )); then
  BYTES_WRITTEN=$(ionice -c2 -n7 nice -n 19 bash -c "$PIPE_CMD")
else
  BYTES_WRITTEN=$(bash -c "$PIPE_CMD")
fi

# shrink preallocated file to the exact stream size
if [[ "$BYTES_WRITTEN" =~ ^[0-9]+$ ]]; then
  truncate -s "$BYTES_WRITTEN" "$OUT_TAR" || true
fi

# ensure all writes hit the device before stopping the clock
sync

t1_ns=$(date +%s%N)
SSD_S1="$(read_stats "$SSD_PART")"
HDD_S1="$(read_stats "$HDD_PART")"

# ---- metrics: device bytes + wall time (device-true) ----
SSD_MB_READ=$(delta_bytes_MB "$SSD_S0" "$SSD_S1" "$SSD_SEC_BYTES" 3)    # r_sectors
HDD_MB_WRITTEN=$(delta_bytes_MB "$HDD_S0" "$HDD_S1" "$HDD_SEC_BYTES" 7) # w_sectors
WALL_S=$(awk -v a="$t0_ns" -v b="$t1_ns" 'BEGIN{printf "%.3f",(b-a)/1e9}')
SSD_R_MBPS=$(awk -v mb="$SSD_MB_READ" -v s="$WALL_S" 'BEGIN{printf "%.2f", mb/s}')
HDD_W_MBPS=$(awk -v mb="$HDD_MB_WRITTEN" -v s="$WALL_S" 'BEGIN{printf "%.2f", mb/s}')

# ---- fragmentation hint ----
EXTENTS_INFO="n/a"
if command -v filefrag >/dev/null 2>&1; then
  ex=$(filefrag -v "$OUT_TAR" 2>/dev/null | awk '/^ *[0-9]+:/ {c++} END{print c+0}')
  [[ -n "$ex" ]] && EXTENTS_INFO="extents_for_archive_tar=${ex}"
fi

OUT_MB=$(awk -v b="$BYTES_WRITTEN" 'BEGIN{printf "%.2f", b/1048576}')

echo "================= AVS ARCHIVE BENCHMARK (STREAM) ================="
echo "Stamp:                         $STAMP"
echo "Mode:                          $([[ $SAFE_MODE -eq 1 ]] && echo safe-mode || echo full-speed)"
echo "SSD partition:                 $SSD_PART"
echo "HDD partition:                 $HDD_PART"
echo "SSD sources:                   $SSD_IMG , $SSD_LAZ"
echo "HDD archive:                   $OUT_TAR"
echo
echo "Archive size (MB):             $OUT_MB"
echo "Wall time (s):                 $WALL_S"
echo "SSD read (MB):                 $SSD_MB_READ"
echo "SSD read throughput (MB/s):    $SSD_R_MBPS"
echo "HDD written (MB):              $HDD_MB_WRITTEN"
echo "HDD write throughput (MB/s):   $HDD_W_MBPS"
echo "Fragmentation hint (HDD):      $EXTENTS_INFO"
echo "=================================================================="
