#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# AVS setup.sh
# Assumption:
#   ROS 2 is already installed on this machine.
#
# What this script does:
#   Installs ONLY system packages (apt) needed to satisfy the
#   CMakeLists.txt dependencies for AVS, including ROS 2 packages
#   that provide headers and CMake configs such as cv_bridge,
#   pcl_conversions, gps_msgs, etc.
#
# What this script does NOT do:
#   Does not install ROS 2 base itself.
#   It will not add ROS apt repositories or change ROS setup.
# ============================================================

log() { echo "[AVS-SETUP] $*"; }
die() { echo "[AVS-SETUP][ERROR] $*" >&2; exit 1; }

if [[ $EUID -ne 0 ]]; then
  die "Run with sudo: sudo bash setup.sh"
fi

ARCH="$(dpkg --print-architecture)"
. /etc/os-release

log "Operating System : Ubuntu ${VERSION_ID}"
log "Architecture     : ${ARCH}"
log "ROS 2            : assumed installed"

# -----------------------------
# ROS_DISTRO detection
# -----------------------------
ROS_DISTRO="${ROS_DISTRO:-}"
if [[ -z "${ROS_DISTRO}" ]]; then
  # Best effort detection from /opt/ros
  if [[ -d /opt/ros ]]; then
    # pick the newest directory name lexicographically as a heuristic
    ROS_DISTRO="$(ls -1 /opt/ros 2>/dev/null | sort | tail -n 1 || true)"
  fi
fi
if [[ -z "${ROS_DISTRO}" || ! -d "/opt/ros/${ROS_DISTRO}" ]]; then
  die "Cannot detect ROS_DISTRO. Export it first, e.g. export ROS_DISTRO=jazzy (or humble), and rerun."
fi
log "ROS_DISTRO        : ${ROS_DISTRO}"

log "Updating apt index"
apt-get update -y

# ============================================================
# 1) Base build tooling
# ============================================================
log "Installing build tooling"
apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  pkg-config \
  ninja-build \
  git \
  curl \
  ca-certificates \
  python3 \
  python3-pip \
  python3-colcon-common-extensions

# ============================================================
# 2) ROS 2 development packages required by find_package(...)
# ============================================================
log "Installing ROS 2 development packages required by AVS:"
log "  - ament_cmake, rclcpp, sensor_msgs"
log "  - cv_bridge + vision_opencv (CMake config for cv_bridge)"
log "  - pcl_conversions"
log "  - gps_msgs"
apt-get install -y --no-install-recommends \
  "ros-${ROS_DISTRO}-ament-cmake" \
  "ros-${ROS_DISTRO}-rclcpp" \
  "ros-${ROS_DISTRO}-sensor-msgs" \
  "ros-${ROS_DISTRO}-cv-bridge" \
  "ros-${ROS_DISTRO}-vision-opencv" \
  "ros-${ROS_DISTRO}-pcl-conversions" \
  "ros-${ROS_DISTRO}-gps-msgs"

# ============================================================
# 3) Non ROS libraries from CMakeLists.txt
# ============================================================
log "Installing non ROS system libraries required by AVS:"
log "  - OpenCV (imgcodecs, highgui) dev files"
log "  - yaml-cpp"
log "  - PCL (common, io, visualization)"
log "  - Boost filesystem"
log "  - SQLite3, RocksDB, libarchive"
log "  - OpenSSL (Crypto)"
apt-get install -y --no-install-recommends \
  libopencv-dev \
  libyaml-cpp-dev \
  libpcl-dev \
  libboost-filesystem-dev \
  libsqlite3-dev \
  librocksdb-dev \
  libarchive-dev \
  libssl-dev

# ============================================================
# 4) VTK stack (used by avs_retrieve_view)
# ============================================================
log "Installing VTK and OpenGL runtime dependencies (for viewer)"
apt-get install -y --no-install-recommends \
  libvtk9-dev \
  libvtk9.1 \
  libvtk9-qt-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev

# ============================================================
# 5) LASzip (for .laz)
# ============================================================
log "Installing LASzip (for LiDAR LAZ compression)"
if apt-cache show liblaszip-dev >/dev/null 2>&1; then
  log "  - Installing liblaszip-dev from apt"
  apt-get install -y --no-install-recommends liblaszip-dev
else
  log "  - liblaszip-dev not available via apt; building from source into /usr/local"
  apt-get install -y --no-install-recommends zlib1g-dev

  TMPDIR="$(mktemp -d)"
  pushd "${TMPDIR}" >/dev/null

  git clone --depth=1 https://github.com/LASzip/LASzip.git
  cmake -S LASzip -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j"$(nproc)"
  cmake --install build

  popd >/dev/null
  rm -rf "${TMPDIR}"
  ldconfig
  log "  - LASzip installed to /usr/local"
fi

# ============================================================
# 6) Sanity checks (informational)
# ============================================================
log "Sanity checks (informational, not fatal)"

# Check ROS environment presence
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  log "WARNING: /opt/ros/${ROS_DISTRO}/setup.bash not found"
fi

# pkg-config checks for modules you use via pkg_check_modules(...)
pkg-config --exists sqlite3 || log "WARNING: pkg-config cannot find sqlite3"
pkg-config --exists rocksdb || log "WARNING: pkg-config cannot find rocksdb"
pkg-config --exists libarchive || log "WARNING: pkg-config cannot find libarchive"

# VTK cmake directory hints
VTK_AARCH64="/usr/lib/aarch64-linux-gnu/cmake/vtk-9.1"
VTK_X86_64="/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1"
if [[ -d "${VTK_AARCH64}" ]]; then
  log "Found VTK CMake dir (aarch64): ${VTK_AARCH64}"
elif [[ -d "${VTK_X86_64}" ]]; then
  log "Found VTK CMake dir (x86_64): ${VTK_X86_64}"
  log "NOTE: Your CMakeLists hardcodes VTK_DIR to aarch64. Adjust if building on x86_64."
else
  log "WARNING: VTK CMake dir vtk-9.1 not found in common paths"
fi

# LASzip checks
if ! (ldconfig -p | grep -q "liblaszip" || ls /usr/lib/*/liblaszip.so* >/dev/null 2>&1 || ls /usr/local/lib/liblaszip.so* >/dev/null 2>&1); then
  log "WARNING: liblaszip not found by ldconfig; linker may fail until cache updates"
fi

cat <<EOF

============================================================
AVS dependency installation COMPLETE
============================================================

Installed categories:

A) ROS 2 dev packages (headers + CMake config):
   - ros-${ROS_DISTRO}-ament-cmake
   - ros-${ROS_DISTRO}-rclcpp
   - ros-${ROS_DISTRO}-sensor-msgs
   - ros-${ROS_DISTRO}-cv-bridge
   - ros-${ROS_DISTRO}-vision-opencv
   - ros-${ROS_DISTRO}-pcl-conversions
   - ros-${ROS_DISTRO}-gps-msgs

B) System libraries:
   - OpenCV, yaml-cpp, PCL
   - Boost filesystem
   - SQLite3, RocksDB, libarchive
   - OpenSSL
   - VTK (+ OpenGL runtime)
   - LASzip (apt or source build)

Next build commands:

  source /opt/ros/${ROS_DISTRO}/setup.bash
  cd <your_colcon_ws>
  colcon build --symlink-install
  source install/setup.bash

Note about VTK_DIR in your CMakeLists:
  - aarch64 path: /usr/lib/aarch64-linux-gnu/cmake/vtk-9.1
  - x86_64 path : /usr/lib/x86_64-linux-gnu/cmake/vtk-9.1

============================================================

EOF
