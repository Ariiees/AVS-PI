<h1 align="center">
  <img src="https://raw.githubusercontent.com/Ariiees/AVS-PI/main/img/CARLab-logo.png" alt="CAR Lab logo" width="56" align="absmiddle">
  Autonomous Vehicle Storage (AVS)
</h1>

<p align="center">
  Computational and hierarchical onboard data management for autonomous vehicle streams
</p>

AVS is a ROS 2 prototype that transforms transient camera, LiDAR, and GPS streams into compact, long-horizon, queryable vehicle history. It performs modality-aware reduction during ingest, writes processed records to an append-only SSD hot tier, and archives completed topic-days into indexed tar files on an HDD cold tier. Both tiers support time-range queries.

## Demo

The demo shows AVS querying retained vehicle data by sensor topic and time range, including image, LiDAR, and GPS records.

[Watch the AVS query demo (MP4)](https://github.com/Ariiees/AVS-PI/blob/main/img/avs.mp4)

## System Overview

AVS addresses three vehicle-data management requirements:

1. **Ingest-time reduction:** image perceptual-hash deduplication and JPEG compression, LiDAR voxel-grid downsampling and LAZ compression, and compact GPS records.
2. **Sustained ingest:** chunk-indexed append-only trip logs minimize random writes and write amplification.
3. **Queryable retention:** recent records remain on SSD while older topic-days are archived to HDD as directly queryable indexed tar files.

The current prototype uses these paper-selected defaults:

| Modality | Ingest processing | Default |
|---|---|---|
| Camera | pHash deduplication, then JPEG encoding | Hamming threshold `2`, JPEG quality `95` |
| LiDAR | Voxel-grid downsampling, then LAZ encoding | Leaf size `0.2 m` |
| GPS | Compact trajectory-oriented record | Latitude, longitude, altitude, and covariance |

Processed records are grouped into chunks and stored as `trip_XX.log` plus `trip_XX.idx`. A `global.sqlite3` catalog maps topics and time ranges to trips. Archival packs a completed topic-day into `Y-M-D.tar` plus an index, allowing cold queries without unpacking the archive.

## Hardware Requirements

The paper prototype uses:

| Component | Prototype hardware |
|---|---|
| Compute | Raspberry Pi 5, Broadcom BCM2712, 4-core Arm Cortex-A76 at 2.4 GHz |
| Memory | 8 GB RAM |
| Hot tier | 256 GB NVMe SSD, with a dedicated 100 GB data partition |
| Cold tier | 1 TB WD10SPZX HDD connected through a USB-to-SATA bridge |
| Vehicle link | Ethernet connection to the autonomous vehicle main computer |
| Filesystem | XFS on the SSD data partition and HDD |
| Operating system | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |

The evaluated vehicle workload contains a 10 Hz Hesai Pandar64 LiDAR, a 10 Hz Basler Ace mono8 front camera, and 50 Hz NovAtel OEM7 GNSS. Other sensors can be used by changing the topic and processing configuration.

## Hardware Connection

AVS runs on a separate computing unit so data processing and storage do not interfere with the vehicle's safety-critical main computer. The vehicle publishes ROS 2 sensor topics over Ethernet; the Raspberry Pi processes and stores recent data on NVMe, then archives older data to the USB-connected HDD.

[![AVS hardware connection diagram](https://raw.githubusercontent.com/Ariiees/AVS-PI/main/img/hydraD.png)](https://raw.githubusercontent.com/Ariiees/AVS-PI/main/img/hydraD.png)

Before running AVS:

1. Connect the Raspberry Pi and vehicle computer to the same Ethernet network.
2. Confirm ROS 2 discovery and topic visibility with `ros2 topic list`.
3. Attach and mount the NVMe data partition and HDD.
4. Confirm the configured roots are writable:

```bash
mkdir -p /home/avs/DATA/SSD /home/avs/DATA/HDD
test -w /home/avs/DATA/SSD
test -w /home/avs/DATA/HDD
```

Formatting or repartitioning storage destroys data. Use `lsblk -f` and configure mounts for your hardware before changing filesystems.

## Setup

### 1. Install ROS 2

Install ROS 2 Jazzy on Ubuntu 24.04 and verify that `/opt/ros/jazzy/setup.bash` exists. The included setup script installs AVS dependencies, but it does not install ROS 2 itself.

### 2. Clone at the prototype path

Several prototype tools currently default to `/home/avs/AVS-PI` and `/home/avs/DATA`. Clone at this path for the paper configuration:

```bash
cd /home/avs
git clone <repository-url> AVS-PI
cd AVS-PI
```

### 3. Install dependencies

```bash
export ROS_DISTRO=jazzy
sudo -E bash setup.bash
sudo apt-get install -y python3-psutil python3-yaml
```

The setup script installs the ROS 2 development packages and native libraries required by AVS, including OpenCV, PCL, VTK, SQLite, RocksDB, libarchive, OpenSSL, and LASzip.

### 4. Configure topics and storage

Edit:

- [`src/avs/config/avs_config.yaml`](src/avs/config/avs_config.yaml): ROS topics, SSD/HDD roots, and reduction settings.
- [`src/avs/config/topics.yaml`](src/avs/config/topics.yaml): mapping from each ROS topic to its sensor type and storage folder.

Verify the configured topics are live:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /my_camera/pylon_ros2_camera_node/image_raw
ros2 topic hz /sensing/lidar/top/pointcloud
ros2 topic hz /novatel/oem7/gps
```

### 5. Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/avs/AVS-PI
colcon build --symlink-install
source install/setup.bash
```

The CMake configuration currently selects the Raspberry Pi ARM64 VTK path. Building on x86-64 requires changing `VTK_DIR` in `src/avs/CMakeLists.txt`.

## Run AVS

Start all three ingest nodes with the append-only logger:

```bash
source /opt/ros/jazzy/setup.bash
source /home/avs/AVS-PI/install/setup.bash
ros2 launch avs avs_store.launch.py storage_backend:=append
```

Run an individual subscriber with explicit configuration paths:

```bash
ros2 run avs image_subscriber --ros-args \
  -p config_path:=/home/avs/AVS-PI/src/avs/config/avs_config.yaml \
  -p topic_map_path:=/home/avs/AVS-PI/src/avs/config/topics.yaml \
  -p storage_backend:=append
```

Replace `image_subscriber` with `lidar_subscriber` or `gps_subscriber` as needed. `storage_backend:=rocksdb` is available for backend comparison.

## Query Retained Data

Queries use a sensor topic and an inclusive nanosecond timestamp range. The interactive viewer queries the SSD hot-tier catalog. Use `cold_retrieve_report.py`, described under [Paper Benchmarks](#paper-benchmarks), for direct indexed queries over HDD archives.

```bash
# Image query
ros2 run avs retrieve_view \
  --topic /my_camera/pylon_ros2_camera_node/image_raw \
  --start <start_ts_ns> --end <end_ts_ns> --image

# LiDAR query
ros2 run avs retrieve_view \
  --topic /sensing/lidar/top/pointcloud \
  --start <start_ts_ns> --end <end_ts_ns> --lidar

# GPS query
ros2 run avs retrieve_view \
  --topic /novatel/oem7/gps \
  --start <start_ts_ns> --end <end_ts_ns> --gps
```

The image and LiDAR viewers use `n` or the right arrow for next and `p` or the left arrow for previous. The GPS viewer draws the complete trajectory for the requested time range; move the mouse over the line to inspect each GPS record. Press `q` to exit.

## Archive to the HDD Tier

Archive every completed topic-day older than a cutoff date:

```bash
ros2 run avs archive 2026-06-01
```

Archive only one topic:

```bash
ros2 run avs archive 2026-06-01 /sensing/lidar/top/pointcloud
```

The archive command writes indexed tar files under `/home/avs/DATA/HDD`, updates the HDD catalog, and removes the archived SSD copy only after the cold-tier artifacts are durable.

## Paper Benchmarks

Run benchmark scripts from their containing directories after sourcing ROS 2 and the workspace.

| Paper evaluation | Repository entry point | Purpose |
|---|---|---|
| Modality reduction throughput | `src/avs/src/prototype_benchmark/reduct_report.py` | Measures live camera, LiDAR, and GPS preprocessing |
| Logger backend comparison | `ros2 run avs db_benchmark` | Compares SQLite, RocksDB, and append-only ingest/query behavior on one captured stream |
| End-to-end ingest | `avs_report.py` and `rosbag_report.py` | Compares AVS resource use and footprint with rosbag2 |
| SSD/HDD behavior | `src/avs/src/benchmark/*.sh` | Measures SSD reads, archive writes, and HDD sequential reads |
| Hot/open-trip queries | `retrieve_report.py` and `retrieve_current.py` | Measures time-range retrieval over SSD logs |
| Cold queries | `cold_retrieve_report.py` | Measures direct retrieval from indexed HDD tar archives |
| Archival | `archive_report.py` | Measures topic-day migration from SSD to HDD |
| Power-loss recovery | `src/avs/src/prototype_benchmark/power_loss/` | Reboots during ingest and validates prefix recovery |
| Selective offload | `ros2 run avs offload ...` | Retrieves and streams one selected topic/time range |
| Encryption compatibility | `ros2 run avs encryption_report ...` | Measures AES-256-GCM encryption plus append-only logging and verifies authenticated decryption |

Examples:

```bash
cd /home/avs/AVS-PI/src/avs/src/prototype_benchmark

# Live reduction throughput
./reduct_report.py --duration 30 --sensors camera,lidar,gps

# AVS end-to-end ingest
./avs_report.py --duration 648

# rosbag2 comparison
./rosbag_report.py --duration 648
./rosbag_report.py --duration 648 --compression --storage-id mcap

# Closed-trip SSD query
python3 retrieve_report.py /novatel/oem7/gps <start_ts_ns> <end_ts_ns> bench 1000

# HDD archive query
python3 cold_retrieve_report.py /sensing/lidar/top/pointcloud \
  <start_ts_ns> <end_ts_ns> bench 2000

# Archive benchmark
./archive_report.py 2026-06-01
```

Encryption compatibility test:

```bash
# Uses at most 271 encoded image frames by default, matching the paper test size.
ros2 run avs encryption_report \
  --input-dir /path/to/encoded/image/frames \
  --output-root /tmp/avs_encryption_report \
  --max-frames 271
```

The benchmark reads each encoded frame, applies AES-256-GCM with a fresh 96-bit IV, appends `IV + authentication tag + ciphertext` through the AVS append-only logger, verifies authenticated decryption, and reports average, p50, p95, p99, and maximum encrypt-and-append latency. Its default output is under `/tmp`; keys are generated in memory and are not written to the repository.

Logger backend comparison:

```bash
ros2 run avs db_benchmark \
  --duration 120 \
  --sensors camera,lidar,gps \
  --config-path /home/avs/AVS-PI/src/avs/config/avs_config.yaml \
  --topic-map-path /home/avs/AVS-PI/src/avs/config/topics.yaml
```

Power-loss experiments intentionally reboot the machine. Read [`src/avs/src/prototype_benchmark/power_loss/README.md`](src/avs/src/prototype_benchmark/power_loss/README.md) before running them.

## Paper Results

On the Raspberry Pi 5 prototype and three days of real Level-4 vehicle traces, AVS:

- Reduced storage by `8.0-8.7x` compared with raw rosbag2.
- Returned the first hot-tier record in `0.9-9.1 ms`.
- Returned the first cold-tier record in `25.6-44.0 ms` without unpacking archives.
- Recovered after power loss in `9.98 s` with a bounded `0.24 s` loss window.
- Sustained approximately `107-108 MB/s` SSD-to-HDD archival throughput.

These results describe the paper's evaluated hardware, sensors, traces, and configuration; performance on another deployment will vary.

## Repository Layout

```text
.
├── img/                         # Logo, hardware figure, and demo media
├── setup.bash                   # Native dependency installer
└── src/avs/
    ├── config/                  # Topics, storage roots, and processing policy
    ├── include/avs/             # AVS headers
    ├── launch/                  # ROS 2 launch files
    └── src/
        ├── archive/             # SSD-to-HDD indexed tar archival
        ├── benchmark/           # Backend and filesystem benchmarks
        ├── logger/              # Append-only and comparison loggers
        ├── offload/             # Selective network offload
        ├── process/             # Modality-specific reduction
        ├── prototype_benchmark/ # Paper prototype evaluation scripts
        ├── retrieve/            # Hot/cold query API and viewer
        └── subscriber/          # Camera, LiDAR, and GPS ingest nodes
```

## Current Prototype Scope

AVS is a research prototype. The evaluated implementation handles one camera, one LiDAR, and one GPS stream; uses fixed, dataset-calibrated reduction policies; and provides prefix-durable append recovery rather than full transactional isolation. Review the code and configuration before using it for production or safety-critical workloads.
