# Retrieve Retained Data

The installed interactive retrieve executable is `retrieve_view`. It queries the SSD hot-tier catalog by sensor topic and inclusive nanosecond timestamp range.

Build and source the workspace before running it:

```bash
cd /home/avs/AVS-PI
colcon build --packages-select avs
source install/setup.bash
```

## Interactive Viewer

```bash
# Image
ros2 run avs retrieve_view \
  --topic /my_camera/pylon_ros2_camera_node/image_raw \
  --start <start_ts_ns> --end <end_ts_ns> --image

# LiDAR
ros2 run avs retrieve_view \
  --topic /sensing/lidar/top/pointcloud \
  --start <start_ts_ns> --end <end_ts_ns> --lidar

# GPS
ros2 run avs retrieve_view \
  --topic /novatel/oem7/gps \
  --start <start_ts_ns> --end <end_ts_ns> --gps
```

Image and LiDAR controls:

- Next frame: `n` or right arrow
- Previous frame: `p` or left arrow
- Quit: `q`

The GPS viewer draws the complete trajectory for the requested range. Move the mouse over the trajectory to inspect a GPS record, and press `q` to quit.

## Query Reports

The query report scripts are benchmark and listing tools, not interactive viewers:

```bash
cd /home/avs/AVS-PI/src/avs/src/prototype_benchmark

# List or benchmark records from closed SSD trips
python3 retrieve_report.py <sensor_topic> <start_ts_ns> <end_ts_ns> list
python3 retrieve_report.py <sensor_topic> <start_ts_ns> <end_ts_ns> bench [max_frames]

# Benchmark the current open SSD trip
python3 retrieve_current.py <sensor_topic> [max_frames]

# List or benchmark records from indexed HDD archives
python3 cold_retrieve_report.py <sensor_topic> <start_ts_ns> <end_ts_ns> list
python3 cold_retrieve_report.py <sensor_topic> <start_ts_ns> <end_ts_ns> bench [max_frames]
```
