# Benchmark avs v.s rosbag overall
```
./rosbag_report.py --duration 648
./rosbag_report.py --duration 648 --compression --storage-id mcap
./avs_report.py --duration 648
```

# Reduction throughput report (live topics, no ros2bag/file output)
```
./reduct_report.py --duration 30 --sensors camera,lidar,gps \
  --camera-topic /my_camera/pylon_ros2_camera_node/image_raw \
  --gps-topic /novatel/oem7/gps \
  --lidar-topic /sensing/lidar/top/pointcloud
```

# Check SSD SMART Log
```
lsblk
sudo smartctl --all /dev/nvme0n1p3 > day1_avs_before.txt
```

# Record SMART Example
```
sudo smartctl --all /dev/nvme0n1p3 > day1_avs_after.txt
```

# Benchmark query 
```
python3 retrieve_report.py /novatel/oem7/gps 1766013563038953891 1766013894106025761 bench 100
python3 cold_retrieve_report.py /sensing/lidar/top/pointcloud 1767982505628828344 1767983229373791665 bench 2000
SUMMARY	records	2000
```

# Benchmark archive
```
./archive_report.py 2025-09-01 /novatel/oem7/gps
```
