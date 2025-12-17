# Benchmark avs v.s rosbag overall
```
./rosbag_report.py --duration 648
./rosbag_report.py --duration 648 --compression
./avs_report.py --duration 648
```

# Record SMART Example
```
sudo smartctl --all /dev/nvme0n1p3 > day1_avs_after.txt
```

# Benchmark archive
```
./archive_move_report.py --before 2025-09-01
```

# Benchmark retrive 
```
source ~/AVS-PI/.venv/bin/activate

python retrive_report.py   --avs-image-db /home/avs/DATA/SSD/db/avs_image.sqlite3   --avs-lidar-db /home/avs/DATA/SSD/db/avs_lidar.sqlite3 --gps-root /home/avs/DATA/SSD/gps   --image-topic /my_camera/pylon_ros2_camera_node/image_raw   --lidar-topic /sensing/lidar/top/pointcloud   --start 2025-08-28_16-35 --end 2025-08-30_18-06   --windows 6 --dur 75 --modality all   --retrieve-cli /home/avs/AVS-PI/install/avs/lib/avs/retrieve_cl
```

# Check SSD SMART Log
lsblk
sudo smartctl --all /dev/nvme0n1p3 > day1_avs_before.txt
