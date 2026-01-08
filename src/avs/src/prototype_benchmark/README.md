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

# Benchmark query 
./retrive_report.py /novatel/oem7/gps 1766013563038953891 1766013894106025761 bench 100


# Check SSD SMART Log
lsblk
sudo smartctl --all /dev/nvme0n1p3 > day1_avs_before.txt
