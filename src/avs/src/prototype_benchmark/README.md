# Benchmark avs v.s rosbag overall
```
./rosbag_report.py --duration 648
./rosbag_report.py --duration 648 --compression
./avs_report.py --duration 648
```

# Check SSD SMART Log
lsblk
sudo smartctl --all /dev/nvme0n1p3 > day1_avs_before.txt

# Record SMART Example
```
sudo smartctl --all /dev/nvme0n1p3 > day1_avs_after.txt
```

# Benchmark query 
./retrive_report.py /novatel/oem7/gps 1766013563038953891 1766013894106025761 bench 100

# Benchmark archive
```
./archive_report.py 2025-09-01
```
