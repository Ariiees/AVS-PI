# Run SSD Real-time Write Benchmark
ros2 run avs fs_img_write --ros-args -p output_dir:=/home/avs/DATA/HDD/images device_name:=sda1
ros2 run avs fs_img_write --ros-args -p output_dir:=/home/avs/DATA/SSD/images device_name:=nvme0n1p3

ros2 run avs fs_lidar_write --ros-args -p output_dir:=/home/avs/DATA/HDD/lidar_laz device_name:=sda1
ros2 run avs fs_lidar_write --ros-args -p output_dir:=/home/avs/DATA/SSD/lidar_laz device_name:=nvme0n1p3

# Run SSD Read Benchmark
sudo ./ssd_read_benchmark.sh --root /home/avs/DATA/SSD --loops 3 --rand-reads 5000 --drop-caches

# Run SSD to HDD Archive Benchmark
Example:
  ./archive_benchmark.sh \
    --ssd-images /home/avs/DATA/SSD/images \
    --ssd-lidar  /home/avs/DATA/SSD/lidar_laz \
    --ssd-part   nvme0n1p3 \
    --hdd-root   /home/avs/DATA/HDD/archive \
    --hdd-part   sda1 \
    --tmp        /home/avs/DATA/SSD/tmp_archives \
    --drop-caches             # optional (needs root)
    <!-- If you want to be gentle on a busy system:
    --safe-mode (adds nice/ionice) -->

sudo ./archive_benchmark.sh   --ssd-images /home/avs/DATA/SSD/images   --ssd-lidar  /home/avs/DATA/SSD/lidar_laz   --ssd-part   nvme0n1p3   --hdd-root   /home/avs/DATA/HDD/ext4/archive   --hdd-part   sda1   --drop-caches

# Run HDD Read Benchmark
sudo ./hdd_tar_bench.sh \
  --tar-dir /home/avs/DATA/HDD/ext4/archive \
  --bs 1M \
  --loops 1
  <!-- If you have multiple tar and want to output to a csv file:
  --csv /home/avs/DATA/HDD/hdd_tar_bench.csv \ -->


# Run DB Benchmark
ros2 run avs db_benchmark 
(optional)  [--images DIR] [--lidar DIR] [--sqlite FILE] [--rocks DIR]  [--ranges 1000] [--window-ms 1000]