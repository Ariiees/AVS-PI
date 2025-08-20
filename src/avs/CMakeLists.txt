cmake_minimum_required(VERSION 3.10)
project(avs)

cmake_policy(SET CMP0144 NEW)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(SQLITE3 REQUIRED sqlite3)
pkg_check_modules(ROCKSDB REQUIRED rocksdb)

# LASzip
find_path(LASZIP_INCLUDE_DIR
  NAMES laszip_api.h laszip_common.h
  PATH_SUFFIXES laszip
  HINTS /usr/local/include /usr/include
)

find_library(LASZIP_LIBRARY NAMES laszip HINTS /usr/local/lib /usr/lib)

# DB
# find_path(SQLITE3_INCLUDE_DIRS sqlite3.h)
# find_library(SQLITE3_LIBRARY sqlite3)

# find_path(ROCKSDB_INCLUDE_DIRS rocksdb/db.h)
# find_library(ROCKSDB_LIBRARY rocksdb)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${SQLITE3_INCLUDE_DIRS}
  ${ROCKSDB_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${LASZIP_INCLUDE_DIR}
)

# ========== Image Reduction and Save ==========
add_executable(image_subscriber
  src/subscriber/image_subscriber.cpp
  src/process/img_dedup.cpp
  src/process/img_compress.cpp
  src/db_operation/db_operation.cpp
)

target_link_libraries(image_subscriber
  ${OpenCV_LIBS}
  ${SQLITE3_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(image_subscriber
  rclcpp sensor_msgs cv_bridge OpenCV yaml-cpp
)

# ========== LiDAR Reduction and Save ==========
add_executable(lidar_subscriber
  src/subscriber/lidar_subscriber.cpp
  src/process/lidar_downsample.cpp
  src/process/lidar_compress.cpp
  src/db_operation/db_operation.cpp
)

target_link_libraries(lidar_subscriber
  ${PCL_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${LASZIP_LIBRARY}
  ${SQLITE3_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(lidar_subscriber
  rclcpp sensor_msgs pcl_conversions yaml-cpp
)

# ========== File System Benchmark Image Write ==========
add_executable(fs_img_write
  src/benchmark/img_write_fs_benchmark.cpp
)

target_link_libraries(fs_img_write
  ${OpenCV_LIBS}
  yaml-cpp
)

ament_target_dependencies(fs_img_write
  rclcpp sensor_msgs cv_bridge OpenCV yaml-cpp
)  

# ========== File System Benchmark Lidar Write ==========
add_executable(fs_lidar_write
  src/benchmark/lidar_write_fs_benchmark.cpp
  src/process/lidar_compress.cpp
)

target_link_libraries(fs_lidar_write
  ${PCL_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${LASZIP_LIBRARY}
  yaml-cpp
)

ament_target_dependencies(fs_lidar_write
   rclcpp sensor_msgs pcl_conversions yaml-cpp
)  


# ========== DB Benchmark ==========
add_executable(db_benchmark
  src/benchmark/db_benchmark.cpp
)

target_include_directories(db_benchmark 
PRIVATE
  ${SQLITE3_INCLUDE_DIRS}
  ${ROCKSDB_INCLUDE_DIRS}
)

target_link_libraries(db_benchmark
  ${SQLITE3_LIBRARIES}
  ${ROCKSDB_LIBRARIES}
  Threads::Threads
  dl
)

# ========== Install Targets ==========
install(TARGETS
  image_subscriber
  lidar_subscriber
  fs_img_write
  fs_lidar_write
  db_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
