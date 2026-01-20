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
find_package(gps_msgs REQUIRED)  
find_package(OpenSSL REQUIRED)


pkg_check_modules(SQLITE3 REQUIRED sqlite3)
pkg_check_modules(ROCKSDB REQUIRED rocksdb)
pkg_check_modules(LIBARCHIVE REQUIRED libarchive)

# LASzip
find_path(LASZIP_INCLUDE_DIR
  NAMES laszip_api.h laszip_common.h
  PATH_SUFFIXES laszip
  HINTS /usr/local/include /usr/include
)

find_library(LASZIP_LIBRARY NAMES laszip HINTS /usr/local/lib /usr/lib)

# OpenCV (need highgui + imgcodecs for viewing .jpg)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs)

# PCL (need visualization for point cloud viewer)
find_package(PCL REQUIRED COMPONENTS common io visualization)

# --- VTK ---
set(VTK_DIR "/usr/lib/aarch64-linux-gnu/cmake/vtk-9.1")

find_package(VTK REQUIRED COMPONENTS
  CommonCore
  CommonDataModel
  FiltersCore
  IOLegacy
  RenderingCore
  RenderingOpenGL2
  InteractionStyle
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${SQLITE3_INCLUDE_DIRS}
  ${ROCKSDB_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${LASZIP_INCLUDE_DIR}
  ${VTK_INCLUDE_DIRS}
  ${LIBARCHIVE_INCLUDE_DIRS}
)

# # ========== File System Benchmark Image Write ==========
# add_executable(fs_img_write
#   src/benchmark/img_write_fs_benchmark.cpp
# )

# target_link_libraries(fs_img_write
#   ${OpenCV_LIBS}
#   yaml-cpp
# )

# ament_target_dependencies(fs_img_write
#   rclcpp sensor_msgs cv_bridge OpenCV yaml-cpp
# )  

# # ========== File System Benchmark Lidar Write ==========
# add_executable(fs_lidar_write
#   src/benchmark/lidar_write_fs_benchmark.cpp
#   src/process/lidar_compress.cpp
# )

# target_link_libraries(fs_lidar_write
#   ${PCL_LIBRARIES}
#   ${PCL_IO_LIBRARIES}
#   ${LASZIP_LIBRARY}
#   yaml-cpp
# )

# ament_target_dependencies(fs_lidar_write
#    rclcpp sensor_msgs pcl_conversions yaml-cpp
# )  


# # ========== DB Benchmark ==========
# add_executable(db_benchmark
#   src/benchmark/db_benchmark.cpp
# )

# target_include_directories(db_benchmark 
# PRIVATE
#   ${SQLITE3_INCLUDE_DIRS}
#   ${ROCKSDB_INCLUDE_DIRS}
# )

# target_link_libraries(db_benchmark
#   ${SQLITE3_LIBRARIES}
#   ${ROCKSDB_LIBRARIES}
#   Threads::Threads
#   dl
# )

# ========== Archive Move Manager ==========s
add_executable(archive
  src/archive/archive.cpp
  src/logger/topic_map.cpp
)

target_link_libraries(archive
  ${SQLITE3_LIBRARIES}
  yaml-cpp
  OpenSSL::Crypto
)

# ========== Image Reduct and Store ==========
add_executable(image_subscriber
  src/subscriber/image_subscriber.cpp
  src/process/img_dedup.cpp
  src/logger/append_logger.cpp
  src/logger/topic_map.cpp
  src/logger/trip_manager.cpp

)

target_link_libraries(image_subscriber
  ${OpenCV_LIBS}
  ${SQLITE3_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(image_subscriber
  rclcpp sensor_msgs cv_bridge OpenCV yaml-cpp
)

# ========== Lidar Downsample and Store ==========
add_executable(lidar_subscriber
  src/subscriber/lidar_subscriber.cpp
  src/process/lidar_compress.cpp
  src/process/lidar_downsample.cpp
  src/logger/append_logger.cpp
  src/logger/topic_map.cpp
  src/logger/trip_manager.cpp

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

# ========== GPS Reduced and Store ==========

add_executable(gps_subscriber
  src/subscriber/gps_subscriber.cpp
  src/logger/append_logger.cpp
  src/logger/topic_map.cpp
  src/logger/trip_manager.cpp 
)

target_link_libraries(gps_subscriber 
   ${SQLITE3_LIBRARIES}
   yaml-cpp
)

ament_target_dependencies(gps_subscriber 
  rclcpp gps_msgs yaml-cpp
) 


# ========== Retrieve View ==========
add_executable(retrieve_view
  src/retrieve/retrieve_cli.cpp
  src/retrieve/retrieve_api.cpp
)

target_link_libraries(retrieve_view
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${LASZIP_LIBRARY}
  ${SQLITE3_LIBRARIES}
  ${VTK_LIBRARIES}
)

ament_target_dependencies(retrieve_view
  rclcpp sensor_msgs cv_bridge OpenCV yaml-cpp pcl_conversions
)

# ========== UDP Offload ==========
add_executable(offload
  src/offload/offload.cpp
  src/retrieve/retrieve_api.cpp
)

ament_target_dependencies(offload
  rclcpp
)

target_link_libraries(offload
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${LASZIP_LIBRARY}
  ${SQLITE3_LIBRARIES}
  ${VTK_LIBRARIES}
)

# ========== Install Targets ==========
install(TARGETS
  image_subscriber
  lidar_subscriber
  gps_subscriber
  retrieve_view
  archive
  offload
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/avs
)

ament_package()
