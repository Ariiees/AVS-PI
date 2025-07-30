cmake_minimum_required(VERSION 3.10)
project(avs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sqlite3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_subscriber
  src/subscriber/image_subscriber.cpp
  src/process/img_dedup.cpp
)

target_link_libraries(image_subscriber
  ${OpenCV_LIBS}
  sqlite3
)

ament_target_dependencies(image_subscriber
  rclcpp sensor_msgs cv_bridge OpenCV sqlite3
)

install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
