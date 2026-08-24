cmake_minimum_required(VERSION 3.8)
project(webots_camera_front)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher
  rclcpp
  sensor_msgs
  OpenCV
)

install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()