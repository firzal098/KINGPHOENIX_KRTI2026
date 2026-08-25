cmake_minimum_required(VERSION 3.8)
project(mavros_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Node Executable
add_executable(mavros_gate_estimator src/mavros_gate_estimator.cpp)

target_include_directories(mavros_gate_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(mavros_gate_estimator
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# Install executable
install(TARGETS
  mavros_gate_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

ament_package()