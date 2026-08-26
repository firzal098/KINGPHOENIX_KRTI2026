cmake_minimum_required(VERSION 3.8)
project(mavros_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate custom service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetString.srv"
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find ONNX Runtime (checks standard system paths /usr/include, /usr/local/include)
find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATH_SUFFIXES onnxruntime onnxruntime/core/session
)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "ONNX Runtime not found! Please ensure ONNX Runtime C++ headers and libraries are installed.")
endif()

# ==========================================
# Node Executable — Gate Estimator
# ==========================================
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

# ==========================================
# Node Executable — Controller
# ==========================================
add_executable(controller src/controller.cpp)

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

ament_target_dependencies(controller
  rclcpp
  ament_index_cpp
  geometry_msgs
  sensor_msgs
  std_srvs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(controller
  "${cpp_typesupport_target}"
  ${ONNXRUNTIME_LIB}
)

# ==========================================
# Install Targets & Assets
# ==========================================
install(TARGETS
  mavros_gate_estimator
  controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install include headers
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

# Install models directory to package share space
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()

ament_package()