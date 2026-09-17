cmake_minimum_required(VERSION 3.16)
project(adla_gate_inference)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Executable target: gate_perception
add_executable(gate_perception
  src/gate_perception_node.cpp
  src/adla_engine.cpp
  src/yolo_pose_decoder.cpp
)

target_include_directories(gate_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gate_perception
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(gate_perception
  ${OpenCV_LIBRARIES}
)

# Executable target: test_offline_inference
add_executable(test_offline_inference
  src/test_offline_inference.cpp
  src/adla_engine.cpp
  src/yolo_pose_decoder.cpp
)

target_include_directories(test_offline_inference PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(test_offline_inference
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(test_offline_inference
  ${OpenCV_LIBRARIES}
)

# Architecture-specific NNSDK link configuration
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  message(STATUS "[adla_gate_inference] Target architecture is ARM64 (aarch64). Configuring native AMLogic NNSDK.")
  
  find_library(NNSDK_LIB NAMES nnsdk PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64
    /usr/lib
    /usr/local/lib
    NO_DEFAULT_PATH
  )
  if(NOT NNSDK_LIB)
    find_library(NNSDK_LIB NAMES nnsdk)
  endif()

  if(NNSDK_LIB)
    message(STATUS "[adla_gate_inference] Linking against NNSDK library: ${NNSDK_LIB}")
    target_link_libraries(gate_perception ${NNSDK_LIB})
    target_link_libraries(test_offline_inference ${NNSDK_LIB})
    
    # Configure RPATH so the runtime linker finds libnnsdk.so
    set_target_properties(gate_perception PROPERTIES
      BUILD_RPATH "$ORIGIN/../lib:${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64"
      INSTALL_RPATH "$ORIGIN/../lib"
    )
    set_target_properties(test_offline_inference PROPERTIES
      BUILD_RPATH "$ORIGIN/../lib:${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64"
      INSTALL_RPATH "$ORIGIN/../lib"
    )
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64/libnnsdk.so DESTINATION lib)
  else()
    message(WARNING "[adla_gate_inference] libnnsdk.so not found on aarch64. Running in stub mode.")
  endif()
else()
  message(STATUS "[adla_gate_inference] Compiling on host (${CMAKE_SYSTEM_PROCESSOR}). Simulation mode active (stubbed NPU).")
endif()

# Installation
install(TARGETS
  gate_perception
  test_offline_inference
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY tests
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

