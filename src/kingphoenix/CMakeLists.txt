cmake_minimum_required(VERSION 3.8)
project(kingphoenix)

find_package(ament_cmake REQUIRED)

# Install launch and configuration folders to the share directory
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()