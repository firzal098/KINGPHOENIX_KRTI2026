cmake_minimum_required(VERSION 3.10)
project(ardupilot_vehicle_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Webots home
if(NOT DEFINED WEBOTS_HOME)
    if(DEFINED ENV{WEBOTS_HOME})
        set(WEBOTS_HOME $ENV{WEBOTS_HOME})
    elseif(EXISTS "/mnt/c/Program Files/Webots")
        set(WEBOTS_HOME "/mnt/c/Program Files/Webots")
    elseif(EXISTS "/usr/local/webots")
        set(WEBOTS_HOME "/usr/local/webots")
    elseif(EXISTS "/snap/webots/current/usr/share/webots")
        set(WEBOTS_HOME "/snap/webots/current/usr/share/webots")
    elseif(EXISTS "/opt/webots")
        set(WEBOTS_HOME "/opt/webots")
    endif()
endif()

message(STATUS "Using WEBOTS_HOME: ${WEBOTS_HOME}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    "${WEBOTS_HOME}/include/controller/cpp"
    "${WEBOTS_HOME}/include/controller/c"
)

link_directories("${WEBOTS_HOME}/lib/controller")

add_executable(ardupilot_vehicle_controller
    ardupilot_vehicle_controller.cpp
    webots_vehicle.cpp
    toojpeg.cpp
)

find_package(Threads REQUIRED)
target_link_libraries(ardupilot_vehicle_controller
    PRIVATE
    CppController
    Controller
    Threads::Threads
)
