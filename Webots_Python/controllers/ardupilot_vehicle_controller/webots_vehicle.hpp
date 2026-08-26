/*
   Class representing an ArduPilot controlled Webots Vehicle in C++
*/

#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace webots {
class Robot;
class Motor;
class Accelerometer;
class Gyro;
class InertialUnit;
class GPS;
class Camera;
class RangeFinder;
class Connector;
}

#pragma pack(push, 1)
struct ServoPacket {
    float motor_speed[16];
};

struct FdmPacket {
    double timestamp;
    double imu_angular_velocity_rpy[3];
    double imu_linear_acceleration_xyz[3];
    double imu_orientation_rpy[3];
    double velocity_xyz[3];
    double position_xyz[3];
};
#pragma pack(pop)

static_assert(sizeof(ServoPacket) == 64, "ServoPacket size mismatch with SITL");
static_assert(sizeof(FdmPacket) == 128, "FdmPacket size mismatch with SITL");

struct VehicleConfig {
    std::vector<std::string> motor_names = {"m1_motor", "m2_motor", "m3_motor", "m4_motor"};
    std::vector<int> reversed_motors;
    bool bidirectional_motors = false;
    bool uses_propellers = true;
    double motor_velocity_cap = std::numeric_limits<double>::infinity();

    std::string accel_name = "accelerometer";
    std::string imu_name = "inertial unit";
    std::string gyro_name = "gyro";
    std::string gps_name = "gps";

    std::string camera_name = "";
    int camera_fps = 10;
    int camera_stream_port = -1;

    std::string camera2_name = "";
    int camera2_fps = 10;
    int camera2_stream_port = -1;

    std::string rangefinder_name = "";
    int rangefinder_fps = 10;
    int rangefinder_stream_port = -1;

    int instance = 0;
    std::string sitl_address = "127.0.0.1";
};

class WebotsArduVehicle {
public:
    explicit WebotsArduVehicle(const VehicleConfig& config);
    ~WebotsArduVehicle();

    // Disable copy
    WebotsArduVehicle(const WebotsArduVehicle&) = delete;
    WebotsArduVehicle& operator=(const WebotsArduVehicle&) = delete;

    bool webots_connected() const {
        return webots_connected_.load();
    }

    void stop_motors();
    webots::Robot* get_robot() {
        return robot_.get();
    }

private:
    void handle_sitl();
    void handle_image_stream_camera(webots::Camera* cam, int port, int fps);
    void handle_image_stream_rangefinder(webots::RangeFinder* rf, int port, int fps);

    FdmPacket get_fdm_struct();
    void handle_controls(const ServoPacket& pkt);

    VehicleConfig config_;
    std::unique_ptr<webots::Robot> robot_;
    int timestep_ = 1;

    std::atomic<bool> webots_connected_{true};
    std::atomic<bool> cargo_released_{false};

    // Webots devices
    webots::Accelerometer* accel_ = nullptr;
    webots::InertialUnit* imu_ = nullptr;
    webots::Gyro* gyro_ = nullptr;
    webots::GPS* gps_ = nullptr;

    std::vector<webots::Motor*> motors_;
    webots::Motor* servo5_ = nullptr;
    webots::Connector* connector_ = nullptr;

    webots::Camera* camera_ = nullptr;
    webots::Camera* camera2_ = nullptr;
    webots::RangeFinder* rangefinder_ = nullptr;

    // Background threads
    std::thread sitl_thread_;
    std::thread camera_thread_;
    std::thread camera2_thread_;
    std::thread rangefinder_thread_;
};
