/*
   General ArduPilot vehicle controller for Webots in C++
*/

#include "webots_vehicle.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int sig) {
    (void)sig;
    g_shutdown_requested.store(true);
}

std::vector<std::string> split_string(const std::string& str, char delim) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        // Trim leading and trailing whitespace
        size_t first = token.find_first_not_of(" \t\r\n");
        if (first != std::string::npos) {
            size_t last = token.find_last_not_of(" \t\r\n");
            tokens.push_back(token.substr(first, (last - first + 1)));
        }
    }
    return tokens;
}

bool parse_bool(const std::string& val) {
    if (val == "true" || val == "True" || val == "1" || val == "TRUE") {
        return true;
    }
    return false;
}

void print_help(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options]\n\n"
              << "Options:\n"
              << "  -m, --motors <list>             Comma-spaced list of motor names (default: \"m1_motor, m2_motor, m3_motor, m4_motor\")\n"
              << "  -r, --reversed-motors <list>    Comma-spaced list of motors to reverse (1-indexed)\n"
              << "      --bidirectional-motors [b]  Motors are bidirectional (e.g. rovers, default: false)\n"
              << "      --uses-propellers [b]       Whether vehicle uses propellers for thrust linearization (default: true)\n"
              << "      --motor-cap <float>         Motor velocity cap (default: inf)\n"
              << "      --accel <name>              Webots accelerometer name (default: \"accelerometer\")\n"
              << "      --imu <name>                Webots IMU name (default: \"inertial unit\")\n"
              << "      --gyro <name>               Webots gyro name (default: \"gyro\")\n"
              << "      --gps <name>                Webots GPS name (default: \"gps\")\n"
              << "      --camera <name>             Webots Camera name (optional)\n"
              << "      --camera-fps <int>          Camera FPS (default: 10)\n"
              << "      --camera-port <int>         Port to stream camera MJPEG over UDP\n"
              << "      --camera2 <name>            Webots second Camera name (optional)\n"
              << "      --camera2-fps <int>         Second camera FPS (default: 10)\n"
              << "      --camera2-port <int>        Port to stream second camera MJPEG over UDP\n"
              << "      --rangefinder <name>        Webots RangeFinder name (optional)\n"
              << "      --rangefinder-fps <int>     Rangefinder FPS (default: 10)\n"
              << "      --rangefinder-port <int>    Port to stream rangefinder MJPEG over UDP\n"
              << "  -i, --instance <int>            Drone instance to match SITL (default: 0)\n"
              << "      --sitl-address <ip>         IP address of SITL (default: \"127.0.0.1\")\n"
              << "  -h, --help                      Show this help message\n";
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    VehicleConfig config;
    std::string motors_str = "m1_motor, m2_motor, m3_motor, m4_motor";
    std::string reversed_motors_str = "";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            print_help(argv[0]);
            return 0;
        } else if ((arg == "-m" || arg == "--motors") && i + 1 < argc) {
            motors_str = argv[++i];
        } else if ((arg == "-r" || arg == "--reversed-motors") && i + 1 < argc) {
            reversed_motors_str = argv[++i];
        } else if (arg == "--bidirectional-motors") {
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                config.bidirectional_motors = parse_bool(argv[++i]);
            } else {
                config.bidirectional_motors = true;
            }
        } else if (arg == "--uses-propellers") {
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                config.uses_propellers = parse_bool(argv[++i]);
            } else {
                config.uses_propellers = true;
            }
        } else if (arg == "--motor-cap" && i + 1 < argc) {
            config.motor_velocity_cap = std::stod(argv[++i]);
        } else if (arg == "--accel" && i + 1 < argc) {
            config.accel_name = argv[++i];
        } else if (arg == "--imu" && i + 1 < argc) {
            config.imu_name = argv[++i];
        } else if (arg == "--gyro" && i + 1 < argc) {
            config.gyro_name = argv[++i];
        } else if (arg == "--gps" && i + 1 < argc) {
            config.gps_name = argv[++i];
        } else if (arg == "--camera" && i + 1 < argc) {
            config.camera_name = argv[++i];
        } else if (arg == "--camera-fps" && i + 1 < argc) {
            config.camera_fps = std::stoi(argv[++i]);
        } else if (arg == "--camera-port" && i + 1 < argc) {
            config.camera_stream_port = std::stoi(argv[++i]);
        } else if (arg == "--camera2" && i + 1 < argc) {
            config.camera2_name = argv[++i];
        } else if (arg == "--camera2-fps" && i + 1 < argc) {
            config.camera2_fps = std::stoi(argv[++i]);
        } else if (arg == "--camera2-port" && i + 1 < argc) {
            config.camera2_stream_port = std::stoi(argv[++i]);
        } else if (arg == "--rangefinder" && i + 1 < argc) {
            config.rangefinder_name = argv[++i];
        } else if (arg == "--rangefinder-fps" && i + 1 < argc) {
            config.rangefinder_fps = std::stoi(argv[++i]);
        } else if (arg == "--rangefinder-port" && i + 1 < argc) {
            config.rangefinder_stream_port = std::stoi(argv[++i]);
        } else if ((arg == "-i" || arg == "--instance") && i + 1 < argc) {
            config.instance = std::stoi(argv[++i]);
        } else if (arg == "--sitl-address" && i + 1 < argc) {
            config.sitl_address = argv[++i];
        }
    }

    config.motor_names = split_string(motors_str, ',');

    if (!reversed_motors_str.empty()) {
        auto rev_tokens = split_string(reversed_motors_str, ',');
        for (const auto& token : rev_tokens) {
            try {
                config.reversed_motors.push_back(std::stoi(token));
            } catch (...) {
            }
        }
    }

    WebotsArduVehicle vehicle(config);

    while (vehicle.webots_connected() && !g_shutdown_requested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    vehicle.stop_motors();
    return 0;
}
