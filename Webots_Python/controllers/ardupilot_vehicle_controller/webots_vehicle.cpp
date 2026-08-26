/*
   Implementation of WebotsArduVehicle C++ bridge for ArduPilot SITL
*/

#include "webots_vehicle.hpp"
#include "toojpeg.hpp"

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Connector.hpp>
#include <webots/Device.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <unordered_set>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef SOCKET socket_t;
typedef int socklen_t;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <errno.h>
typedef int socket_t;
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
#define closesocket close
#endif

namespace {

#ifdef _WIN32
struct WinsockInit {
    WinsockInit() {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
    }
    ~WinsockInit() {
        WSACleanup();
    }
};
static WinsockInit s_winsock_init;
#endif

void set_socket_nonblocking(socket_t sock) {
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(sock, FIONBIO, &mode);
#else
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags != -1) {
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    }
#endif
}

} // namespace

WebotsArduVehicle::WebotsArduVehicle(const VehicleConfig& config)
    : config_(config)
{
    robot_ = std::make_unique<webots::Robot>();
    timestep_ = static_cast<int>(robot_->getBasicTimeStep());

    std::unordered_set<std::string> available_devices;
    for (int i = 0; i < robot_->getNumberOfDevices(); ++i) {
        webots::Device* dev = robot_->getDeviceByIndex(i);
        if (dev != nullptr) {
            available_devices.insert(dev->getName());
        }
    }

    // Init core sensors
    if (available_devices.count(config_.accel_name)) {
        accel_ = robot_->getAccelerometer(config_.accel_name);
        if (accel_) {
            accel_->enable(timestep_);
        }
    }

    if (available_devices.count(config_.imu_name)) {
        imu_ = robot_->getInertialUnit(config_.imu_name);
        if (imu_) {
            imu_->enable(timestep_);
        }
    }

    if (available_devices.count(config_.gyro_name)) {
        gyro_ = robot_->getGyro(config_.gyro_name);
        if (gyro_) {
            gyro_->enable(timestep_);
        }
    }

    if (available_devices.count(config_.gps_name)) {
        gps_ = robot_->getGPS(config_.gps_name);
        if (gps_) {
            gps_->enable(timestep_);
        }
    }

    // Init cargo drop servo (SERVO5) or connector
    if (available_devices.count("servo5")) {
        try {
            servo5_ = robot_->getMotor("servo5");
            if (servo5_) {
                servo5_->setPosition(0.0);
                servo5_->setVelocity(5.0);
            }
        } catch (...) {
            servo5_ = nullptr;
        }
    }

    if (available_devices.count("connector")) {
        try {
            connector_ = robot_->getConnector("connector");
            if (connector_) {
                connector_->enablePresence(timestep_);
                connector_->lock();
            }
        } catch (...) {
            connector_ = nullptr;
        }
    }

    // Init Camera 1
    if (!config_.camera_name.empty() && available_devices.count(config_.camera_name)) {
        try {
            camera_ = robot_->getCamera(config_.camera_name);
            if (camera_) {
                camera_->enable(1000 / config_.camera_fps);
                if (config_.camera_stream_port > 0) {
                    camera_thread_ = std::thread(&WebotsArduVehicle::handle_image_stream_camera, this,
                                                 camera_, config_.camera_stream_port, config_.camera_fps);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to init camera '" << config_.camera_name << "': " << e.what() << "\n";
            camera_ = nullptr;
        }
    }

    // Init Camera 2
    if (!config_.camera2_name.empty() && available_devices.count(config_.camera2_name)) {
        try {
            camera2_ = robot_->getCamera(config_.camera2_name);
            if (camera2_) {
                camera2_->enable(1000 / config_.camera2_fps);
                if (config_.camera2_stream_port > 0) {
                    camera2_thread_ = std::thread(&WebotsArduVehicle::handle_image_stream_camera, this,
                                                  camera2_, config_.camera2_stream_port, config_.camera2_fps);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to init camera2 '" << config_.camera2_name << "': " << e.what() << "\n";
            camera2_ = nullptr;
        }
    }

    // Init RangeFinder
    if (!config_.rangefinder_name.empty() && available_devices.count(config_.rangefinder_name)) {
        try {
            rangefinder_ = robot_->getRangeFinder(config_.rangefinder_name);
            if (rangefinder_) {
                rangefinder_->enable(1000 / config_.rangefinder_fps);
                if (config_.rangefinder_stream_port > 0) {
                    rangefinder_thread_ = std::thread(&WebotsArduVehicle::handle_image_stream_rangefinder, this,
                                                      rangefinder_, config_.rangefinder_stream_port, config_.rangefinder_fps);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to init rangefinder '" << config_.rangefinder_name << "': " << e.what() << "\n";
            rangefinder_ = nullptr;
        }
    }

    // Init motors
    motors_.reserve(config_.motor_names.size());
    for (const auto& name : config_.motor_names) {
        webots::Motor* m = nullptr;
        if (available_devices.count(name)) {
            m = robot_->getMotor(name);
        }
        if (m) {
            m->setPosition(INFINITY);
            m->setVelocity(0.0);
            motors_.push_back(m);
        } else {
            std::cerr << "Warning: Motor '" << name << "' not found on robot (I" << config_.instance << ")\n";
        }
    }

    // Start SITL communication thread
    sitl_thread_ = std::thread(&WebotsArduVehicle::handle_sitl, this);
}

WebotsArduVehicle::~WebotsArduVehicle() {
    webots_connected_.store(false);
    if (sitl_thread_.joinable()) {
        sitl_thread_.join();
    }
    if (camera_thread_.joinable()) {
        camera_thread_.join();
    }
    if (camera2_thread_.joinable()) {
        camera2_thread_.join();
    }
    if (rangefinder_thread_.joinable()) {
        rangefinder_thread_.join();
    }
}

void WebotsArduVehicle::stop_motors() {
    for (auto* m : motors_) {
        if (m) {
            m->setPosition(INFINITY);
            m->setVelocity(0.0);
        }
    }
}

FdmPacket WebotsArduVehicle::get_fdm_struct() {
    FdmPacket pkt;
    std::memset(&pkt, 0, sizeof(pkt));

    pkt.timestamp = robot_->getTime();

    const double* g = gyro_ ? gyro_->getValues() : nullptr;
    if (g) {
        pkt.imu_angular_velocity_rpy[0] = g[0];
        pkt.imu_angular_velocity_rpy[1] = -g[1];
        pkt.imu_angular_velocity_rpy[2] = -g[2];
    }

    const double* a = accel_ ? accel_->getValues() : nullptr;
    if (a) {
        pkt.imu_linear_acceleration_xyz[0] = a[0];
        pkt.imu_linear_acceleration_xyz[1] = -a[1];
        pkt.imu_linear_acceleration_xyz[2] = -a[2];
    }

    const double* i = imu_ ? imu_->getRollPitchYaw() : nullptr;
    if (i) {
        pkt.imu_orientation_rpy[0] = i[0];
        pkt.imu_orientation_rpy[1] = -i[1];
        pkt.imu_orientation_rpy[2] = -i[2];
    }

    const double* gps_vel = gps_ ? gps_->getSpeedVector() : nullptr;
    if (gps_vel) {
        pkt.velocity_xyz[0] = gps_vel[0];
        pkt.velocity_xyz[1] = -gps_vel[1];
        pkt.velocity_xyz[2] = -gps_vel[2];
    }

    const double* gps_pos = gps_ ? gps_->getValues() : nullptr;
    if (gps_pos) {
        pkt.position_xyz[0] = gps_pos[0];
        pkt.position_xyz[1] = -gps_pos[1];
        pkt.position_xyz[2] = -gps_pos[2];
    }

    return pkt;
}

void WebotsArduVehicle::handle_controls(const ServoPacket& pkt) {
    size_t num_motors = motors_.size();
    std::vector<float> commands(num_motors, 0.0f);

    for (size_t i = 0; i < num_motors && i < 16; ++i) {
        commands[i] = pkt.motor_speed[i];
    }

    // Bidirectional motors scaling (-1.0 to 1.0)
    if (config_.bidirectional_motors) {
        for (size_t i = 0; i < num_motors; ++i) {
            commands[i] = commands[i] * 2.0f - 1.0f;
        }
    }

    // Linearize propeller thrust
    if (config_.uses_propellers) {
        for (size_t i = 0; i < num_motors; ++i) {
            float v = commands[i];
            commands[i] = std::sqrt(std::abs(v)) * (v >= 0.0f ? 1.0f : -1.0f);
        }
    }

    // SERVO5 cargo release
    float servo5_cmd = pkt.motor_speed[4];
    if (servo5_ != nullptr && servo5_cmd >= 0.0f) {
        double target_pos = static_cast<double>(servo5_cmd) * 1.5708;
        servo5_->setPosition(target_pos);
    }
    if (connector_ != nullptr && servo5_cmd >= 0.0f) {
        if (servo5_cmd > 0.5f) {
            cargo_released_.store(true);
            connector_->unlock();
        } else {
            cargo_released_.store(false);
            connector_->lock();
        }
    }

    // Reverse specified motors (1-indexed)
    for (int rev : config_.reversed_motors) {
        if (rev >= 1 && static_cast<size_t>(rev) <= num_motors) {
            commands[rev - 1] *= -1.0f;
        }
    }

    // Set motor velocities
    for (size_t i = 0; i < num_motors; ++i) {
        webots::Motor* m = motors_[i];
        if (m) {
            double max_v = m->getMaxVelocity();
            double cap = std::min(max_v, config_.motor_velocity_cap);
            m->setVelocity(static_cast<double>(commands[i]) * cap);
        }
    }
}

void WebotsArduVehicle::handle_sitl() {
    int port = 9002 + 10 * config_.instance;
    int sitl_in_port = port + 1;

    socket_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == INVALID_SOCKET) {
        std::cerr << "Failed to create SITL UDP socket (I" << config_.instance << ")\n";
        webots_connected_.store(false);
        return;
    }

    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));

#ifdef _WIN32
    // Disable WSAECONNRESET on Windows
    DWORD dwBytesReturned = 0;
    BOOL bNewBehavior = FALSE;
    WSAIoctl(s, SIO_UDP_CONNRESET, &bNewBehavior, sizeof(bNewBehavior), NULL, 0, &dwBytesReturned, NULL, NULL);
#endif

    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(static_cast<uint16_t>(port));

    if (bind(s, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind SITL UDP socket on port " << port << " (I" << config_.instance << ")\n";
        closesocket(s);
        webots_connected_.store(false);
        return;
    }

    sockaddr_in sitl_dest_addr;
    std::memset(&sitl_dest_addr, 0, sizeof(sitl_dest_addr));
    sitl_dest_addr.sin_family = AF_INET;
    sitl_dest_addr.sin_port = htons(static_cast<uint16_t>(sitl_in_port));
    inet_pton(AF_INET, config_.sitl_address.c_str(), &sitl_dest_addr.sin_addr);

    std::cout << "Listening for ArduPilot SITL (I" << config_.instance << ") at 127.0.0.1:" << port << "\n" << std::flush;
    robot_->step(timestep_);

    // Wait for SITL packets
    while (webots_connected_.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s, &rfds);

        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000; // 10ms

        int res = select(static_cast<int>(s + 1), &rfds, nullptr, nullptr, &tv);
        if (res > 0 && FD_ISSET(s, &rfds)) {
            break;
        }

        if (connector_ != nullptr && !cargo_released_.load()) {
            connector_->lock();
        }

        if (robot_->step(timestep_) == -1) {
            closesocket(s);
            webots_connected_.store(false);
            return;
        }
    }

    std::cout << "Connected to ArduPilot SITL (I" << config_.instance << ")\n" << std::flush;

    // Main communication loop
    while (webots_connected_.load()) {
        fd_set rfds, wfds;
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_SET(s, &rfds);
        FD_SET(s, &wfds);

        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000; // 1ms

        int res = select(static_cast<int>(s + 1), &rfds, &wfds, nullptr, &tv);
        if (res < 0) {
            break;
        }

        if (FD_ISSET(s, &wfds)) {
            FdmPacket fdm = get_fdm_struct();
            sendto(s, reinterpret_cast<const char*>(&fdm), sizeof(fdm), 0,
                   reinterpret_cast<const sockaddr*>(&sitl_dest_addr), sizeof(sitl_dest_addr));
        }

        if (FD_ISSET(s, &rfds)) {
            ServoPacket pkt;
            sockaddr_in from_addr;
            socklen_t from_len = sizeof(from_addr);

            int n = recvfrom(s, reinterpret_cast<char*>(&pkt), sizeof(pkt), 0,
                             reinterpret_cast<sockaddr*>(&from_addr), &from_len);
            if (n >= static_cast<int>(sizeof(ServoPacket))) {
                handle_controls(pkt);

                int step_res = robot_->step(timestep_);
                if (step_res == -1) {
                    break;
                }
            }
        }
    }

    closesocket(s);
    webots_connected_.store(false);
    std::cout << "Lost connection to Webots (I" << config_.instance << ")\n";
}

void WebotsArduVehicle::handle_image_stream_camera(webots::Camera* cam, int port, int fps) {
    if (!cam) {
        return;
    }

    int width = cam->getWidth();
    int height = cam->getHeight();
    int sample_period_ms = cam->getSamplingPeriod();
    if (sample_period_ms <= 0) {
        sample_period_ms = 1000 / fps;
    }

    socket_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == INVALID_SOCKET) {
        std::cerr << "Failed to create camera UDP socket on port " << port << " (I" << config_.instance << ")\n";
        return;
    }

    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
    int sndbuf = 2 * 1024 * 1024;
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char*>(&sndbuf), sizeof(sndbuf));

#ifdef _WIN32
    DWORD dwBytesReturned = 0;
    BOOL bNewBehavior = FALSE;
    WSAIoctl(s, SIO_UDP_CONNRESET, &bNewBehavior, sizeof(bNewBehavior), NULL, 0, &dwBytesReturned, NULL, NULL);
#endif

    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(static_cast<uint16_t>(port));

    if (bind(s, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind camera UDP socket on port " << port << " (I" << config_.instance << ")\n";
        closesocket(s);
        return;
    }

    set_socket_nonblocking(s);

    std::cout << "Camera MJPEG stream started on UDP port " << port << " (I" << config_.instance << ") ("
              << width << "x" << height << " @ " << (1000.0 / sample_period_ms) << "fps)\n" << std::flush;

    struct ClientInfo {
        sockaddr_in addr;
        std::chrono::steady_clock::time_point last_seen;
    };
    std::map<std::string, ClientInfo> clients;

    std::vector<uint8_t> rgb_buffer(width * height * 3);
    std::vector<uint8_t> jpg_buffer;

    while (webots_connected_.load()) {
        auto start_time = std::chrono::steady_clock::now();

        // Process client registration / ping packets
        while (true) {
            char dummy[1024];
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int n = recvfrom(s, dummy, sizeof(dummy), 0, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
            if (n <= 0) {
                break;
            }

            char ip_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
            std::string client_key = std::string(ip_str) + ":" + std::to_string(ntohs(client_addr.sin_port));

            if (clients.find(client_key) == clients.end()) {
                std::cout << "Connected to camera UDP client " << client_key << " (I" << config_.instance << ")\n";
            }
            clients[client_key] = {client_addr, std::chrono::steady_clock::now()};
        }

        const unsigned char* image_bgra = cam->getImage();
        if (image_bgra != nullptr && !clients.empty()) {
            // Convert Webots BGRA to RGB
            for (int i = 0; i < width * height; ++i) {
                rgb_buffer[i * 3 + 0] = image_bgra[i * 4 + 2]; // R
                rgb_buffer[i * 3 + 1] = image_bgra[i * 4 + 1]; // G
                rgb_buffer[i * 3 + 2] = image_bgra[i * 4 + 0]; // B
            }

            if (TooJpeg::encodeJpeg(rgb_buffer.data(), static_cast<uint16_t>(width),
                                    static_cast<uint16_t>(height), true, 80, jpg_buffer)) {
                auto now = std::chrono::steady_clock::now();
                std::vector<std::string> dead_clients;

                for (const auto& kv : clients) {
                    auto elapsed_client = std::chrono::duration_cast<std::chrono::seconds>(now - kv.second.last_seen).count();
                    if (elapsed_client > 10) {
                        dead_clients.push_back(kv.first);
                        continue;
                    }
                    sendto(s, reinterpret_cast<const char*>(jpg_buffer.data()), static_cast<int>(jpg_buffer.size()), 0,
                           reinterpret_cast<const sockaddr*>(&kv.second.addr), sizeof(kv.second.addr));
                }

                for (const auto& dead : dead_clients) {
                    clients.erase(dead);
                    std::cout << "Camera UDP client disconnected " << dead << " (I" << config_.instance << ")\n";
                }
            }
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed < sample_period_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sample_period_ms - elapsed));
        }
    }

    closesocket(s);
}

void WebotsArduVehicle::handle_image_stream_rangefinder(webots::RangeFinder* rf, int port, int fps) {
    if (!rf) {
        return;
    }

    int width = rf->getWidth();
    int height = rf->getHeight();
    int sample_period_ms = rf->getSamplingPeriod();
    if (sample_period_ms <= 0) {
        sample_period_ms = 1000 / fps;
    }
    double min_range = rf->getMinRange();
    double max_range = rf->getMaxRange();
    double range_span = max_range - min_range;

    socket_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == INVALID_SOCKET) {
        std::cerr << "Failed to create RangeFinder UDP socket on port " << port << " (I" << config_.instance << ")\n";
        return;
    }

    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
    int sndbuf = 2 * 1024 * 1024;
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char*>(&sndbuf), sizeof(sndbuf));

#ifdef _WIN32
    DWORD dwBytesReturned = 0;
    BOOL bNewBehavior = FALSE;
    WSAIoctl(s, SIO_UDP_CONNRESET, &bNewBehavior, sizeof(bNewBehavior), NULL, 0, &dwBytesReturned, NULL, NULL);
#endif

    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(static_cast<uint16_t>(port));

    if (bind(s, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) == SOCKET_ERROR) {
        std::cerr << "Failed to bind RangeFinder UDP socket on port " << port << " (I" << config_.instance << ")\n";
        closesocket(s);
        return;
    }

    set_socket_nonblocking(s);

    std::cout << "RangeFinder MJPEG stream started on UDP port " << port << " (I" << config_.instance << ") ("
              << width << "x" << height << " @ " << (1000.0 / sample_period_ms) << "fps)\n" << std::flush;

    struct ClientInfo {
        sockaddr_in addr;
        std::chrono::steady_clock::time_point last_seen;
    };
    std::map<std::string, ClientInfo> clients;

    std::vector<uint8_t> gray_buffer(width * height);
    std::vector<uint8_t> jpg_buffer;

    while (webots_connected_.load()) {
        auto start_time = std::chrono::steady_clock::now();

        while (true) {
            char dummy[1024];
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int n = recvfrom(s, dummy, sizeof(dummy), 0, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
            if (n <= 0) {
                break;
            }

            char ip_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
            std::string client_key = std::string(ip_str) + ":" + std::to_string(ntohs(client_addr.sin_port));

            if (clients.find(client_key) == clients.end()) {
                std::cout << "Connected to RangeFinder UDP client " << client_key << " (I" << config_.instance << ")\n";
            }
            clients[client_key] = {client_addr, std::chrono::steady_clock::now()};
        }

        const float* range_image = rf->getRangeImage();
        if (range_image != nullptr && !clients.empty() && range_span > 0.0) {
            for (int i = 0; i < width * height; ++i) {
                float d = range_image[i];
                if (std::isinf(d) || d >= max_range) {
                    gray_buffer[i] = 255;
                } else if (d <= min_range) {
                    gray_buffer[i] = 0;
                } else {
                    float norm = static_cast<float>((d - min_range) / range_span);
                    gray_buffer[i] = static_cast<uint8_t>(std::clamp<float>(norm * 255.0f, 0.0f, 255.0f));
                }
            }

            if (TooJpeg::encodeJpeg(gray_buffer.data(), static_cast<uint16_t>(width),
                                    static_cast<uint16_t>(height), false, 80, jpg_buffer)) {
                auto now = std::chrono::steady_clock::now();
                std::vector<std::string> dead_clients;

                for (const auto& kv : clients) {
                    auto elapsed_client = std::chrono::duration_cast<std::chrono::seconds>(now - kv.second.last_seen).count();
                    if (elapsed_client > 10) {
                        dead_clients.push_back(kv.first);
                        continue;
                    }
                    sendto(s, reinterpret_cast<const char*>(jpg_buffer.data()), static_cast<int>(jpg_buffer.size()), 0,
                           reinterpret_cast<const sockaddr*>(&kv.second.addr), sizeof(kv.second.addr));
                }

                for (const auto& dead : dead_clients) {
                    clients.erase(dead);
                    std::cout << "RangeFinder UDP client disconnected " << dead << " (I" << config_.instance << ")\n";
                }
            }
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
        if (elapsed < sample_period_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sample_period_ms - elapsed));
        }
    }

    closesocket(s);
}
