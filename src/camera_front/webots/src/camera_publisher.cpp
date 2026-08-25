#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class WebotsUdpCameraBridge : public rclcpp::Node
{
public:
  WebotsUdpCameraBridge()
  : Node("webots_camera_bridge"), running_(true)
  {
    // Declare and retrieve parameters
    this->declare_parameter<std::string>("server_ip", "127.0.0.1");
    this->declare_parameter<int>("server_port", 5600);
    this->declare_parameter<std::string>("frame_id", "camera_front_optical_frame");
    this->declare_parameter<double>("fov", 1.0); // ~60 degrees horizontal FOV

    server_ip_ = this->get_parameter("server_ip").as_string();
    server_port_ = this->get_parameter("server_port").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    fov_ = this->get_parameter("fov").as_double();

    // Create ROS 2 Publishers
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

    RCLCPP_INFO(this->get_logger(), "Connecting to Webots UDP Stream at %s:%d",
                server_ip_.c_str(), server_port_);

    // Start background UDP receiver worker thread
    receiver_thread_ = std::thread(&WebotsUdpCameraBridge::udp_receiver_loop, this);
  }

  ~WebotsUdpCameraBridge() override
  {
    running_ = false;
    if (receiver_thread_.joinable()) {
      receiver_thread_.join();
    }
  }

private:
  void udp_receiver_loop()
  {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket: %s", strerror(errno));
      return;
    }

    // Set socket receive buffer size
    int rx_buf_size = 2 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rx_buf_size, sizeof(rx_buf_size));

    // Set receive timeout so the loop can check running_ flag periodically
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000; // 200 ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port_);
    inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr);

    std::vector<uint8_t> buffer(65535);
    auto last_ping_time = std::chrono::steady_clock::now() - 5s;

    while (running_ && rclcpp::ok()) {
      auto now = std::chrono::steady_clock::now();

      // Send ping packet every 2 seconds to keep the client registration alive in Webots
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_ping_time).count() >= 2) {
        const char ping_msg[] = "PING";
        sendto(sock, ping_msg, sizeof(ping_msg), 0,
               reinterpret_cast<struct sockaddr *>(&server_addr), sizeof(server_addr));
        last_ping_time = now;
      }

      // Receive UDP packet
      sockaddr_in src_addr {};
      socklen_t addr_len = sizeof(src_addr);
      ssize_t bytes_received = recvfrom(
        sock, buffer.data(), buffer.size(), 0,
        reinterpret_cast<struct sockaddr *>(&src_addr), &addr_len);

      if (bytes_received <= 0) {
        continue;
      }

      // Decode MJPEG frame buffer
      cv::Mat raw_bytes(1, static_cast<int>(bytes_received), CV_8UC1, buffer.data());
      cv::Mat decoded_frame = cv::imdecode(raw_bytes, cv::IMREAD_COLOR);

      if (decoded_frame.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Corrupted or incomplete JPEG frame received.");
        continue;
      }

      publish_camera_data(decoded_frame);
    }

    close(sock);
  }

  void publish_camera_data(const cv::Mat & frame)
  {
    auto stamp = this->now();

    // 1. Construct and publish Image message
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = stamp;
    img_msg.header.frame_id = frame_id_;
    img_msg.height = static_cast<uint32_t>(frame.rows);
    img_msg.width = static_cast<uint32_t>(frame.cols);
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_msg.is_bigendian = 0;
    img_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.cols * frame.elemSize());
    img_msg.data.resize(img_msg.step * img_msg.height);
    std::memcpy(img_msg.data.data(), frame.data, img_msg.data.size());

    image_pub_->publish(img_msg);

    // 2. Construct and publish CameraInfo message
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header.stamp = stamp;
    info_msg.header.frame_id = frame_id_;
    info_msg.width = img_msg.width;
    info_msg.height = img_msg.height;
    info_msg.distortion_model = "plumb_bob";
    info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // Calculate focal length from horizontal field of view
    double cx = img_msg.width / 2.0;
    double cy = img_msg.height / 2.0;
    double fx = cx / std::tan(fov_ / 2.0);
    double fy = fx;

    info_msg.k = {
      fx,  0.0, cx,
      0.0, fy,  cy,
      0.0, 0.0, 1.0
    };

    info_msg.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    info_msg.p = {
      fx,  0.0, cx,  0.0,
      0.0, fy,  cy,  0.0,
      0.0, 0.0, 1.0, 0.0
    };

    info_pub_->publish(info_msg);
  }

  // Members
  std::string server_ip_;
  int server_port_;
  std::string frame_id_;
  double fov_;
  std::atomic<bool> running_;
  std::thread receiver_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsUdpCameraBridge>());
  rclcpp::shutdown();
  return 0;
}