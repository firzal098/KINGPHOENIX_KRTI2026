#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <mutex>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include "adla_gate_inference/adla_engine.hpp"
#include "adla_gate_inference/yolo_pose_decoder.hpp"

class GatePerceptionNode : public rclcpp::Node {
public:
    GatePerceptionNode()
    : Node("gate_perception_node") {
        // Declare and retrieve parameters matching cuda_gate_inference
        std::string default_model;
        try {
            default_model = ament_index_cpp::get_package_share_directory("adla_gate_inference") +
                            "/resource/gate_yolo_pose_int8.adla";
        } catch (...) {
            default_model = "gate_yolo_pose_int8.adla";
        }

        this->declare_parameter<std::string>("model_path", default_model);
        this->declare_parameter<std::string>("method", "pnp");
        this->declare_parameter<double>("conf_threshold", 0.50);
        this->declare_parameter<double>("corner_conf_threshold", 0.15);
        this->declare_parameter<double>("iou_threshold", 0.45);
        this->declare_parameter<bool>("publish_debug_image", true);

        model_path_ = this->get_parameter("model_path").as_string();
        method_ = this->get_parameter("method").as_string();
        conf_threshold_ = static_cast<float>(this->get_parameter("conf_threshold").as_double());
        corner_conf_thresh_ = static_cast<float>(this->get_parameter("corner_conf_threshold").as_double());
        iou_threshold_ = static_cast<float>(this->get_parameter("iou_threshold").as_double());
        publish_debug_ = this->get_parameter("publish_debug_image").as_bool();

        // Initialize ADLA NPU Engine
        if (!engine_.init(model_path_, 640, 640, 3)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ADLA Engine with model: %s", model_path_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "ADLA Gate Perception Node successfully initialized with model: %s", model_path_.c_str());
        }

        // Subscriptions
        auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos_sub,
            std::bind(&GatePerceptionNode::image_callback, this, std::placeholders::_1));

        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        sub_projected_pixels_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/estimator/projected_gate_pixels", qos_reliable,
            std::bind(&GatePerceptionNode::projected_pixels_callback, this, std::placeholders::_1));

        // 2D Corner Keypoints Publisher for C++ Estimator
        // Format per gate: [score, u0, v0, c0, u1, v1, c1, u2, v2, c2, u3, v3, c3]
        pub_corners_2d_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/perception/gate_corners_2d", 10);

        if (publish_debug_) {
            pub_debug_img_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/perception/debug_image", 10);
            pub_debug_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                "/perception/debug_image/compressed", 10);
        }

        RCLCPP_INFO(this->get_logger(), "Gate ADLA Perception Node active (Method: '%s', Model: %s)",
                    method_.c_str(), model_path_.c_str());
    }

private:
    void projected_pixels_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg && !msg->data.empty()) {
            std::lock_guard<std::mutex> lock(proj_mutex_);
            latest_projected_pixels_ = msg->data;
        }
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (!engine_.is_initialized()) {
            return;
        }

        // 1. Decode ROS Image to OpenCV Mat
        cv::Mat frame;
        if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
            frame = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()), msg->step);
        } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
            cv::Mat rgb(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()), msg->step);
            cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Unsupported image encoding: %s", msg->encoding.c_str());
            return;
        }

        float orig_h = static_cast<float>(frame.rows);
        float orig_w = static_cast<float>(frame.cols);

        // 2. Preprocess: Resize to 640x640 and convert BGR -> RGB for ADLA model
        cv::Mat resized_bgr;
        cv::resize(frame, resized_bgr, cv::Size(640, 640));
        cv::Mat resized_rgb;
        cv::cvtColor(resized_bgr, resized_rgb, cv::COLOR_BGR2RGB);

        // Ensure buffer is continuous
        if (!resized_rgb.isContinuous()) {
            resized_rgb = resized_rgb.clone();
        }

        // 3. Execute ADLA NPU Inference
        size_t num_output_floats = 0;
        const float* raw_outputs = engine_.run_inference_rgb(
            resized_rgb.data,
            640 * 640 * 3,
            num_output_floats
        );

        if (!raw_outputs || num_output_floats == 0) {
            return;
        }

        // 4. Decode Keypoints & Filter 4 Gate Corners (TL, TR, BR, BL)
        std::vector<GateDetection> detections = decoder_.decode(
            raw_outputs,
            num_output_floats,
            conf_threshold_,
            corner_conf_thresh_,
            iou_threshold_,
            orig_w,
            orig_h,
            640.0f,
            640.0f
        );

        // 5. Pack 2D Corner Measurements for downstream Estimator
        // Header prefix: [sec, nanosec]
        std_msgs::msg::Float64MultiArray corners_msg;
        corners_msg.data.reserve(2 + detections.size() * 13);
        corners_msg.data.push_back(static_cast<double>(msg->header.stamp.sec));
        corners_msg.data.push_back(static_cast<double>(msg->header.stamp.nanosec));

        for (const auto& det : detections) {
            corners_msg.data.push_back(static_cast<double>(det.score));
            // 4 corners: 0=TL, 1=TR, 2=BR, 3=BL
            for (int i = 0; i < 4; ++i) {
                corners_msg.data.push_back(static_cast<double>(det.corners[i].x));
                corners_msg.data.push_back(static_cast<double>(det.corners[i].y));
                corners_msg.data.push_back(static_cast<double>(det.corners[i].conf));
            }
        }

        pub_corners_2d_->publish(corners_msg);

        // 6. Debug Visualization (only when subscribers are active)
        if (publish_debug_) {
            bool has_img_subs = (pub_debug_img_->get_subscription_count() > 0);
            bool has_comp_subs = (pub_debug_compressed_->get_subscription_count() > 0);

            if (has_img_subs || has_comp_subs) {
                publish_debug_overlay(frame, detections, msg->header, has_img_subs, has_comp_subs);
            }
        }
    }

    void publish_debug_overlay(
        const cv::Mat& frame,
        const std::vector<GateDetection>& detections,
        const std_msgs::msg::Header& header,
        bool publish_raw,
        bool publish_compressed
    ) {
        cv::Mat vis_img = frame.clone();

        // 1. Render YOLO Detections
        if (!detections.empty()) {
            bool is_pixel_innov = (method_.find("pixel_innovation") != std::string::npos ||
                                   method_.find("pixel-innovation") != std::string::npos);

            if (is_pixel_innov) {
                cv::Mat overlay = vis_img.clone();
                for (const auto& det : detections) {
                    int x1 = static_cast<int>(det.cx - det.w * 0.5f);
                    int y1 = static_cast<int>(det.cy - det.h * 0.5f);
                    int x2 = static_cast<int>(det.cx + det.w * 0.5f);
                    int y2 = static_cast<int>(det.cy + det.h * 0.5f);

                    cv::rectangle(overlay, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << "Gate YOLO: " << det.score;
                    cv::putText(overlay, ss.str(), cv::Point(x1, std::max(15, y1 - 6)),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);

                    for (size_t i = 0; i < det.all_keypoints.size(); ++i) {
                        const auto& kp = det.all_keypoints[i];
                        if (kp.conf >= corner_conf_thresh_) {
                            cv::Scalar color = (det.all_keypoints.size() == 4 || (i >= 6 && i <= 9))
                                               ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 250, 0);
                            cv::circle(overlay, cv::Point(static_cast<int>(kp.x), static_cast<int>(kp.y)), 4, color, -1);
                        }
                    }
                }
                cv::addWeighted(overlay, 0.40, vis_img, 0.60, 0.0, vis_img);
            } else {
                for (const auto& det : detections) {
                    int x1 = static_cast<int>(det.cx - det.w * 0.5f);
                    int y1 = static_cast<int>(det.cy - det.h * 0.5f);
                    int x2 = static_cast<int>(det.cx + det.w * 0.5f);
                    int y2 = static_cast<int>(det.cy + det.h * 0.5f);

                    cv::rectangle(vis_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << "Gate: " << det.score;
                    cv::putText(vis_img, ss.str(), cv::Point(x1, std::max(15, y1 - 6)),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

                    for (size_t i = 0; i < det.all_keypoints.size(); ++i) {
                        const auto& kp = det.all_keypoints[i];
                        if (kp.conf >= corner_conf_thresh_) {
                            cv::Scalar color = (det.all_keypoints.size() == 4 || (i >= 6 && i <= 9))
                                               ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 250, 0);
                            cv::circle(vis_img, cv::Point(static_cast<int>(kp.x), static_cast<int>(kp.y)), 4, color, -1);
                        }
                    }
                }
            }
        }

        // 2. Render 3D-to-2D Projected Gate Positions from Estimator Memory
        // Format per visible gate (12 floats): [id, is_active, u_c, v_c, u_tl, v_tl, u_tr, v_tr, u_br, v_br, u_bl, v_bl]
        std::vector<double> proj_copy;
        {
            std::lock_guard<std::mutex> lock(proj_mutex_);
            proj_copy = latest_projected_pixels_;
        }

        if (proj_copy.size() >= 12 && (proj_copy.size() % 12 == 0)) {
            size_t num_mem_gates = proj_copy.size() / 12;
            for (size_t g = 0; g < num_mem_gates; ++g) {
                size_t off = g * 12;
                int gate_id = static_cast<int>(proj_copy[off + 0]);
                bool is_active = (proj_copy[off + 1] > 0.5);

                double u_tl = proj_copy[off + 4], v_tl = proj_copy[off + 5];
                double u_tr = proj_copy[off + 6], v_tr = proj_copy[off + 7];
                double u_br = proj_copy[off + 8], v_br = proj_copy[off + 9];
                double u_bl = proj_copy[off + 10], v_bl = proj_copy[off + 11];

                cv::Scalar color_box = is_active ? cv::Scalar(255, 255, 0) : cv::Scalar(0, 165, 255);
                int thickness = is_active ? 2 : 1;

                std::vector<cv::Point> pts = {
                    cv::Point(static_cast<int>(u_tl), static_cast<int>(v_tl)),
                    cv::Point(static_cast<int>(u_tr), static_cast<int>(v_tr)),
                    cv::Point(static_cast<int>(u_br), static_cast<int>(v_br)),
                    cv::Point(static_cast<int>(u_bl), static_cast<int>(v_bl))
                };

                bool all_valid = true;
                for (const auto& pt : pts) {
                    if (pt.x < -500 || pt.y < -500) {
                        all_valid = false;
                        break;
                    }
                }

                if (all_valid) {
                    const cv::Point* ppt[1] = { pts.data() };
                    int npt[] = { 4 };
                    cv::polylines(vis_img, ppt, npt, 1, true, color_box, thickness);

                    for (const auto& pt : pts) {
                        cv::circle(vis_img, pt, 3, color_box, -1);
                    }

                    std::string label = (is_active ? "TARGET G#" : "MEM G#") + std::to_string(gate_id);
                    cv::putText(vis_img, label, cv::Point(pts[0].x, std::max(15, pts[0].y - 6)),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, color_box, 1);
                }
            }
        }

        // 3. Publish Raw Image
        if (publish_raw && pub_debug_img_) {
            sensor_msgs::msg::Image out_img;
            out_img.header = header;
            out_img.height = static_cast<uint32_t>(vis_img.rows);
            out_img.width = static_cast<uint32_t>(vis_img.cols);
            out_img.encoding = sensor_msgs::image_encodings::BGR8;
            out_img.is_bigendian = 0;
            out_img.step = static_cast<sensor_msgs::msg::Image::_step_type>(vis_img.cols * vis_img.elemSize());
            out_img.data.resize(out_img.step * out_img.height);
            std::memcpy(out_img.data.data(), vis_img.data, out_img.data.size());
            pub_debug_img_->publish(out_img);
        }

        // 4. Publish Compressed Image
        if (publish_compressed && pub_debug_compressed_) {
            sensor_msgs::msg::CompressedImage comp_msg;
            comp_msg.header = header;
            comp_msg.format = "jpeg";
            std::vector<uchar> buf;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
            cv::imencode(".jpg", vis_img, buf, params);
            comp_msg.data = std::move(buf);
            pub_debug_compressed_->publish(comp_msg);
        }
    }

    // Parameters
    std::string model_path_;
    std::string method_;
    float conf_threshold_{0.50f};
    float corner_conf_thresh_{0.15f};
    float iou_threshold_{0.45f};
    bool publish_debug_{true};

    // Subscriptions & Publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_projected_pixels_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_corners_2d_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_img_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_debug_compressed_;

    // Thread-safe storage for projected pixels
    std::mutex proj_mutex_;
    std::vector<double> latest_projected_pixels_;

    // Core inference components
    AdlaEngine engine_;
    YoloPoseDecoder decoder_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GatePerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
