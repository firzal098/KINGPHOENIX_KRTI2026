#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>

#include "adla_gate_inference/adla_engine.hpp"
#include "adla_gate_inference/yolo_pose_decoder.hpp"

namespace fs = std::filesystem;

class OfflineTestRunner : public rclcpp::Node {
public:
    OfflineTestRunner()
    : Node("offline_test_runner") {
        // Resolve default paths dynamically via ament_index_cpp or local directory
        std::string pkg_share = "";
        try {
            pkg_share = ament_index_cpp::get_package_share_directory("adla_gate_inference");
        } catch (...) {
            pkg_share = "";
        }

        std::string default_model = "resource/gate_yolo_pose_int8.adla";
        if (!pkg_share.empty() && fs::exists(pkg_share + "/resource/gate_yolo_pose_int8.adla")) {
            default_model = pkg_share + "/resource/gate_yolo_pose_int8.adla";
        } else if (fs::exists("resource/gate_yolo_pose_int8.adla")) {
            default_model = "resource/gate_yolo_pose_int8.adla";
        }

        std::string default_input = "tests/test_images";
        if (!pkg_share.empty() && fs::exists(pkg_share + "/tests/test_images")) {
            default_input = pkg_share + "/tests/test_images";
        } else if (fs::exists("tests/test_images")) {
            default_input = "tests/test_images";
        }

        std::string default_output = "./output_images";
        if (!pkg_share.empty()) {
            default_output = "./output_images";
        }

        this->declare_parameter<std::string>("model_path", default_model);
        this->declare_parameter<std::string>("input_dir", default_input);
        this->declare_parameter<std::string>("output_dir", default_output);
        this->declare_parameter<double>("conf_threshold", 0.50);
        this->declare_parameter<double>("corner_conf_threshold", 0.15);
        this->declare_parameter<double>("iou_threshold", 0.45);

        model_path_ = this->get_parameter("model_path").as_string();
        input_dir_ = this->get_parameter("input_dir").as_string();
        output_dir_ = this->get_parameter("output_dir").as_string();
        conf_threshold_ = static_cast<float>(this->get_parameter("conf_threshold").as_double());
        corner_conf_thresh_ = static_cast<float>(this->get_parameter("corner_conf_threshold").as_double());
        iou_threshold_ = static_cast<float>(this->get_parameter("iou_threshold").as_double());
    }

    bool run() {
        std::cout << "\n============================================================\n";
        std::cout << "         ADLA Gate Perception - Offline Test Runner         \n";
        std::cout << "============================================================\n";
        std::cout << "Model Path:       " << model_path_ << "\n";
        std::cout << "Input Directory:  " << input_dir_ << "\n";
        std::cout << "Output Directory: " << output_dir_ << "\n";
        std::cout << "Conf Threshold:   " << conf_threshold_ << "\n";
        std::cout << "Corner Threshold: " << corner_conf_thresh_ << "\n";
        std::cout << "IoU Threshold:    " << iou_threshold_ << "\n";
        std::cout << "------------------------------------------------------------\n";

        // Check and create output directory
        if (!fs::exists(output_dir_)) {
            fs::create_directories(output_dir_);
        }

        if (!fs::exists(input_dir_)) {
            std::cerr << "ERROR: Input directory does not exist: " << input_dir_ << std::endl;
            return false;
        }

        // Initialize Engine
        AdlaEngine engine;
        if (!engine.init(model_path_, 640, 640, 3)) {
            std::cerr << "ERROR: Failed to initialize ADLA engine with model: " << model_path_ << std::endl;
            return false;
        }

        YoloPoseDecoder decoder;
        int processed_count = 0;
        int total_gates_detected = 0;

        for (const auto& entry : fs::directory_iterator(input_dir_)) {
            if (!entry.is_regular_file()) continue;

            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext != ".jpg" && ext != ".jpeg" && ext != ".png" && ext != ".bmp") {
                continue;
            }

            std::string filename = entry.path().filename().string();
            cv::Mat frame = cv::imread(entry.path().string());
            if (frame.empty()) {
                std::cerr << "Warning: Could not read image: " << filename << std::endl;
                continue;
            }

            float orig_w = static_cast<float>(frame.cols);
            float orig_h = static_cast<float>(frame.rows);

            // Preprocess: Resize to 640x640 RGB
            cv::Mat resized_bgr;
            cv::resize(frame, resized_bgr, cv::Size(640, 640));
            cv::Mat resized_rgb;
            cv::cvtColor(resized_bgr, resized_rgb, cv::COLOR_BGR2RGB);
            if (!resized_rgb.isContinuous()) {
                resized_rgb = resized_rgb.clone();
            }

            // Inference
            size_t num_output_floats = 0;
            const float* raw_outputs = engine.run_inference_rgb(
                resized_rgb.data,
                640 * 640 * 3,
                num_output_floats
            );

            std::vector<GateDetection> detections;
            if (raw_outputs && num_output_floats > 0) {
                detections = decoder.decode(
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
            }

            std::cout << "[Image: " << filename << "] Detected " << detections.size() << " gate(s)\n";

            // Draw detections on original image
            cv::Mat vis = frame.clone();
            for (size_t i = 0; i < detections.size(); ++i) {
                const auto& det = detections[i];
                total_gates_detected++;

                int x1 = static_cast<int>(det.cx - det.w * 0.5f);
                int y1 = static_cast<int>(det.cy - det.h * 0.5f);
                int x2 = static_cast<int>(det.cx + det.w * 0.5f);
                int y2 = static_cast<int>(det.cy + det.h * 0.5f);

                // Draw bounding box (Green)
                cv::rectangle(vis, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

                std::stringstream ss;
                ss << "Gate #" << (i + 1) << " (" << std::fixed << std::setprecision(2) << det.score << ")";
                cv::putText(vis, ss.str(), cv::Point(x1, std::max(18, y1 - 8)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

                // Corner coordinates
                const char* corner_names[4] = {"TL", "TR", "BR", "BL"};
                std::vector<cv::Point> poly_pts;

                for (int c = 0; c < 4; ++c) {
                    int kx = static_cast<int>(det.corners[c].x);
                    int ky = static_cast<int>(det.corners[c].y);
                    poly_pts.emplace_back(kx, ky);

                    // Draw corner point (Red dot)
                    cv::circle(vis, cv::Point(kx, ky), 6, cv::Scalar(0, 0, 255), -1);

                    // Corner label
                    std::string klabel = corner_names[c];
                    cv::putText(vis, klabel, cv::Point(kx + 8, ky - 4),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);

                    std::cout << "   " << corner_names[c] << ": (" << kx << ", " << ky
                              << ") conf=" << std::fixed << std::setprecision(2) << det.corners[c].conf << "\n";
                }

                // Draw wireframe connecting 4 corners (Yellow polygon)
                if (poly_pts.size() == 4) {
                    const cv::Point* ppt[1] = { poly_pts.data() };
                    int npt[] = { 4 };
                    cv::polylines(vis, ppt, npt, 1, true, cv::Scalar(0, 255, 255), 2);
                }
            }

            std::string out_path = (fs::path(output_dir_) / filename).string();
            cv::imwrite(out_path, vis);
            std::cout << "   Saved result -> " << out_path << "\n\n";
            processed_count++;
        }

        std::cout << "============================================================\n";
        std::cout << "Verification Complete!\n";
        std::cout << "Processed Images:       " << processed_count << "\n";
        std::cout << "Total Gates Highlighted: " << total_gates_detected << "\n";
        std::cout << "Output Folder:          " << output_dir_ << "\n";
        std::cout << "============================================================\n";
        return true;
    }

private:
    std::string model_path_;
    std::string input_dir_;
    std::string output_dir_;
    float conf_threshold_{0.50f};
    float corner_conf_thresh_{0.15f};
    float iou_threshold_{0.45f};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto runner = std::make_shared<OfflineTestRunner>();
    runner->run();
    rclcpp::shutdown();
    return 0;
}
