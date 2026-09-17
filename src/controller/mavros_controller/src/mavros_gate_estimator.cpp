#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <cmath>
#include <limits>
#include <chrono>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

struct StampedPose {
    rclcpp::Time stamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

struct GatePriorRDF {
    int id;
    double x_rdf;
    double y_rdf;
    double z_rdf;
    double nx_rdf;
    double ny_rdf;
    double nz_rdf;
};

struct GateState {
    int id;
    Eigen::Vector3d position_enu;   // 3D position in local ENU frame
    Eigen::Vector3d smoothed_pos_enu; // Continuously smoothed 30Hz position stream for controller
    Eigen::Vector3d prior_pos_enu;  // Initial surveyed prior position in local ENU
    Eigen::Matrix3d covariance;     // 3x3 error covariance matrix
    Eigen::Vector3d normal_enu;     // Gate normal vector in ENU frame
    Eigen::Quaterniond orientation; // Orientation quaternion in ENU
    rclcpp::Time last_update_time{0, 0, RCL_ROS_TIME};
};

class MavrosGateEstimator : public rclcpp::Node {
public:
    MavrosGateEstimator() 
    : Node("mavros_gate_estimator"), 
      drone_pose_received_(false), 
      initial_pose_captured_(false),
      camera_info_received_(false),
      active_target_gate_idx_(0)
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("method", "pnp"); // "pnp" or "pixel_innovation"
        this->declare_parameter<double>("pixel_noise_sigma", 4.0);
        this->declare_parameter<double>("gate_prior_sigma", 1.5);
        this->declare_parameter<double>("drone_pose_sigma", 2.0);
        this->declare_parameter<double>("pnp_vision_sigma", 4.0);
        this->declare_parameter<double>("association_max_dist", 6.0);
        this->declare_parameter<double>("mahalanobis_thresh_sq", 11.345);
        this->declare_parameter<double>("max_prior_deviation_m", 12.0);
        this->declare_parameter<double>("max_refine_tilt_deg", 20.0);
        this->declare_parameter<double>("process_noise_q", 0.5);
        this->declare_parameter<double>("max_refine_distance_m", 36.0);
        this->declare_parameter<double>("camera_pitch_deg", 0.0);
        this->declare_parameter<double>("gate_width_m", 1.78);
        this->declare_parameter<double>("gate_height_m", 1.88);
        this->declare_parameter<bool>("enable_monte_carlo_cov", true);
        this->declare_parameter<int>("mc_samples", 20);
        this->declare_parameter<double>("corner_noise_sigma", 2.0);
        this->declare_parameter<bool>("publish_initial_pos", true);
        this->declare_parameter<bool>("blend_gate5_with_mean_1_2", true);
        this->declare_parameter<bool>("tunnel_blend_gate1_and_2", true);
        this->declare_parameter<bool>("use_1d_right_axis_offset", false);
        this->declare_parameter<bool>("enable_gate3_pnp_refinement", true);
        this->declare_parameter<bool>("v7", false);

        // Safe parameter reading
        method_                     = this->get_parameter("method").as_string();
        pixel_noise_sigma_          = get_param_as_double("pixel_noise_sigma", 4.0);
        prior_sigma_                = get_param_as_double("gate_prior_sigma", 1.5);
        drone_sigma_                = get_param_as_double("drone_pose_sigma", 2.0);
        pnp_sigma_                  = get_param_as_double("pnp_vision_sigma", 4.0);
        process_noise_q_            = get_param_as_double("process_noise_q", 0.5);
        max_dist_                   = get_param_as_double("association_max_dist", 6.0);
        mahalanobis_max_sq_         = get_param_as_double("mahalanobis_thresh_sq", 11.345);
        max_prior_deviation_        = get_param_as_double("max_prior_deviation_m", 12.0);
        max_tilt_rad_               = get_param_as_double("max_refine_tilt_deg", 20.0) * (M_PI / 180.0);
        max_refine_dist_            = get_param_as_double("max_refine_distance_m", 36.0);
        camera_pitch_deg_           = get_param_as_double("camera_pitch_deg", 0.0);
        gate_w_                     = get_param_as_double("gate_width_m", 1.78);
        gate_h_                     = get_param_as_double("gate_height_m", 1.88);
        enable_mc_cov_              = this->get_parameter("enable_monte_carlo_cov").as_bool();
        mc_samples_                 = static_cast<int>(this->get_parameter("mc_samples").as_int());
        corner_noise_sigma_         = get_param_as_double("corner_noise_sigma", 2.0);
        publish_initial_pos_        = this->get_parameter("publish_initial_pos").as_bool();
        blend_gate5_with_mean_1_2_  = this->get_parameter("blend_gate5_with_mean_1_2").as_bool();
        tunnel_blend_gate1_and_2_   = this->get_parameter("tunnel_blend_gate1_and_2").as_bool();
        use_1d_right_axis_offset_   = this->get_parameter("use_1d_right_axis_offset").as_bool();
        enable_gate3_pnp_refinement_= this->get_parameter("enable_gate3_pnp_refinement").as_bool();
        v7_                         = this->get_parameter("v7").as_bool();

        // 3D object model corner points (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
        float hw = static_cast<float>(gate_w_ / 2.0);
        float hh = static_cast<float>(gate_h_ / 2.0);
        object_points_4p_ = {
            cv::Point3f(-hw, -hh, 0.0f), // Corner 6 (Top-Left)
            cv::Point3f( hw, -hh, 0.0f), // Corner 7 (Top-Right)
            cv::Point3f( hw,  hh, 0.0f), // Corner 8 (Bottom-Right)
            cv::Point3f(-hw,  hh, 0.0f)  // Corner 9 (Bottom-Left)
        };

        // Initialize zero offset defaults until first MAVROS pose is received
        initial_drone_pos_ = Eigen::Vector3d::Zero();
        initial_drone_rot_ = Eigen::Quaterniond::Identity();

        // Initialize default gate priors at origin
        initialize_gate_priors();

        // Define Camera Optical (RDF) to Drone Body (FLU) rotation matrix with pitch tilt
        // RDF X (Right) -> Body -Y
        // RDF Y (Down)  -> Body [sin(pitch), 0, -cos(pitch)]
        // RDF Z (Fwd)   -> Body [cos(pitch), 0,  sin(pitch)]
        double pitch_rad = camera_pitch_deg_ * (M_PI / 180.0);
        R_cam_to_body_ <<  0.0,  std::sin(pitch_rad),  std::cos(pitch_rad),
                          -1.0,                  0.0,                  0.0,
                           0.0, -std::cos(pitch_rad),  std::sin(pitch_rad);

        RCLCPP_INFO(
            this->get_logger(),
            "Camera mounting pitch: %.1f deg (RDF to FLU rotation configured). Estimation Method: '%s'",
            camera_pitch_deg_, method_.c_str()
        );

        auto sensor_qos = rclcpp::SensorDataQoS();

        // Subscribers
        sub_drone_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", sensor_qos,
            std::bind(&MavrosGateEstimator::drone_pose_callback, this, std::placeholders::_1)
        );

        sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10,
            std::bind(&MavrosGateEstimator::camera_info_callback, this, std::placeholders::_1)
        );

        // 2D Gate Keypoint Corners Subscriber from cuda_gate_inference
        sub_gate_corners_2d_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/perception/gate_corners_2d", sensor_qos,
            std::bind(&MavrosGateEstimator::gate_corners_2d_callback, this, std::placeholders::_1)
        );

        // Direct 3D PnP Pose Subscriber (backward compatibility)
        sub_pnp_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/perception/gate_poses_3d", sensor_qos,
            std::bind(&MavrosGateEstimator::pnp_poses_callback, this, std::placeholders::_1)
        );

        sub_pnp_covariances_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/perception/gate_covariances_3d", sensor_qos,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg) {
                    latest_pnp_covariances_ = *msg;
                    has_pnp_covariances_ = true;
                }
            }
        );

        // Subscribe to active target gate index from controller (ensures only active gate is refined)
        sub_target_gate_ = this->create_subscription<std_msgs::msg::Int32>(
            "/controller/target_gate_index", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg && msg->data >= 0) {
                    active_target_gate_idx_ = msg->data;
                }
            }
        );

        // Publishers for Refined Estimates
        pub_refined_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/estimator/refined_gate_poses", 10
        );

        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/estimator/gate_markers", 10
        );

        // Publisher for 3D-to-2D Projected Gate Memory Pixels (for FPV debug overlay)
        pub_projected_pixels_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/estimator/projected_gate_pixels", 10
        );

        // Publishers for Unrefined Initial Priors
        if (publish_initial_pos_) {
            pub_initial_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
                "/estimator/initial_gate_poses", 10
            );
            pub_initial_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/estimator/initial_gate_markers", 10
            );
        }

        // Service Server: Reset Gate Positions
        srv_reset_gates_ = this->create_service<std_srvs::srv::Trigger>(
            "/estimator/reset_gates",
            std::bind(&MavrosGateEstimator::handle_reset_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Landing Pad Publisher & Services
        pub_landing_pad_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/estimator/landing_pad", 10
        );
        pub_landing_pad_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/estimator/landing_pad_marker", 10
        );
        pub_manual_pad_active_ = this->create_publisher<std_msgs::msg::Bool>(
            "/estimator/manual_landing_pad_active", 10
        );
        srv_tag_landing_pad_ = this->create_service<std_srvs::srv::Trigger>(
            "/estimator/tag_landing_pad",
            std::bind(&MavrosGateEstimator::handle_tag_landing_pad, this, std::placeholders::_1, std::placeholders::_2)
        );
        srv_reset_landing_pad_ = this->create_service<std_srvs::srv::Trigger>(
            "/estimator/reset_landing_pad",
            std::bind(&MavrosGateEstimator::handle_reset_landing_pad, this, std::placeholders::_1, std::placeholders::_2)
        );

        // 30 Hz wall timer (33,333 microseconds)
        pub_timer_ = this->create_wall_timer(
            std::chrono::microseconds(33333),
            std::bind(&MavrosGateEstimator::timer_callback, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "MAVROS Gate Estimator Node Initialized (Method: %s, 30 Hz streaming).",
            method_.c_str()
        );
    }

private:
    double get_param_as_double(const std::string &name, double default_val) {
        if (!this->has_parameter(name)) return default_val;
        auto param = this->get_parameter(name);
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            return static_cast<double>(param.as_int());
        } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            return param.as_double();
        }
        return default_val;
    }

    void timer_callback() {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "map";

        publish_refined_poses(header);
        publish_rviz_markers(header);

        if (publish_initial_pos_) {
            publish_initial_poses(header);
        }

        // Compute and publish 3D-to-2D projected gate memory pixels
        compute_and_publish_projected_pixels();

        // Update and publish landing pad pose & marker
        update_and_publish_landing_pad(header);
    }

    void initialize_gate_priors() {
        gates_.clear();
        initial_poses_msg_.poses.clear();
        initial_markers_msg_.markers.clear();

        std::vector<GatePriorRDF> priors_rdf = {
            {1,  0.41, -0.85, 29.33,  0.0, 0.0, 1.0},
            {2,  5.46, -0.85, 19.31,  0.0, 0.0, 1.0},
            {3,  9.49, -0.85, 10.51,  1.0, 0.0, 0.0},
            {4, 12.41, -0.85, 12.43,  0.0, 0.0, 1.0},
            {5, 17.47, -0.85, 29.38,  0.0, 0.0, 1.0}
        };

        const double initial_var = prior_sigma_ * prior_sigma_;

        for (const auto &p : priors_rdf) {
            GateState state;
            state.id = p.id;

            Eigen::Vector3d rel_pos_enu(p.z_rdf, -p.x_rdf, -p.y_rdf);
            Eigen::Vector3d rel_norm_enu(p.nz_rdf, -p.nx_rdf, -p.ny_rdf);

            state.position_enu  = initial_drone_pos_ + (initial_drone_rot_ * rel_pos_enu);
            state.smoothed_pos_enu = state.position_enu;
            state.prior_pos_enu = state.position_enu;
            state.normal_enu    = (initial_drone_rot_ * rel_norm_enu).normalized();
            state.normal_enu.z() = 0.0;
            state.normal_enu.normalize();
            state.covariance    = Eigen::Matrix3d::Identity() * initial_var;

            Eigen::Vector3d default_facing(1.0, 0.0, 0.0);
            state.orientation = Eigen::Quaterniond::FromTwoVectors(default_facing, state.normal_enu);

            gates_.push_back(state);

            geometry_msgs::msg::Pose initial_pose;
            initial_pose.position.x = state.position_enu.x();
            initial_pose.position.y = state.position_enu.y();
            initial_pose.position.z = state.position_enu.z();
            initial_pose.orientation.x = state.orientation.x();
            initial_pose.orientation.y = state.orientation.y();
            initial_pose.orientation.z = state.orientation.z();
            initial_pose.orientation.w = state.orientation.w();
            initial_poses_msg_.poses.push_back(initial_pose);

            visualization_msgs::msg::Marker init_box;
            init_box.ns = "initial_gate_boxes";
            init_box.id = state.id;
            init_box.type = visualization_msgs::msg::Marker::CUBE;
            init_box.action = visualization_msgs::msg::Marker::ADD;
            init_box.pose = initial_pose;
            init_box.scale.x = 0.08;
            init_box.scale.y = 1.9;
            init_box.scale.z = 2.0;
            init_box.color.r = 1.0f;
            init_box.color.g = 0.5f;
            init_box.color.b = 0.0f;
            init_box.color.a = 0.4f;
            initial_markers_msg_.markers.push_back(init_box);

            visualization_msgs::msg::Marker init_text;
            init_text.ns = "initial_gate_labels";
            init_text.id = state.id + 200;
            init_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            init_text.action = visualization_msgs::msg::Marker::ADD;
            init_text.pose.position.x = state.position_enu.x();
            init_text.pose.position.y = state.position_enu.y();
            init_text.pose.position.z = state.position_enu.z() + 1.8;
            init_text.scale.z = 0.4;
            init_text.color.r = 1.0f;
            init_text.color.g = 0.6f;
            init_text.color.b = 0.0f;
            init_text.color.a = 0.8f;
            init_text.text = "Gate " + std::to_string(state.id) + " (Initial)";
            initial_markers_msg_.markers.push_back(init_text);

            // Add Initial Sub-Gates for Gate 3 and Gate 4
            std::vector<double> sub_offsets;
            if (v7_) {
                if (state.id == 3) {
                    sub_offsets = {1.0, 2.5};
                } else if (state.id == 4) {
                    sub_offsets = {1.0, 2.0, 3.0};
                }
            } else {
                if (state.id == 3) {
                    sub_offsets = {1.0};
                } else if (state.id == 4) {
                    sub_offsets = {1.0, 2.0};
                }
            }

            for (size_t sub_k = 0; sub_k < sub_offsets.size(); ++sub_k) {
                double s_off = sub_offsets[sub_k];
                Eigen::Vector3d sub_pos = state.position_enu + s_off * state.normal_enu;
                if (state.id == 3 && sub_k == 1) { // Gate 3.2
                    Eigen::Vector3d right_enu(state.normal_enu.y(), -state.normal_enu.x(), 0.0);
                    sub_pos += 0.2 * right_enu;
                }
                int sub_id_num = state.id * 10 + (sub_k + 1);

                visualization_msgs::msg::Marker init_sub_box = init_box;
                init_sub_box.id = sub_id_num;
                init_sub_box.pose.position.x = sub_pos.x();
                init_sub_box.pose.position.y = sub_pos.y();
                init_sub_box.pose.position.z = sub_pos.z();
                init_sub_box.color.a = 0.25f;
                initial_markers_msg_.markers.push_back(init_sub_box);

                visualization_msgs::msg::Marker init_sub_text = init_text;
                init_sub_text.id = sub_id_num + 200;
                init_sub_text.pose.position.x = sub_pos.x();
                init_sub_text.pose.position.y = sub_pos.y();
                init_sub_text.pose.position.z = sub_pos.z() + 1.8;
                init_sub_text.text = "Gate " + std::to_string(state.id) + "." + std::to_string(sub_k + 1) + " (Sub)";
                initial_markers_msg_.markers.push_back(init_sub_text);
            }
        }

        // Initialize Landing Pad Prior: RDF = [1.74, 0.0, 54.48] m
        // rel_pos_enu = (z_rdf, -x_rdf, -y_rdf) = (54.48, -1.74, 0.0)
        Eigen::Vector3d pad_rel_enu(54.48, -1.74, 0.0);
        prior_landing_pad_enu_ = initial_drone_pos_ + (initial_drone_rot_ * pad_rel_enu);
        if (!has_manual_landing_pad_) {
            refined_landing_pad_enu_ = prior_landing_pad_enu_;
            smoothed_landing_pad_enu_ = prior_landing_pad_enu_;
        }
    }

    bool handle_reset_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        active_target_gate_idx_ = 0;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (drone_pose_received_) {
                initial_drone_pos_ = latest_drone_pos_;
                initial_drone_rot_ = latest_drone_rot_;
            }
        }

        initialize_gate_priors();

        response->success = true;
        response->message = "Gate positions and covariances successfully reset and re-anchored.";
        RCLCPP_INFO(
            this->get_logger(),
            "All gate states successfully reset and re-anchored to current pose [%.2f, %.2f, %.2f].",
            initial_drone_pos_.x(), initial_drone_pos_.y(), initial_drone_pos_.z()
        );
        return true;
    }

    bool handle_tag_landing_pad(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!drone_pose_received_) {
            response->success = false;
            response->message = "Cannot tag landing pad: No drone pose received yet.";
            return true;
        }

        manual_landing_pad_enu_ = latest_drone_pos_;
        manual_landing_pad_enu_.z() = 0.0; // Landing pad sits on ground plane
        has_manual_landing_pad_ = true;
        refined_landing_pad_enu_ = manual_landing_pad_enu_;
        smoothed_landing_pad_enu_ = manual_landing_pad_enu_;

        response->success = true;
        response->message = "Landing pad manually tagged at: [" +
                            std::to_string(manual_landing_pad_enu_.x()) + ", " +
                            std::to_string(manual_landing_pad_enu_.y()) + ", " +
                            std::to_string(manual_landing_pad_enu_.z()) + "]";
        RCLCPP_INFO(this->get_logger(), "MANUAL LANDING PAD SET: %s", response->message.c_str());
        return true;
    }

    bool handle_reset_landing_pad(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        has_manual_landing_pad_ = false;
        response->success = true;
        response->message = "Manual landing pad cleared; reverted to auto-refined prior.";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        return true;
    }

    void update_and_publish_landing_pad(const std_msgs::msg::Header &header) {
        // Calculate mean translation correction across all gates
        Eigen::Vector3d sum_gate_offset = Eigen::Vector3d::Zero();
        int count = 0;
        for (const auto &g : gates_) {
            sum_gate_offset += (g.position_enu - g.prior_pos_enu);
            count++;
        }
        Eigen::Vector3d mean_offset = Eigen::Vector3d::Zero();
        if (count > 0) {
            mean_offset = sum_gate_offset / static_cast<double>(count);
        }

        if (has_manual_landing_pad_) {
            refined_landing_pad_enu_ = manual_landing_pad_enu_;
        } else {
            refined_landing_pad_enu_ = prior_landing_pad_enu_ + mean_offset;
        }

        const double alpha = 0.35;
        smoothed_landing_pad_enu_ = (1.0 - alpha) * smoothed_landing_pad_enu_ + alpha * refined_landing_pad_enu_;

        // 1. Publish PoseStamped
        geometry_msgs::msg::PoseStamped pad_msg;
        pad_msg.header = header;
        pad_msg.pose.position.x = smoothed_landing_pad_enu_.x();
        pad_msg.pose.position.y = smoothed_landing_pad_enu_.y();
        pad_msg.pose.position.z = smoothed_landing_pad_enu_.z();
        pad_msg.pose.orientation.w = 1.0;
        pub_landing_pad_->publish(pad_msg);

        // 2. Publish manual status flag
        std_msgs::msg::Bool active_msg;
        active_msg.data = has_manual_landing_pad_;
        pub_manual_pad_active_->publish(active_msg);

        // 3. Publish RViz Marker
        visualization_msgs::msg::Marker pad_marker;
        pad_marker.header = header;
        pad_marker.ns = "landing_pad";
        pad_marker.id = 999;
        pad_marker.type = visualization_msgs::msg::Marker::CUBE;
        pad_marker.action = visualization_msgs::msg::Marker::ADD;
        pad_marker.pose = pad_msg.pose;
        pad_marker.scale.x = 2.4;
        pad_marker.scale.y = 2.4;
        pad_marker.scale.z = 0.06;
        pad_marker.color.r = has_manual_landing_pad_ ? 1.0f : 0.0f;
        pad_marker.color.g = has_manual_landing_pad_ ? 0.8f : 0.9f;
        pad_marker.color.b = has_manual_landing_pad_ ? 0.1f : 0.2f;
        pad_marker.color.a = 0.85f;
        pub_landing_pad_marker_->publish(pad_marker);
    }

    void drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_drone_pos_ = Eigen::Vector3d(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        latest_drone_rot_ = Eigen::Quaterniond(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
        );

        rclcpp::Time msg_stamp(msg->header.stamp);
        pose_history_.push_back({msg_stamp, latest_drone_pos_, latest_drone_rot_});

        while (!pose_history_.empty() && (msg_stamp - pose_history_.front().stamp).seconds() > 3.0) {
            pose_history_.pop_front();
        }

        drone_pose_received_ = true;

        if (!initial_pose_captured_) {
            initial_drone_pos_ = latest_drone_pos_;
            initial_drone_rot_ = latest_drone_rot_;
            initial_pose_captured_ = true;

            initialize_gate_priors();

            RCLCPP_INFO(
                this->get_logger(),
                "Captured initial MAVROS pose offset: [%.2f, %.2f, %.2f]. Gate priors anchored.",
                initial_drone_pos_.x(), initial_drone_pos_.y(), initial_drone_pos_.z()
            );
        }
    }

    bool get_interpolated_drone_pose(const rclcpp::Time &query_stamp, Eigen::Vector3d &pos_out, Eigen::Quaterniond &rot_out) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (pose_history_.empty()) {
            pos_out = latest_drone_pos_;
            rot_out = latest_drone_rot_;
            return false;
        }

        if (query_stamp <= pose_history_.front().stamp) {
            pos_out = pose_history_.front().position;
            rot_out = pose_history_.front().orientation;
            return true;
        }

        if (query_stamp >= pose_history_.back().stamp) {
            pos_out = pose_history_.back().position;
            rot_out = pose_history_.back().orientation;
            return true;
        }

        for (size_t i = 0; i + 1 < pose_history_.size(); ++i) {
            const auto &p0 = pose_history_[i];
            const auto &p1 = pose_history_[i + 1];

            if (p0.stamp <= query_stamp && query_stamp <= p1.stamp) {
                double dt_seg = (p1.stamp - p0.stamp).seconds();
                if (dt_seg < 1e-6) {
                    pos_out = p1.position;
                    rot_out = p1.orientation;
                    return true;
                }
                double alpha = (query_stamp - p0.stamp).seconds() / dt_seg;
                alpha = std::clamp(alpha, 0.0, 1.0);

                pos_out = (1.0 - alpha) * p0.position + alpha * p1.position;
                rot_out = p0.orientation.slerp(alpha, p1.orientation);
                return true;
            }
        }

        pos_out = latest_drone_pos_;
        rot_out = latest_drone_rot_;
        return true;
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!camera_info_received_) {
            K_ = (cv::Mat_<double>(3, 3) <<
                msg->k[0], msg->k[1], msg->k[2],
                msg->k[3], msg->k[4], msg->k[5],
                msg->k[6], msg->k[7], msg->k[8]);
            
            fx_ = msg->k[0];
            fy_ = msg->k[4];
            cx_ = msg->k[2];
            cy_ = msg->k[5];
            img_w_ = msg->width > 0 ? static_cast<double>(msg->width) : (cx_ * 2.0);
            img_h_ = msg->height > 0 ? static_cast<double>(msg->height) : (cy_ * 2.0);

            if (!msg->d.empty()) {
                D_ = cv::Mat(msg->d).clone();
            } else {
                D_ = cv::Mat::zeros(5, 1, CV_64F);
            }
            camera_info_received_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Camera intrinsics registered in Estimator: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f (Image: %.0fx%.0f)",
                fx_, fy_, cx_, cy_, img_w_, img_h_
            );
        }
    }

    /**
     * @brief Solves the exact 2D pixel coordinates of all gates currently stored in estimator memory.
     * Accounts for full drone 6-DOF pose and camera tilt pitch (10 deg up causes gates to appear lower).
     * Publishes 12 floats per visible gate on /estimator/projected_gate_pixels:
     * [id, is_active, u_center, v_center, u_tl, v_tl, u_tr, v_tr, u_br, v_br, u_bl, v_bl]
     */
    void compute_and_publish_projected_pixels() {
        if (!drone_pose_received_ || !camera_info_received_) {
            return;
        }

        Eigen::Vector3d drone_pos;
        Eigen::Quaterniond drone_rot;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            drone_pos = latest_drone_pos_;
            drone_rot = latest_drone_rot_;
        }

        const Eigen::Matrix3d R_drone = drone_rot.toRotationMatrix();
        const Eigen::Matrix3d R_total = R_drone * R_cam_to_body_; // RDF to World ENU
        const Eigen::Matrix3d R_world_to_cam = R_total.transpose(); // World ENU to RDF

        const double hw = gate_w_ / 2.0;
        const double hh = gate_h_ / 2.0;
        const Eigen::Vector3d up_enu(0.0, 0.0, 1.0);

        std::vector<double> projected_data;

        for (size_t idx = 0; idx < gates_.size(); ++idx) {
            const auto &gate = gates_[idx];
            double is_active = (static_cast<int>(idx) == active_target_gate_idx_) ? 1.0 : 0.0;

            // Gate coordinate axes in World ENU
            Eigen::Vector3d r_gate = (gate.normal_enu.cross(up_enu)).normalized();
            if (r_gate.norm() < 0.1) {
                r_gate = Eigen::Vector3d(0.0, 1.0, 0.0);
            }
            Eigen::Vector3d u_gate = up_enu;

            // 3D points: Center, TL (6), TR (7), BR (8), BL (9)
            std::vector<Eigen::Vector3d> pts_3d = {
                gate.smoothed_pos_enu,
                gate.smoothed_pos_enu - hw * r_gate + hh * u_gate, // TL
                gate.smoothed_pos_enu + hw * r_gate + hh * u_gate, // TR
                gate.smoothed_pos_enu + hw * r_gate - hh * u_gate, // BR
                gate.smoothed_pos_enu - hw * r_gate - hh * u_gate  // BL
            };

            // Transform Center to Camera RDF
            Eigen::Vector3d p_cam_center = R_world_to_cam * (pts_3d[0] - drone_pos);
            
            // Check if gate center is in front of the camera (Z > 0.5m)
            if (p_cam_center.z() <= 0.5) {
                continue;
            }

            // Project all 5 points to 2D pixels (u, v)
            std::vector<double> px_coords;
            bool any_in_fov = false;

            for (const auto &p_world : pts_3d) {
                Eigen::Vector3d p_cam = R_world_to_cam * (p_world - drone_pos);
                if (p_cam.z() <= 0.2) {
                    px_coords.push_back(-999.0);
                    px_coords.push_back(-999.0);
                    continue;
                }

                double u = fx_ * (p_cam.x() / p_cam.z()) + cx_;
                double v = fy_ * (p_cam.y() / p_cam.z()) + cy_;

                px_coords.push_back(u);
                px_coords.push_back(v);

                // Margin check around image frame
                if (u >= -150.0 && u <= (img_w_ + 150.0) && v >= -150.0 && v <= (img_h_ + 150.0)) {
                    any_in_fov = true;
                }
            }

            if (any_in_fov && px_coords.size() == 10) {
                // Pack 12 floats per visible gate
                projected_data.push_back(static_cast<double>(gate.id));
                projected_data.push_back(is_active);
                projected_data.insert(projected_data.end(), px_coords.begin(), px_coords.end());
            }
        }

        std_msgs::msg::Float64MultiArray proj_msg;
        proj_msg.data = projected_data;
        pub_projected_pixels_->publish(proj_msg);
    }

    void gate_corners_2d_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (!drone_pose_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /mavros/local_position/pose before processing 2D gate measurements..."
            );
            return;
        }

        if (!camera_info_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /camera/camera_info before processing vision..."
            );
            return;
        }

        if (msg->data.empty()) {
            return;
        }

        Eigen::Vector3d drone_pos = latest_drone_pos_;
        Eigen::Quaterniond drone_rot = latest_drone_rot_;
        size_t off_start = 0;

        if (msg->data.size() >= 2 && (msg->data.size() - 2) % 13 == 0) {
            int32_t sec = static_cast<int32_t>(msg->data[0]);
            uint32_t nanosec = static_cast<uint32_t>(msg->data[1]);
            rclcpp::Time img_stamp(sec, nanosec, RCL_ROS_TIME);
            get_interpolated_drone_pose(img_stamp, drone_pos, drone_rot);
            off_start = 2;
        } else if (msg->data.size() % 13 == 0) {
            drone_pos = latest_drone_pos_;
            drone_rot = latest_drone_rot_;
            off_start = 0;
        } else {
            return;
        }

        size_t num_detections = (msg->data.size() - off_start) / 13;
        if (num_detections == 0) {
            return;
        }

        // 1. Attitude Gating: Skip vision updates when drone is in high pitch/roll maneuvers to prevent projection errors
        double qw = drone_rot.w();
        double qx = drone_rot.x();
        double qy = drone_rot.y();
        double qz = drone_rot.z();

        double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
        double sin_pitch = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
        double pitch = std::asin(sin_pitch);

        if (std::abs(roll) > max_tilt_rad_ || std::abs(pitch) > max_tilt_rad_) {
            return;
        }

        // 2. Target Gate Isolation: Only active target gate is refined
        int target_idx = active_target_gate_idx_;
        if (target_idx < 0 || target_idx >= static_cast<int>(gates_.size())) {
            return;
        }

        // Special Rule: Skip direct refinement for Gate 4 (idx 3); inherits offset from tunnel entrance
        if (target_idx == 3) {
            return;
        }

        const Eigen::Matrix3d R_drone = drone_rot.toRotationMatrix();
        const Eigen::Matrix3d R_total = R_drone * R_cam_to_body_; // RDF to ENU
        const Eigen::Matrix3d R_world_to_cam = R_total.transpose(); // ENU to RDF

        // =========================================================================
        // METHOD A1: Pixel Innovation EKF (4 Corners Averaged to 1 Center Point)
        // =========================================================================
        if (method_ == "pixel_innovation" || method_ == "pixel-innovation") {
            Eigen::Vector3d p_cam_pred = R_world_to_cam * (gates_[target_idx].position_enu - drone_pos);
            if (p_cam_pred.z() <= 0.5) {
                return;
            }

            double z_inv = 1.0 / p_cam_pred.z();
            double z_inv2 = z_inv * z_inv;
            double u_pred = fx_ * p_cam_pred.x() * z_inv + cx_;
            double v_pred = fy_ * p_cam_pred.y() * z_inv + cy_;

            // Find best matching detection based on 2D pixel distance to predicted center
            int best_det_idx = -1;
            double min_pix_dist = std::numeric_limits<double>::max();
            Eigen::Vector2d best_z_center(0.0, 0.0);

            for (size_t i = 0; i < num_detections; ++i) {
                size_t off = off_start + i * 13;
                double u0 = msg->data[off + 1], v0 = msg->data[off + 2];
                double u1 = msg->data[off + 4], v1 = msg->data[off + 5];
                double u2 = msg->data[off + 7], v2 = msg->data[off + 8];
                double u3 = msg->data[off + 10], v3 = msg->data[off + 11];

                double u_c = 0.25 * (u0 + u1 + u2 + u3);
                double v_c = 0.25 * (v0 + v1 + v2 + v3);

                double dist = std::hypot(u_c - u_pred, v_c - v_pred);
                if (dist < min_pix_dist) {
                    min_pix_dist = dist;
                    best_det_idx = static_cast<int>(i);
                    best_z_center << u_c, v_c;
                }
            }

            // Pixel distance gating (maximum 300 pixels innovation window)
            if (best_det_idx != -1 && min_pix_dist < 300.0) {
                // Time-dependent covariance propagation (process noise Q)
                // Adds process noise to prevent covariance collapse so EKF remains sensitive to vision
                rclcpp::Time current_time = this->now();
                double dt = 0.033;
                if (gates_[target_idx].last_update_time.nanoseconds() > 0) {
                    dt = std::clamp((current_time - gates_[target_idx].last_update_time).seconds(), 0.001, 1.0);
                }
                gates_[target_idx].last_update_time = current_time;

                Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * (process_noise_q_ * dt);
                Eigen::Matrix3d P_prior = gates_[target_idx].covariance + Q;

                // Measurement Jacobian H = d(h)/d(p_cam) * R_world_to_cam (2x3)
                Eigen::Matrix<double, 2, 3> dh_dpc;
                dh_dpc << fx_ * z_inv, 0.0, -fx_ * p_cam_pred.x() * z_inv2,
                          0.0, fy_ * z_inv, -fy_ * p_cam_pred.y() * z_inv2;

                Eigen::Matrix<double, 2, 3> H = dh_dpc * R_world_to_cam;

                // 2D Pixel Innovation: y = z_meas - h(x_hat)
                Eigen::Vector2d y_innov = best_z_center - Eigen::Vector2d(u_pred, v_pred);

                // Distance-scaled pixel measurement covariance
                // At 30m, 1 pixel has a larger lever arm (8.4cm/px) than at 15m (4.2cm/px).
                // Scaling R_pix at long range prevents single-frame YOLO noise from causing drift,
                // while retaining crisp responsiveness as the drone closes in (< 15m).
                double dist_scale = std::max(1.0, p_cam_pred.z() / 15.0);
                double eff_sigma = pixel_noise_sigma_ * dist_scale;
                Eigen::Matrix2d R_pix = Eigen::Matrix2d::Identity() * (eff_sigma * eff_sigma);

                // Innovation Covariance S = H * P_prior * H^T + R
                Eigen::Matrix2d S = H * P_prior * H.transpose() + R_pix;

                // Kalman Gain K = P_prior * H^T * S^-1 (3x2)
                Eigen::Matrix<double, 3, 2> K = P_prior * H.transpose() * S.inverse();

                // State & Covariance Update
                gates_[target_idx].position_enu += K * y_innov;
                gates_[target_idx].covariance = (Eigen::Matrix3d::Identity() - K * H) * P_prior;

                // Enforce covariance floor so sensitivity never dies
                for (int d = 0; d < 3; ++d) {
                    if (gates_[target_idx].covariance(d, d) < 0.04) {
                        gates_[target_idx].covariance(d, d) = 0.04;
                    }
                }

                // Anti-drift prior clamp
                clamp_gate_position(gates_[target_idx]);

                // Propagate offset downstream
                propagate_offsets(target_idx);
            }
            return;
        }

        // =========================================================================
        // METHOD A2: Pixel Innovation Advanced (Locked Depth & Elevation, 1D Lateral EKF)
        // =========================================================================
        if (method_ == "pixel_innovation_advanced" || method_ == "pixel-innovation-advanced") {
            auto &gate = gates_[target_idx];

            // Gate coordinate axes in World ENU
            Eigen::Vector3d up_enu(0.0, 0.0, 1.0);
            Eigen::Vector3d r_gate = (gate.normal_enu.cross(up_enu)).normalized();
            if (r_gate.norm() < 0.1) {
                r_gate = Eigen::Vector3d(0.0, 1.0, 0.0);
            }
            Eigen::Vector3d u_gate = up_enu;
            double hw = gate_w_ / 2.0;
            double hh = gate_h_ / 2.0;

            // 4 Corner offsets relative to gate center in World ENU:
            std::vector<Eigen::Vector3d> corner_offsets_enu = {
                -hw * r_gate + hh * u_gate, // TL (0)
                 hw * r_gate + hh * u_gate, // TR (1)
                 hw * r_gate - hh * u_gate, // BR (2)
                -hw * r_gate - hh * u_gate  // BL (3)
            };

            // Predict 4 corner positions in camera RDF frame
            std::vector<Eigen::Vector3d> p_cam_pred(4);
            Eigen::Matrix<double, 8, 1> z_pred;
            bool valid_projection = true;

            for (size_t k = 0; k < 4; ++k) {
                Eigen::Vector3d corner_enu = gate.position_enu + corner_offsets_enu[k];
                p_cam_pred[k] = R_world_to_cam * (corner_enu - drone_pos);

                if (p_cam_pred[k].z() <= 0.5) {
                    valid_projection = false;
                    break;
                }

                double z_inv = 1.0 / p_cam_pred[k].z();
                z_pred(2 * k)     = fx_ * p_cam_pred[k].x() * z_inv + cx_;
                z_pred(2 * k + 1) = fy_ * p_cam_pred[k].y() * z_inv + cy_;
            }

            if (!valid_projection) {
                return;
            }

            // Predicted center pixel for data association
            double u_pred_center = 0.25 * (z_pred(0) + z_pred(2) + z_pred(4) + z_pred(6));
            double v_pred_center = 0.25 * (z_pred(1) + z_pred(3) + z_pred(5) + z_pred(7));

            // Find best matching detection based on 2D pixel distance to predicted center
            int best_det_idx = -1;
            double min_pix_dist = std::numeric_limits<double>::max();
            Eigen::Matrix<double, 8, 1> z_meas;

            for (size_t i = 0; i < num_detections; ++i) {
                size_t off = off_start + i * 13;
                double u0 = msg->data[off + 1], v0 = msg->data[off + 2];
                double u1 = msg->data[off + 4], v1 = msg->data[off + 5];
                double u2 = msg->data[off + 7], v2 = msg->data[off + 8];
                double u3 = msg->data[off + 10], v3 = msg->data[off + 11];

                double u_c = 0.25 * (u0 + u1 + u2 + u3);
                double v_c = 0.25 * (v0 + v1 + v2 + v3);

                double dist = std::hypot(u_c - u_pred_center, v_c - v_pred_center);
                if (dist < min_pix_dist) {
                    min_pix_dist = dist;
                    best_det_idx = static_cast<int>(i);
                    z_meas << u0, v0, u1, v1, u2, v2, u3, v3;
                }
            }

            // Gating: Only accept detection if center is within 180 pixels of prediction
            if (best_det_idx != -1 && min_pix_dist < 180.0) {
                // Lateral vector transformed to camera RDF frame: v_lat_cam = R_world_to_cam * r_gate
                Eigen::Vector3d v_lat_cam = R_world_to_cam * r_gate;

                // 8x1 Measurement Jacobian w.r.t 1D scalar lateral offset d_lat
                Eigen::Matrix<double, 8, 1> H = Eigen::Matrix<double, 8, 1>::Zero();
                for (size_t k = 0; k < 4; ++k) {
                    double x_k = p_cam_pred[k].x();
                    double y_k = p_cam_pred[k].y();
                    double z_k = p_cam_pred[k].z();
                    double z_inv = 1.0 / z_k;
                    double z_inv2 = z_inv * z_inv;

                    H(2 * k, 0)     = fx_ * (z_inv * v_lat_cam.x() - x_k * z_inv2 * v_lat_cam.z());
                    H(2 * k + 1, 0) = fy_ * (z_inv * v_lat_cam.y() - y_k * z_inv2 * v_lat_cam.z());
                }

                // 8D Innovation: y = z_meas - z_pred
                Eigen::Matrix<double, 8, 1> y_innov = z_meas - z_pred;

                // 8x8 Measurement noise covariance R
                Eigen::Matrix<double, 8, 8> R_pix = Eigen::Matrix<double, 8, 8>::Identity() * (pixel_noise_sigma_ * pixel_noise_sigma_);

                // 1D Scalar Lateral Variance P_lat (with process noise)
                double P_lat = (r_gate.transpose() * gate.covariance * r_gate)(0, 0);
                P_lat = std::clamp(P_lat + 0.02, 0.001, 25.0);

                // 8x8 Innovation Covariance S = H * P_lat * H^T + R_pix
                Eigen::Matrix<double, 8, 8> S = H * P_lat * H.transpose() + R_pix;

                // 1x8 Kalman Gain K = P_lat * H^T * S^-1
                Eigen::Matrix<double, 1, 8> K = P_lat * H.transpose() * S.inverse();

                // 1D Scalar Lateral Update
                double delta_d_lat = (K * y_innov)(0, 0);

                // Compute current lateral offset along r_gate
                double current_d_lat = (gate.position_enu - gate.prior_pos_enu).dot(r_gate);
                double new_d_lat = std::clamp(current_d_lat + delta_d_lat, -max_prior_deviation_, max_prior_deviation_);

                // State Update: Exactly locked to prior in forward and elevation, lateral along r_gate
                gate.position_enu = gate.prior_pos_enu + new_d_lat * r_gate;

                // 1D Joseph-form Covariance Update
                double I_minus_KH = 1.0 - (K * H)(0, 0);
                double P_lat_new = I_minus_KH * P_lat * I_minus_KH + (K * R_pix * K.transpose())(0, 0);
                gate.covariance = P_lat_new * (r_gate * r_gate.transpose()) + 0.01 * Eigen::Matrix3d::Identity();

                // Propagate offset downstream
                propagate_offsets(target_idx);
            }
            return;
        }

        // =========================================================================
        // METHOD B: 3D Perspective-n-Point (IPPE + Monte-Carlo Covariance)
        // =========================================================================
        const Eigen::Matrix3d R_drone_pos = Eigen::Matrix3d::Identity() * (drone_sigma_ * drone_sigma_);
        int best_pnp_idx = -1;
        double min_mahalanobis_sq = std::numeric_limits<double>::max();
        Eigen::Vector3d best_z_meas = Eigen::Vector3d::Zero();
        Eigen::Matrix3d best_R_meas = Eigen::Matrix3d::Identity();

        std::mt19937 rng(1337);
        std::normal_distribution<double> noise_dist(0.0, corner_noise_sigma_);

        for (size_t i = 0; i < num_detections; ++i) {
            size_t off = off_start + i * 13;
            double u0 = msg->data[off + 1], v0 = msg->data[off + 2];
            double u1 = msg->data[off + 4], v1 = msg->data[off + 5];
            double u2 = msg->data[off + 7], v2 = msg->data[off + 8];
            double u3 = msg->data[off + 10], v3 = msg->data[off + 11];

            std::vector<cv::Point2f> image_points = {
                cv::Point2f(static_cast<float>(u0), static_cast<float>(v0)),
                cv::Point2f(static_cast<float>(u1), static_cast<float>(v1)),
                cv::Point2f(static_cast<float>(u2), static_cast<float>(v2)),
                cv::Point2f(static_cast<float>(u3), static_cast<float>(v3))
            };

            cv::Mat rvec, tvec;
            bool success = cv::solvePnP(
                object_points_4p_,
                image_points,
                K_,
                D_,
                rvec,
                tvec,
                false,
                cv::SOLVEPNP_IPPE
            );

            if (!success || tvec.empty()) {
                continue;
            }

            double tx = tvec.at<double>(0);
            double ty = tvec.at<double>(1);
            double tz = tvec.at<double>(2);

            if (tz <= 0.3) {
                continue;
            }

            Eigen::Vector3d p_pnp_rdf(tx, ty, tz);
            double dist_to_cam = p_pnp_rdf.norm();

            if (dist_to_cam > max_refine_dist_) {
                continue;
            }

            // Fast C++ Monte-Carlo Covariance
            Eigen::Matrix3d R_pnp_cam = Eigen::Matrix3d::Identity();
            if (enable_mc_cov_) {
                std::vector<Eigen::Vector3d> sampled_translations;
                for (int s = 0; s < mc_samples_; ++s) {
                    std::vector<cv::Point2f> perturbed_pts = image_points;
                    for (auto &pt : perturbed_pts) {
                        pt.x += static_cast<float>(noise_dist(rng));
                        pt.y += static_cast<float>(noise_dist(rng));
                    }
                    cv::Mat r_s, t_s;
                    if (cv::solvePnP(object_points_4p_, perturbed_pts, K_, D_, r_s, t_s, false, cv::SOLVEPNP_IPPE)) {
                        if (t_s.at<double>(2) > 0.3) {
                            sampled_translations.emplace_back(
                                t_s.at<double>(0),
                                t_s.at<double>(1),
                                t_s.at<double>(2)
                            );
                        }
                    }
                }

                if (sampled_translations.size() >= static_cast<size_t>(std::max(5, mc_samples_ / 2))) {
                    Eigen::MatrixXd samples(sampled_translations.size(), 3);
                    for (size_t s = 0; s < sampled_translations.size(); ++s) {
                        samples.row(s) = sampled_translations[s];
                    }
                    Eigen::Vector3d mean = samples.colwise().mean();
                    Eigen::MatrixXd centered = samples.rowwise() - mean.transpose();
                    R_pnp_cam = (centered.transpose() * centered) / static_cast<double>(sampled_translations.size() - 1);
                    R_pnp_cam += Eigen::Matrix3d::Identity() * 0.0025;
                } else {
                    double sig = (dist_to_cam > 20.0) ? 8.0 : (1.5 + (dist_to_cam / 20.0) * 6.5);
                    R_pnp_cam = Eigen::Matrix3d::Identity() * (sig * sig);
                }
            } else {
                double eff_pnp_sigma = pnp_sigma_;
                if (dist_to_cam > 36.0) {
                    eff_pnp_sigma = 10.0;
                } else if (dist_to_cam > 5.0) {
                    double ratio = (dist_to_cam - 5.0) / (36.0 - 5.0);
                    eff_pnp_sigma = pnp_sigma_ + ratio * (10.0 - pnp_sigma_);
                }
                R_pnp_cam = Eigen::Matrix3d::Identity() * (eff_pnp_sigma * eff_pnp_sigma);
            }

            const Eigen::Matrix3d R_pnp_world = R_total * R_pnp_cam * R_total.transpose();
            const Eigen::Matrix3d R_meas = R_pnp_world + R_drone_pos;

            Eigen::Vector3d z_meas = drone_pos + (R_total * p_pnp_rdf);

            Eigen::Vector3d y = z_meas - gates_[target_idx].position_enu;
            Eigen::Matrix3d S = gates_[target_idx].covariance + R_meas;

            double m_dist_sq = y.transpose() * S.inverse() * y;
            double euc_dist = y.norm();
            double dynamic_max_dist = std::max(max_dist_, 3.0 * std::sqrt(R_meas.diagonal().maxCoeff()));

            if (m_dist_sq < min_mahalanobis_sq && euc_dist <= dynamic_max_dist && m_dist_sq <= mahalanobis_max_sq_) {
                min_mahalanobis_sq = m_dist_sq;
                best_pnp_idx = static_cast<int>(i);
                best_z_meas = z_meas;
                best_R_meas = R_meas;
            }
        }

        if (best_pnp_idx != -1) {
            // Step 1: Primary 3D PnP Kalman Filter Update
            apply_gate_update(target_idx, best_z_meas, best_R_meas, min_mahalanobis_sq);

            // Step 2: Sequential Lateral 1D Pixel Innovation Update (Bearing Fine-Centering)
            size_t best_off = off_start + best_pnp_idx * 13;
            double u0 = msg->data[best_off + 1], u1 = msg->data[best_off + 4];
            double u2 = msg->data[best_off + 7], u3 = msg->data[best_off + 10];
            double u_center_meas = 0.25 * (u0 + u1 + u2 + u3);

            apply_lateral_pixel_innovation(target_idx, u_center_meas, drone_pos, R_world_to_cam);
            propagate_offsets(target_idx);
        }
    }

    void pnp_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (!drone_pose_received_ || msg->poses.empty()) {
            return;
        }

        Eigen::Vector3d drone_pos = latest_drone_pos_;
        Eigen::Quaterniond drone_rot = latest_drone_rot_;
        rclcpp::Time pnp_stamp(msg->header.stamp);
        get_interpolated_drone_pose(pnp_stamp, drone_pos, drone_rot);

        // Attitude Gating
        double qw = drone_rot.w();
        double qx = drone_rot.x();
        double qy = drone_rot.y();
        double qz = drone_rot.z();

        double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
        double sin_pitch = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
        double pitch = std::asin(sin_pitch);

        if (std::abs(roll) > max_tilt_rad_ || std::abs(pitch) > max_tilt_rad_) {
            return;
        }

        int target_idx = active_target_gate_idx_;
        if (target_idx < 0 || target_idx >= static_cast<int>(gates_.size()) || target_idx == 3) {
            return;
        }

        const Eigen::Matrix3d R_drone = drone_rot.toRotationMatrix();
        const Eigen::Matrix3d R_total = R_drone * R_cam_to_body_;
        const Eigen::Matrix3d R_drone_pos = Eigen::Matrix3d::Identity() * (drone_sigma_ * drone_sigma_);

        int best_pnp_idx = -1;
        double min_mahalanobis_sq = std::numeric_limits<double>::max();
        Eigen::Vector3d best_z_meas = Eigen::Vector3d::Zero();
        Eigen::Matrix3d best_R_meas = Eigen::Matrix3d::Identity();

        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto &pnp_pose = msg->poses[i];
            Eigen::Vector3d p_pnp_rdf(pnp_pose.position.x, pnp_pose.position.y, pnp_pose.position.z);
            double dist_to_cam = p_pnp_rdf.norm();

            if (dist_to_cam > max_refine_dist_) {
                continue;
            }

            Eigen::Matrix3d R_pnp_cam = Eigen::Matrix3d::Identity();
            if (has_pnp_covariances_ && latest_pnp_covariances_.data.size() >= (i + 1) * 9) {
                size_t off = i * 9;
                R_pnp_cam << latest_pnp_covariances_.data[off + 0], latest_pnp_covariances_.data[off + 1], latest_pnp_covariances_.data[off + 2],
                             latest_pnp_covariances_.data[off + 3], latest_pnp_covariances_.data[off + 4], latest_pnp_covariances_.data[off + 5],
                             latest_pnp_covariances_.data[off + 6], latest_pnp_covariances_.data[off + 7], latest_pnp_covariances_.data[off + 8];
            } else {
                double eff_pnp_sigma = pnp_sigma_;
                if (dist_to_cam > 36.0) {
                    eff_pnp_sigma = 10.0;
                } else if (dist_to_cam > 5.0) {
                    double ratio = (dist_to_cam - 5.0) / (36.0 - 5.0);
                    eff_pnp_sigma = pnp_sigma_ + ratio * (10.0 - pnp_sigma_);
                }
                R_pnp_cam = Eigen::Matrix3d::Identity() * (eff_pnp_sigma * eff_pnp_sigma);
            }

            const Eigen::Matrix3d R_pnp_world = R_total * R_pnp_cam * R_total.transpose();
            const Eigen::Matrix3d R_meas = R_pnp_world + R_drone_pos;

            Eigen::Vector3d z_meas = drone_pos + (R_total * p_pnp_rdf);
            Eigen::Vector3d y = z_meas - gates_[target_idx].position_enu;
            Eigen::Matrix3d S = gates_[target_idx].covariance + R_meas;

            double m_dist_sq = y.transpose() * S.inverse() * y;
            double euc_dist = y.norm();
            double dynamic_max_dist = std::max(max_dist_, 3.0 * std::sqrt(R_meas.diagonal().maxCoeff()));

            if (m_dist_sq < min_mahalanobis_sq && euc_dist <= dynamic_max_dist && m_dist_sq <= mahalanobis_max_sq_) {
                min_mahalanobis_sq = m_dist_sq;
                best_pnp_idx = static_cast<int>(i);
                best_z_meas = z_meas;
                best_R_meas = R_meas;
            }
        }

        if (best_pnp_idx != -1) {
            apply_gate_update(target_idx, best_z_meas, best_R_meas, min_mahalanobis_sq);
        }
    }

    void apply_lateral_pixel_innovation(int target_idx, double u_meas, const Eigen::Vector3d &drone_pos, const Eigen::Matrix3d &R_world_to_cam) {
        if (target_idx < 0 || target_idx >= static_cast<int>(gates_.size())) {
            return;
        }

        auto &gate = gates_[target_idx];
        const Eigen::Vector3d up_enu(0.0, 0.0, 1.0);
        Eigen::Vector3d r_gate = (gate.normal_enu.cross(up_enu)).normalized();
        if (r_gate.norm() < 0.1) {
            r_gate = Eigen::Vector3d(0.0, 1.0, 0.0);
        }

        Eigen::Vector3d p_cam_pred = R_world_to_cam * (gate.position_enu - drone_pos);
        if (p_cam_pred.z() <= 0.5) {
            return;
        }

        double z_inv = 1.0 / p_cam_pred.z();
        double z_inv2 = z_inv * z_inv;
        double u_pred = fx_ * p_cam_pred.x() * z_inv + cx_;

        // Lateral direction vector in camera RDF frame: v_lat_cam = R_world_to_cam * r_gate
        Eigen::Vector3d v_lat_cam = R_world_to_cam * r_gate;

        // 1D Jacobian: d(u_pred) / d(d_lat) along r_gate
        double H_lat = fx_ * (v_lat_cam.x() * z_inv - p_cam_pred.x() * v_lat_cam.z() * z_inv2);

        // 1D Scalar Lateral Variance P_lat along r_gate
        double P_lat = (r_gate.transpose() * gate.covariance * r_gate)(0, 0);
        P_lat = std::clamp(P_lat, 0.001, 25.0);

        // Measurement noise covariance (scaled by distance to balance trust with PnP)
        double dist_scale = std::max(1.0, p_cam_pred.z() / 15.0);
        double eff_lat_sigma = pixel_noise_sigma_ * dist_scale;
        double R_lat = eff_lat_sigma * eff_lat_sigma;

        // 1D Kalman Innovation & Gain
        double S_lat = H_lat * P_lat * H_lat + R_lat;
        if (std::abs(S_lat) > 1e-6) {
            double K_lat = (P_lat * H_lat) / S_lat;
            double y_u = u_meas - u_pred;

            double delta_d_lat = K_lat * y_u;
            gate.position_enu += delta_d_lat * r_gate;

            double P_lat_new = (1.0 - K_lat * H_lat) * P_lat;
            gate.covariance += (P_lat_new - P_lat) * (r_gate * r_gate.transpose());
        }

        clamp_gate_position(gate);
    }

    void apply_gate_update(int target_idx, const Eigen::Vector3d &best_z_meas, const Eigen::Matrix3d &best_R_meas, double min_mahalanobis_sq) {
        if (target_idx == 4 && blend_gate5_with_mean_1_2_) {
            // Target Gate 5: blend raw PnP offset (60%) with Gate 2 offset (40%)
            Eigen::Vector3d offset_2 = gates_[1].position_enu - gates_[1].prior_pos_enu;
            Eigen::Vector3d offset_5_raw = best_z_meas - gates_[4].prior_pos_enu;

            Eigen::Vector3d blended_offset = 0.40 * offset_2 + 0.60 * offset_5_raw;
            Eigen::Vector3d z_meas_blended = gates_[4].prior_pos_enu + blended_offset;

            update_gate_kalman(gates_[4], z_meas_blended, best_R_meas, min_mahalanobis_sq);
        } else if (target_idx == 2) {
            // Target Gate 3: blend 50% Gate 3 PnP with 50% upstream Gate 1/2 mean offset
            Eigen::Vector3d offset_1 = gates_[0].position_enu - gates_[0].prior_pos_enu;
            Eigen::Vector3d offset_2 = gates_[1].position_enu - gates_[1].prior_pos_enu;
            Eigen::Vector3d raw_offset_1_2 = tunnel_blend_gate1_and_2_ ? (0.5 * (offset_1 + offset_2)) : offset_2;

            Eigen::Vector3d offset_1_2_horiz(raw_offset_1_2.x(), raw_offset_1_2.y(), 0.0);
            Eigen::Vector3d offset_3_pnp_raw = best_z_meas - gates_[2].prior_pos_enu;
            Eigen::Vector3d offset_3_pnp_horiz(offset_3_pnp_raw.x(), offset_3_pnp_raw.y(), 0.0);

            Eigen::Vector3d blended_horiz = 0.50 * offset_1_2_horiz + 0.50 * offset_3_pnp_horiz;
            Eigen::Vector3d z_meas_gate3 = gates_[2].prior_pos_enu + blended_horiz;
            z_meas_gate3.z() = best_z_meas.z();

            update_gate_kalman(gates_[2], z_meas_gate3, best_R_meas, min_mahalanobis_sq);

            // Synchronize Gate 4 offset to match Gate 3
            Eigen::Vector3d total_offset_3 = gates_[2].position_enu - gates_[2].prior_pos_enu;
            if (gates_.size() > 3) {
                gates_[3].position_enu = gates_[3].prior_pos_enu + total_offset_3;
            }
        } else {
            update_gate_kalman(gates_[target_idx], best_z_meas, best_R_meas, min_mahalanobis_sq);
        }

        propagate_offsets(target_idx);
    }

    void propagate_offsets(int target_idx) {
        if (target_idx == 0 || target_idx == 1) {
            Eigen::Vector3d offset_1 = gates_[0].position_enu - gates_[0].prior_pos_enu;
            Eigen::Vector3d offset_2 = gates_[1].position_enu - gates_[1].prior_pos_enu;
            
            Eigen::Vector3d raw_offset = Eigen::Vector3d::Zero();
            bool should_propagate = false;

            if (tunnel_blend_gate1_and_2_) {
                raw_offset = (target_idx == 0) ? offset_1 : (0.5 * (offset_1 + offset_2));
                should_propagate = true;
            } else {
                if (target_idx == 1) {
                    raw_offset = offset_2;
                    should_propagate = true;
                }
            }

            if (should_propagate) {
                Eigen::Vector3d offset_to_apply(raw_offset.x(), raw_offset.y(), 0.0);

                if (target_idx == 0 && gates_.size() > 1) {
                    gates_[1].position_enu = gates_[1].prior_pos_enu + offset_to_apply;
                }
                if (gates_.size() > 2) {
                    gates_[2].position_enu = gates_[2].prior_pos_enu + offset_to_apply;
                }
                if (gates_.size() > 3) {
                    gates_[3].position_enu = gates_[3].prior_pos_enu + offset_to_apply;
                }
                if (gates_.size() > 4 && blend_gate5_with_mean_1_2_) {
                    gates_[4].position_enu = gates_[4].prior_pos_enu + 0.40 * offset_to_apply;
                }
            }
        } else if (target_idx == 2) {
            // Synchronize Gate 4 to maintain the exact same total world-space offset as Gate 3
            if (gates_.size() > 3) {
                Eigen::Vector3d total_offset_3 = gates_[2].position_enu - gates_[2].prior_pos_enu;
                gates_[3].position_enu = gates_[3].prior_pos_enu + total_offset_3;
            }
        }
    }

    void clamp_gate_position(GateState &gate) {
        Eigen::Vector3d dev = gate.position_enu - gate.prior_pos_enu;
        double horiz_dev = std::hypot(dev.x(), dev.y());
        if (horiz_dev > max_prior_deviation_) {
            double scale = max_prior_deviation_ / horiz_dev;
            gate.position_enu.x() = gate.prior_pos_enu.x() + dev.x() * scale;
            gate.position_enu.y() = gate.prior_pos_enu.y() + dev.y() * scale;
        }
        gate.position_enu.z() = gate.prior_pos_enu.z();
    }

    void update_gate_kalman(GateState &gate, const Eigen::Vector3d &z_meas, const Eigen::Matrix3d &R_meas, [[maybe_unused]] double mahalanobis_sq) {
        rclcpp::Time current_time = this->now();
        double dt = 0.033;
        if (gate.last_update_time.nanoseconds() > 0) {
            dt = std::clamp((current_time - gate.last_update_time).seconds(), 0.001, 1.0);
        }
        gate.last_update_time = current_time;

        Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * (process_noise_q_ * dt);
        Eigen::Matrix3d P_prior = gate.covariance + Q;

        const Eigen::Vector3d y = z_meas - gate.position_enu;
        const Eigen::Matrix3d S = P_prior + R_meas;
        const Eigen::Matrix3d K = P_prior * S.inverse();

        gate.position_enu += K * y;
        gate.covariance = (Eigen::Matrix3d::Identity() - K) * P_prior;

        for (int d = 0; d < 3; ++d) {
            if (gate.covariance(d, d) < 0.04) {
                gate.covariance(d, d) = 0.04;
            }
        }

        clamp_gate_position(gate);
    }

    void publish_refined_poses(const std_msgs::msg::Header &header) {
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = header.stamp;
        msg.header.frame_id = "map";

        const double alpha = 0.35; // 30Hz exponential smoothing (tau ~ 75ms)

        for (auto &gate : gates_) {
            // Smooth discrete 10Hz vision jumps across continuous 30Hz timer ticks
            gate.smoothed_pos_enu = (1.0 - alpha) * gate.smoothed_pos_enu + alpha * gate.position_enu;

            geometry_msgs::msg::Pose p;
            p.position.x = gate.smoothed_pos_enu.x();
            p.position.y = gate.smoothed_pos_enu.y();
            p.position.z = gate.smoothed_pos_enu.z();

            p.orientation.x = gate.orientation.x();
            p.orientation.y = gate.orientation.y();
            p.orientation.z = gate.orientation.z();
            p.orientation.w = gate.orientation.w();

            msg.poses.push_back(p);
        }

        pub_refined_poses_->publish(msg);
    }

    void publish_initial_poses(const std_msgs::msg::Header &header) {
        geometry_msgs::msg::PoseArray pose_msg = initial_poses_msg_;
        pose_msg.header.stamp = header.stamp;
        pose_msg.header.frame_id = "map";
        pub_initial_poses_->publish(pose_msg);

        visualization_msgs::msg::MarkerArray marker_msg = initial_markers_msg_;
        for (auto &marker : marker_msg.markers) {
            marker.header.stamp = header.stamp;
            marker.header.frame_id = "map";
        }
        pub_initial_markers_->publish(marker_msg);
    }

    void publish_rviz_markers(const std_msgs::msg::Header &header) {
        visualization_msgs::msg::MarkerArray array;

        for (const auto &gate : gates_) {
            visualization_msgs::msg::Marker box;
            box.header.stamp = header.stamp;
            box.header.frame_id = "map";
            box.ns = "gate_boxes";
            box.id = gate.id;
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose.position.x = gate.smoothed_pos_enu.x();
            box.pose.position.y = gate.smoothed_pos_enu.y();
            box.pose.position.z = gate.smoothed_pos_enu.z();
            box.pose.orientation.x = gate.orientation.x();
            box.pose.orientation.y = gate.orientation.y();
            box.pose.orientation.z = gate.orientation.z();
            box.pose.orientation.w = gate.orientation.w();
            box.scale.x = 0.1;
            box.scale.y = 1.9;
            box.scale.z = 2.0;
            box.color.r = 0.0f;
            box.color.g = 0.8f;
            box.color.b = 1.0f;
            box.color.a = 0.6f;
            array.markers.push_back(box);

            visualization_msgs::msg::Marker text;
            text.header.stamp = header.stamp;
            text.header.frame_id = "map";
            text.ns = "gate_labels";
            text.id = gate.id + 100;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = gate.smoothed_pos_enu.x();
            text.pose.position.y = gate.smoothed_pos_enu.y();
            text.pose.position.z = gate.smoothed_pos_enu.z() + 1.5;
            text.scale.z = 0.5;
            text.color.r = 1.0f;
            text.color.g = 1.0f;
            text.color.b = 1.0f;
            text.color.a = 1.0f;
            text.text = "Gate " + std::to_string(gate.id);
            array.markers.push_back(text);

            // Add Refined Sub-Gates for Gate 3 and Gate 4
            std::vector<double> sub_offsets;
            if (v7_) {
                if (gate.id == 3) {
                    sub_offsets = {1.0, 2.5};
                } else if (gate.id == 4) {
                    sub_offsets = {1.0, 2.0, 3.0};
                }
            } else {
                if (gate.id == 3) {
                    sub_offsets = {1.0, 2.5};
                } else if (gate.id == 4) {
                    sub_offsets = {1.0, 2.0};
                }
            }

            for (size_t sub_k = 0; sub_k < sub_offsets.size(); ++sub_k) {
                double s_off = sub_offsets[sub_k];
                Eigen::Vector3d sub_pos = gate.position_enu + s_off * gate.normal_enu;
                if (gate.id == 3 && sub_k == 1) { // Gate 3.2
                    Eigen::Vector3d right_enu(gate.normal_enu.y(), -gate.normal_enu.x(), 0.0);
                    sub_pos += 0.2 * right_enu;
                }
                int sub_id_num = gate.id * 10 + (sub_k + 1);

                visualization_msgs::msg::Marker sub_box = box;
                sub_box.id = sub_id_num;
                sub_box.pose.position.x = sub_pos.x();
                sub_box.pose.position.y = sub_pos.y();
                sub_box.pose.position.z = sub_pos.z();
                sub_box.color.r = 0.2f;
                sub_box.color.g = 0.9f;
                sub_box.color.b = 0.8f;
                sub_box.color.a = 0.45f;
                array.markers.push_back(sub_box);

                visualization_msgs::msg::Marker sub_text = text;
                sub_text.id = sub_id_num + 100;
                sub_text.pose.position.x = sub_pos.x();
                sub_text.pose.position.y = sub_pos.y();
                sub_text.pose.position.z = sub_pos.z() + 1.5;
                sub_text.text = "Gate " + std::to_string(gate.id) + "." + std::to_string(sub_k + 1) + " (Sub)";
                array.markers.push_back(sub_text);
            }
        }

        pub_markers_->publish(array);
    }

    // Parameters
    std::string method_{"pnp"};
    double pixel_noise_sigma_{4.0};
    double prior_sigma_;
    double drone_sigma_;
    double pnp_sigma_;
    double max_dist_;
    double mahalanobis_max_sq_;
    double max_prior_deviation_;
    double max_tilt_rad_;
    double max_refine_dist_;
    double process_noise_q_{0.5};
    double camera_pitch_deg_{0.0};
    double gate_w_{1.9};
    double gate_h_{2.0};
    bool enable_mc_cov_{true};
    int mc_samples_{20};
    double corner_noise_sigma_{2.0};
    bool publish_initial_pos_;
    bool blend_gate5_with_mean_1_2_{true};
    bool tunnel_blend_gate1_and_2_{true};
    bool use_1d_right_axis_offset_{false};
    bool enable_gate3_pnp_refinement_{true};
    bool v7_{false};

    // Data structures & matrices
    std::vector<GateState> gates_;
    geometry_msgs::msg::PoseArray initial_poses_msg_;
    visualization_msgs::msg::MarkerArray initial_markers_msg_;
    Eigen::Matrix3d R_cam_to_body_;

    Eigen::Vector3d latest_drone_pos_;
    Eigen::Quaterniond latest_drone_rot_;
    bool drone_pose_received_;
    std::deque<StampedPose> pose_history_;
    std::mutex pose_mutex_;

    Eigen::Vector3d initial_drone_pos_;
    Eigen::Quaterniond initial_drone_rot_;
    bool initial_pose_captured_;

    // Landing Pad state & priors
    Eigen::Vector3d prior_landing_pad_enu_{0.0, 0.0, 0.0};
    Eigen::Vector3d refined_landing_pad_enu_{0.0, 0.0, 0.0};
    Eigen::Vector3d smoothed_landing_pad_enu_{0.0, 0.0, 0.0};
    Eigen::Vector3d manual_landing_pad_enu_{0.0, 0.0, 0.0};
    bool has_manual_landing_pad_{false};

    // Camera calibration & 3D object geometry
    bool camera_info_received_{false};
    cv::Mat K_;
    cv::Mat D_;
    double fx_{0.0};
    double fy_{0.0};
    double cx_{0.0};
    double cy_{0.0};
    double img_w_{640.0};
    double img_h_{640.0};
    std::vector<cv::Point3f> object_points_4p_;

    // ROS 2 Comms
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_gate_corners_2d_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pnp_poses_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pnp_covariances_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_target_gate_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_refined_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_projected_pixels_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_initial_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_initial_markers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_landing_pad_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_landing_pad_marker_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_manual_pad_active_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_gates_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_tag_landing_pad_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_landing_pad_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    std_msgs::msg::Float64MultiArray latest_pnp_covariances_;
    bool has_pnp_covariances_{false};
    int active_target_gate_idx_{0};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavrosGateEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}