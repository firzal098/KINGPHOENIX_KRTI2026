#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    Eigen::Vector3d prior_pos_enu;  // Initial surveyed prior position in local ENU
    Eigen::Matrix3d covariance;     // 3x3 error covariance matrix
    Eigen::Vector3d normal_enu;     // Gate normal vector in ENU frame
    Eigen::Quaterniond orientation; // Orientation quaternion in ENU
};

class MavrosGateEstimator : public rclcpp::Node {
public:
    MavrosGateEstimator() 
    : Node("mavros_gate_estimator"), 
      drone_pose_received_(false), 
      initial_pose_captured_(false),
      active_target_gate_idx_(0)
    {
        // Declare parameters with default double values
        this->declare_parameter<double>("gate_prior_sigma", 1.5);
        this->declare_parameter<double>("drone_pose_sigma", 2.0);
        this->declare_parameter<double>("pnp_vision_sigma", 4.0);
        this->declare_parameter<double>("association_max_dist", 6.0);
        this->declare_parameter<double>("mahalanobis_thresh_sq", 11.345);
        this->declare_parameter<double>("max_prior_deviation_m", 1.5);
        this->declare_parameter<double>("max_refine_tilt_deg", 18.0);
        this->declare_parameter<double>("max_refine_distance_m", 36.0);
        this->declare_parameter<bool>("publish_initial_pos", true);

        // Safe parameter reading (handles int or double from launch files without crashing)
        prior_sigma_          = get_param_as_double("gate_prior_sigma", 1.5);
        drone_sigma_          = get_param_as_double("drone_pose_sigma", 2.0);
        pnp_sigma_            = get_param_as_double("pnp_vision_sigma", 4.0);
        max_dist_             = get_param_as_double("association_max_dist", 6.0);
        mahalanobis_max_sq_   = get_param_as_double("mahalanobis_thresh_sq", 11.345);
        max_prior_deviation_  = get_param_as_double("max_prior_deviation_m", 1.5);
        max_tilt_rad_         = get_param_as_double("max_refine_tilt_deg", 18.0) * (M_PI / 180.0);
        max_refine_dist_      = get_param_as_double("max_refine_distance_m", 36.0);
        publish_initial_pos_  = this->get_parameter("publish_initial_pos").as_bool();

        // Initialize zero offset defaults until first MAVROS pose is received
        initial_drone_pos_ = Eigen::Vector3d::Zero();
        initial_drone_rot_ = Eigen::Quaterniond::Identity();

        // Initialize default gate priors at origin
        initialize_gate_priors();

        // Define Camera Optical (RDF) to Drone Body (FLU) rotation matrix
        R_cam_to_body_ <<  0.0,  0.0,  1.0,
                          -1.0,  0.0,  0.0,
                           0.0, -1.0,  0.0;

        auto sensor_qos = rclcpp::SensorDataQoS();

        // Subscribers
        sub_drone_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", sensor_qos,
            std::bind(&MavrosGateEstimator::drone_pose_callback, this, std::placeholders::_1)
        );

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

        // 30 Hz wall timer (33,333 microseconds)
        pub_timer_ = this->create_wall_timer(
            std::chrono::microseconds(33333),
            std::bind(&MavrosGateEstimator::timer_callback, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "MAVROS Gate Estimator Node Initialized (30 Hz streaming, Max Dist: %.1fm, Mahalanobis Sq: %.3f).",
            max_dist_, mahalanobis_max_sq_
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
    }

    void initialize_gate_priors() {
        gates_.clear();
        initial_poses_msg_.poses.clear();
        initial_markers_msg_.markers.clear();

        std::vector<GatePriorRDF> priors_rdf = {
            {1,  0.41, -0.75, 29.33,  0.0, 0.0, 1.0},
            {2,  5.46, -0.75, 19.31,  0.0, 0.0, 1.0},
            {3,  9.49, -0.75, 10.51,  1.0, 0.0, 0.0},
            {4, 12.41, -0.75, 12.43,  0.0, 0.0, 1.0},
            {5, 17.47, -0.75, 29.38,  0.0, 0.0, 1.0}
        };

        const double initial_var = prior_sigma_ * prior_sigma_;

        for (const auto &p : priors_rdf) {
            GateState state;
            state.id = p.id;

            Eigen::Vector3d rel_pos_enu(p.z_rdf, -p.x_rdf, -p.y_rdf);
            Eigen::Vector3d rel_norm_enu(p.nz_rdf, -p.nx_rdf, -p.ny_rdf);

            state.position_enu  = initial_drone_pos_ + (initial_drone_rot_ * rel_pos_enu);
            state.prior_pos_enu = state.position_enu;
            state.normal_enu    = (initial_drone_rot_ * rel_norm_enu).normalized();
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

            // Add Initial Sub-Gates for Gate 3 (1 sub-gate) and Gate 4 (2 sub-gates)
            std::vector<double> sub_offsets;
            if (state.id == 3) {
                sub_offsets = {1.0};
            } else if (state.id == 4) {
                sub_offsets = {1.0, 2.0};
            }

            for (size_t sub_k = 0; sub_k < sub_offsets.size(); ++sub_k) {
                double s_off = sub_offsets[sub_k];
                Eigen::Vector3d sub_pos = state.position_enu + s_off * state.normal_enu;
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
    }

    void handle_reset_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (drone_pose_received_) {
            initial_drone_pos_ = latest_drone_pos_;
            initial_drone_rot_ = latest_drone_rot_;
        }
        active_target_gate_idx_ = 0;
        initialize_gate_priors();
        response->success = true;
        response->message = "Gate positions and covariances reset to initial priors anchored at current drone position.";
        RCLCPP_INFO(
            this->get_logger(),
            "All gate states successfully reset and re-anchored to current drone pose: [%.2f, %.2f, %.2f].",
            initial_drone_pos_.x(), initial_drone_pos_.y(), initial_drone_pos_.z()
        );
    }

    void drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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

    void pnp_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (!drone_pose_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /mavros/local_position/pose before processing PnP gate measurements..."
            );
            return;
        }

        if (msg->poses.empty()) {
            return;
        }

        // 1. Attitude Gating: Skip vision updates when drone is in high pitch/roll maneuvers to prevent projection errors
        double qw = latest_drone_rot_.w();
        double qx = latest_drone_rot_.x();
        double qy = latest_drone_rot_.y();
        double qz = latest_drone_rot_.z();

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

        // Special Rule: Skip direct PnP refinement for Gate 3 (idx 2) and Gate 4 (idx 3);
        // they are refined using the mean offset from Gate 1 and Gate 2.
        if (target_idx == 2 || target_idx == 3) {
            return;
        }

        const Eigen::Matrix3d R_drone = latest_drone_rot_.toRotationMatrix();
        const Eigen::Matrix3d R_total = R_drone * R_cam_to_body_;
        const Eigen::Matrix3d R_drone_pos = Eigen::Matrix3d::Identity() * (drone_sigma_ * drone_sigma_);

        // Find the best matching PnP measurement for ONLY the active target gate
        int best_pnp_idx = -1;
        double min_mahalanobis_sq = std::numeric_limits<double>::max();
        Eigen::Vector3d best_z_meas = Eigen::Vector3d::Zero();
        Eigen::Matrix3d best_R_meas = Eigen::Matrix3d::Identity();

        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto &pnp_pose = msg->poses[i];
            Eigen::Vector3d p_pnp_rdf(pnp_pose.position.x, pnp_pose.position.y, pnp_pose.position.z);
            double dist_to_cam = p_pnp_rdf.norm();

            // 3. Distance Gating: Skip detections beyond max_refine_dist_ (depth noise is too high)
            if (dist_to_cam > max_refine_dist_) {
                continue;
            }

            // Extract directional 3x3 Measurement Covariance R_pnp_cam from Swift Monte-Carlo sampling (if available)
            Eigen::Matrix3d R_pnp_cam = Eigen::Matrix3d::Identity();
            if (has_pnp_covariances_ && latest_pnp_covariances_.data.size() >= (i + 1) * 9) {
                size_t off = i * 9;
                R_pnp_cam << latest_pnp_covariances_.data[off + 0], latest_pnp_covariances_.data[off + 1], latest_pnp_covariances_.data[off + 2],
                             latest_pnp_covariances_.data[off + 3], latest_pnp_covariances_.data[off + 4], latest_pnp_covariances_.data[off + 5],
                             latest_pnp_covariances_.data[off + 6], latest_pnp_covariances_.data[off + 7], latest_pnp_covariances_.data[off + 8];
            } else {
                // Distance-dependent Error Sigma Fallback (smoothly scaling up to max_refine_dist_)
                double eff_pnp_sigma = pnp_sigma_;
                if (dist_to_cam > 36.0) {
                    eff_pnp_sigma = 10.0;
                } else if (dist_to_cam > 5.0) {
                    double ratio = (dist_to_cam - 5.0) / (36.0 - 5.0);
                    eff_pnp_sigma = pnp_sigma_ + ratio * (10.0 - pnp_sigma_);
                } else {
                    eff_pnp_sigma = pnp_sigma_;
                }
                R_pnp_cam = Eigen::Matrix3d::Identity() * (eff_pnp_sigma * eff_pnp_sigma);
            }

            const Eigen::Matrix3d R_pnp_world = R_total * R_pnp_cam * R_total.transpose();
            const Eigen::Matrix3d R_meas = R_pnp_world + R_drone_pos;

            // Global measured gate position in ENU frame
            Eigen::Vector3d z_meas = latest_drone_pos_ + (R_total * p_pnp_rdf);

            // Associate strictly against active target gate
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

        // Apply EKF update exclusively to the active target gate
        if (best_pnp_idx != -1) {
            if (target_idx == 4) {
                // When targeting Gate 5, blend its raw PnP detection offset with the mean offset of Gate 1 & 2
                Eigen::Vector3d offset_1 = gates_[0].position_enu - gates_[0].prior_pos_enu;
                Eigen::Vector3d offset_2 = gates_[1].position_enu - gates_[1].prior_pos_enu;
                Eigen::Vector3d offset_5_raw = best_z_meas - gates_[4].prior_pos_enu;

                Eigen::Vector3d mean_offset_1_2_5 = (offset_1 + offset_2 + offset_5_raw) / 3.0;
                Eigen::Vector3d z_meas_blended = gates_[4].prior_pos_enu + mean_offset_1_2_5;

                update_gate_kalman(gates_[4], z_meas_blended, best_R_meas, min_mahalanobis_sq);
            } else {
                update_gate_kalman(gates_[target_idx], best_z_meas, best_R_meas, min_mahalanobis_sq);
            }

            // When Gate 1 or Gate 2 is refined, propagate their mean correction offset to Gate 3, 4, and 5
            if (target_idx == 0 || target_idx == 1) {
                Eigen::Vector3d offset_1 = gates_[0].position_enu - gates_[0].prior_pos_enu;
                Eigen::Vector3d offset_2 = gates_[1].position_enu - gates_[1].prior_pos_enu;
                Eigen::Vector3d mean_offset = (target_idx == 0) ? offset_1 : (0.5 * (offset_1 + offset_2));

                if (gates_.size() > 2) {
                    gates_[2].position_enu = gates_[2].prior_pos_enu + mean_offset;
                }
                if (gates_.size() > 3) {
                    gates_[3].position_enu = gates_[3].prior_pos_enu + mean_offset;
                }
                if (gates_.size() > 4) {
                    gates_[4].position_enu = gates_[4].prior_pos_enu + mean_offset;
                }
            }
        }
    }

    void update_gate_kalman(GateState &gate, const Eigen::Vector3d &z_meas, const Eigen::Matrix3d &R_meas, [[maybe_unused]] double mahalanobis_sq) {
        const Eigen::Vector3d y = z_meas - gate.position_enu;
        const Eigen::Matrix3d S = gate.covariance + R_meas;
        const Eigen::Matrix3d K = gate.covariance * S.inverse();

        gate.position_enu += K * y;
        gate.covariance = (Eigen::Matrix3d::Identity() - K) * gate.covariance;

        // Anti-Drift Prior Anchor Clamping: Prevent the gate from wandering more than max_prior_deviation_ from surveyed prior
        Eigen::Vector3d dev = gate.position_enu - gate.prior_pos_enu;
        double horiz_dev = std::hypot(dev.x(), dev.y());
        if (horiz_dev > max_prior_deviation_) {
            double scale = max_prior_deviation_ / horiz_dev;
            gate.position_enu.x() = gate.prior_pos_enu.x() + dev.x() * scale;
            gate.position_enu.y() = gate.prior_pos_enu.y() + dev.y() * scale;
        }

        // Tightly clamp vertical Z (gate center nominal height is fixed)
        gate.position_enu.z() = std::clamp(gate.position_enu.z(), gate.prior_pos_enu.z() - 0.35, gate.prior_pos_enu.z() + 0.35);
    }

    void publish_refined_poses(const std_msgs::msg::Header &header) {
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = header.stamp;
        msg.header.frame_id = "map";

        for (const auto &gate : gates_) {
            geometry_msgs::msg::Pose p;
            p.position.x = gate.position_enu.x();
            p.position.y = gate.position_enu.y();
            p.position.z = gate.position_enu.z();

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
            box.pose.position.x = gate.position_enu.x();
            box.pose.position.y = gate.position_enu.y();
            box.pose.position.z = gate.position_enu.z();
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
            text.pose.position.x = gate.position_enu.x();
            text.pose.position.y = gate.position_enu.y();
            text.pose.position.z = gate.position_enu.z() + 1.5;
            text.scale.z = 0.5;
            text.color.r = 1.0f;
            text.color.g = 1.0f;
            text.color.b = 1.0f;
            text.color.a = 1.0f;
            text.text = "Gate " + std::to_string(gate.id);
            array.markers.push_back(text);

            // Add Refined Sub-Gates for Gate 3 (1 sub-gate) and Gate 4 (2 sub-gates)
            std::vector<double> sub_offsets;
            if (gate.id == 3) {
                sub_offsets = {1.0};
            } else if (gate.id == 4) {
                sub_offsets = {1.0, 2.0};
            }

            for (size_t sub_k = 0; sub_k < sub_offsets.size(); ++sub_k) {
                double s_off = sub_offsets[sub_k];
                Eigen::Vector3d sub_pos = gate.position_enu + s_off * gate.normal_enu;
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
    double prior_sigma_;
    double drone_sigma_;
    double pnp_sigma_;
    double max_dist_;
    double mahalanobis_max_sq_;
    double max_prior_deviation_;
    double max_tilt_rad_;
    double max_refine_dist_;
    bool publish_initial_pos_;

    // Data structures & matrices
    std::vector<GateState> gates_;
    geometry_msgs::msg::PoseArray initial_poses_msg_;
    visualization_msgs::msg::MarkerArray initial_markers_msg_;
    Eigen::Matrix3d R_cam_to_body_;

    Eigen::Vector3d latest_drone_pos_;
    Eigen::Quaterniond latest_drone_rot_;
    bool drone_pose_received_;

    Eigen::Vector3d initial_drone_pos_;
    Eigen::Quaterniond initial_drone_rot_;
    bool initial_pose_captured_;

    // ROS 2 Comms
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pnp_poses_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pnp_covariances_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_target_gate_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_refined_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_initial_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_initial_markers_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_gates_;
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