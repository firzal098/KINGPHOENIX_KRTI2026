#ifndef POLICY_HPP_
#define POLICY_HPP_

#include <cstddef>
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

// ONNX Runtime C++ API Header
#include <onnxruntime_cxx_api.h>

class Policy {
public:
    Policy()
    : current_gate_target_index_(0),
      current_sub_gate_index_(0),
      current_preview_gate_index_(1),
      current_preview_sub_gate_index_(0),
      prev_dot_product_(1.0),
      has_prev_drone_pos_(false),
      triple_gate_pass_method_(1),
      a_max_(5.6638),
      has_imu_(false),
      onnx_loaded_(false)
    {
        observation_vector_.fill(0.0);
        prev_action_.fill(0.0);
        prev_drone_pos_.fill(0.0);
    }

    /**
     * @brief Initializes Subscribers, Telemetry Publishers, and ONNX Runtime Session.
     * @param node Pointer to parent ROS 2 Node.
     * @param model_path Absolute or package path to policy.onnx.
     */
    void init(rclcpp::Node* node, const std::string& model_path = "models/policy.onnx")
    {
        node_ = node;

        if (node_->has_parameter("triple_gate_pass_method")) {
            triple_gate_pass_method_ = node_->get_parameter("triple_gate_pass_method").as_int();
        }
        if (node_->has_parameter("max_accel")) {
            a_max_ = node_->get_parameter("max_accel").as_double();
        }
        if (node_->has_parameter("enable_gate_1_1")) {
            enable_gate_1_1_ = node_->get_parameter("enable_gate_1_1").as_bool();
        }
        if (node_->has_parameter("max_action_magnitude")) {
            max_action_magnitude_ = node_->get_parameter("max_action_magnitude").as_double();
        }
        if (node_->has_parameter("max_yaw_rate_deg")) {
            setMaxYawRateDeg(node_->get_parameter("max_yaw_rate_deg").as_double());
        }
        if (node_->has_parameter("gate1_straight_servoing")) {
            gate1_straight_servoing_ = node_->get_parameter("gate1_straight_servoing").as_bool();
        }
        if (node_->has_parameter("gate1_servoing_speed")) {
            gate1_servoing_speed_ = node_->get_parameter("gate1_servoing_speed").as_double();
        }
        if (node_->has_parameter("gate1_servoing_kp")) {
            gate1_servoing_kp_ = node_->get_parameter("gate1_servoing_kp").as_double();
        }
        if (node_->has_parameter("gate3_ungrip_delay")) {
            gate3_ungrip_delay_ = node_->get_parameter("gate3_ungrip_delay").as_double();
        }

        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // 1. Subscribe to /estimator/refined_gate_poses (Reliable QoS matching estimator)
        gate_poses_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/estimator/refined_gate_poses", qos_reliable,
            std::bind(&Policy::gatePosesCallback, this, std::placeholders::_1));

        // 2. Subscribe to /mavros/local_position/pose (Best Effort QoS matching MAVROS)
        local_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_best_effort,
            std::bind(&Policy::localPoseCallback, this, std::placeholders::_1));

        // 3. Subscribe to /mavros/local_position/velocity_local
        local_vel_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", qos_best_effort,
            std::bind(&Policy::localVelCallback, this, std::placeholders::_1));

        // 4. Subscribe to /mavros/imu/data for Body Angular Velocity (omega_b in FLU)
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", qos_best_effort,
            std::bind(&Policy::imuCallback, this, std::placeholders::_1));

        // 5. Telemetry Publishers for Observation (42D), Action Space (3D), and Subgate Telemetry
        pub_obs_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/policy/observation", 10);
        pub_action_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/policy/action", 10);
        pub_target_subgate_ = node_->create_publisher<std_msgs::msg::Int32>("/policy/target_subgate", 10);

        // 6. Gripper Async Service Client & Command Publisher
        gripper_client_ = node_->create_client<std_srvs::srv::SetBool>("/gripper/set_state");
        gripper_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/gripper/command", 10);

        // 7. Initialize ONNX Runtime Session
        try {
            env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "PureJaxRL_Policy");
            session_options_ = std::make_unique<Ort::SessionOptions>();
            session_options_->SetIntraOpNumThreads(1);
            session_options_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

            session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), *session_options_);
            memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

            // Retrieve ONNX input/output node names dynamically
            Ort::AllocatorWithDefaultOptions allocator;
            
            auto input_name_ptr = session_->GetInputNameAllocated(0, allocator);
            input_node_name_ = input_name_ptr.get();

            auto output_name_ptr = session_->GetOutputNameAllocated(0, allocator);
            output_node_name_ = output_name_ptr.get();

            onnx_loaded_ = true;
            RCLCPP_INFO(node_->get_logger(), 
                "ONNX Policy model successfully loaded from: '%s'. Input: '%s', Output: '%s'",
                model_path.c_str(), input_node_name_.c_str(), output_node_name_.c_str());
        } 
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load ONNX model file '%s': %s", model_path.c_str(), e.what());
            onnx_loaded_ = false;
        }
    }

    /**
     * @brief Transforms 3D vector from World ENU frame to Drone Body FLU frame (+X Forward, +Y Left, +Z Up).
     * Exact 1:1 match with R^T * [dx, dy, dz]^T in CrazyflowGateEnv.obs() / PureJaxGateEnv.get_obs()
     */
    std::array<double, 3> transformWorldENUtoBodyFLU(double dx, double dy, double dz) const
    {
        double qw = current_local_pose_.pose.orientation.w;
        double qx = current_local_pose_.pose.orientation.x;
        double qy = current_local_pose_.pose.orientation.y;
        double qz = current_local_pose_.pose.orientation.z;

        // Quaternion safety check to handle uninitialized poses before MAVROS connects
        double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (q_norm < 1e-6) {
            qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;
        } else {
            qw /= q_norm; qx /= q_norm; qy /= q_norm; qz /= q_norm;
        }

        double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
        double r01 = 2.0 * (qx * qy - qw * qz);
        double r02 = 2.0 * (qx * qz + qw * qy);

        double r10 = 2.0 * (qx * qy + qw * qz);
        double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
        double r12 = 2.0 * (qy * qz - qw * qx);

        double r20 = 2.0 * (qx * qz - qw * qy);
        double r21 = 2.0 * (qy * qz + qw * qx);
        double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

        // Transpose multiplication R^T * [dx, dy, dz]^T (World ENU -> Body FLU)
        double flu_x = r00 * dx + r10 * dy + r20 * dz;
        double flu_y = r01 * dx + r11 * dy + r21 * dz;
        double flu_z = r02 * dx + r12 * dy + r22 * dz;

        return {flu_x, flu_y, flu_z};
    }

    /**
     * @brief Number of sub-gates for a given main gate.
     * When v7 is true:
     * Gate 3 (2): 3 gates total (1 main + 2 sub-gates)
     * Gate 4 (3): 4 gates total (1 main + 3 sub-gates)
     * Default:
     * Gate 3 (2): 2 gates (1 main + 1 sub-gate)
     * Gate 4 (3): 3 gates (1 main + 2 sub-gates)
     * All others: 1 gate
     */
    size_t getSubgateCount(size_t main_gate_idx) const
    {
        if (main_gate_idx == 0) return enable_gate_1_1_ ? 2 : 1; // Gate 1: 1 main + optional virtual waypoint (+1.0m before Gate 2)
        if (v7_) {
            if (main_gate_idx == 1) return 2; // Gate 2: 1 main + 1 virtual waypoint (+1.5m fwd, +1.0m right)
            if (main_gate_idx == 2) return 3; // Gate 3: 1 main + 2 sub-gates (total 3 gates)
            if (main_gate_idx == 3) return 4; // Gate 4: 1 main + 3 sub-gates (total 4 gates)
            return 1;
        } else {
            if (main_gate_idx == 1) return 2; // Gate 2: 1 main + 1 virtual waypoint (+1.5m fwd, +1.0m right)
            if (main_gate_idx == 2) return 2; // Gate 3: 1 main + 1 sub-gate
            if (main_gate_idx == 3) return 3; // Gate 4: 1 main + 2 sub-gates
            return 1;
        }
    }

    /**
     * @brief Returns relative forward distance offset along the gate normal for a given sub-gate.
     */
    double getSubgateOffset(size_t main_gate_idx, size_t sub_idx) const
    {
        if (v7_) {
            if (main_gate_idx == 2) {
                if (sub_idx == 1) return 1.0;
                if (sub_idx == 2) return 2.5; // v7: Gate 3.1 at 1.0m, Gate 3.2 at 2.5m (1.5m spacing from 3.1)
            } else if (main_gate_idx == 3) {
                // v7: 4 sub gates with 1m distance each (0m, 1m, 2m, 3m)
                if (sub_idx == 1) return 1.0;
                if (sub_idx == 2) return 2.0;
                if (sub_idx == 3) return 3.0;
            }
            return 0.0;
        } else {
            if (main_gate_idx == 2) {
                if (sub_idx == 1) return 1.0;
            } else if (main_gate_idx == 3) {
                if (sub_idx == 1) return 1.0;
                if (sub_idx == 2) return 2.0;
            }
            return 0.0;
        }
    }

    /**
     * @brief Derives the 3D Pose for a specific gate / sub-gate.
     * @param main_gate_idx 0-indexed main gate index.
     * @param sub_idx 0-indexed sub-gate index.
     * @param default_px, default_py, default_pz Fallback position if pose array empty.
     */
    geometry_msgs::msg::Pose getGatePose(size_t main_gate_idx, size_t sub_idx, double default_px = 0.0, double default_py = 0.0, double default_pz = 0.0) const
    {
        geometry_msgs::msg::Pose pose;
        if (!current_gate_poses_.poses.empty() && main_gate_idx < current_gate_poses_.poses.size()) {
            pose = current_gate_poses_.poses[main_gate_idx];
        } else {
            pose.position.x = default_px + 2.0;
            pose.position.y = default_py;
            pose.position.z = default_pz;
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
        }

        if (main_gate_idx == 0 && sub_idx == 1) {
            // Virtual Approach Waypoint: 1.0m before Gate 2 entrance along Gate 2 normal (+1.0*nx)
            geometry_msgs::msg::Pose gate2_pose;
            if (!current_gate_poses_.poses.empty() && 1 < current_gate_poses_.poses.size()) {
                gate2_pose = current_gate_poses_.poses[1];
            } else {
                gate2_pose.position.x = default_px + 2.0;
                gate2_pose.position.y = default_py;
                gate2_pose.position.z = default_pz;
                gate2_pose.orientation.w = 1.0;
                gate2_pose.orientation.x = 0.0;
                gate2_pose.orientation.y = 0.0;
                gate2_pose.orientation.z = 0.0;
            }
            double gw = gate2_pose.orientation.w;
            double gx = gate2_pose.orientation.x;
            double gy = gate2_pose.orientation.y;
            double gz = gate2_pose.orientation.z;
            double g_norm = std::sqrt(gw * gw + gx * gx + gy * gy + gz * gz);
            if (g_norm > 1e-6) {
                gw /= g_norm; gx /= g_norm; gy /= g_norm; gz /= g_norm;
            } else {
                gw = 1.0; gx = 0.0; gy = 0.0; gz = 0.0;
            }
            double nx = 1.0 - 2.0 * (gy * gy + gz * gz);
            double ny = 2.0 * (gx * gy + gw * gz);
            double nz = 2.0 * (gx * gz - gw * gy);

            gate2_pose.position.x += 1.0 * nx;
            gate2_pose.position.y += 1.0 * ny;
            gate2_pose.position.z += 1.0 * nz;
            return gate2_pose;
        }

        if (main_gate_idx == 1 && sub_idx == 1) {
            // Virtual Waypoint: 1.5m past Gate 2 exit along flight path (-nx) and 1.0m to the right (+rx towards Gate 3)
            double gw = pose.orientation.w;
            double gx = pose.orientation.x;
            double gy = pose.orientation.y;
            double gz = pose.orientation.z;
            double g_norm = std::sqrt(gw * gw + gx * gx + gy * gy + gz * gz);
            if (g_norm > 1e-6) {
                gw /= g_norm; gx /= g_norm; gy /= g_norm; gz /= g_norm;
            } else {
                gw = 1.0; gx = 0.0; gy = 0.0; gz = 0.0;
            }
            double nx = 1.0 - 2.0 * (gy * gy + gz * gz);
            double ny = 2.0 * (gx * gy + gw * gz);
            double nz = 2.0 * (gx * gz - gw * gy);

            // Right vector = normal x up (where up is [0, 0, 1]) -> [ny, -nx, 0]
            double rx = ny;
            double ry = -nx;
            double rz = 0.0;

            pose.position.x += -1.5 * nx - 1.0 * rx;
            pose.position.y += -1.5 * ny - 1.0 * ry;
            pose.position.z += -1.5 * nz - 1.0 * rz;
            return pose;
        }

        double offset = getSubgateOffset(main_gate_idx, sub_idx);
        if (std::abs(offset) > 1e-4) {
            // Extract gate normal from quaternion: R_gate * [1, 0, 0]^T
            double gw = pose.orientation.w;
            double gx = pose.orientation.x;
            double gy = pose.orientation.y;
            double gz = pose.orientation.z;
            double g_norm = std::sqrt(gw * gw + gx * gx + gy * gy + gz * gz);
            if (g_norm > 1e-6) {
                gw /= g_norm; gx /= g_norm; gy /= g_norm; gz /= g_norm;
            } else {
                gw = 1.0; gx = 0.0; gy = 0.0; gz = 0.0;
            }
            double nx = 1.0 - 2.0 * (gy * gy + gz * gz);
            double ny = 2.0 * (gx * gy + gw * gz);
            double nz = 2.0 * (gx * gz - gw * gy);

            pose.position.x += offset * nx;
            pose.position.y += offset * ny;
            pose.position.z += offset * nz;
        }

        return pose;
    }

    /**
     * @brief Computes 42D observation space vector precisely matching crazyflow_gate_env.py specification.
     * Layout:
     *   [0:3]   vel_B: Linear velocity in Body FLU frame [v_fwd, v_left, v_up]
     *   [3:6]   grav_B: Projected gravity vector in Body FLU frame [r20, r21, r22]
     *   [6:9]   omega_B: Angular velocity in Body FLU frame [roll_rate, pitch_rate, yaw_rate]
     *   [9:24]  active_gate_15D: Relative active gate/sub-gate corners & center in Body FLU frame [c_tl, c_tr, c_bl, c_br, p_gate]
     *   [24:39] next_gate_15D: Relative next main gate preview corners & center in Body FLU frame (or 0 if last gate)
     *   [39:42] prev_action: Previous 3D control action [v_fwd, v_left, yaw_rate]
     */
    const std::array<double, 42>& get_observation_spaces()
    {
        // 1. Body Linear Velocity in FLU [0:3]
        double vx_W = current_local_vel_.twist.linear.x;
        double vy_W = current_local_vel_.twist.linear.y;
        double vz_W = current_local_vel_.twist.linear.z;
        auto vel_flu = transformWorldENUtoBodyFLU(vx_W, vy_W, vz_W);

        observation_vector_[0] = vel_flu[0]; // v_x^FLU (Forward)
        observation_vector_[1] = vel_flu[1]; // v_y^FLU (Left)
        observation_vector_[2] = vel_flu[2]; // v_z^FLU (Up)

        // 2. Projected Gravity Vector in Body FLU Frame [3:6] (Row 2 of R: [r20, r21, r22])
        double qw = current_local_pose_.pose.orientation.w;
        double qx = current_local_pose_.pose.orientation.x;
        double qy = current_local_pose_.pose.orientation.y;
        double qz = current_local_pose_.pose.orientation.z;

        double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (q_norm < 1e-6) {
            qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;
        } else {
            qw /= q_norm; qx /= q_norm; qy /= q_norm; qz /= q_norm;
        }

        double r20 = 2.0 * (qx * qz - qw * qy);
        double r21 = 2.0 * (qy * qz + qw * qx);
        double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

        observation_vector_[3] = r20; // grav_B.x
        observation_vector_[4] = r21; // grav_B.y
        observation_vector_[5] = r22; // grav_B.z

        // 3. Body Angular Velocity (omega_B) in Body FLU [6:9]
        if (has_imu_) {
            observation_vector_[6] = current_imu_.angular_velocity.x; // Roll rate (p)
            observation_vector_[7] = current_imu_.angular_velocity.y; // Pitch rate (q)
            observation_vector_[8] = current_imu_.angular_velocity.z; // Yaw rate (r)
        } else {
            // Fallback from twist.angular if IMU data not yet received
            double wx_W = current_local_vel_.twist.angular.x;
            double wy_W = current_local_vel_.twist.angular.y;
            double wz_W = current_local_vel_.twist.angular.z;
            auto omega_flu = transformWorldENUtoBodyFLU(wx_W, wy_W, wz_W);
            observation_vector_[6] = omega_flu[0];
            observation_vector_[7] = omega_flu[1];
            observation_vector_[8] = omega_flu[2];
        }

        // Drone current position in World ENU
        double px = current_local_pose_.pose.position.x;
        double py = current_local_pose_.pose.position.y;
        double pz = current_local_pose_.pose.position.z;

        const double half_w = 0.75;
        const double half_h = 0.75;

        // 4. Active Gate 15D Features in Body FLU [9:24] (Targeting currently active sub-gate)
        geometry_msgs::msg::Pose gate_pose = getGatePose(current_gate_target_index_, current_sub_gate_index_, px, py, pz);

        // Active Gate Yaw in World ENU Frame
        double gw = gate_pose.orientation.w;
        double gx = gate_pose.orientation.x;
        double gy = gate_pose.orientation.y;
        double gz = gate_pose.orientation.z;
        double g_norm = std::sqrt(gw * gw + gx * gx + gy * gy + gz * gz);
        if (g_norm > 1e-6) {
            gw /= g_norm; gx /= g_norm; gy /= g_norm; gz /= g_norm;
        } else {
            gw = 1.0; gx = 0.0; gy = 0.0; gz = 0.0;
        }
        double gate_yaw = std::atan2(2.0 * (gw * gz + gx * gy), 1.0 - 2.0 * (gy * gy + gz * gz));

        double lat_x = -std::sin(gate_yaw);
        double lat_y =  std::cos(gate_yaw);
        double lat_z =  0.0;

        double vert_x = 0.0;
        double vert_y = 0.0;
        double vert_z = 1.0;

        // 4 Active Gate Outer Corners in World ENU
        double c_tl_x = gate_pose.position.x - half_w * lat_x + half_h * vert_x;
        double c_tl_y = gate_pose.position.y - half_w * lat_y + half_h * vert_y;
        double c_tl_z = gate_pose.position.z - half_w * lat_z + half_h * vert_z;

        double c_tr_x = gate_pose.position.x + half_w * lat_x + half_h * vert_x;
        double c_tr_y = gate_pose.position.y + half_w * lat_x + half_h * vert_y;
        double c_tr_z = gate_pose.position.z + half_w * lat_z + half_h * vert_z;

        double c_bl_x = gate_pose.position.x - half_w * lat_x - half_h * vert_x;
        double c_bl_y = gate_pose.position.y - half_w * lat_y - half_h * vert_y;
        double c_bl_z = gate_pose.position.z - half_w * lat_z - half_h * vert_z;

        double c_br_x = gate_pose.position.x + half_w * lat_x - half_h * vert_x;
        double c_br_y = gate_pose.position.y + half_w * lat_y - half_h * vert_y;
        double c_br_z = gate_pose.position.z + half_w * lat_z - half_h * vert_z;

        // Transform active gate corners and center into Body FLU frame
        auto c_tl_flu = transformWorldENUtoBodyFLU(c_tl_x - px, c_tl_y - py, c_tl_z - pz);
        auto c_tr_flu = transformWorldENUtoBodyFLU(c_tr_x - px, c_tr_y - py, c_tr_z - pz);
        auto c_bl_flu = transformWorldENUtoBodyFLU(c_bl_x - px, c_bl_y - py, c_bl_z - pz);
        auto c_br_flu = transformWorldENUtoBodyFLU(c_br_x - px, c_br_y - py, c_br_z - pz);
        auto p_gate_flu = transformWorldENUtoBodyFLU(gate_pose.position.x - px, gate_pose.position.y - py, gate_pose.position.z - pz);

        // Top-Left [9:12]
        observation_vector_[9]  = c_tl_flu[0];
        observation_vector_[10] = c_tl_flu[1];
        observation_vector_[11] = c_tl_flu[2];

        // Top-Right [12:15]
        observation_vector_[12] = c_tr_flu[0];
        observation_vector_[13] = c_tr_flu[1];
        observation_vector_[14] = c_tr_flu[2];

        // Bottom-Left [15:18]
        observation_vector_[15] = c_bl_flu[0];
        observation_vector_[16] = c_bl_flu[1];
        observation_vector_[17] = c_bl_flu[2];

        // Bottom-Right [18:21]
        observation_vector_[18] = c_br_flu[0];
        observation_vector_[19] = c_br_flu[1];
        observation_vector_[20] = c_br_flu[2];

        // Relative Gate Center [21:24]
        observation_vector_[21] = p_gate_flu[0];
        observation_vector_[22] = p_gate_flu[1];
        observation_vector_[23] = p_gate_flu[2];

        // 5. Next Gate Preview 15D Features in Body FLU [24:39]
        // Previews next sub-gate ahead (sub-gate k+1) until the last sub-gate, which previews next main gate.
        // For Gate 5 (terminal gate), projects a virtual gate 3.0m in front of Gate 5 along its normal vector.
        size_t preview_main_idx = current_gate_target_index_;
        size_t preview_sub_idx = 0;
        bool has_next = false;
        geometry_msgs::msg::Pose next_gate_pose;

        size_t total_subgates = getSubgateCount(current_gate_target_index_);
        if (v7_ && current_gate_target_index_ == 3) {
            // v7: When Gate 4 is active target, preview is 1 virtual gate 3.0m away from the last sub-gate (+6.0m from Gate 4 entrance)
            preview_main_idx = 3;
            preview_sub_idx = 4; // Virtual Gate (+6.0m)
            has_next = (!current_gate_poses_.poses.empty() && 3 < current_gate_poses_.poses.size());
            if (has_next) {
                geometry_msgs::msg::Pose gate4_pose = getGatePose(3, 0, px, py, pz);
                next_gate_pose = gate4_pose;

                double qw = gate4_pose.orientation.w;
                double qx = gate4_pose.orientation.x;
                double qy = gate4_pose.orientation.y;
                double qz = gate4_pose.orientation.z;
                double g_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
                if (g_norm > 1e-6) {
                    qw /= g_norm; qx /= g_norm; qy /= g_norm; qz /= g_norm;
                } else {
                    qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;
                }
                double nx = 1.0 - 2.0 * (qy * qy + qz * qz);
                double ny = 2.0 * (qx * qy + qw * qz);
                double nz = 2.0 * (qx * qz - qw * qy);

                // 3.0m away from last sub-gate (last sub-gate 3 is at 3.0m, so total 6.0m)
                next_gate_pose.position.x += 6.0 * nx;
                next_gate_pose.position.y += 6.0 * ny;
                next_gate_pose.position.z += 6.0 * nz;
            }
        } else if (current_gate_target_index_ == 3 && triple_gate_pass_method_ == 2) {
            // Method 2: Force preview gate to target next main gate (Gate 5) during Gate 4 active target
            preview_main_idx = current_gate_target_index_ + 1;
            preview_sub_idx = 0;
            has_next = (!current_gate_poses_.poses.empty() && preview_main_idx < current_gate_poses_.poses.size());
            if (has_next) {
                next_gate_pose = getGatePose(preview_main_idx, preview_sub_idx, px, py, pz);
            }
        } else if (current_gate_target_index_ == 3 && triple_gate_pass_method_ == 3) {
            // Method 3: Preview a virtual gate 3.0m in front of Gate 4 main entrance until reaching the last sub-gate, then preview Gate 5
            if (current_sub_gate_index_ < total_subgates - 1) {
                preview_main_idx = 3;
                preview_sub_idx = 3; // Virtual Gate 4 (+3.0m from entrance)
                has_next = (!current_gate_poses_.poses.empty() && 3 < current_gate_poses_.poses.size());
                if (has_next) {
                    geometry_msgs::msg::Pose gate4_pose = getGatePose(3, 0, px, py, pz);
                    next_gate_pose = gate4_pose;

                    double qw = gate4_pose.orientation.w;
                    double qx = gate4_pose.orientation.x;
                    double qy = gate4_pose.orientation.y;
                    double qz = gate4_pose.orientation.z;
                    double g_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
                    if (g_norm > 1e-6) {
                        qw /= g_norm; qx /= g_norm; qy /= g_norm; qz /= g_norm;
                    } else {
                        qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;
                    }
                    double nx = 1.0 - 2.0 * (qy * qy + qz * qz);
                    double ny = 2.0 * (qx * qy + qw * qz);
                    double nz = 2.0 * (qx * qz - qw * qy);

                    next_gate_pose.position.x += 3.0 * nx;
                    next_gate_pose.position.y += 3.0 * ny;
                    next_gate_pose.position.z += 3.0 * nz;
                }
            } else {
                preview_main_idx = 4; // Gate 5
                preview_sub_idx = 0;
                has_next = (!current_gate_poses_.poses.empty() && preview_main_idx < current_gate_poses_.poses.size());
                if (has_next) {
                    next_gate_pose = getGatePose(preview_main_idx, preview_sub_idx, px, py, pz);
                }
            }
        } else if (current_sub_gate_index_ + 1 < total_subgates) {
            // Method 1: Preview immediate next sub-gate in the tunnel
            preview_main_idx = current_gate_target_index_;
            preview_sub_idx = current_sub_gate_index_ + 1;
            has_next = (!current_gate_poses_.poses.empty() && preview_main_idx < current_gate_poses_.poses.size());
            if (has_next) {
                next_gate_pose = getGatePose(preview_main_idx, preview_sub_idx, px, py, pz);
            }
        } else if (current_gate_target_index_ + 1 < current_gate_poses_.poses.size()) {
            preview_main_idx = current_gate_target_index_ + 1;
            preview_sub_idx = 0;
            has_next = true;
            next_gate_pose = getGatePose(preview_main_idx, preview_sub_idx, px, py, pz);
        } else if (current_gate_target_index_ == 4 && !current_gate_poses_.poses.empty()) {
            // Virtual Preview Gate 3.0m in front of Gate 5 along normal
            preview_main_idx = 4;
            preview_sub_idx = 1;
            has_next = true;
            geometry_msgs::msg::Pose gate5_pose = getGatePose(4, 0, px, py, pz);
            next_gate_pose = gate5_pose;

            double qw = gate5_pose.orientation.w;
            double qx = gate5_pose.orientation.x;
            double qy = gate5_pose.orientation.y;
            double qz = gate5_pose.orientation.z;
            double g_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            if (g_norm > 1e-6) {
                qw /= g_norm; qx /= g_norm; qy /= g_norm; qz /= g_norm;
            } else {
                qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;
            }
            double nx = 1.0 - 2.0 * (qy * qy + qz * qz);
            double ny = 2.0 * (qx * qy + qw * qz);
            double nz = 2.0 * (qx * qz - qw * qy);

            next_gate_pose.position.x += 3.0 * nx;
            next_gate_pose.position.y += 3.0 * ny;
            next_gate_pose.position.z += 3.0 * nz;
        }

        current_preview_gate_index_ = has_next ? preview_main_idx : 999;
        current_preview_sub_gate_index_ = has_next ? preview_sub_idx : 0;

        if (has_next) {
            double ngw = next_gate_pose.orientation.w;
            double ngx = next_gate_pose.orientation.x;
            double ngy = next_gate_pose.orientation.y;
            double ngz = next_gate_pose.orientation.z;
            double ng_norm = std::sqrt(ngw * ngw + ngx * ngx + ngy * ngy + ngz * ngz);
            if (ng_norm > 1e-6) {
                ngw /= ng_norm; ngx /= ng_norm; ngy /= ng_norm; ngz /= ng_norm;
            } else {
                ngw = 1.0; ngx = 0.0; ngy = 0.0; ngz = 0.0;
            }

            double next_gate_yaw = std::atan2(2.0 * (ngw * ngz + ngx * ngy), 1.0 - 2.0 * (ngy * ngy + ngz * ngz));

            double nlat_x = -std::sin(next_gate_yaw);
            double nlat_y =  std::cos(next_gate_yaw);
            double nlat_z =  0.0;

            double nvert_x = 0.0;
            double nvert_y = 0.0;
            double nvert_z = 1.0;

            double nc_tl_x = next_gate_pose.position.x - half_w * nlat_x + half_h * nvert_x;
            double nc_tl_y = next_gate_pose.position.y - half_w * nlat_y + half_h * nvert_y;
            double nc_tl_z = next_gate_pose.position.z - half_w * nlat_z + half_h * nvert_z;

            double nc_tr_x = next_gate_pose.position.x + half_w * nlat_x + half_h * nvert_x;
            double nc_tr_y = next_gate_pose.position.y + half_w * nlat_y + half_h * nvert_y;
            double nc_tr_z = next_gate_pose.position.z + half_w * nlat_z + half_h * nvert_z;

            double nc_bl_x = next_gate_pose.position.x - half_w * nlat_x - half_h * nvert_x;
            double nc_bl_y = next_gate_pose.position.y - half_w * nlat_y - half_h * nvert_y;
            double nc_bl_z = next_gate_pose.position.z - half_w * nlat_z - half_h * nvert_z;

            double nc_br_x = next_gate_pose.position.x + half_w * nlat_x - half_h * nvert_x;
            double nc_br_y = next_gate_pose.position.y + half_w * nlat_y - half_h * nvert_y;
            double nc_br_z = next_gate_pose.position.z - half_w * nlat_z - half_h * nvert_z;

            auto nc_tl_flu = transformWorldENUtoBodyFLU(nc_tl_x - px, nc_tl_y - py, nc_tl_z - pz);
            auto nc_tr_flu = transformWorldENUtoBodyFLU(nc_tr_x - px, nc_tr_y - py, nc_tr_z - pz);
            auto nc_bl_flu = transformWorldENUtoBodyFLU(nc_bl_x - px, nc_bl_y - py, nc_bl_z - pz);
            auto nc_br_flu = transformWorldENUtoBodyFLU(nc_br_x - px, nc_br_y - py, nc_br_z - pz);
            auto np_gate_flu = transformWorldENUtoBodyFLU(next_gate_pose.position.x - px, next_gate_pose.position.y - py, next_gate_pose.position.z - pz);

            // Next Gate Top-Left [24:27]
            observation_vector_[24] = nc_tl_flu[0];
            observation_vector_[25] = nc_tl_flu[1];
            observation_vector_[26] = nc_tl_flu[2];

            // Next Gate Top-Right [27:30]
            observation_vector_[27] = nc_tr_flu[0];
            observation_vector_[28] = nc_tr_flu[1];
            observation_vector_[29] = nc_tr_flu[2];

            // Next Gate Bottom-Left [30:33]
            observation_vector_[30] = nc_bl_flu[0];
            observation_vector_[31] = nc_bl_flu[1];
            observation_vector_[32] = nc_bl_flu[2];

            // Next Gate Bottom-Right [33:36]
            observation_vector_[33] = nc_br_flu[0];
            observation_vector_[34] = nc_br_flu[1];
            observation_vector_[35] = nc_br_flu[2];

            // Next Gate Center [36:39]
            observation_vector_[36] = np_gate_flu[0];
            observation_vector_[37] = np_gate_flu[1];
            observation_vector_[38] = np_gate_flu[2];
        } else {
            for (size_t i = 24; i < 39; ++i) {
                observation_vector_[i] = 0.0;
            }
        }

        // 6. Previous Control Action in Body FLU [39:42]
        observation_vector_[39] = prev_action_[0]; // v_fwd_cmd
        observation_vector_[40] = prev_action_[1]; // v_left_cmd
        observation_vector_[41] = prev_action_[2]; // yaw_rate_cmd

        return observation_vector_;
    }

    /**
     * @brief Computes 3D Action Space [v_fwd, v_left, yaw_rate] in Body FLU using ONNX policy.
     * Applies action bound clamping matching training limits.
     */
    std::array<double, 3> get_action_spaces()
    {
        if (!onnx_loaded_ || !session_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "ONNX model not loaded. Returning zero actions.");
            return {0.0, 0.0, 0.0};
        }

        // Convert double 42D observation array to float tensor
        std::array<float, 42> input_tensor_values;
        for (size_t i = 0; i < 42; ++i) {
            input_tensor_values[i] = static_cast<float>(observation_vector_[i]);
        }

        std::array<int64_t, 2> input_shape = {1, 42};

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info_, input_tensor_values.data(), input_tensor_values.size(),
            input_shape.data(), input_shape.size());

        const char* input_names[] = { input_node_name_.c_str() };
        const char* output_names[] = { output_node_name_.c_str() };

        try {
            auto output_tensors = session_->Run(
                Ort::RunOptions{nullptr},
                input_names, &input_tensor, 1,
                output_names, 1);

            float* output_data = output_tensors[0].GetTensorMutableData<float>();

            double raw_vfwd     = static_cast<double>(output_data[0]);
            double raw_vleft    = static_cast<double>(output_data[1]);
            double raw_yaw_rate = static_cast<double>(output_data[2]);

            // Clamp to environment action boundaries [-4..16], [-8..8], [-15.71..15.71]
            prev_action_[0] = std::clamp(raw_vfwd,     -4.0,  16.0);
            prev_action_[1] = std::clamp(raw_vleft,    -8.0,   8.0);
            prev_action_[2] = std::clamp(raw_yaw_rate, -15.707963, 15.707963);

            // When targeting Gate 1 main (idx 0, sub 0), if distance is above 20m, enforce minimum action space magnitude of 10.0
            if (current_gate_target_index_ == 0 && current_sub_gate_index_ == 0) {
                double dist_to_gate = std::sqrt(observation_vector_[21] * observation_vector_[21] +
                                                observation_vector_[22] * observation_vector_[22] +
                                                observation_vector_[23] * observation_vector_[23]);
                if (dist_to_gate > 20.0) {
                    double mag = std::sqrt(prev_action_[0] * prev_action_[0] + prev_action_[1] * prev_action_[1]);
                    if (mag < 10.0) {
                        if (mag > 1e-4) {
                            prev_action_[0] = (prev_action_[0] / mag) * 10.0;
                            prev_action_[1] = (prev_action_[1] / mag) * 10.0;
                        } else {
                            prev_action_[0] = 10.0;
                            prev_action_[1] = 0.0;
                        }
                    }
                }
            }

            // When targeting Gate 3 (idx 2), limit horizontal action magnitude [v_fwd, v_left] to 2.0
            if (current_gate_target_index_ == 2) {
                double mag = std::sqrt(prev_action_[0] * prev_action_[0] + prev_action_[1] * prev_action_[1]);
                if (mag > 2.0) {
                    prev_action_[0] = (prev_action_[0] / mag) * 2.0;
                    prev_action_[1] = (prev_action_[1] / mag) * 2.0;
                }
            }

            // When targeting Gate 5 (idx 4), limit horizontal action magnitude [v_fwd, v_left] to 4.0
            if (current_gate_target_index_ == 4) {
                double mag = std::sqrt(prev_action_[0] * prev_action_[0] + prev_action_[1] * prev_action_[1]);
                if (mag > 4.0) {
                    prev_action_[0] = (prev_action_[0] / mag) * 4.0;
                    prev_action_[1] = (prev_action_[1] / mag) * 4.0;
                }
            }

            // Absolute Maximum Action Limits (Enforced by GCS / user ceiling across all gates)
            // 1. Combined horizontal velocity magnitude [v_fwd, v_left]
            double horiz_mag = std::hypot(prev_action_[0], prev_action_[1]);
            if (horiz_mag > max_action_magnitude_ && horiz_mag > 1e-6) {
                double scale = max_action_magnitude_ / horiz_mag;
                prev_action_[0] *= scale;
                prev_action_[1] *= scale;
            }

            // 2. Absolute Maximum Yaw Rotation Rate
            prev_action_[2] = std::clamp(prev_action_[2], -max_yaw_rate_rad_, max_yaw_rate_rad_);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "ONNX Inference step failed: %s", e.what());
        }

        return prev_action_;
    }

    /**
     * @brief Direct Line-of-Sight & Enforced Proportional Visual Servoing Controller for Gate 1.
     * Computes Body FLU velocity action [v_fwd, v_left, yaw_rate] straight toward Gate 1 center
     * with enforced cross-track lateral correction and bearing alignment.
     */
    std::array<double, 3> compute_gate1_servoing_action()
    {
        double px = current_local_pose_.pose.position.x;
        double py = current_local_pose_.pose.position.y;
        double pz = current_local_pose_.pose.position.z;

        geometry_msgs::msg::Pose gate_pose = getGatePose(0, 0, px, py, pz);

        // Transform gate center offset into Body FLU frame
        auto p_gate_flu = transformWorldENUtoBodyFLU(gate_pose.position.x - px, gate_pose.position.y - py, gate_pose.position.z - pz);
        double d_fwd  = p_gate_flu[0];
        double d_left = p_gate_flu[1];

        // Total 2D horizontal distance to Gate 1 center
        double dist_to_gate = std::hypot(d_fwd, d_left);
        double safe_dist = std::max(0.5, dist_to_gate);

        // 1. Cruise / Approach Speed Selection (slows down to gate1_slow_speed_ at <= gate1_slow_distance_)
        double target_speed = (dist_to_gate <= gate1_slow_distance_) ? gate1_slow_speed_ : gate1_servoing_speed_;
        target_speed = std::clamp(target_speed, 1.0, max_action_magnitude_);

        // 2. Line of Sight (LOS) Vector Projection
        // Directly directs the velocity vector along the straight ray to gate center
        double u_fwd  = std::max(0.0, d_fwd / safe_dist);
        double u_left = d_left / safe_dist;

        double v_fwd_base  = target_speed * u_fwd;
        double v_left_base = target_speed * u_left;

        // 3. Enforced Cross-Track Lateral Correction
        // Adds high-gain lateral proportional feedback (v_left_corr = Kp * d_left)
        // to forcefully pull the drone onto the gate centerline
        double v_left_corr = gate1_servoing_kp_ * d_left;
        double v_left = std::clamp(v_left_base + v_left_corr, -8.0, 8.0);
        double v_fwd  = std::clamp(v_fwd_base, 1.0, max_action_magnitude_);

        // 4. Proportional yaw alignment to gate bearing
        double bearing = std::atan2(d_left, std::max(0.5, d_fwd));
        double yaw_rate = std::clamp(4.0 * bearing, -max_yaw_rate_rad_, max_yaw_rate_rad_);

        // Apply global horizontal magnitude limit
        double horiz_mag = std::hypot(v_fwd, v_left);
        if (horiz_mag > max_action_magnitude_ && horiz_mag > 1e-6) {
            double scale = max_action_magnitude_ / horiz_mag;
            v_fwd *= scale;
            v_left *= scale;
        }

        prev_action_[0] = v_fwd;
        prev_action_[1] = v_left;
        prev_action_[2] = yaw_rate;

        return {v_fwd, v_left, yaw_rate};
    }

    /**
     * @brief Evaluates whether the drone has passed the active target sub-gate plane.
     * Uses robust bidirectional ray-plane intersection and proximity fallback.
     * If active sub-gate is passed, advances to sub-gate k+1; if no more sub-gates left,
     * advances to next main gate.
     */
    void checkGatePassage()
    {
        if (current_gate_poses_.poses.empty() || current_gate_target_index_ >= current_gate_poses_.poses.size()) {
            return;
        }

        double px = current_local_pose_.pose.position.x;
        double py = current_local_pose_.pose.position.y;
        double pz = current_local_pose_.pose.position.z;

        const auto gate_pose = getGatePose(current_gate_target_index_, current_sub_gate_index_, px, py, pz);

        // 1. Extract Gate Normal Vector directly from Pose Quaternion: R_gate * [1, 0, 0]^T
        double gw = gate_pose.orientation.w;
        double gx = gate_pose.orientation.x;
        double gy = gate_pose.orientation.y;
        double gz = gate_pose.orientation.z;
        double g_norm = std::sqrt(gw * gw + gx * gx + gy * gy + gz * gz);
        if (g_norm > 1e-6) {
            gw /= g_norm; gx /= g_norm; gy /= g_norm; gz /= g_norm;
        } else {
            gw = 1.0; gx = 0.0; gy = 0.0; gz = 0.0;
        }

        // Gate unit normal vector in World ENU (forward axis of gate frame)
        double nx = 1.0 - 2.0 * (gy * gy + gz * gz);
        double ny = 2.0 * (gx * gy + gw * gz);
        double nz = 2.0 * (gx * gz - gw * gy);

        double dx = px - gate_pose.position.x;
        double dy = py - gate_pose.position.y;
        double dz = pz - gate_pose.position.z;

        double curr_dot = dx * nx + dy * ny + dz * nz;
        double dist_to_center = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (!has_prev_drone_pos_) {
            prev_drone_pos_ = {px, py, pz};
            prev_dot_product_ = curr_dot;
            has_prev_drone_pos_ = true;
            return;
        }

        // 2. Continuous Ray-Plane Segment Crossing Detection:
        // Segment from p_prev to p_curr crosses the plane if s_prev and s_curr have opposite signs
        double s_prev = prev_dot_product_;
        double s_curr = curr_dot;

        bool sign_flip = (s_prev * s_curr <= 0.0) && (std::abs(s_prev - s_curr) > 1e-4);
        bool passed = false;

        if (sign_flip && dist_to_center < 3.5) {
            // Interpolate exact crossing point
            double denom = s_prev - s_curr;
            double t = (std::abs(denom) > 1e-6) ? std::clamp(s_prev / denom, 0.0, 1.0) : 0.5;
            double cx = prev_drone_pos_[0] + t * (px - prev_drone_pos_[0]) - gate_pose.position.x;
            double cy = prev_drone_pos_[1] + t * (py - prev_drone_pos_[1]) - gate_pose.position.y;
            double cz = prev_drone_pos_[2] + t * (pz - prev_drone_pos_[2]) - gate_pose.position.z;
            double cross_dist = std::sqrt(cx * cx + cy * cy + cz * cz);

            if (cross_dist < 2.5) {
                passed = true;
            }
        }

        // 3. Proximity Fallback: Drone came within 1.2m of gate center and is now flying away
        double vx = current_local_vel_.twist.linear.x;
        double vy = current_local_vel_.twist.linear.y;
        double vz = current_local_vel_.twist.linear.z;
        double radial_vel = (dx * vx + dy * vy + dz * vz); // > 0 means moving away from gate center
        if (dist_to_center < 1.20 && radial_vel > 0.0) {
            passed = true;
        }

        if (passed) {
            size_t total_subgates = getSubgateCount(current_gate_target_index_);

            // Automatically open gripper asynchronously after passing Gate 3.2 (or final sub-gate of Gate 3)
            if (current_gate_target_index_ == 2 && (current_sub_gate_index_ >= 2 || current_sub_gate_index_ == total_subgates - 1)) {
                triggerDelayedGripperOpen();
            }

            if (current_sub_gate_index_ + 1 < total_subgates) {
                current_sub_gate_index_++;
                RCLCPP_INFO(node_->get_logger(), 
                    "Successfully PASSED Gate #%zu Sub-gate #%zu! (Dist: %.2fm). Advancing active target to Sub-gate #%zu.", 
                    current_gate_target_index_ + 1, current_sub_gate_index_, dist_to_center, current_sub_gate_index_ + 1);
            } else {
                current_gate_target_index_++;
                current_sub_gate_index_ = 0;
                RCLCPP_INFO(node_->get_logger(), 
                    "Successfully PASSED Gate #%zu (All sub-gates cleared)! (Dist: %.2fm). Advancing to Gate #%zu.", 
                    current_gate_target_index_, dist_to_center, current_gate_target_index_ + 1);
            }
            has_prev_drone_pos_ = false;
        } else {
            prev_dot_product_ = curr_dot;
            prev_drone_pos_ = {px, py, pz};
        }
    }

    /**
     * @brief Triggers asynchronous opening of gripper after Gate 3.2, with an optional configurable delay.
     * Non-blocking so flight control frequency is never impacted.
     */
    void triggerDelayedGripperOpen()
    {
        double delay_sec = gate3_ungrip_delay_;
        if (delay_sec <= 0.001) {
            openGripperAsync();
            return;
        }

        uint32_t epoch = ++ungrip_epoch_;
        RCLCPP_INFO(node_->get_logger(),
            "⏳ [GRIPPER] Gate 3.2 cleared! Waiting %.2fs before opening gripper (async worker)...", delay_sec);

        std::thread([this, delay_sec, epoch]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(delay_sec * 1000.0)));
            if (ungrip_epoch_ == epoch) {
                openGripperAsync();
            } else {
                RCLCPP_INFO(node_->get_logger(), "⏹️ [GRIPPER] Delayed ungrip cancelled due to reset.");
            }
        }).detach();
    }

    /**
     * @brief Asynchronously opens the gripper via /gripper/set_state service and /gripper/command topic.
     * Non-blocking so real-time flight control loop frequency is never stalled.
     */
    void openGripperAsync()
    {
        RCLCPP_INFO(node_->get_logger(), "🔓 [GRIPPER] Triggering ASYNC Gripper Open after Gate 3.2...");

        // 1. Instant publication to /gripper/command topic
        if (gripper_cmd_pub_) {
            std_msgs::msg::Bool cmd_msg;
            cmd_msg.data = true;
            gripper_cmd_pub_->publish(cmd_msg);
        }

        // 2. Non-blocking asynchronous service request to /gripper/set_state
        if (gripper_client_) {
            if (gripper_client_->service_is_ready()) {
                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req->data = true;

                gripper_client_->async_send_request(
                    req,
                    [logger = node_->get_logger()](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                        try {
                            auto res = future.get();
                            if (res->success) {
                                RCLCPP_INFO(logger, "✅ [GRIPPER] Async open succeeded: %s", res->message.c_str());
                            } else {
                                RCLCPP_WARN(logger, "⚠️ [GRIPPER] Async open response returned false: %s", res->message.c_str());
                            }
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR(logger, "❌ [GRIPPER] Async open service call error: %s", e.what());
                        }
                    });
            } else {
                RCLCPP_INFO(node_->get_logger(), "ℹ️ [GRIPPER] Service /gripper/set_state not ready yet; command sent via topic.");
            }
        }
    }

    /**
     * @brief Computes 42D observation space and publishes to /policy/observation telemetry topic.
     * Can be called anytime (both in RUN and non-RUN states) to keep live telemetry updating.
     */
    void publishObservation()
    {
        get_observation_spaces();
        if (pub_obs_) {
            std_msgs::msg::Float64MultiArray obs_msg;
            obs_msg.data.assign(observation_vector_.begin(), observation_vector_.end());
            pub_obs_->publish(obs_msg);
        }
        if (pub_target_subgate_) {
            std_msgs::msg::Int32 subgate_msg;
            subgate_msg.data = static_cast<int32_t>(current_sub_gate_index_);
            pub_target_subgate_->publish(subgate_msg);
        }
    }

    /**
     * @brief Executes 1 policy step (Obs -> Inference/Servoing -> Gate Check -> Telemetry Pub -> MAVROS PositionTarget).
     * @return mavros_msgs::msg::PositionTarget formatted setpoint for /mavros/setpoint_raw/local
     */
    mavros_msgs::msg::PositionTarget step()
    {
        // 1. Compute 42D Observation Vector & Publish Telemetry
        publishObservation();

        // 2. Compute 3D Action Vector:
        // Use direct visual servoing when targeting Gate 1 main, switch to RL ONNX policy for all subsequent gates
        std::array<double, 3> action;
        if (gate1_straight_servoing_ && current_gate_target_index_ == 0 && current_sub_gate_index_ == 0) {
            action = compute_gate1_servoing_action();
        } else {
            action = get_action_spaces();
        }

        // 3. Publish Action Telemetry (3D) to /policy/action
        if (pub_action_) {
            std_msgs::msg::Float64MultiArray action_msg;
            action_msg.data.assign(action.begin(), action.end());
            pub_action_->publish(action_msg);
        }

        // 4. Evaluate gate passing logic
        checkGatePassage();

        // 5. Construct MAVROS setpoint_raw PositionTarget message in Earth Frame (FRAME_LOCAL_NED)
        mavros_msgs::msg::PositionTarget setpoint;
        setpoint.header.stamp = node_->now();
        setpoint.header.frame_id = "map";
        
        // Earth frame velocity setpoint (MAV_FRAME_LOCAL_NED)
        setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        // Type Mask: Ignore horizontal Position (X, Y), all Acceleration, and Yaw Angle.
        // Enforce closed-loop Altitude Hold at Position Z or fixed velocity.
        setpoint.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                             mavros_msgs::msg::PositionTarget::IGNORE_PY |
                             mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                             mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                             mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::msg::PositionTarget::IGNORE_YAW;

        // Extract yaw heading from current drone orientation
        double qw = current_local_pose_.pose.orientation.w;
        double qx = current_local_pose_.pose.orientation.x;
        double qy = current_local_pose_.pose.orientation.y;
        double qz = current_local_pose_.pose.orientation.z;
        double q_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (q_norm > 1e-6) {
            qw /= q_norm; qx /= q_norm; qy /= q_norm; qz /= q_norm;
        } else {
            qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;
        }
        double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        double cos_y = std::cos(yaw);
        double sin_y = std::sin(yaw);

        // Rotate horizontal action [v_fwd, v_left] into World ENU frame (1:1 match with training step in crazyflow_gate_env.py)
        double vx_world = cos_y * action[0] - sin_y * action[1];
        double vy_world = sin_y * action[0] + cos_y * action[1];

        // Target nominal gate center (z = 1.0m)
        double target_z = 1.0;
        if (!current_gate_poses_.poses.empty() && current_gate_target_index_ < current_gate_poses_.poses.size()) {
            double gz = current_gate_poses_.poses[current_gate_target_index_].position.z;
            if (gz > 0.3 && gz < 3.5) {
                target_z = gz;
            }
        }

        // Initialize filter state from current velocity if not yet set
        if (!has_filtered_vel_) {
            filtered_vx_ = current_local_vel_.twist.linear.x;
            filtered_vy_ = current_local_vel_.twist.linear.y;
            filtered_vz_ = current_local_vel_.twist.linear.z;
            has_filtered_vel_ = true;
        }

        // 7. Acceleration Slew-Rate Limiter (a_max parameter)
        // Prevents excessive pitch tilt that causes vertical lift loss
        const double dt = 0.02; // 50 Hz control period
        const double max_dv = a_max_ * dt;

        double dvx = vx_world - filtered_vx_;
        double dvy = vy_world - filtered_vy_;
        double dv_norm = std::hypot(dvx, dvy);
        if (dv_norm > max_dv) {
            filtered_vx_ += (dvx / dv_norm) * max_dv;
            filtered_vy_ += (dvy / dv_norm) * max_dv;
        } else {
            filtered_vx_ = vx_world;
            filtered_vy_ = vy_world;
        }

        setpoint.position.z = target_z;
        setpoint.velocity.x = filtered_vx_;
        setpoint.velocity.y = filtered_vy_;
        setpoint.velocity.z = 0.0;
        setpoint.yaw_rate = action[2];

        return setpoint;
    }

    /**
     * @brief Resets policy internal state, actuator filters, and gate trackers.
     */
    void reset()
    {
        has_filtered_vel_ = false;
        filtered_vx_ = 0.0;
        filtered_vy_ = 0.0;
        filtered_vz_ = 0.0;
        prev_action_ = {0.0, 0.0, 0.0};
        observation_vector_.fill(0.0);
        has_prev_drone_pos_ = false;
        prev_drone_pos_.fill(0.0);
        current_gate_target_index_ = 0;
        current_sub_gate_index_ = 0;
        prev_dot_product_ = 1.0;
        ungrip_epoch_++;
    }

    // Getters and Setters
    bool isAllGatesCleared() const
    {
        if (current_gate_poses_.poses.empty()) return false;
        return current_gate_target_index_ >= current_gate_poses_.poses.size();
    }

    size_t getTargetGateIndex() const { return current_gate_target_index_; }
    size_t getTargetSubGateIndex() const { return current_sub_gate_index_; }
    size_t getPreviewGateIndex() const { return current_preview_gate_index_; }
    size_t getPreviewSubGateIndex() const { return current_preview_sub_gate_index_; }
    void setTargetGateIndex(size_t index, size_t sub_index = 0) {
        current_gate_target_index_ = index;
        current_sub_gate_index_ = sub_index;
        has_prev_drone_pos_ = false;
        prev_dot_product_ = 1.0;
    }
    void setTripleGatePassMethod(int method) { triple_gate_pass_method_ = method; }
    int getTripleGatePassMethod() const { return triple_gate_pass_method_; }
    void setMaxAccel(double a_max) { a_max_ = a_max; }
    double getMaxAccel() const { return a_max_; }
    void setV7(bool v7) { v7_ = v7; }
    bool getV7() const { return v7_; }
    void setEnableGate1_1(bool enable) { enable_gate_1_1_ = enable; }
    bool getEnableGate1_1() const { return enable_gate_1_1_; }
    void setMaxActionMagnitude(double max_mag) { max_action_magnitude_ = std::clamp(max_mag, 1.0, 16.0); }
    double getMaxActionMagnitude() const { return max_action_magnitude_; }
    void setMaxYawRateRad(double max_yaw_rate) { max_yaw_rate_rad_ = std::clamp(max_yaw_rate, 20.0 * M_PI / 180.0, 360.0 * M_PI / 180.0); }
    void setMaxYawRateDeg(double max_yaw_deg) { setMaxYawRateRad(max_yaw_deg * M_PI / 180.0); }
    double getMaxYawRateRad() const { return max_yaw_rate_rad_; }
    double getMaxYawRateDeg() const { return max_yaw_rate_rad_ * 180.0 / M_PI; }
    void setGate1StraightServoing(bool enable) { gate1_straight_servoing_ = enable; }
    bool getGate1StraightServoing() const { return gate1_straight_servoing_; }
    void setGate1ServoingSpeed(double speed) { gate1_servoing_speed_ = std::clamp(speed, 2.0, 16.0); }
    double getGate1ServoingSpeed() const { return gate1_servoing_speed_; }
    void setGate1ServoingKp(double kp) { gate1_servoing_kp_ = std::clamp(kp, 0.1, 10.0); }
    double getGate1ServoingKp() const { return gate1_servoing_kp_; }
    void setGate1SlowDistance(double dist) { gate1_slow_distance_ = std::max(0.0, dist); }
    double getGate1SlowDistance() const { return gate1_slow_distance_; }
    void setGate1SlowSpeed(double speed) { gate1_slow_speed_ = std::clamp(speed, 1.0, 16.0); }
    double getGate1SlowSpeed() const { return gate1_slow_speed_; }
    void setGate3UngripDelay(double delay) { gate3_ungrip_delay_ = std::max(0.0, delay); }
    double getGate3UngripDelay() const { return gate3_ungrip_delay_; }
    const std::array<double, 42>& getObservationVector() const { return observation_vector_; }

private:
    void gatePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        current_gate_poses_ = *msg;
    }

    void localPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_local_pose_ = *msg;
    }

    void localVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        current_local_vel_ = *msg;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        current_imu_ = *msg;
        has_imu_ = true;
    }

    rclcpp::Node* node_{nullptr};
    size_t current_gate_target_index_{0};
    size_t current_sub_gate_index_{0};
    size_t current_preview_gate_index_{1};
    size_t current_preview_sub_gate_index_{0};
    double prev_dot_product_{1.0};
    bool has_prev_drone_pos_{false};
    std::array<double, 3> prev_drone_pos_{};
    int triple_gate_pass_method_{1};
    double a_max_{5.6638};
    bool v7_{false};
    bool enable_gate_1_1_{true};
    double max_action_magnitude_{16.0};
    double max_yaw_rate_rad_{2.0 * M_PI};
    bool gate1_straight_servoing_{true};
    double gate1_servoing_speed_{12.0};
    double gate1_servoing_kp_{4.0};
    double gate1_slow_distance_{15.0};
    double gate1_slow_speed_{3.0};

    double filtered_vx_{0.0};
    double filtered_vy_{0.0};
    double filtered_vz_{0.0};
    bool has_filtered_vel_{false};

    geometry_msgs::msg::PoseArray current_gate_poses_;
    geometry_msgs::msg::PoseStamped current_local_pose_;
    geometry_msgs::msg::TwistStamped current_local_vel_;
    sensor_msgs::msg::Imu current_imu_;
    bool has_imu_{false};

    std::array<double, 42> observation_vector_{};
    std::array<double, 3> prev_action_{};

    // ONNX Runtime Session Members
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo memory_info_{nullptr};
    std::string input_node_name_;
    std::string output_node_name_;
    bool onnx_loaded_{false};

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_poses_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_action_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_target_subgate_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripper_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_pub_;
    double gate3_ungrip_delay_{0.0};
    std::atomic<uint32_t> ungrip_epoch_{0};
};

#endif // POLICY_HPP_