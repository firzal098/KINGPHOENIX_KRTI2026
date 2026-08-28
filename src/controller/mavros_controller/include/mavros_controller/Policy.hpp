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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// ONNX Runtime C++ API Header
#include <onnxruntime_cxx_api.h>

class Policy {
public:
    Policy()
    : current_gate_target_index_(0),
      prev_dot_product_(1.0),
      has_prev_dot_(false),
      onnx_loaded_(false)
    {
        observation_vector_.fill(0.0);
        prev_action_.fill(0.0);
    }

    /**
     * @brief Initializes Subscribers, Telemetry Publishers, and ONNX Runtime Session.
     * @param node Pointer to parent ROS 2 Node.
     * @param model_path Absolute or package path to policy.onnx.
     */
    void init(rclcpp::Node* node, const std::string& model_path = "models/policy.onnx")
    {
        node_ = node;
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

        // 4. Telemetry Publishers for Observation (28D) and Action Space (4D)
        pub_obs_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/policy/observation", 10);
        pub_action_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/policy/action", 10);

        // 5. Initialize ONNX Runtime Session
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
     * @brief Computes 40D observation space vector precisely matching crazyflow_gate_env.py specification.
     * Layout:
     *   [0:3]   vel_B: Linear velocity in Body FLU frame [v_fwd, v_left, v_up]
     *   [3:6]   grav_B: Projected gravity vector in Body FLU frame [r20, r21, r22]
     *   [6:21]  active_gate_15D: Relative active gate corners & center in Body FLU frame [c_tl, c_tr, c_bl, c_br, p_gate]
     *   [21:36] next_gate_15D: Relative next gate preview corners & center in Body FLU frame (or 0 if last gate)
     *   [36:40] prev_action: Previous 4D control action [v_fwd, v_left, v_up, yaw_rate]
     */
    const std::array<double, 40>& get_observation_spaces()
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

        // Drone current position in World ENU
        double px = current_local_pose_.pose.position.x;
        double py = current_local_pose_.pose.position.y;
        double pz = current_local_pose_.pose.position.z;

        const double half_w = 0.75;
        const double half_h = 0.75;

        // 3. Active Gate 15D Features in Body FLU [6:21]
        geometry_msgs::msg::Pose gate_pose;
        if (!current_gate_poses_.poses.empty() && current_gate_target_index_ < current_gate_poses_.poses.size()) {
            gate_pose = current_gate_poses_.poses[current_gate_target_index_];
        } else {
            gate_pose.position.x = px + 2.0;
            gate_pose.position.y = py;
            gate_pose.position.z = pz;
            gate_pose.orientation.w = 1.0;
            gate_pose.orientation.x = 0.0;
            gate_pose.orientation.y = 0.0;
            gate_pose.orientation.z = 0.0;
        }

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
        double c_tr_y = gate_pose.position.y + half_w * lat_y + half_h * vert_y;
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

        // Top-Left [6:9]
        observation_vector_[6]  = c_tl_flu[0];
        observation_vector_[7]  = c_tl_flu[1];
        observation_vector_[8]  = c_tl_flu[2];

        // Top-Right [9:12]
        observation_vector_[9]  = c_tr_flu[0];
        observation_vector_[10] = c_tr_flu[1];
        observation_vector_[11] = c_tr_flu[2];

        // Bottom-Left [12:15]
        observation_vector_[12] = c_bl_flu[0];
        observation_vector_[13] = c_bl_flu[1];
        observation_vector_[14] = c_bl_flu[2];

        // Bottom-Right [15:18]
        observation_vector_[15] = c_br_flu[0];
        observation_vector_[16] = c_br_flu[1];
        observation_vector_[17] = c_br_flu[2];

        // Relative Gate Center [18:21]
        observation_vector_[18] = p_gate_flu[0];
        observation_vector_[19] = p_gate_flu[1];
        observation_vector_[20] = p_gate_flu[2];

        // 4. Next Gate Preview 15D Features in Body FLU [21:36]
        size_t next_gate_idx = current_gate_target_index_ + 1;
        bool has_next = (!current_gate_poses_.poses.empty() && next_gate_idx < current_gate_poses_.poses.size());

        if (has_next) {
            const auto& next_gate_pose = current_gate_poses_.poses[next_gate_idx];

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
            double nc_br_z = next_gate_pose.position.z + half_w * nlat_z - half_h * nvert_z;

            auto nc_tl_flu = transformWorldENUtoBodyFLU(nc_tl_x - px, nc_tl_y - py, nc_tl_z - pz);
            auto nc_tr_flu = transformWorldENUtoBodyFLU(nc_tr_x - px, nc_tr_y - py, nc_tr_z - pz);
            auto nc_bl_flu = transformWorldENUtoBodyFLU(nc_bl_x - px, nc_bl_y - py, nc_bl_z - pz);
            auto nc_br_flu = transformWorldENUtoBodyFLU(nc_br_x - px, nc_br_y - py, nc_br_z - pz);
            auto np_gate_flu = transformWorldENUtoBodyFLU(next_gate_pose.position.x - px, next_gate_pose.position.y - py, next_gate_pose.position.z - pz);

            // Next Gate Top-Left [21:24]
            observation_vector_[21] = nc_tl_flu[0];
            observation_vector_[22] = nc_tl_flu[1];
            observation_vector_[23] = nc_tl_flu[2];

            // Next Gate Top-Right [24:27]
            observation_vector_[24] = nc_tr_flu[0];
            observation_vector_[25] = nc_tr_flu[1];
            observation_vector_[26] = nc_tr_flu[2];

            // Next Gate Bottom-Left [27:30]
            observation_vector_[27] = nc_bl_flu[0];
            observation_vector_[28] = nc_bl_flu[1];
            observation_vector_[29] = nc_bl_flu[2];

            // Next Gate Bottom-Right [30:33]
            observation_vector_[30] = nc_br_flu[0];
            observation_vector_[31] = nc_br_flu[1];
            observation_vector_[32] = nc_br_flu[2];

            // Next Gate Center [33:36]
            observation_vector_[33] = np_gate_flu[0];
            observation_vector_[34] = np_gate_flu[1];
            observation_vector_[35] = np_gate_flu[2];
        } else {
            for (size_t i = 21; i < 36; ++i) {
                observation_vector_[i] = 0.0;
            }
        }

        // 5. Previous Control Action in Body FLU [36:40]
        observation_vector_[36] = prev_action_[0]; // v_fwd_cmd
        observation_vector_[37] = prev_action_[1]; // v_left_cmd
        observation_vector_[38] = prev_action_[2]; // v_up_cmd
        observation_vector_[39] = prev_action_[3]; // yaw_rate_cmd

        return observation_vector_;
    }

    /**
     * @brief Computes 4D Action Space [v_fwd, v_left, v_up, yaw_rate] in Body FLU using ONNX policy.
     * Applies action bound clamping matching training limits.
     */
    std::array<double, 4> get_action_spaces()
    {
        if (!onnx_loaded_ || !session_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "ONNX model not loaded. Returning zero actions.");
            return {0.0, 0.0, 0.0, 0.0};
        }

        // Convert double 40D observation array to float tensor
        std::array<float, 40> input_tensor_values;
        for (size_t i = 0; i < 40; ++i) {
            input_tensor_values[i] = static_cast<float>(observation_vector_[i]);
        }

        std::array<int64_t, 2> input_shape = {1, 40};

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
            double raw_vup      = static_cast<double>(output_data[2]);
            double raw_yaw_rate = static_cast<double>(output_data[3]);

            // Clamp to environment action boundaries [-4..16], [-8..8], [-6..6], [-9.42..9.42]
            prev_action_[0] = std::clamp(raw_vfwd,     -4.0,  16.0);
            prev_action_[1] = std::clamp(raw_vleft,    -8.0,   8.0);
            prev_action_[2] = std::clamp(raw_vup,      -6.0,   6.0);
            prev_action_[3] = std::clamp(raw_yaw_rate, -9.424778, 9.424778);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "ONNX Inference step failed: %s", e.what());
        }

        return prev_action_;
    }

    /**
     * @brief Evaluates whether the drone has passed the active target gate plane using dot product.
     */
    void checkGatePassage()
    {
        if (current_gate_poses_.poses.empty() || current_gate_target_index_ >= current_gate_poses_.poses.size()) {
            return;
        }

        const auto& gate_pose = current_gate_poses_.poses[current_gate_target_index_];

        // Gate Yaw & Normal Vector n_gate in World ENU Frame
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

        // Entrance normal vector n_gate = [-cos(yaw), -sin(yaw), 0]
        double nx = -std::cos(gate_yaw);
        double ny = -std::sin(gate_yaw);
        double nz = 0.0;

        // Drone position relative to target gate center
        double dx = current_local_pose_.pose.position.x - gate_pose.position.x;
        double dy = current_local_pose_.pose.position.y - gate_pose.position.y;
        double dz = current_local_pose_.pose.position.z - gate_pose.position.z;

        // Dot product representing signed plane distance
        double curr_dot = dx * nx + dy * ny + dz * nz;
        double dist_to_center = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (!has_prev_dot_) {
            prev_dot_product_ = curr_dot;
            has_prev_dot_ = true;
            return;
        }

        // Plane crossing condition: dot product transitions from positive (front) to negative (behind)
        if (prev_dot_product_ > 0.0 && curr_dot <= 0.0 && dist_to_center < 3.5) {
            RCLCPP_INFO(node_->get_logger(), 
                "Successfully PASSED Gate #%zu! Advancing target to next gate.", current_gate_target_index_);
            current_gate_target_index_++;
            prev_dot_product_ = 1.0;
            has_prev_dot_ = false; // Reset first-frame latch for next gate
        } else {
            prev_dot_product_ = curr_dot;
        }
    }

    /**
     * @brief Executes 1 policy step (Obs -> Inference -> Gate Check -> Telemetry Pub -> MAVROS PositionTarget).
     * @return mavros_msgs::msg::PositionTarget formatted setpoint for /mavros/setpoint_raw/local
     */
    mavros_msgs::msg::PositionTarget step()
    {
        // 1. Compute 40D Observation Vector
        get_observation_spaces();

        // 2. Publish Observation Telemetry (40D) to /policy/observation
        if (pub_obs_ && pub_obs_->get_subscription_count() > 0) {
            std_msgs::msg::Float64MultiArray obs_msg;
            obs_msg.data.assign(observation_vector_.begin(), observation_vector_.end());
            pub_obs_->publish(obs_msg);
        }

        // 3. Compute 4D Action Vector via ONNX model (in Body FLU)
        auto action = get_action_spaces();

        // 4. Publish Action Telemetry (4D) to /policy/action
        if (pub_action_ && pub_action_->get_subscription_count() > 0) {
            std_msgs::msg::Float64MultiArray action_msg;
            action_msg.data.assign(action.begin(), action.end());
            pub_action_->publish(action_msg);
        }

        // 5. Evaluate gate passing logic
        checkGatePassage();

        // 6. Construct MAVROS setpoint_raw PositionTarget message in Earth Frame (FRAME_LOCAL_NED)
        mavros_msgs::msg::PositionTarget setpoint;
        setpoint.header.stamp = node_->now();
        setpoint.header.frame_id = "map";
        
        // Earth frame velocity setpoint (MAV_FRAME_LOCAL_NED)
        setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

        // Type Mask: Ignore horizontal Position (X, Y), all Acceleration, and Yaw Angle.
        // Enforce closed-loop Altitude Hold at Position Z (target_z = 1.05m / gate center height).
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
            if (gz >= 0.5 && gz <= 2.5) {
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

        // 7. Acceleration Slew-Rate Limiter (a_max = 5.6638 m/s^2 from crazyflow_gate_env.py)
        // Prevents excessive pitch tilt that causes vertical lift loss
        const double dt = 0.02; // 50 Hz control period
        const double a_max = 5.6638; // Maximum acceleration [m/s^2]
        const double max_dv = a_max * dt; // 0.1133 m/s max velocity change per step

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

        // Commanded targets streamed to MAVROS
        // setpoint.position.z = target_z;just disabling this temporarily
        setpoint.velocity.x = filtered_vx_;
        setpoint.velocity.y = filtered_vy_;
        // setpoint.velocity.z = action[2]; just disabling this temporarily
        setpoint.velocity.z = 0;

        // Yaw rate (CCW positive in FLU/ENU)
        setpoint.yaw_rate = action[3];

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
        prev_action_ = {0.0, 0.0, 0.0, 0.0};
        has_prev_dot_ = false;
    }

    // Getters and Setters
    size_t getTargetGateIndex() const { return current_gate_target_index_; }
    void setTargetGateIndex(size_t index) {
        current_gate_target_index_ = index;
        has_prev_dot_ = false;
        prev_dot_product_ = 1.0;
    }
    const std::array<double, 40>& getObservationVector() const { return observation_vector_; }

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

    rclcpp::Node* node_{nullptr};
    size_t current_gate_target_index_{0};
    double prev_dot_product_{1.0};
    bool has_prev_dot_{false};

    double filtered_vx_{0.0};
    double filtered_vy_{0.0};
    double filtered_vz_{0.0};
    bool has_filtered_vel_{false};

    geometry_msgs::msg::PoseArray current_gate_poses_;
    geometry_msgs::msg::PoseStamped current_local_pose_;
    geometry_msgs::msg::TwistStamped current_local_vel_;

    std::array<double, 40> observation_vector_{};
    std::array<double, 4> prev_action_{};

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

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_action_;
};

#endif // POLICY_HPP_