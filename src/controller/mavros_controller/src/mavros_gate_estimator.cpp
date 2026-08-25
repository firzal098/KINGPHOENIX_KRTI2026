#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
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
    Eigen::Matrix3d covariance;     // 3x3 error covariance matrix
    Eigen::Vector3d normal_enu;     // Gate normal vector in ENU frame
    Eigen::Quaterniond orientation; // Orientation quaternion in ENU
};

class MavrosGateEstimator : public rclcpp::Node {
public:
    MavrosGateEstimator() : Node("mavros_gate_estimator"), drone_pose_received_(false), initial_pose_captured_(false) {
        // Declare tunable parameters
        this->declare_parameter<double>("gate_prior_sigma", 1.5);
        this->declare_parameter<double>("drone_pose_sigma", 2.0);
        this->declare_parameter<double>("pnp_vision_sigma", 4.0);
        this->declare_parameter<double>("association_max_dist", 6.0);
        
        // Chi-squared 3 DOF gating threshold (11.345 = 99% confidence interval)
        this->declare_parameter<double>("mahalanobis_thresh_sq", 11.345);

        // New parameter: Toggle publishing unrefined static initial gate poses
        this->declare_parameter<bool>("publish_initial_pos", true);

        prior_sigma_         = this->get_parameter("gate_prior_sigma").as_double();
        drone_sigma_         = this->get_parameter("drone_pose_sigma").as_double();
        pnp_sigma_           = this->get_parameter("pnp_vision_sigma").as_double();
        max_dist_            = this->get_parameter("association_max_dist").as_double();
        mahalanobis_max_sq_  = this->get_parameter("mahalanobis_thresh_sq").as_double();
        publish_initial_pos_ = this->get_parameter("publish_initial_pos").as_bool();

        // Initialize zero offset defaults until first MAVROS pose is received
        initial_drone_pos_ = Eigen::Vector3d::Zero();
        initial_drone_rot_ = Eigen::Quaterniond::Identity();

        // Initialize default gate priors
        initialize_gate_priors();

        // Define Camera Optical (RDF) to Drone Body (FLU) rotation matrix
        // FLU: +X Forward, +Y Left, +Z Up
        // RDF: +X Right, +Y Down, +Z Forward (Depth)
        R_cam_to_body_ <<  0.0,  0.0,  1.0,
                          -1.0,  0.0,  0.0,
                           0.0, -1.0,  0.0;

        // Use SensorDataQoS (Best Effort Reliability) to match MAVROS & Perception node publishers
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

        RCLCPP_INFO(
            this->get_logger(),
            "MAVROS Gate Estimator Node Initialized (Mahalanobis Threshold Sq: %.3f, Publish Initial Poses: %s).",
            mahalanobis_max_sq_, publish_initial_pos_ ? "ENABLED" : "DISABLED"
        );
    }

private:
    void initialize_gate_priors() {
        gates_.clear();
        initial_poses_msg_.poses.clear();
        initial_markers_msg_.markers.clear();

        // Raw gate priors provided in RDF relative to drone initialization point
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

            // Raw conversion from RDF relative to drone body ENU frame:
            // X_rel_enu = Z_rdf (Forward)
            // Y_rel_enu = -X_rdf (Left)
            // Z_rel_enu = -Y_rdf (Up)
            Eigen::Vector3d rel_pos_enu(p.z_rdf, -p.x_rdf, -p.y_rdf);
            Eigen::Vector3d rel_norm_enu(p.nz_rdf, -p.nx_rdf, -p.ny_rdf);

            // Add captured initial drone pose offset to anchor ENU gate positions
            state.position_enu = initial_drone_pos_ + (initial_drone_rot_ * rel_pos_enu);
            state.normal_enu   = (initial_drone_rot_ * rel_norm_enu).normalized();

            // Initial 3x3 diagonal covariance matrix P_0
            state.covariance = Eigen::Matrix3d::Identity() * initial_var;

            // Calculate rotation quaternion orienting the gate's normal vector in ENU
            Eigen::Vector3d default_facing(1.0, 0.0, 0.0);
            state.orientation = Eigen::Quaterniond::FromTwoVectors(default_facing, state.normal_enu);

            gates_.push_back(state);

            // Construct static Pose message for initial unrefined position
            geometry_msgs::msg::Pose initial_pose;
            initial_pose.position.x = state.position_enu.x();
            initial_pose.position.y = state.position_enu.y();
            initial_pose.position.z = state.position_enu.z();
            initial_pose.orientation.x = state.orientation.x();
            initial_pose.orientation.y = state.orientation.y();
            initial_pose.orientation.z = state.orientation.z();
            initial_pose.orientation.w = state.orientation.w();
            initial_poses_msg_.poses.push_back(initial_pose);

            // Construct static visual Marker message (Amber/Orange) for initial unrefined position
            visualization_msgs::msg::Marker init_box;
            init_box.ns = "initial_gate_boxes";
            init_box.id = state.id;
            init_box.type = visualization_msgs::msg::Marker::CUBE;
            init_box.action = visualization_msgs::msg::Marker::ADD;
            init_box.pose = initial_pose;
            init_box.scale.x = 0.08;
            init_box.scale.y = 2.0;
            init_box.scale.z = 2.0;
            init_box.color.r = 1.0f; // Orange / Amber color for unrefined priors
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

            RCLCPP_INFO(
                this->get_logger(),
                "Initialized Gate %d -> Anchored ENU Pos: [%.2f, %.2f, %.2f], Normal: [%.2f, %.2f, %.2f]",
                state.id, state.position_enu.x(), state.position_enu.y(), state.position_enu.z(),
                state.normal_enu.x(), state.normal_enu.y(), state.normal_enu.z()
            );
        }
    }

    void handle_reset_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        initialize_gate_priors();
        response->success = true;
        response->message = "Gate positions and error covariances reset to initial anchored priors.";
        RCLCPP_INFO(this->get_logger(), "All gate states successfully reset.");
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

        // Capture first received MAVROS pose as reference offset point and re-anchor gate priors
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

        const Eigen::Matrix3d R_drone = latest_drone_rot_.toRotationMatrix();
        const Eigen::Matrix3d R_total = R_drone * R_cam_to_body_;

        // Camera PnP covariance in optical frame
        const Eigen::Matrix3d R_pnp_cam = Eigen::Matrix3d::Identity() * (pnp_sigma_ * pnp_sigma_);
        
        // Transform PnP covariance to world ENU frame: R_world = R_total * R_pnp_cam * R_total^T
        const Eigen::Matrix3d R_pnp_world = R_total * R_pnp_cam * R_total.transpose();
        
        // Drone position uncertainty covariance
        const Eigen::Matrix3d R_drone_pos = Eigen::Matrix3d::Identity() * (drone_sigma_ * drone_sigma_);

        // Total measurement error covariance R_meas
        const Eigen::Matrix3d R_meas = R_pnp_world + R_drone_pos;

        for (const auto &pnp_pose : msg->poses) {
            Eigen::Vector3d p_pnp_rdf(pnp_pose.position.x, pnp_pose.position.y, pnp_pose.position.z);

            // Compute global gate position measurement z in ENU frame
            Eigen::Vector3d z_meas = latest_drone_pos_ + (R_total * p_pnp_rdf);

            // Data Association using Mahalanobis Distance
            int best_idx = -1;
            double min_mahalanobis_sq = std::numeric_limits<double>::max();
            double corresponding_euc_dist = 0.0;

            for (size_t i = 0; i < gates_.size(); ++i) {
                Eigen::Vector3d y = z_meas - gates_[i].position_enu;    // Innovation residual
                Eigen::Matrix3d S = gates_[i].covariance + R_meas;       // Innovation covariance
                
                // Mahalanobis distance squared: D_M^2 = y^T * S^-1 * y
                double m_dist_sq = y.transpose() * S.inverse() * y;
                double euc_dist  = y.norm();

                // Check Euclidean max distance fallback as well
                if (m_dist_sq < min_mahalanobis_sq && euc_dist <= max_dist_) {
                    min_mahalanobis_sq = m_dist_sq;
                    corresponding_euc_dist = euc_dist;
                    best_idx = static_cast<int>(i);
                }
            }

            // Perform Kalman update if measurement falls within Chi-Square Mahalanobis threshold
            if (best_idx != -1 && min_mahalanobis_sq <= mahalanobis_max_sq_) {
                update_gate_kalman(gates_[best_idx], z_meas, R_meas, min_mahalanobis_sq);
            } else if (best_idx != -1) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Gate candidate %d rejected by Mahalanobis check (D_M^2 = %.2f > %.2f, Euc Dist = %.2fm)",
                    gates_[best_idx].id, min_mahalanobis_sq, mahalanobis_max_sq_, corresponding_euc_dist
                );
            }
        }

        // Publish updated refined results
        publish_refined_poses(msg->header);
        publish_rviz_markers(msg->header);

        // Publish static unrefined initial gate poses if enabled
        if (publish_initial_pos_) {
            publish_initial_poses(msg->header);
        }
    }

    void update_gate_kalman(GateState &gate, const Eigen::Vector3d &z_meas, const Eigen::Matrix3d &R_meas, [[maybe_unused]] double mahalanobis_sq) {
        const Eigen::Vector3d y = z_meas - gate.position_enu;    // Innovation residual
        const Eigen::Matrix3d S = gate.covariance + R_meas;       // Innovation covariance
        const Eigen::Matrix3d K = gate.covariance * S.inverse(); // Kalman Gain

        // State & Covariance Update
        gate.position_enu += K * y;
        gate.covariance = (Eigen::Matrix3d::Identity() - K) * gate.covariance;

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Updated Gate %d -> Refined ENU Pos: [%.2f, %.2f, %.2f], Pos Uncertainty std_x: %.2fm, D_M^2: %.2f",
        //     gate.id, gate.position_enu.x(), gate.position_enu.y(), gate.position_enu.z(),
        //     std::sqrt(gate.covariance(0,0)), mahalanobis_sq
        // );
    }

    void publish_refined_poses(const std_msgs::msg::Header &header) {
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = header.stamp;
        msg.header.frame_id = "map"; // Standard ENU world frame

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
            // Gate Visual Box
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
            box.scale.x = 0.1; // Frame thickness
            box.scale.y = 2.0; // Gate width
            box.scale.z = 2.0; // Gate height
            box.color.r = 0.0f;
            box.color.g = 0.8f;
            box.color.b = 1.0f;
            box.color.a = 0.6f;
            array.markers.push_back(box);

            // Gate ID Text
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
        }

        pub_markers_->publish(array);
    }

    // Config parameters
    double prior_sigma_;
    double drone_sigma_;
    double pnp_sigma_;
    double max_dist_;
    double mahalanobis_max_sq_;
    bool publish_initial_pos_;

    // Data structures & matrices
    std::vector<GateState> gates_;
    geometry_msgs::msg::PoseArray initial_poses_msg_;
    visualization_msgs::msg::MarkerArray initial_markers_msg_;
    Eigen::Matrix3d R_cam_to_body_;

    Eigen::Vector3d latest_drone_pos_;
    Eigen::Quaterniond latest_drone_rot_;
    bool drone_pose_received_;

    // Initial offset anchor variables
    Eigen::Vector3d initial_drone_pos_;
    Eigen::Quaterniond initial_drone_rot_;
    bool initial_pose_captured_;

    // ROS 2 Comms
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pnp_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_refined_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_initial_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_initial_markers_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_gates_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavrosGateEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}