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
    MavrosGateEstimator() : Node("mavros_gate_estimator"), drone_pose_received_(false) {
        // Declare tunable parameters
        this->declare_parameter<double>("gate_prior_sigma", 1.5);
        this->declare_parameter<double>("drone_pose_sigma", 2.0);
        this->declare_parameter<double>("pnp_vision_sigma", 4.0);
        this->declare_parameter<double>("association_max_dist", 6.0);
        
        // Chi-squared 3 DOF gating threshold (11.345 = 99% confidence interval)
        this->declare_parameter<double>("mahalanobis_thresh_sq", 11.345);

        prior_sigma_         = this->get_parameter("gate_prior_sigma").as_double();
        drone_sigma_         = this->get_parameter("drone_pose_sigma").as_double();
        pnp_sigma_           = this->get_parameter("pnp_vision_sigma").as_double();
        max_dist_            = this->get_parameter("association_max_dist").as_double();
        mahalanobis_max_sq_  = this->get_parameter("mahalanobis_thresh_sq").as_double();

        // Initialize 5 gates from raw RDF priors to global ENU
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

        // Publishers
        pub_refined_poses_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/estimator/refined_gate_poses", 10
        );

        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/estimator/gate_markers", 10
        );

        // Service Server: Reset Gate Positions
        srv_reset_gates_ = this->create_service<std_srvs::srv::Trigger>(
            "/estimator/reset_gates",
            std::bind(&MavrosGateEstimator::handle_reset_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "MAVROS Gate Estimator Node Initialized (Mahalanobis Threshold Sq: %.3f).", mahalanobis_max_sq_);
    }

private:
    void initialize_gate_priors() {
        gates_.clear();

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

            // Conversion from RDF to ENU:
            // X_enu = Z_rdf (Forward)
            // Y_enu = -X_rdf (Left)
            // Z_enu = -Y_rdf (Up)
            state.position_enu = Eigen::Vector3d(p.z_rdf, -p.x_rdf, -p.y_rdf);
            state.normal_enu   = Eigen::Vector3d(p.nz_rdf, -p.nx_rdf, -p.ny_rdf).normalized();

            // Initial 3x3 diagonal covariance matrix P_0
            state.covariance = Eigen::Matrix3d::Identity() * initial_var;

            // Calculate rotation quaternion orienting the gate's normal vector in ENU
            Eigen::Vector3d default_facing(1.0, 0.0, 0.0);
            state.orientation = Eigen::Quaterniond::FromTwoVectors(default_facing, state.normal_enu);

            gates_.push_back(state);

            RCLCPP_INFO(
                this->get_logger(),
                "Initialized Gate %d -> ENU Position: [%.2f, %.2f, %.2f], Normal: [%.2f, %.2f, %.2f]",
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
        response->message = "Gate positions and error covariances reset to initial priors.";
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

        // Publish updated results
        publish_refined_poses(msg->header);
        publish_rviz_markers(msg->header);
    }

    void update_gate_kalman(GateState &gate, const Eigen::Vector3d &z_meas, const Eigen::Matrix3d &R_meas, double mahalanobis_sq) {
        const Eigen::Vector3d y = z_meas - gate.position_enu;    // Innovation residual
        const Eigen::Matrix3d S = gate.covariance + R_meas;       // Innovation covariance
        const Eigen::Matrix3d K = gate.covariance * S.inverse(); // Kalman Gain

        // State & Covariance Update
        gate.position_enu += K * y;
        gate.covariance = (Eigen::Matrix3d::Identity() - K) * gate.covariance;

        RCLCPP_INFO(
            this->get_logger(),
            "Updated Gate %d -> Refined ENU Pos: [%.2f, %.2f, %.2f], Pos Uncertainty std_x: %.2fm, D_M^2: %.2f",
            gate.id, gate.position_enu.x(), gate.position_enu.y(), gate.position_enu.z(),
            std::sqrt(gate.covariance(0,0)), mahalanobis_sq
        );
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

    // Data structures & matrices
    std::vector<GateState> gates_;
    Eigen::Matrix3d R_cam_to_body_;

    Eigen::Vector3d latest_drone_pos_;
    Eigen::Quaterniond latest_drone_rot_;
    bool drone_pose_received_;

    // ROS 2 Comms
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pnp_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_refined_poses_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_gates_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavrosGateEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}