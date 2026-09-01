#include <chrono>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_controller/srv/set_string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "mavros_controller/Policy.hpp"

using namespace std::chrono_literals;

enum class FSMState {
    OFF,
    WAIT_FOR_FCU_CONNECT,
    SET_MODE_GUIDED,
    ARMING,
    TAKEOFF,
    CLIMBING,
    HOVER,
    RUN,
    HOME,
    FREE,
    LANDING
};

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
    : Node("controller"),
      current_fsm_state_(FSMState::OFF),
      target_altitude_(1.0),
      altitude_tolerance_(0.05),
      has_home_waypoint_(false),
      last_request_time_(this->now()),
      last_state_pub_time_(this->now())
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        // Subscriptions
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos,
            std::bind(&ControllerNode::stateCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos,
            std::bind(&ControllerNode::poseCallback, this, std::placeholders::_1));

        home_sub_ = this->create_subscription<mavros_msgs::msg::HomePosition>(
            "/mavros/home_position/home", qos,
            std::bind(&ControllerNode::homePositionCallback, this, std::placeholders::_1));

        custom_home_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/controller/home_pose", qos_reliable,
            std::bind(&ControllerNode::customHomePoseCallback, this, std::placeholders::_1));

        // Publishers for both local position setpoint (HOVER / HOME) and local raw setpoint (RUN)
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
            
        local_raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);

        // Publisher for Current FSM State telemetry
        current_state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/controller/current_state", 10);

        // Publisher for Active Target Gate Index telemetry (0 = Gate #1, 1 = Gate #2, etc.)
        target_gate_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/controller/target_gate_index", 10);
        target_subgate_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/controller/target_subgate_index", 10);

        preview_gate_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/controller/preview_gate_index", 10);

        preview_subgate_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/controller/preview_subgate_index", 10);

        // Subscriber to manually set Active Target Gate Index from GCS
        set_target_gate_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/controller/set_target_gate", qos_reliable,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg && msg->data >= 0 && msg->data < 5) {
                    policy_.setTargetGateIndex(static_cast<size_t>(msg->data), 0);
                    RCLCPP_INFO(this->get_logger(), "Active Target Gate switched manually to: Gate #%d", msg->data + 1);
                    publishCurrentState();
                }
            });

        // Service Clients
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        command_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
        land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

        // Service Server for State Transitions
        change_state_srv_ = this->create_service<mavros_controller::srv::SetString>(
            "~/change_state",
            std::bind(&ControllerNode::handleChangeState, this, std::placeholders::_1, std::placeholders::_2));

         // Resolve absolute package share directory for default model path
        std::string default_model_path;
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("mavros_controller");
            default_model_path = pkg_share + "/models/policy.onnx";
        } catch (const std::exception & e) {
            default_model_path = "models/policy.onnx";
        }

        // Declare model_path parameter (defaults to installed share directory path)
        this->declare_parameter<std::string>("model_path", default_model_path);
        std::string model_path = this->get_parameter("model_path").as_string();

        this->declare_parameter<int>("triple_gate_pass_method", 1);
        int triple_gate_pass_method = this->get_parameter("triple_gate_pass_method").as_int();

        this->declare_parameter<double>("max_accel", 5.6638);
        double max_accel = this->get_parameter("max_accel").as_double();

        this->declare_parameter<bool>("v7", false);
        bool v7 = this->get_parameter("v7").as_bool();

        this->declare_parameter<bool>("enable_gate_1_1", true);
        bool enable_gate_1_1 = this->get_parameter("enable_gate_1_1").as_bool();

        policy_.init(this, model_path);
        policy_.setTripleGatePassMethod(triple_gate_pass_method);
        policy_.setMaxAccel(max_accel);
        policy_.setV7(v7);
        policy_.setEnableGate1_1(enable_gate_1_1);

        this->declare_parameter<double>("target_altitude", 1.0);
        target_altitude_ = this->get_parameter("target_altitude").as_double();

        // 50 Hz timer loop (20 ms period)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ControllerNode::fsmLoop, this));

        RCLCPP_INFO(this->get_logger(), "ArduPilot MAVROS Controller Node Started (50 Hz). Initial state: OFF.");
    }

    std::string getFSMStateString(FSMState state) const
    {
        switch (state) {
            case FSMState::OFF: return "OFF";
            case FSMState::WAIT_FOR_FCU_CONNECT: return "WAIT_FOR_FCU_CONNECT";
            case FSMState::SET_MODE_GUIDED: return "SET_MODE_GUIDED";
            case FSMState::ARMING: return "ARMING";
            case FSMState::TAKEOFF: return "TAKEOFF";
            case FSMState::CLIMBING: return "CLIMBING";
            case FSMState::HOVER: return "HOVER";
            case FSMState::RUN: return "RUN";
            case FSMState::HOME: return "HOME";
            case FSMState::FREE: return "FREE";
            case FSMState::LANDING: return "LANDING";
            default: return "UNKNOWN";
        }
    }

    void publishCurrentState()
    {
        if (current_state_pub_) {
            std_msgs::msg::String msg;
            msg.data = getFSMStateString(current_fsm_state_);
            current_state_pub_->publish(msg);
        }

        if (target_gate_pub_) {
            std_msgs::msg::Int32 gate_msg;
            gate_msg.data = static_cast<int32_t>(policy_.getTargetGateIndex());
            target_gate_pub_->publish(gate_msg);
        }

        if (target_subgate_pub_) {
            std_msgs::msg::Int32 subgate_msg;
            subgate_msg.data = static_cast<int32_t>(policy_.getTargetSubGateIndex());
            target_subgate_pub_->publish(subgate_msg);
        }

        if (preview_gate_pub_) {
            std_msgs::msg::Int32 prev_gate_msg;
            prev_gate_msg.data = static_cast<int32_t>(policy_.getPreviewGateIndex());
            preview_gate_pub_->publish(prev_gate_msg);
        }

        if (preview_subgate_pub_) {
            std_msgs::msg::Int32 prev_sub_msg;
            prev_sub_msg.data = static_cast<int32_t>(policy_.getPreviewSubGateIndex());
            preview_subgate_pub_->publish(prev_sub_msg);
        }
    }

    bool changeState(const std::string & target_state)
    {
        std::string state_upper = target_state;
        std::transform(state_upper.begin(), state_upper.end(), state_upper.begin(), ::toupper);

        if (state_upper == "OFF") {
            double current_alt = current_pose_.pose.position.z;
            bool on_ground = (!current_mavros_state_.armed || current_alt < 0.25);

            if (!on_ground && (current_fsm_state_ == FSMState::HOVER ||
                               current_fsm_state_ == FSMState::RUN ||
                               current_fsm_state_ == FSMState::HOME ||
                               current_fsm_state_ == FSMState::FREE ||
                               current_fsm_state_ == FSMState::CLIMBING ||
                               current_fsm_state_ == FSMState::TAKEOFF))
            {
                RCLCPP_INFO(this->get_logger(), "State change request 'OFF': Drone is airborne (alt: %.2fm). Initiating LANDING sequence.", current_alt);
                current_fsm_state_ = FSMState::LANDING;
                requestLand();
            } else {
                RCLCPP_INFO(this->get_logger(), "State change request 'OFF': Drone is on ground or near surface (alt: %.2fm). Entering OFF state directly and force disarming.", current_alt);
                current_fsm_state_ = FSMState::OFF;
                if (current_mavros_state_.armed) {
                    requestForceDisarm();
                }
            }
            publishCurrentState();
            return true;
        }
        else if (state_upper == "HOVER") {
            if (current_fsm_state_ == FSMState::OFF || current_fsm_state_ == FSMState::LANDING) {
                RCLCPP_INFO(this->get_logger(), "State change request 'HOVER': Initiating takeoff sequence towards HOVER.");
                current_fsm_state_ = FSMState::WAIT_FOR_FCU_CONNECT;
                publishCurrentState();
                return true;
            } else {
                RCLCPP_INFO(this->get_logger(), "Transitioning to / remaining in HOVER mode.");
                hover_pose_ = current_pose_;
                if (hover_pose_.pose.position.z < 0.3) {
                    hover_pose_.pose.position.z = target_altitude_;
                }
                if (current_mavros_state_.mode != "GUIDED") {
                    requestSetMode("GUIDED");
                }
                current_fsm_state_ = FSMState::HOVER;
                publishCurrentState();
                return true;
            }
        }
        else if (state_upper == "RUN") {
            if (current_fsm_state_ == FSMState::HOVER || current_fsm_state_ == FSMState::HOME || current_fsm_state_ == FSMState::FREE) {
                RCLCPP_INFO(this->get_logger(), "State change request 'RUN': Switching to RUN.");
                if (current_mavros_state_.mode != "GUIDED") {
                    requestSetMode("GUIDED");
                }
                policy_.reset();
                run_start_time_ = this->now();
                current_fsm_state_ = FSMState::RUN;
                publishCurrentState();
                return true;
            } else if (current_fsm_state_ == FSMState::OFF || current_fsm_state_ == FSMState::LANDING) {
                RCLCPP_WARN(this->get_logger(), "Cannot transition to 'RUN' directly from OFF/LANDING. Takeoff to HOVER first.");
                return false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Already in or transitioning to RUN mode.");
                policy_.reset();
                run_start_time_ = this->now();
                current_fsm_state_ = FSMState::RUN;
                publishCurrentState();
                return true;
            }
        }
        else if (state_upper == "HOME") {
            if (!has_home_waypoint_) {
                RCLCPP_WARN(this->get_logger(), "Cannot transition to 'HOME': No home waypoint available yet.");
                return false;
            }

            if (current_fsm_state_ == FSMState::OFF || current_fsm_state_ == FSMState::LANDING) {
                RCLCPP_WARN(this->get_logger(), "Cannot fly to 'HOME' directly from OFF/LANDING. Takeoff to HOVER first.");
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "State change request 'HOME': Flying towards Home Waypoint (x: %.2f, y: %.2f, z: %.2f).",
                home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
            if (current_mavros_state_.mode != "GUIDED") {
                requestSetMode("GUIDED");
            }
            current_fsm_state_ = FSMState::HOME;
            publishCurrentState();
            return true;
        }
        else if (state_upper == "FREE") {
            RCLCPP_INFO(this->get_logger(), "State change request 'FREE': Giving full manual/RC control. Automated setpoints & arm/disarm watchdog disabled.");
            current_fsm_state_ = FSMState::FREE;
            publishCurrentState();
            return true;
        }

        RCLCPP_WARN(this->get_logger(), "Invalid state request: '%s'. Valid states: 'OFF', 'HOVER', 'RUN', 'HOME', 'FREE'.", target_state.c_str());
        return false;
    }

private:
    void handleChangeState(
        const std::shared_ptr<mavros_controller::srv::SetString::Request> request,
        std::shared_ptr<mavros_controller::srv::SetString::Response> response)
    {
        bool success = changeState(request->data);
        response->success = success;
        if (success) {
            response->message = "State change request to '" + request->data + "' successfully accepted.";
        } else {
            response->message = "Failed to initiate transition to '" + request->data + "'. Allowed values: 'OFF', 'HOVER', 'RUN', 'HOME', 'FREE'.";
        }
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_mavros_state_ = *msg;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        // Auto-capture initial position before takeoff if home waypoint is not yet set
        if (!has_home_waypoint_ && !current_mavros_state_.armed && current_pose_.pose.position.z < 0.3) {
            home_pose_ = current_pose_;
            home_pose_.header.frame_id = "map";
            home_pose_.pose.position.z = target_altitude_;
            has_home_waypoint_ = true;
            RCLCPP_INFO_ONCE(this->get_logger(), "Initial ground position auto-captured as fallback home waypoint: (x: %.2f, y: %.2f, z: %.2f)",
                home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
        }
    }

    void homePositionCallback(const mavros_msgs::msg::HomePosition::SharedPtr msg)
    {
        home_pose_.header = msg->header;
        home_pose_.header.frame_id = "map";
        home_pose_.pose.position = msg->position;
        home_pose_.pose.orientation = msg->orientation;
        if (home_pose_.pose.position.z < 0.2) {
            home_pose_.pose.position.z = target_altitude_;
        }
        has_home_waypoint_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "MAVROS HomePosition received: (x: %.2f, y: %.2f, z: %.2f)",
            home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
    }

    void customHomePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        home_pose_ = *msg;
        if (home_pose_.header.frame_id.empty()) {
            home_pose_.header.frame_id = "map";
        }
        if (home_pose_.pose.position.z < 0.2) {
            home_pose_.pose.position.z = target_altitude_;
        }
        has_home_waypoint_ = true;
        RCLCPP_INFO(this->get_logger(), "Custom home waypoint updated: (x: %.2f, y: %.2f, z: %.2f)",
            home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z);
    }

    void fsmLoop()
    {
        rclcpp::Time current_time = this->now();

        // Periodically broadcast current state telemetry (10 Hz)
        if ((current_time - last_state_pub_time_).seconds() >= 0.1) {
            publishCurrentState();
            last_state_pub_time_ = current_time;
        }

        // Calculate Ground-Relative Altitude (AGL)
        // If on the ground and disarmed in OFF/ARMING, continuously track ground level
        if (!current_mavros_state_.armed && current_fsm_state_ <= FSMState::ARMING) {
            ground_altitude_ = current_pose_.pose.position.z;
            has_ground_altitude_ = true;
        }

        double ground_z = has_ground_altitude_ ? ground_altitude_ : 0.0;
        double current_alt_agl = current_pose_.pose.position.z - ground_z;
        double target_local_z = ground_z + target_altitude_;

        // Continuously update and stream Observation Space (42D) telemetry in all states
        if (current_fsm_state_ != FSMState::RUN) {
            policy_.publishObservation();
        }

        switch (current_fsm_state_) {
            case FSMState::OFF: {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "FSM State: OFF (Unarmed / Standby). Awaiting 'HOVER' command.");

                if (current_mavros_state_.armed && (current_time - last_request_time_).seconds() > 1.5) {
                    requestForceDisarm();
                    last_request_time_ = current_time;
                }
                break;
            }

            case FSMState::WAIT_FOR_FCU_CONNECT: {
                if (current_mavros_state_.connected) {
                    RCLCPP_INFO(this->get_logger(), "FCU Link OK. Transitioning to SET_MODE_GUIDED.");
                    current_fsm_state_ = FSMState::SET_MODE_GUIDED;
                    last_request_time_ = current_time;
                } else {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "Waiting for FCU connection...");
                }
                break;
            }

            case FSMState::SET_MODE_GUIDED: {
                if (current_mavros_state_.mode == "GUIDED") {
                    RCLCPP_INFO(this->get_logger(), "GUIDED mode confirmed. Switching to ARMING state.");
                    current_fsm_state_ = FSMState::ARMING;
                } else if ((current_time - last_request_time_).seconds() > 2.0) {
                    requestSetMode("GUIDED");
                    last_request_time_ = current_time;
                }
                break;
            }

            case FSMState::ARMING: {
                if (current_mavros_state_.armed) {
                    RCLCPP_INFO(this->get_logger(), "Drone armed successfully. Latching ground altitude at %.2fm. Switching to TAKEOFF.", ground_altitude_);
                    current_fsm_state_ = FSMState::TAKEOFF;
                } else if ((current_time - last_request_time_).seconds() > 2.0) {
                    requestArming(true);
                    last_request_time_ = current_time;
                }
                break;
            }

            case FSMState::TAKEOFF: {
                if ((current_time - last_request_time_).seconds() > 2.0) {
                    hover_pose_ = current_pose_;
                    hover_pose_.pose.position.z = target_local_z;
                    requestTakeoff(static_cast<float>(target_altitude_));
                    RCLCPP_INFO(this->get_logger(), "Takeoff command sent to height %.2fm AGL (local z: %.2fm). Transitioning to CLIMBING.", target_altitude_, target_local_z);
                    current_fsm_state_ = FSMState::CLIMBING;
                    last_request_time_ = current_time;
                }
                break;
            }

            case FSMState::CLIMBING: {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 500,
                    "Climbing... Target: %.2fm AGL (local: %.2fm), Current: %.2fm AGL (local: %.2fm)", 
                    target_altitude_, target_local_z, current_alt_agl, current_pose_.pose.position.z);

                // If drone hasn't lifted off (< 0.15m AGL) after 2.5s, re-trigger arming & takeoff
                if (current_alt_agl < 0.15 && (current_time - last_request_time_).seconds() > 2.5) {
                    if (!current_mavros_state_.armed) {
                        RCLCPP_WARN(this->get_logger(), "Drone not armed in CLIMBING. Sending force arm.");
                        requestForceArm();
                    }
                    if (current_mavros_state_.mode != "GUIDED") {
                        requestSetMode("GUIDED");
                    }
                    RCLCPP_INFO(this->get_logger(), "Re-sending takeoff command...");
                    requestTakeoff(static_cast<float>(target_altitude_));
                    last_request_time_ = current_time;
                } else if (current_alt_agl >= 0.20) {
                    // Actively stream target altitude setpoint to guide ArduPilot past initial takeoff threshold
                    hover_pose_.header.stamp = this->now();
                    hover_pose_.header.frame_id = "map";
                    hover_pose_.pose.position.z = target_local_z;
                    local_pos_pub_->publish(hover_pose_);
                }

                if (current_alt_agl >= (target_altitude_ - altitude_tolerance_)) {
                    RCLCPP_INFO(this->get_logger(), "Target altitude reached (%.2fm AGL). Transitioning to HOVER.", current_alt_agl);
                    hover_pose_ = current_pose_;
                    hover_pose_.pose.position.z = target_local_z;
                    current_fsm_state_ = FSMState::HOVER;
                    publishCurrentState();
                }
                break;
            }

            case FSMState::HOVER: {
                if (current_alt_agl < 0.20) {
                    RCLCPP_WARN(this->get_logger(), "Ground sink detected in HOVER (alt: %.2fm AGL < 0.20m). Auto-disarming to OFF.", current_alt_agl);
                    current_fsm_state_ = FSMState::OFF;
                    requestForceDisarm();
                    break;
                }

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "FSM State: HOVER. Maintaining altitude at %.2fm AGL (local: %.2fm).", target_altitude_, hover_pose_.pose.position.z);

                hover_pose_.header.stamp = this->now();
                hover_pose_.header.frame_id = "map";
                hover_pose_.pose.position.z = target_local_z;
                local_pos_pub_->publish(hover_pose_);
                break;
            }

            case FSMState::RUN: {
                if (current_alt_agl < 0.20 && (current_time - run_start_time_).seconds() > 2.0) {
                    RCLCPP_WARN(this->get_logger(), "Ground sink detected in RUN (alt: %.2fm AGL < 0.20m). Auto-disarming to OFF.", current_alt_agl);
                    current_fsm_state_ = FSMState::OFF;
                    requestForceDisarm();
                    break;
                }

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "FSM State: RUN. Streaming Policy velocity setpoints to /mavros/setpoint_raw/local (Gate Target Index: %zu).", 
                    policy_.getTargetGateIndex());

                // Execute policy step and publish PositionTarget setpoint to /mavros/setpoint_raw/local
                mavros_msgs::msg::PositionTarget raw_setpoint = policy_.step();

                if (policy_.isAllGatesCleared()) {
                    RCLCPP_INFO(this->get_logger(), "All gates successfully passed! Automatically transitioning from RUN to HOVER state.");
                    hover_pose_ = current_pose_;
                    hover_pose_.header.stamp = this->now();
                    hover_pose_.header.frame_id = "map";
                    hover_pose_.pose.position.z = current_pose_.pose.position.z;
                    current_fsm_state_ = FSMState::HOVER;
                    local_pos_pub_->publish(hover_pose_);
                    break;
                }

                local_raw_pub_->publish(raw_setpoint);
                break;
            }

            case FSMState::HOME: {
                if (current_alt_agl < 0.20) {
                    RCLCPP_WARN(this->get_logger(), "Ground sink detected in HOME (alt: %.2fm AGL < 0.20m). Auto-disarming to OFF.", current_alt_agl);
                    current_fsm_state_ = FSMState::OFF;
                    requestForceDisarm();
                    break;
                }

                if (current_mavros_state_.mode != "GUIDED" && (current_time - last_request_time_).seconds() > 2.0) {
                    requestSetMode("GUIDED");
                    last_request_time_ = current_time;
                }

                double dx = home_pose_.pose.position.x - current_pose_.pose.position.x;
                double dy = home_pose_.pose.position.y - current_pose_.pose.position.y;
                double dz = home_pose_.pose.position.z - current_pose_.pose.position.z;
                double dist_horiz = std::hypot(dx, dy);
                double dist_3d = std::sqrt(dx * dx + dy * dy + dz * dz);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "FSM State: HOME. Flying towards Home Waypoint (x: %.2f, y: %.2f, z: %.2f) | Dist: %.2fm (horiz: %.2fm).",
                    home_pose_.pose.position.x, home_pose_.pose.position.y, home_pose_.pose.position.z,
                    dist_3d, dist_horiz);

                home_pose_.header.stamp = this->now();
                home_pose_.header.frame_id = "map";
                local_pos_pub_->publish(home_pose_);
                break;
            }

            case FSMState::FREE: {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "FSM State: FREE. Full manual / RC control active. Controller passive (no setpoints, no auto-arm/disarm).");
                // In FREE state: passive, no setpoints published, no auto arm/disarm commands sent
                break;
            }

            case FSMState::LANDING: {
                double current_alt = current_pose_.pose.position.z;
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "FSM State: LANDING... Current Altitude: %.2fm", current_alt);

                if ((current_time - last_request_time_).seconds() > 3.0 && current_mavros_state_.mode != "LAND") {
                    requestLand();
                    last_request_time_ = current_time;
                }

                if (!current_mavros_state_.armed || current_alt <= 0.20) {
                    RCLCPP_INFO(this->get_logger(), "Touchdown confirmed (alt: %.2fm). Transitioning to OFF state.", current_alt);
                    if (current_mavros_state_.armed) {
                        requestForceDisarm();
                    }
                    current_fsm_state_ = FSMState::OFF;
                }
                break;
            }
        }
    }

    void requestLand()
    {
        if (!land_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Land service unavailable. Requesting LAND mode via SetMode.");
            requestSetMode("LAND");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = 0.0f;
        request->latitude = 0.0f;
        request->longitude = 0.0f;

        land_client_->async_send_request(
            request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Land command executed successfully.");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Land service rejected request. Setting LAND mode.");
                        requestSetMode("LAND");
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Land service call failed: %s", e.what());
                }
            });
    }

    void requestSetMode(const std::string & mode)
    {
        if (!set_mode_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "SetMode service unavailable.");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        set_mode_client_->async_send_request(
            request,
            [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->mode_sent) {
                        RCLCPP_INFO(this->get_logger(), "Mode set to: %s", mode.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Mode change to '%s' rejected by FCU.", mode.c_str());
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "SetMode service call failed: %s", e.what());
                }
            });
    }

    void requestForceDisarm()
    {
        if (!command_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Command service unavailable for force disarm. Attempting standard disarm.");
            requestArming(false);
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->broadcast = false;
        request->command = 400; // MAV_CMD_COMPONENT_ARM_DISARM
        request->confirmation = 0;
        request->param1 = 0.0f;     // 0 = Disarm
        request->param2 = 21196.0f; // ArduPilot force disarm magic number (ARMING_CHECK_FORCE_DISARM)
        request->param3 = 0.0f;
        request->param4 = 0.0f;
        request->param5 = 0.0f;
        request->param6 = 0.0f;
        request->param7 = 0.0f;

        command_client_->async_send_request(
            request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Force disarm executed successfully.");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Force disarm returned result code: %d", response->result);
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Force disarm service call failed: %s", e.what());
                }
            });
    }

    void requestForceArm()
    {
        if (!command_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Command service unavailable for force arm. Attempting standard arm.");
            requestArming(true);
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->broadcast = false;
        request->command = 400; // MAV_CMD_COMPONENT_ARM_DISARM
        request->confirmation = 0;
        request->param1 = 1.0f;     // 1 = Arm
        request->param2 = 21196.0f; // ArduPilot force arm (ARMING_CHECK_FORCE)
        request->param3 = 0.0f;
        request->param4 = 0.0f;
        request->param5 = 0.0f;
        request->param6 = 0.0f;
        request->param7 = 0.0f;

        command_client_->async_send_request(
            request,
            [this](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Force arm executed successfully.");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Force arm returned result code: %d", response->result);
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Force arm service call failed: %s", e.what());
                }
            });
    }

    void requestArming(bool arm)
    {
        if (!arming_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Arming service unavailable.");
            if (arm) requestForceArm();
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;

        arming_client_->async_send_request(
            request,
            [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Arming command executed: %s", arm ? "ARM" : "DISARM");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Arming request (%s) rejected by FCU.", arm ? "ARM" : "DISARM");
                        if (!arm) {
                            RCLCPP_INFO(this->get_logger(), "Falling back to force disarm.");
                            requestForceDisarm();
                        } else {
                            RCLCPP_INFO(this->get_logger(), "Falling back to force arm.");
                            requestForceArm();
                        }
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Arming service call failed: %s", e.what());
                    if (!arm) {
                        requestForceDisarm();
                    } else {
                        requestForceArm();
                    }
                }
            });
    }

    void requestTakeoff(float altitude)
    {
        if (!takeoff_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Takeoff service unavailable.");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = altitude;
        request->latitude = 0.0f;
        request->longitude = 0.0f;
        request->min_pitch = 0.0f;
        request->yaw = 0.0f;

        takeoff_client_->async_send_request(
            request,
            [this, altitude](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Takeoff command accepted for altitude: %.2fm", altitude);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Takeoff request rejected by FCU.");
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Takeoff service call failed: %s", e.what());
                }
            });
    }

    FSMState current_fsm_state_;
    mavros_msgs::msg::State current_mavros_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped hover_pose_;
    geometry_msgs::msg::PoseStamped home_pose_;

    double target_altitude_;
    double altitude_tolerance_;
    double ground_altitude_{0.0};
    bool has_ground_altitude_{false};
    bool has_home_waypoint_{false};
    rclcpp::Time last_request_time_;
    rclcpp::Time last_state_pub_time_;
    rclcpp::Time run_start_time_;

    Policy policy_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr custom_home_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_target_gate_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_raw_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_gate_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_subgate_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr preview_gate_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr preview_subgate_pub_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::Service<mavros_controller::srv::SetString>::SharedPtr change_state_srv_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}