#include <chrono>
#include <memory>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
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
      last_request_time_(this->now()),
      last_state_pub_time_(this->now())
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Subscriptions
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos,
            std::bind(&ControllerNode::stateCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos,
            std::bind(&ControllerNode::poseCallback, this, std::placeholders::_1));

        // Publishers for both local position setpoint (HOVER) and local raw setpoint (RUN)
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);
            
        local_raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 10);

        // Publisher for Current FSM State telemetry
        current_state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/controller/current_state", 10);

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
        policy_.init(this, model_path);
        // 50 Hz timer loop (20 ms period)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ControllerNode::fsmLoop, this));

        RCLCPP_INFO(this->get_logger(), "ArduPilot MAVROS Controller Node Started (50 Hz). Initial state: OFF.");
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
            return true;
        }
        else if (state_upper == "HOVER") {
            if (current_fsm_state_ == FSMState::OFF || current_fsm_state_ == FSMState::LANDING) {
                RCLCPP_INFO(this->get_logger(), "State change request 'HOVER': Initiating takeoff sequence towards HOVER.");
                current_fsm_state_ = FSMState::WAIT_FOR_FCU_CONNECT;
                return true;
            } else {
                RCLCPP_INFO(this->get_logger(), "Transitioning to / remaining in HOVER mode.");
                current_fsm_state_ = FSMState::HOVER;
                return true;
            }
        }
        else if (state_upper == "RUN") {
            if (current_fsm_state_ == FSMState::HOVER) {
                RCLCPP_INFO(this->get_logger(), "State change request 'RUN': Switching from HOVER to RUN.");
                current_fsm_state_ = FSMState::RUN;
                return true;
            } else if (current_fsm_state_ == FSMState::OFF || current_fsm_state_ == FSMState::LANDING) {
                RCLCPP_WARN(this->get_logger(), "Cannot transition to 'RUN' directly from OFF/LANDING. Takeoff to HOVER first.");
                return false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Already in or transitioning to RUN mode.");
                current_fsm_state_ = FSMState::RUN;
                return true;
            }
        }

        RCLCPP_WARN(this->get_logger(), "Invalid state request: '%s'. Valid states: 'OFF', 'HOVER', 'RUN'.", target_state.c_str());
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
            response->message = "Failed to initiate transition to '" + request->data + "'. Allowed values: 'OFF', 'HOVER', 'RUN'.";
        }
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_mavros_state_ = *msg;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
    }

    void fsmLoop()
    {
        rclcpp::Time current_time = this->now();

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
                    RCLCPP_INFO(this->get_logger(), "FCU connected. Switching to SET_MODE_GUIDED.");
                    current_fsm_state_ = FSMState::SET_MODE_GUIDED;
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for FCU connection...");
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
                    RCLCPP_INFO(this->get_logger(), "Drone armed successfully. Switching to TAKEOFF state.");
                    current_fsm_state_ = FSMState::TAKEOFF;
                } else if ((current_time - last_request_time_).seconds() > 2.0) {
                    requestArming(true);
                    last_request_time_ = current_time;
                }
                break;
            }

            case FSMState::TAKEOFF: {
                if ((current_time - last_request_time_).seconds() > 2.0) {
                    requestTakeoff(static_cast<float>(target_altitude_));
                    RCLCPP_INFO(this->get_logger(), "Takeoff command sent to height %.2fm. Transitioning to CLIMBING.", target_altitude_);
                    current_fsm_state_ = FSMState::CLIMBING;
                    last_request_time_ = current_time;
                }
                break;
            }

            case FSMState::CLIMBING: {
                double current_alt = current_pose_.pose.position.z;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 500,
                    "Climbing... Target: %.2fm, Current: %.2fm", target_altitude_, current_alt);

                if (current_alt >= (target_altitude_ - altitude_tolerance_)) {
                    RCLCPP_INFO(this->get_logger(), "Target altitude reached (%.2fm). Transitioning to HOVER.", current_alt);
                    hover_pose_ = current_pose_;
                    hover_pose_.pose.position.z = target_altitude_;
                    current_fsm_state_ = FSMState::HOVER;
                }
                break;
            }

            case FSMState::HOVER: {
                double current_alt = current_pose_.pose.position.z;
                if (current_alt < 0.20) {
                    RCLCPP_WARN(this->get_logger(), "Ground sink / low altitude detected in HOVER (alt: %.2fm < 0.20m). Auto-disarming to OFF.", current_alt);
                    current_fsm_state_ = FSMState::OFF;
                    requestForceDisarm();
                    break;
                }

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "FSM State: HOVER. Maintaining altitude at %.2fm.", hover_pose_.pose.position.z);

                hover_pose_.header.stamp = this->now();
                hover_pose_.header.frame_id = "map";
                local_pos_pub_->publish(hover_pose_);
                break;
            }

            case FSMState::RUN: {
                double current_alt = current_pose_.pose.position.z;
                if (current_alt < 0.20) {
                    RCLCPP_WARN(this->get_logger(), "Ground sink / low altitude detected in RUN (alt: %.2fm < 0.20m). Auto-disarming to OFF.", current_alt);
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
                local_raw_pub_->publish(raw_setpoint);
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

    void requestArming(bool arm)
    {
        if (!arming_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Arming service unavailable.");
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
                        }
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "Arming service call failed: %s", e.what());
                    if (!arm) {
                        requestForceDisarm();
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

    double target_altitude_;
    double altitude_tolerance_;
    rclcpp::Time last_request_time_;

    Policy policy_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_raw_pub_;

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