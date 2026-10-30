// Offboard velocity control driven by keyboard commands.
//
// Consumes the NED velocity command published by keyboard_velocity and streams
// the matching px4_msgs offboard setpoints over uXRCE-DDS.  The node owns the
// flight state machine:
//
//     disarmed + climb request   -> arm, then switch to OFFBOARD
//     OFFBOARD                   -> stream keyboard velocity
//     land request ('g')         -> LAND until AUTO_LAND, then stop streaming
//     disarmed after landing     -> idle, ready for another takeoff
//
// The node only takes over when the operator explicitly asks to fly (i).  It
// never requests OFFBOARD while merely armed in some other mode, so it cannot
// steal an AUTO_LAND / AUTO_RTL / AUTO_TAKEOFF owned by another controller.  If
// the vehicle leaves OFFBOARD after a takeoff, the node disengages and stops
// streaming instead of fighting the new mode.
//
// An automatic takeoff is latched when arming is triggered: the vehicle climbs
// at takeoff_climb_rate to takeoff_altitude before the keyboard vertical
// command takes over.
//
// Everything is PX4 local NED: x north, y east, z down (negative up).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

constexpr uint32_t kMavCmdComponentArmDisarm = 400;
constexpr uint32_t kMavCmdDoSetMode = 176;
constexpr uint32_t kMavCmdNavLand = 21;

constexpr uint8_t kPx4CustomMainModeOffboard = 6;
constexpr uint8_t kNavigationStateOffboard = 14;
constexpr uint8_t kNavigationStateAutoLand = 18;
constexpr uint8_t kArmingStateDisarmed = 1;

float nan()
{
  return std::numeric_limits<float>::quiet_NaN();
}

rclcpp::QoS px4_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
}

}  // namespace

class KeyboardVelocityControl : public rclcpp::Node
{
public:
  KeyboardVelocityControl()
  : Node("keyboard_velocity_control")
  {
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 1.0);
    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", 1.5);
    takeoff_climb_rate_ = declare_parameter<double>("takeoff_climb_rate", 0.5);
    climb_trigger_ = declare_parameter<double>("climb_trigger", 0.05);
    retry_ = declare_parameter<double>("command_retry_s", 1.0);

    const rclcpp::QoS qos = px4_qos();
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", qos);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", qos);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos);

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos,
      std::bind(&KeyboardVelocityControl::on_status, this, std::placeholders::_1));
    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&KeyboardVelocityControl::on_position, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/keyboard/velocity_cmd", 10,
      std::bind(&KeyboardVelocityControl::on_cmd, this, std::placeholders::_1));
    land_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/keyboard/land", 10,
      std::bind(&KeyboardVelocityControl::on_land, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&KeyboardVelocityControl::tick, this));
    RCLCPP_INFO(
      get_logger(),
      "keyboard velocity control ready: press i in the keyboard node to arm");
  }

private:
  // -------------------------------------------------------------- callbacks
  void on_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    status_ = *msg;
    have_status_ = true;
  }

  void on_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    z_ = static_cast<double>(msg->z);
    have_z_ = true;
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_ = *msg;
    cmd_stamp_ = now_s();
    if (stale_) {
      stale_ = false;
      RCLCPP_INFO(get_logger(), "keyboard commands resumed");
    }
  }

  void on_land(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      land_requested_ = true;
    }
  }

  // ----------------------------------------------------------------- helpers
  double now_s()
  {
    return static_cast<double>(this->now().nanoseconds()) / 1e9;
  }

  uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(this->now().nanoseconds()) / 1000;
  }

  // Returns the current keyboard command, zeroed if the source is stale.
  geometry_msgs::msg::Twist active_cmd()
  {
    if (!cmd_stamp_ || now_s() - *cmd_stamp_ > cmd_timeout_) {
      if (!stale_) {
        stale_ = true;
        RCLCPP_WARN(
          get_logger(), "no keyboard command for %.1f s; commanding zero velocity",
          cmd_timeout_);
      }
      return geometry_msgs::msg::Twist();
    }
    return cmd_;
  }

  void send_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestamp_us();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  void stream(const geometry_msgs::msg::Twist & cmd)
  {
    px4_msgs::msg::OffboardControlMode heartbeat;
    heartbeat.timestamp = timestamp_us();
    heartbeat.position = false;
    heartbeat.velocity = true;
    heartbeat.acceleration = false;
    heartbeat.attitude = false;
    heartbeat.body_rate = false;
    heartbeat.thrust_and_torque = false;
    heartbeat.direct_actuator = false;
    offboard_mode_pub_->publish(heartbeat);

    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = timestamp_us();
    setpoint.position = {nan(), nan(), nan()};
    setpoint.velocity = {
      static_cast<float>(cmd.linear.x),
      static_cast<float>(cmd.linear.y),
      static_cast<float>(cmd.linear.z)};
    setpoint.acceleration = {nan(), nan(), nan()};
    setpoint.jerk = {nan(), nan(), nan()};
    setpoint.yaw = nan();
    setpoint.yawspeed = static_cast<float>(cmd.angular.z);
    setpoint_pub_->publish(setpoint);
  }

  // ------------------------------------------------------------- state machine
  void request_arm(double now)
  {
    if (now - last_arm_ < retry_) {
      return;
    }
    last_arm_ = now;
    RCLCPP_INFO(get_logger(), "arming for keyboard flight");
    send_command(kMavCmdComponentArmDisarm, 1.0f);
  }

  void request_offboard(double now)
  {
    if (now - last_mode_ < retry_) {
      return;
    }
    last_mode_ = now;
    RCLCPP_INFO(get_logger(), "requesting OFFBOARD mode");
    send_command(kMavCmdDoSetMode, 1.0f, kPx4CustomMainModeOffboard);
  }

  void request_land(double now)
  {
    if (now - last_land_ < retry_) {
      return;
    }
    last_land_ = now;
    RCLCPP_INFO(get_logger(), "sending LAND command");
    send_command(kMavCmdNavLand);
  }

  // Runs the state machine and returns the setpoint to stream (nullopt = idle).
  std::optional<geometry_msgs::msg::Twist> update(
    geometry_msgs::msg::Twist cmd,
    const px4_msgs::msg::VehicleStatus & status, double now)
  {
    if (status.arming_state == kArmingStateDisarmed) {
      if (engaged_) {
        RCLCPP_INFO(get_logger(), "disarmed; keyboard control idle");
      }
      engaged_ = false;
      takeoff_active_ = false;
      if (land_requested_) {
        land_requested_ = false;
        RCLCPP_INFO(get_logger(), "landing finished");
      }
      if (cmd.linear.z < -climb_trigger_) {
        engaged_ = true;
        takeoff_active_ = true;
        request_arm(now);
        return geometry_msgs::msg::Twist();  // feed offboard while arming
      }
      return std::nullopt;
    }

    if (land_requested_) {
      if (cmd.linear.z < -climb_trigger_) {
        land_requested_ = false;
        engaged_ = true;
        takeoff_active_ = true;
        RCLCPP_INFO(get_logger(), "landing cancelled; climbing");
      } else {
        if (status.nav_state == kNavigationStateAutoLand) {
          // LAND accepted: hands off, do not stream offboard anymore.
          if (engaged_) {
            RCLCPP_INFO(get_logger(), "AUTO_LAND active; offboard stream stopped");
          }
          engaged_ = false;
          return std::nullopt;
        }
        request_land(now);
        return geometry_msgs::msg::Twist();
      }
    }

    if (!engaged_) {
      // Explicit engagement only: never take over an armed vehicle in a mode
      // owned by someone else (AUTO_LAND / AUTO_RTL / ...).
      if (status.nav_state == kNavigationStateOffboard &&
        cmd.linear.z < -climb_trigger_)
      {
        engaged_ = true;
        takeoff_active_ = true;
        RCLCPP_INFO(get_logger(), "engaging keyboard control in active OFFBOARD");
      } else {
        return std::nullopt;
      }
    }

    if (status.nav_state != kNavigationStateOffboard) {
      if (!takeoff_active_) {
        RCLCPP_INFO(
          get_logger(), "left OFFBOARD (nav_state=%d); keyboard control disengaged",
          static_cast<int>(status.nav_state));
        engaged_ = false;
        return std::nullopt;
      }
      request_offboard(now);
      geometry_msgs::msg::Twist climb;
      climb.linear.z = -takeoff_climb_rate_;
      return climb;
    }

    if (takeoff_active_) {
      if (cmd.linear.z > 0.0) {
        takeoff_active_ = false;
        RCLCPP_INFO(get_logger(), "takeoff cancelled by descend command");
      } else if (have_z_ && z_ <= -takeoff_altitude_) {
        takeoff_active_ = false;
        RCLCPP_INFO(
          get_logger(), "takeoff altitude reached (%.1f m)", takeoff_altitude_);
      } else {
        cmd.linear.z = -takeoff_climb_rate_;
      }
    }
    return cmd;
  }

  // ------------------------------------------------------------------- tick
  void tick()
  {
    const geometry_msgs::msg::Twist cmd = active_cmd();
    if (!have_status_) {
      return;
    }
    const std::optional<geometry_msgs::msg::Twist> out = update(cmd, status_, now_s());
    if (out) {
      stream(*out);
    }
  }

  double cmd_timeout_ = 1.0;
  double takeoff_altitude_ = 1.5;
  double takeoff_climb_rate_ = 0.5;
  double climb_trigger_ = 0.05;
  double retry_ = 1.0;

  geometry_msgs::msg::Twist cmd_;
  std::optional<double> cmd_stamp_;
  bool stale_ = false;
  bool have_status_ = false;
  bool have_z_ = false;
  px4_msgs::msg::VehicleStatus status_;
  double z_ = 0.0;
  bool land_requested_ = false;
  bool engaged_ = false;
  bool takeoff_active_ = false;
  double last_arm_ = -std::numeric_limits<double>::infinity();
  double last_mode_ = -std::numeric_limits<double>::infinity();
  double last_land_ = -std::numeric_limits<double>::infinity();

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardVelocityControl>());
  rclcpp::shutdown();
  return 0;
}
