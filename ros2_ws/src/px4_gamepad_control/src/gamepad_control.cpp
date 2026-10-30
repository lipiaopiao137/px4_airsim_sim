// Xbox gamepad teleoperation for PX4 offboard flight.
//
// Ported from ref/sdk_examples/zero_gamepad_control_planner/gamepad_control.hpp:
// the axis mapping, dead zone, tanh response curve and the RB/LB takeoff and
// land buttons are unchanged.  Instead of the Zero co_processor API the node
// talks to PX4 through uXRCE-DDS:
//
//     /joy                       sensor_msgs/Joy, published by joy_node
//     /fmu/in/offboard_control_mode
//     /fmu/in/trajectory_setpoint
//     /fmu/in/vehicle_command
//
// Default mapping (rc_control_mode:=USA):
//
//     left stick X   yaw rate (positive = clockwise / right)
//     left stick Y   climb / descend
//     right stick X  east / west
//     right stick Y  north / south (forward)
//     RB             arm, switch to OFFBOARD and take off automatically
//     LB             land (ignored while the automatic takeoff is running)
//
// JAPAN and CHINA follow the reference layouts: forward on left stick Y,
// climb on right stick Y, and yaw on left X (JAPAN) or right X (CHINA).
// Everything is PX4 local NED: x north, y east, z down (negative up).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace
{

constexpr uint32_t kMavCmdComponentArmDisarm = 400;
constexpr uint32_t kMavCmdDoSetMode = 176;
constexpr uint32_t kMavCmdNavLand = 21;

constexpr uint8_t kPx4CustomMainModeOffboard = 6;
constexpr uint8_t kNavigationStateOffboard = 14;
constexpr uint8_t kNavigationStateAutoLand = 18;
constexpr uint8_t kArmingStateArmed = 2;
constexpr uint8_t kArmingStateDisarmed = 1;

// Linux/SDL button indices for an Xbox controller: LB = 4, RB = 5.
constexpr int kButtonLb = 4;
constexpr int kButtonRb = 5;

float nan()
{
  return std::numeric_limits<float>::quiet_NaN();
}

// PX4 subscribes to /fmu/in/* with VOLATILE durability.
rclcpp::QoS px4_in_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

// PX4 publishes /fmu/out/* with TRANSIENT_LOCAL so a late subscriber gets the
// last sample immediately.
rclcpp::QoS px4_out_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
}

}  // namespace

class GamepadControl : public rclcpp::Node
{
public:
  GamepadControl()
  : Node("gamepad_control")
  {
    mode_ = declare_parameter<std::string>("rc_control_mode", "USA");
    if (mode_ != "USA" && mode_ != "JAPAN" && mode_ != "CHINA") {
      RCLCPP_ERROR(
        get_logger(), "invalid rc_control_mode '%s'; falling back to USA", mode_.c_str());
      mode_ = "USA";
    }
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");

    max_xy_vel_ = declare_parameter<double>("control.max_xy_vel", 2.0);
    max_z_vel_ = declare_parameter<double>("control.max_z_vel", 1.5);
    max_yaw_vel_ = declare_parameter<double>("control.max_yaw_vel", 1.0);
    const int interval_ms = std::max(10, static_cast<int>(
      declare_parameter<int>("control.interval_ms", 50)));

    deadzone_ = std::clamp(declare_parameter<double>("gamepad.axis_deadzone", 0.1), 0.0, 0.95);
    btn_debounce_ms_ = declare_parameter<int>("gamepad.btn_debounce_ms", 200);
    nonlinear_scale_ = declare_parameter<double>("gamepad.nonlinear_scale", 2.0);

    left_x_idx_ = declare_parameter<int>("axis_mapping.left_x", 0);
    left_y_idx_ = declare_parameter<int>("axis_mapping.left_y", 1);
    right_x_idx_ = declare_parameter<int>("axis_mapping.right_x", 3);
    right_y_idx_ = declare_parameter<int>("axis_mapping.right_y", 4);

    takeoff_button_ = declare_parameter<int>("button.takeoff", kButtonRb);
    land_button_ = declare_parameter<int>("button.land", kButtonLb);

    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 1.0);
    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", 1.5);
    takeoff_climb_rate_ = declare_parameter<double>("takeoff_climb_rate", 0.5);
    retry_ = declare_parameter<double>("command_retry_s", 1.0);

    const rclcpp::QoS in_qos = px4_in_qos();
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", in_qos);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", in_qos);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", in_qos);

    const rclcpp::QoS out_qos = px4_out_qos();
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", out_qos,
      std::bind(&GamepadControl::on_status, this, std::placeholders::_1));
    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", out_qos,
      std::bind(&GamepadControl::on_position, this, std::placeholders::_1));

    // joy_node publishes with the default (reliable) QoS; a best-effort
    // subscriber accepts that and also works with best-effort joy sources.
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&GamepadControl::on_joy, this, std::placeholders::_1));

    start_time_ = now_s();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(interval_ms),
      std::bind(&GamepadControl::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "gamepad ready: mode=%s, left stick yaw/climb, right stick move, "
      "RB takeoff, LB land (joy topic %s)",
      mode_.c_str(), joy_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "limits: xy=%.1f m/s z=%.1f m/s yaw=%.1f rad/s, deadzone=%.2f, curve=%.1f",
      max_xy_vel_, max_z_vel_, max_yaw_vel_, deadzone_, nonlinear_scale_);
  }

  ~GamepadControl() override = default;

private:
  // ------------------------------------------------------------- callbacks
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

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = now_s();
    if (!have_joy_) {
      have_joy_ = true;
      RCLCPP_INFO(
        get_logger(), "gamepad connected: %zu axes, %zu buttons",
        msg->axes.size(), msg->buttons.size());
    }

    const double lx = axis_value(*msg, left_x_idx_);
    const double ly = axis_value(*msg, left_y_idx_);
    const double rx = axis_value(*msg, right_x_idx_);
    const double ry = axis_value(*msg, right_y_idx_);

    left_x_ = lx;
    right_x_ = rx;
    if (mode_ == "USA") {
      left_y_ = ly;
      right_y_ = -ry;
    } else {
      left_y_ = -ly;
      right_y_ = ry;
    }

    if (rising_edge(*msg, takeoff_button_)) {
      on_takeoff_button();
    }
    if (rising_edge(*msg, land_button_)) {
      on_land_button();
    }
  }

  // --------------------------------------------------------------- helpers
  double now_s()
  {
    return static_cast<double>(this->now().nanoseconds()) / 1e9;
  }

  uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(this->now().nanoseconds()) / 1000;
  }

  // Dead-zone rescaling, same as the reference: values inside the zone become
  // zero, values outside are stretched back to the full [-1, 1] range.
  double axis_value(const sensor_msgs::msg::Joy & msg, int index) const
  {
    if (index < 0 || index >= static_cast<int>(msg.axes.size())) {
      return 0.0;
    }
    const double value = static_cast<double>(msg.axes[index]);
    if (std::abs(value) < deadzone_) {
      return 0.0;
    }
    return std::copysign((std::abs(value) - deadzone_) / (1.0 - deadzone_), value);
  }

  bool rising_edge(const sensor_msgs::msg::Joy & msg, int button)
  {
    const bool pressed =
      button >= 0 && button < static_cast<int>(msg.buttons.size()) &&
      msg.buttons[button] != 0;
    const bool was_pressed = last_button_state_[button];
    last_button_state_[button] = pressed;
    if (!pressed || was_pressed) {
      return false;
    }
    const double now = now_s();
    const auto it = last_button_time_.find(button);
    if (it != last_button_time_.end() &&
      now - it->second < static_cast<double>(btn_debounce_ms_) / 1000.0)
    {
      return false;
    }
    last_button_time_[button] = now;
    return true;
  }

  double curve(double value) const
  {
    if (nonlinear_scale_ <= 1e-6) {
      return value;
    }
    return std::tanh(nonlinear_scale_ * value) / std::tanh(nonlinear_scale_);
  }

  geometry_msgs::msg::Twist mapped_command() const
  {
    geometry_msgs::msg::Twist cmd;
    if (mode_ == "USA") {
      cmd.linear.x = curve(right_y_) * max_xy_vel_;
      cmd.linear.y = curve(right_x_) * max_xy_vel_;
      cmd.linear.z = curve(left_y_) * max_z_vel_;
      cmd.angular.z = curve(left_x_) * max_yaw_vel_;
    } else if (mode_ == "JAPAN") {
      cmd.linear.x = curve(left_y_) * max_xy_vel_;
      cmd.linear.y = curve(right_x_) * max_xy_vel_;
      cmd.linear.z = curve(right_y_) * max_z_vel_;
      cmd.angular.z = curve(left_x_) * max_yaw_vel_;
    } else {  // CHINA
      cmd.linear.x = curve(left_y_) * max_xy_vel_;
      cmd.linear.y = curve(left_x_) * max_xy_vel_;
      cmd.linear.z = curve(right_y_) * max_z_vel_;
      cmd.angular.z = curve(right_x_) * max_yaw_vel_;
    }
    return cmd;
  }

  void on_takeoff_button()
  {
    if (!have_status_) {
      RCLCPP_WARN(get_logger(), "takeoff ignored: no PX4 status yet");
      return;
    }
    if (land_requested_) {
      takeoff_requested_ = true;
      RCLCPP_INFO(get_logger(), "landing cancel requested");
      return;
    }
    if (takeoff_active_ || (engaged_ && status_.nav_state == kNavigationStateOffboard)) {
      RCLCPP_INFO(get_logger(), "already flying under gamepad control; takeoff ignored");
      return;
    }
    takeoff_requested_ = true;
    RCLCPP_INFO(get_logger(), "Takeoff command received");
  }

  void on_land_button()
  {
    if (!have_status_) {
      RCLCPP_WARN(get_logger(), "land ignored: no PX4 status yet");
      return;
    }
    if (status_.arming_state == kArmingStateDisarmed) {
      if (takeoff_requested_) {
        takeoff_requested_ = false;
        RCLCPP_INFO(get_logger(), "takeoff cancelled while disarmed");
      } else {
        RCLCPP_INFO(get_logger(), "already disarmed; land ignored");
      }
      return;
    }
    if (takeoff_active_) {
      RCLCPP_INFO(
        get_logger(), "takeoff in progress; ignoring land command");
      return;
    }
    if (land_requested_) {
      return;
    }
    land_requested_ = true;
    RCLCPP_INFO(get_logger(), "Land command received");
  }

  // --------------------------------------------------------------- commands
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

  // ------------------------------------------------------------ state machine
  void request_arm(double now)
  {
    if (now - last_arm_ < retry_) {
      return;
    }
    last_arm_ = now;
    RCLCPP_INFO(get_logger(), "arming for gamepad flight");
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
        RCLCPP_INFO(get_logger(), "disarmed; gamepad control idle");
      }
      engaged_ = false;
      takeoff_active_ = false;
      if (land_requested_) {
        land_requested_ = false;
        RCLCPP_INFO(get_logger(), "landing finished");
      }
      if (takeoff_requested_) {
        request_arm(now);
        return geometry_msgs::msg::Twist();  // feed offboard while arming
      }
      return std::nullopt;
    }

    if (land_requested_) {
      if (takeoff_requested_) {
        takeoff_requested_ = false;
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
    } else if (takeoff_requested_ && engaged_) {
      // RB pressed again while already under gamepad control: no-op.
      takeoff_requested_ = false;
    }

    if (!engaged_) {
      if (takeoff_requested_) {
        takeoff_requested_ = false;
        engaged_ = true;
        takeoff_active_ = true;
        RCLCPP_INFO(get_logger(), "engaging gamepad control");
      } else {
        return std::nullopt;
      }
    }

    if (status.nav_state != kNavigationStateOffboard) {
      if (!takeoff_active_) {
        RCLCPP_INFO(
          get_logger(), "left OFFBOARD (nav_state=%d); gamepad control disengaged",
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
    const double now = now_s();
    if (!have_status_) {
      return;
    }

    if (!have_joy_ && now - start_time_ > 3.0 && !waiting_logged_) {
      waiting_logged_ = true;
      RCLCPP_WARN(
        get_logger(),
        "waiting for %s; start joy_node or check the controller",
        joy_topic_.c_str());
    }

    const bool stale = !have_joy_ || (now - last_joy_time_ > cmd_timeout_);
    if (stale && have_joy_ && !stale_logged_) {
      stale_logged_ = true;
      RCLCPP_WARN(
        get_logger(), "no gamepad input for %.1f s; commanding zero velocity",
        cmd_timeout_);
    } else if (!stale) {
      stale_logged_ = false;
    }

    const geometry_msgs::msg::Twist cmd =
      stale ? geometry_msgs::msg::Twist() : mapped_command();
    const std::optional<geometry_msgs::msg::Twist> out = update(cmd, status_, now);
    if (out) {
      stream(*out);
    }
  }

  // ------------------------------------------------------------------ state
  std::string mode_;
  std::string joy_topic_;
  double max_xy_vel_ = 2.0;
  double max_z_vel_ = 1.5;
  double max_yaw_vel_ = 1.0;
  double deadzone_ = 0.1;
  int btn_debounce_ms_ = 200;
  double nonlinear_scale_ = 2.0;
  int left_x_idx_ = 0;
  int left_y_idx_ = 1;
  int right_x_idx_ = 3;
  int right_y_idx_ = 4;
  int takeoff_button_ = kButtonRb;
  int land_button_ = kButtonLb;
  double cmd_timeout_ = 1.0;
  double takeoff_altitude_ = 1.5;
  double takeoff_climb_rate_ = 0.5;
  double retry_ = 1.0;

  double left_x_ = 0.0;
  double left_y_ = 0.0;
  double right_x_ = 0.0;
  double right_y_ = 0.0;
  bool have_joy_ = false;
  double last_joy_time_ = 0.0;
  double start_time_ = 0.0;
  bool waiting_logged_ = false;
  bool stale_logged_ = false;
  std::map<int, bool> last_button_state_;
  std::map<int, double> last_button_time_;

  bool have_status_ = false;
  px4_msgs::msg::VehicleStatus status_;
  bool have_z_ = false;
  double z_ = 0.0;

  bool engaged_ = false;
  bool takeoff_active_ = false;
  bool takeoff_requested_ = false;
  bool land_requested_ = false;
  double last_arm_ = -std::numeric_limits<double>::infinity();
  double last_mode_ = -std::numeric_limits<double>::infinity();
  double last_land_ = -std::numeric_limits<double>::infinity();

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GamepadControl>());
  rclcpp::shutdown();
  return 0;
}
