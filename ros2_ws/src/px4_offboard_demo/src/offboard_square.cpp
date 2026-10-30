// PX4 offboard demo: arm, take off and fly a square, then land.
//
// Controlled through PX4's uXRCE-DDS/ROS 2 topics (px4_msgs).  Everything is in
// PX4 local NED coordinates: x = north, y = east, z = down (negative up).
//
// Run after MicroXRCEAgent and PX4 SITL are up, e.g.:
//     ros2 run px4_offboard_demo offboard_square --ros-args -p altitude:=2.5

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

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

float nan()
{
  return std::numeric_limits<float>::quiet_NaN();
}

// PX4 subscribes to /fmu/in/* with VOLATILE durability.  Matching it here also
// keeps a re-created DDS reader from being handed an old command after a
// time-sync reset.
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

class OffboardSquare : public rclcpp::Node
{
public:
  OffboardSquare()
  : Node("px4_offboard_square")
  {
    altitude_ = declare_parameter<double>("altitude", 2.5);
    const double side = declare_parameter<double>("square_size", 5.0);
    leg_seconds_ = declare_parameter<double>("leg_seconds", 8.0);
    hold_seconds_ = declare_parameter<double>("offboard_hold_seconds", 2.0);
    land_timeout_ = declare_parameter<double>("land_timeout", 90.0);

    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_in_qos());
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_in_qos());
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4_in_qos());

    const rclcpp::QoS qos = px4_out_qos();
    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&OffboardSquare::on_position, this, std::placeholders::_1));
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos,
      std::bind(&OffboardSquare::on_status, this, std::placeholders::_1));
    ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      std::bind(&OffboardSquare::on_command_ack, this, std::placeholders::_1));

    phase_start_ = now_s();

    legs_ = {{
      {{0.0, 0.0, -altitude_}},
      {{side, 0.0, -altitude_}},
      {{side, side, -altitude_}},
      {{0.0, side, -altitude_}},
      {{0.0, 0.0, -altitude_}},
    }};

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&OffboardSquare::tick, this));
    RCLCPP_INFO(
      get_logger(), "offboard demo ready: altitude=%g m, side=%g m", altitude_, side);
  }

  bool failed() const {return failed_;}

private:
  // ------------------------------------------------------------------ helpers
  double now_s()
  {
    return static_cast<double>(this->now().nanoseconds()) / 1e9;
  }

  uint64_t timestamp_us()
  {
    return static_cast<uint64_t>(this->now().nanoseconds()) / 1000;
  }

  void on_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    px_ = static_cast<double>(msg->x);
    py_ = static_cast<double>(msg->y);
    pz_ = static_cast<double>(msg->z);
    have_position_ = true;
    dist_bottom_ = static_cast<double>(msg->dist_bottom);
    dist_bottom_valid_ = msg->dist_bottom_valid;
    z_deriv_ = static_cast<double>(msg->z_deriv);
    last_position_time_ = now_s();
  }

  void on_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    status_ = *msg;
    have_status_ = true;
  }

  void on_command_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
  {
    if (msg->command == kMavCmdNavLand ||
      msg->command == kMavCmdComponentArmDisarm ||
      msg->command == kMavCmdDoSetMode)
    {
      RCLCPP_INFO(
        get_logger(), "command %u ack result=%u",
        static_cast<unsigned>(msg->command), static_cast<unsigned>(msg->result));
    }
    if (msg->command == kMavCmdNavLand && msg->result == 0) {
      land_ack_received_ = true;
      land_ack_time_ = now_s();
    }
  }

  bool armed() const
  {
    return have_status_ && status_.arming_state == kArmingStateArmed;
  }

  bool in_offboard() const
  {
    return have_status_ && status_.nav_state == kNavigationStateOffboard;
  }

  void publish_offboard_mode()
  {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = timestamp_us();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_mode_pub_->publish(msg);
  }

  void publish_setpoint(double x, double y, double z)
  {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = timestamp_us();
    msg.position = {
      static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    msg.velocity = {nan(), nan(), nan()};
    msg.acceleration = {nan(), nan(), nan()};
    msg.jerk = {nan(), nan(), nan()};
    msg.yaw = nan();
    msg.yawspeed = nan();
    setpoint_pub_->publish(msg);
  }

  void send_command(
    uint32_t command, float param1 = 0.0f, float param2 = 0.0f,
    float param3 = 0.0f, float param4 = 0.0f, double param5 = 0.0,
    double param6 = 0.0, float param7 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = timestamp_us();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  void switch_phase(const std::string & phase)
  {
    RCLCPP_INFO(get_logger(), "phase -> %s", phase.c_str());
    phase_ = phase;
    phase_start_ = now_s();
  }

  // --------------------------------------------------------------------- tick
  void tick()
  {
    // While OFFBOARD, PX4 leaves the mode if this stream stops.  Once the
    // land phase starts we stop on purpose: AUTO_LAND does not need it, and
    // a live offboard signal would let a stray mode command (e.g. another
    // offboard controller still running) put PX4 back into OFFBOARD and
    // freeze the descent at altitude.
    if (phase_ != "land" && phase_ != "done" && phase_ != "failed") {
      publish_offboard_mode();
      if (phase_ != "wait_position") {
        if (!have_position_) {
          publish_setpoint(0.0, 0.0, -altitude_);
        } else {
          publish_setpoint(px_, py_, pz_);
        }
      }
    }

    if (phase_ == "wait_position") {
      if (have_position_) {
        switch_phase("prestream");
      }
      return;
    }

    const double elapsed = now_s() - phase_start_;

    if (phase_ == "prestream") {
      if (elapsed >= hold_seconds_) {
        switch_phase("arm");
      }
      return;
    }

    if (phase_ == "arm") {
      if (armed()) {
        switch_phase("set_offboard");
        return;
      }
      if (now_s() - last_arm_request_ > 1.0) {
        last_arm_request_ = now_s();
        RCLCPP_INFO(get_logger(), "sending arm command");
        send_command(kMavCmdComponentArmDisarm, 1.0f);
      }
      if (elapsed > 60.0) {
        RCLCPP_ERROR(
          get_logger(), "arming timed out (check pre-flight checks / GPS)");
        switch_phase("failed");
      }
      return;
    }

    if (phase_ == "set_offboard") {
      if (in_offboard()) {
        RCLCPP_INFO(get_logger(), "OFFBOARD engaged");
        switch_phase("takeoff");
        return;
      }
      if (now_s() - last_mode_request_ > 1.0) {
        last_mode_request_ = now_s();
        RCLCPP_INFO(get_logger(), "requesting OFFBOARD mode");
        send_command(kMavCmdDoSetMode, 1.0f, kPx4CustomMainModeOffboard);
      }
      if (elapsed > 30.0) {
        RCLCPP_ERROR(get_logger(), "OFFBOARD mode not accepted");
        switch_phase("failed");
      }
      return;
    }

    if (phase_ == "takeoff") {
      publish_setpoint(0.0, 0.0, -altitude_);
      if (have_position_ && std::abs(pz_ + altitude_) < 0.3) {
        RCLCPP_INFO(get_logger(), "altitude reached, starting square");
        leg_index_ = 0;
        switch_phase("square");
      }
      return;
    }

    if (phase_ == "square") {
      const auto & target = legs_[leg_index_];
      publish_setpoint(target[0], target[1], target[2]);
      if (!leg_start_) {
        leg_start_ = now_s();
        RCLCPP_INFO(
          get_logger(), "waypoint %zu: (%g, %g, %g)", leg_index_,
          target[0], target[1], target[2]);
      } else if (now_s() - *leg_start_ >= leg_seconds_) {
        ++leg_index_;
        leg_start_.reset();
        if (leg_index_ >= legs_.size()) {
          switch_phase("land");
        }
      }
      return;
    }

    if (phase_ == "land") {
      const double now = now_s();
      // If something else switched PX4 back to OFFBOARD after LAND was
      // accepted (another offboard controller, or a replayed mode command),
      // AUTO_LAND was abandoned.  Ask to land again.
      if (land_ack_received_ &&
        now - land_ack_time_ > 1.0 &&
        armed() &&
        status_.nav_state == kNavigationStateOffboard &&
        !left_land_logged_)
      {
        left_land_logged_ = true;
        RCLCPP_WARN(
          get_logger(), "PX4 left AUTO_LAND and is back in OFFBOARD; re-sending LAND");
        land_ack_received_ = false;
      }

      // Send LAND up to three times, one second apart. A single best-effort
      // command can be lost while the DDS time sync is re-converging.  Do not
      // keep resending forever: PX4 restarts its COM_DISARM_LAND timer on
      // every LAND command, so a steady stream lands the vehicle but never
      // disarms it.
      if (!land_ack_received_ && land_attempts_ < 3 &&
        now - last_land_request_ >= 1.0)
      {
        ++land_attempts_;
        last_land_request_ = now;
        RCLCPP_INFO(
          get_logger(),
          "sending LAND command (attempt %d/3, z=%.2f m, dist_bottom=%.2f m)",
          land_attempts_, pz_, dist_bottom_);
        send_command(kMavCmdNavLand);
      }

      if (land_attempts_ >= 3 && !land_ack_received_ && !land_ack_warned_) {
        land_ack_warned_ = true;
        RCLCPP_WARN(get_logger(), "no LAND command ack after 3 attempts");
      }

      if (have_status_ &&
        status_.nav_state == kNavigationStateAutoLand && !auto_land_logged_)
      {
        auto_land_logged_ = true;
        RCLCPP_INFO(get_logger(), "PX4 entered AUTO_LAND");
      }

      if (have_status_ && status_.arming_state == kArmingStateDisarmed) {
        RCLCPP_INFO(get_logger(), "landed and disarmed - demo finished");
        switch_phase("done");
        return;
      }

      if (last_position_time_ > 0.0 &&
        now - last_position_time_ > 2.0 && !telemetry_lost_logged_)
      {
        telemetry_lost_logged_ = true;
        RCLCPP_WARN(
          get_logger(),
          "no local position for 2 s; touchdown detection may be stale");
      }

      // PX4's auto-land is slow in the last metres, so do not rely on a
      // wall-clock deadline.  Use the range/down-velocity estimate to
      // decide when the vehicle is actually on the ground.
      const bool on_ground =
        dist_bottom_valid_ && dist_bottom_ < 0.25 && std::abs(z_deriv_) < 0.15;
      if (on_ground) {
        if (!landed_since_) {
          landed_since_ = elapsed;
          RCLCPP_INFO(
            get_logger(),
            "touchdown detected (dist_bottom=%.2f m), waiting for auto-disarm",
            dist_bottom_);
        } else if (!disarm_requested_ && elapsed - *landed_since_ > 4.0) {
          disarm_requested_ = true;
          RCLCPP_WARN(
            get_logger(), "still armed 4 s after touchdown, sending disarm");
          send_command(kMavCmdComponentArmDisarm, 0.0f);
        }
      } else {
        landed_since_.reset();
      }

      if (elapsed > land_timeout_) {
        RCLCPP_ERROR(get_logger(), "landing timed out");
        switch_phase("failed");
      }
      return;
    }

    if (phase_ == "done") {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "shutting down");
      rclcpp::shutdown();
      return;
    }

    if (phase_ == "failed") {
      timer_->cancel();
      failed_ = true;
      RCLCPP_INFO(get_logger(), "shutting down after failure");
      rclcpp::shutdown();
      return;
    }
  }

  double altitude_ = 2.5;
  double leg_seconds_ = 8.0;
  double hold_seconds_ = 2.0;
  double land_timeout_ = 90.0;

  bool have_position_ = false;
  double px_ = 0.0;
  double py_ = 0.0;
  double pz_ = 0.0;
  double dist_bottom_ = std::numeric_limits<double>::quiet_NaN();
  bool dist_bottom_valid_ = false;
  double z_deriv_ = 0.0;
  double last_position_time_ = 0.0;

  bool have_status_ = false;
  px4_msgs::msg::VehicleStatus status_;

  std::string phase_ = "wait_position";
  double phase_start_ = 0.0;
  double last_arm_request_ = 0.0;
  double last_mode_request_ = 0.0;
  int land_attempts_ = 0;
  double last_land_request_ = 0.0;
  bool land_ack_received_ = false;
  double land_ack_time_ = 0.0;
  bool left_land_logged_ = false;
  bool land_ack_warned_ = false;
  bool auto_land_logged_ = false;
  bool telemetry_lost_logged_ = false;
  bool disarm_requested_ = false;
  std::optional<double> landed_since_;
  bool failed_ = false;

  using Leg = std::array<double, 3>;
  std::array<Leg, 5> legs_{};
  size_t leg_index_ = 0;
  std::optional<double> leg_start_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardSquare>();
  rclcpp::spin(node);
  const bool failed = node->failed();
  rclcpp::shutdown();
  return failed ? 1 : 0;
}
