// Keyboard teleoperation for PX4 offboard flight.
//
// Reads single key presses from the terminal and publishes a velocity command
// in PX4 local NED coordinates (x north, y east, z down) plus a yaw rate:
//
//     /keyboard/velocity_cmd   geometry_msgs/Twist
//         linear.x/y/z   m/s in NED (z positive = descend)
//         angular.z      yaw rate in rad/s (positive = clockwise, i.e. right)
//
//     /keyboard/land           std_msgs/Bool (True on every 'g' press)
//
// A key press steps the corresponding velocity up; when no mapped key is seen
// the command decays back to zero.  Terminals do not report key-up events, so
// this is the same press-to-accelerate scheme used by the px4_keyboard_control project.
//
// Key map
//     w / s   north / south          (x)
//     a / d   west / east            (y)
//     i / k   up / down              (z)
//     j / l   yaw left / right
//     h       hover (zero command)
//     g       land
//     q       quit (Ctrl-C also works)

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace
{

enum Axis
{
  AXIS_X = 0,
  AXIS_Y = 1,
  AXIS_Z = 2,
  AXIS_YAW = 3,
  AXIS_COUNT = 4,
};

const char * kKeyHelp =
  "keyboard ready: w/s north/south, a/d west/east, i/k up/down, "
  "j/l yaw left/right, h hover, g land, q quit";

}  // namespace

class KeyboardVelocity : public rclcpp::Node
{
public:
  KeyboardVelocity()
  : Node("keyboard_velocity")
  {
    const double step_xy = declare_parameter<double>("step_xy", 0.1);
    const double max_xy = declare_parameter<double>("max_xy", 2.0);
    const double step_z = declare_parameter<double>("step_z", 0.1);
    const double max_z = declare_parameter<double>("max_z", 1.5);
    const double step_yaw = declare_parameter<double>("step_yaw", 0.1);
    const double max_yaw = declare_parameter<double>("max_yaw", 1.0);
    const double rate =
      std::max(1.0, declare_parameter<double>("publish_rate_hz", 20.0));

    step_ = {step_xy, step_xy, step_z, step_yaw};
    max_ = {max_xy, max_xy, max_z, max_yaw};
    cmd_.fill(0.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/keyboard/velocity_cmd", 10);
    land_pub_ = create_publisher<std_msgs::msg::Bool>("/keyboard/land", 10);

    tty_ = ::isatty(STDIN_FILENO) != 0;
    if (tty_) {
      if (::tcgetattr(STDIN_FILENO, &old_term_) == 0) {
        have_old_term_ = true;
        struct termios mode = old_term_;
        // Same as Python's tty.setcbreak(): no echo, no line buffering, no
        // signal generation, so Ctrl-C arrives as a 0x03 byte.
        mode.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        mode.c_cflag &= ~(CSIZE | PARENB);
        mode.c_cflag |= CS8;
        mode.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
        mode.c_cc[VMIN] = 1;
        mode.c_cc[VTIME] = 0;
        ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &mode);
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "stdin is not a TTY; keyboard input is disabled "
        "(run this node in a terminal, not backgrounded/redirected)");
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&KeyboardVelocity::tick, this));
    RCLCPP_INFO(get_logger(), "%s", kKeyHelp);
  }

  ~KeyboardVelocity() override
  {
    restore_terminal();
  }

private:
  // ------------------------------------------------------------------ input
  std::string read_keys()
  {
    if (!tty_) {
      return std::string();
    }
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(STDIN_FILENO, &readable);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    const int ready =
      ::select(STDIN_FILENO + 1, &readable, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return std::string();
    }
    char buffer[16];
    const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (count <= 0) {
      return std::string();
    }
    return std::string(buffer, static_cast<size_t>(count));
  }

  void restore_terminal()
  {
    if (have_old_term_) {
      ::tcsetattr(STDIN_FILENO, TCSADRAIN, &old_term_);
      have_old_term_ = false;
    }
  }

  // --------------------------------------------------------------- commands
  void decay_axis(int axis)
  {
    double value = cmd_[axis];
    const double step = step_[axis];
    if (value > 0.0) {
      value = std::max(0.0, value - step);
    } else if (value < 0.0) {
      value = std::min(0.0, value + step);
    }
    if (std::abs(value) < 1e-6) {
      value = 0.0;
    }
    cmd_[axis] = value;
  }

  void decay_all()
  {
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      decay_axis(axis);
    }
  }

  void decay_others(int keep)
  {
    for (int axis = 0; axis < AXIS_COUNT; ++axis) {
      if (axis != keep) {
        decay_axis(axis);
      }
    }
  }

  void bump(int axis, double sign)
  {
    const double step = step_[axis];
    const double limit = max_[axis];
    cmd_[axis] = std::max(-limit, std::min(limit, cmd_[axis] + sign * step));
    decay_others(axis);
  }

  bool handle_key(char key)
  {
    switch (key) {
      case 'w': bump(AXIS_X, +1.0); break;
      case 's': bump(AXIS_X, -1.0); break;
      case 'a': bump(AXIS_Y, -1.0); break;
      case 'd': bump(AXIS_Y, +1.0); break;
      case 'i': bump(AXIS_Z, -1.0); break;
      case 'k': bump(AXIS_Z, +1.0); break;
      case 'j': bump(AXIS_YAW, -1.0); break;
      case 'l': bump(AXIS_YAW, +1.0); break;
      case 'h':
        cmd_.fill(0.0);
        break;
      case 'g': {
          std_msgs::msg::Bool land;
          land.data = true;
          land_pub_->publish(land);
          RCLCPP_INFO(get_logger(), "land requested");
          break;
        }
      default:
        return false;
    }
    return true;
  }

  void publish()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = cmd_[AXIS_X];
    msg.linear.y = cmd_[AXIS_Y];
    msg.linear.z = cmd_[AXIS_Z];
    msg.angular.z = cmd_[AXIS_YAW];
    cmd_pub_->publish(msg);
  }

  // ------------------------------------------------------------------- tick
  void tick()
  {
    bool handled = false;
    for (char key : read_keys()) {
      if (key == 'q' || key == '\x03') {
        RCLCPP_INFO(get_logger(), "quit requested");
        restore_terminal();
        rclcpp::shutdown();
        return;
      }
      if (handle_key(key)) {
        handled = true;
      }
    }
    if (!handled) {
      decay_all();
    }
    publish();
  }

  std::array<double, AXIS_COUNT> step_{};
  std::array<double, AXIS_COUNT> max_{};
  std::array<double, AXIS_COUNT> cmd_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr land_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool tty_ = false;
  bool have_old_term_ = false;
  struct termios old_term_ {};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardVelocity>());
  rclcpp::shutdown();
  return 0;
}
