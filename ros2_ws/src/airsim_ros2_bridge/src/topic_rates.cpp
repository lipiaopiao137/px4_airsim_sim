// Print real publish rates for the /airsim/* topics.
//
// `ros2 topic hz` is unreliable on this machine, so this node subscribes to
// every bridge topic and counts messages for a fixed duration instead:
//
//     ros2 run airsim_ros2_bridge topic_rates --ros-args -p duration_s:=20.0

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace airsim_bridge
{

class TopicRates : public rclcpp::Node
{
public:
  TopicRates()
  : Node("topic_rates")
  {
    duration_ = declare_parameter<double>("duration_s", 20.0);
    start_ = std::chrono::steady_clock::now();

    subscribe<nav_msgs::msg::Odometry>("/airsim/odom");
    subscribe<nav_msgs::msg::Path>("/airsim/path");
    subscribe<sensor_msgs::msg::Image>("/airsim/camera/image_raw");
    subscribe<sensor_msgs::msg::PointCloud2>("/airsim/depth/points");
    subscribe<sensor_msgs::msg::PointCloud2>("/airsim/lidar/points");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&TopicRates::tick, this));
  }

private:
  template<typename MessageT>
  void subscribe(const std::string & topic)
  {
    topics_.push_back(topic);
    counts_.push_back(0);
    const size_t index = topics_.size() - 1;
    subscriptions_.push_back(
      create_subscription<MessageT>(
        topic, 10,
        [this, index](typename MessageT::ConstSharedPtr) {
          ++counts_[index];
        }));
  }

  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - start_).count();
    if (elapsed < duration_) {
      return;
    }
    for (size_t i = 0; i < topics_.size(); ++i) {
      std::printf(
        "%-28s %5zu msgs  %5.2f Hz\n",
        topics_[i].c_str(), counts_[i],
        static_cast<double>(counts_[i]) / elapsed);
    }
    timer_->cancel();
    rclcpp::shutdown();
  }

  double duration_ = 20.0;
  std::chrono::steady_clock::time_point start_;
  std::vector<std::string> topics_;
  std::vector<size_t> counts_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace airsim_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<airsim_bridge::TopicRates>());
  rclcpp::shutdown();
  return 0;
}
