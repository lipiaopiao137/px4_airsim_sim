// Bridge AirSim sensors and vehicle pose into ROS 2.
//
// Published topics
//     /airsim/camera/image_raw      sensor_msgs/Image        (bgr8, "front" camera)
//     /airsim/camera/camera_info    sensor_msgs/CameraInfo
//     /airsim/depth/image           sensor_msgs/Image        (32FC1, metres, DepthPlanar)
//     /airsim/depth/camera_info     sensor_msgs/CameraInfo
//     /airsim/depth/points          sensor_msgs/PointCloud2  (base_link)
//     /airsim/lidar/points          sensor_msgs/PointCloud2  (base_link)
//     /airsim/odom                  nav_msgs/Odometry        (map -> base_link)
//     /airsim/path                  nav_msgs/Path            (trajectory in map)
//     /tf                           map -> base_link
//
// Coordinate conventions
//     * AirSim world / body frames are NED; ROS is ENU / FLU.  All conversions
//       happen here so that RViz can use a normal ENU fixed frame.
//     * AirSim camera optical frame (x right, y down, z forward) is rotated
//       into the ROS body frame (x forward, y left, z up).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "airsim_ros2_bridge/frames.hpp"
#include "airsim_ros2_bridge/rpc.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace airsim_bridge
{

namespace
{

// AirSim ImageType values
constexpr int kImageScene = 0;
constexpr int kImageDepthPlanar = 1;

constexpr double kPi = 3.14159265358979323846;

// Camera offset in the body frame (FLU), mirrors settings.json (X=0.25, Z=-0.10 NED)
const double kCameraOffsetFlu[3] = {0.25, 0.0, 0.10};
// Lidar mounting height above the vehicle origin, metres (Z=-0.3 NED)
const double kLidarOffsetFlu[3] = {0.0, 0.0, 0.3};

double require_double(const msgpack::Value & object, const char * key)
{
  const msgpack::Value * value = object.find(key);
  if (value == nullptr) {
    throw AirSimRpcError(std::string("missing field: ") + key);
  }
  return value->as_double();
}

std::vector<float> to_floats(const msgpack::Value & value)
{
  std::vector<float> numbers;
  if (value.type() == msgpack::Value::Type::Bin) {
    const std::string & raw = value.as_string();
    numbers.resize(raw.size() / sizeof(float));
    if (!numbers.empty()) {
      std::memcpy(numbers.data(), raw.data(), numbers.size() * sizeof(float));
    }
    return numbers;
  }
  if (value.type() == msgpack::Value::Type::Array) {
    numbers.reserve(value.size());
    for (size_t i = 0; i < value.size(); ++i) {
      numbers.push_back(static_cast<float>(value.at(i).as_double()));
    }
    return numbers;
  }
  throw AirSimRpcError("expected a binary or numeric array payload");
}

sensor_msgs::msg::PointCloud2 point_cloud2(
  const std::vector<float> & points, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size() / 3);
  msg.fields.resize(3);
  const char * names[3] = {"x", "y", "z"};
  for (size_t i = 0; i < 3; ++i) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<uint32_t>(i * 4);
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }
  msg.is_bigendian = false;
  msg.point_step = 12;
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = false;
  msg.data.resize(points.size() * sizeof(float));
  if (!points.empty()) {
    std::memcpy(msg.data.data(), points.data(), msg.data.size());
  }
  return msg;
}

sensor_msgs::msg::CameraInfo camera_info(
  int width, int height, double hfov_deg, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(width);
  msg.height = static_cast<uint32_t>(height);
  const double fx =
    (static_cast<double>(width) * 0.5) / std::tan(hfov_deg * kPi / 180.0 * 0.5);
  const double fy = fx;
  const double cx = static_cast<double>(width) * 0.5;
  const double cy = static_cast<double>(height) * 0.5;
  msg.distortion_model = "plumb_bob";
  msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return msg;
}

}  // namespace

class AirSimSensors : public rclcpp::Node
{
public:
  AirSimSensors()
  : Node("airsim_sensors")
  {
    ip_ = declare_parameter<std::string>("airsim_ip", "127.0.0.1");
    port_ = static_cast<int>(declare_parameter<int64_t>("airsim_port", 41451));
    vehicle_ = declare_parameter<std::string>("vehicle_name", "PX4");
    camera_ = declare_parameter<std::string>("camera_name", "front");
    lidar_ = declare_parameter<std::string>("lidar_name", "Lidar");
    world_ = declare_parameter<std::string>("world_frame", "map");
    body_ = declare_parameter<std::string>("body_frame", "base_link");
    rgb_rate_ = std::max(0.5, declare_parameter<double>("rgb_rate_hz", 10.0));
    depth_rate_ = std::max(0.5, declare_parameter<double>("depth_rate_hz", 5.0));
    lidar_rate_ = std::max(0.5, declare_parameter<double>("lidar_rate_hz", 10.0));
    pose_rate_ = std::max(1.0, declare_parameter<double>("pose_rate_hz", 20.0));
    path_rate_ = std::max(0.0, declare_parameter<double>("path_rate_hz", 2.0));
    path_max_ = std::max(0, static_cast<int>(
        declare_parameter<int64_t>("path_max_points", 2000)));
    min_depth_ = declare_parameter<double>("min_depth_m", 0.3);
    max_depth_ = declare_parameter<double>("max_depth_m", 40.0);
    stride_ = std::max(1, static_cast<int>(declare_parameter<int64_t>("depth_stride", 2)));

    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/airsim/camera/image_raw", 10);
    rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/airsim/camera/camera_info", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/airsim/depth/image", 10);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/airsim/depth/camera_info", 10);
    depth_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/airsim/depth/points", 10);
    lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/airsim/lidar/points", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/airsim/odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/airsim/path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (path_max_ > 0) {
      path_ = std::make_unique<std::deque<geometry_msgs::msg::PoseStamped>>();
    }

    threads_.emplace_back(&AirSimSensors::pose_loop, this);
    threads_.emplace_back(&AirSimSensors::rgb_loop, this);
    threads_.emplace_back(&AirSimSensors::depth_loop, this);
    threads_.emplace_back(&AirSimSensors::lidar_loop, this);
  }

  ~AirSimSensors() override
  {
    stop();
  }

  void stop()
  {
    if (stopped_.exchange(true)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(wait_mutex_);
    }
    wait_cv_.notify_all();
    for (auto & thread : threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

private:
  // ------------------------------------------------------------------ utils
  std::unique_ptr<AirSimRpcClient> connect() const
  {
    return std::make_unique<AirSimRpcClient>(ip_, port_, 15.0);
  }

  void log_waiting(const char * what, const std::string & error)
  {
    if (!hinted_ready_.load()) {
      RCLCPP_WARN(
        get_logger(), "waiting for AirSim at %s:%d (%s): %s",
        ip_.c_str(), port_, what, error.c_str());
    }
  }

  void prepare(AirSimRpcClient & client, const char * what)
  {
    if (!hinted_ready_.exchange(true)) {
      RCLCPP_INFO(get_logger(), "AirSim connected, %s stream started", what);
    }
    try {
      msgpack::Value info = client.call(
        "simGetCameraInfo", {
          msgpack::Value::make_string(camera_),
          msgpack::Value::make_string(vehicle_),
          msgpack::Value::make_bool(false),
        });
      const msgpack::Value * fov = info.find("fov");
      if (fov != nullptr) {
        const double value = fov->as_double();
        if (value > 1.0) {
          std::lock_guard<std::mutex> lock(hfov_mutex_);
          hfov_ = value;
        }
      }
    } catch (const std::exception &) {
      // FOV is only used for CameraInfo / depth reprojection; keep the default.
    }
  }

  double hfov()
  {
    std::lock_guard<std::mutex> lock(hfov_mutex_);
    return hfov_;
  }

  void wait_period(double seconds)
  {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    wait_cv_.wait_for(
      lock, std::chrono::duration<double>(seconds),
      [this]() {return stopped_.load();});
  }

  void record_pose(
    const double p_enu[3], const double q_enu[4], const rclcpp::Time & stamp)
  {
    if (!path_ || path_rate_ <= 0.0) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_;
    pose.pose.position.x = p_enu[0];
    pose.pose.position.y = p_enu[1];
    pose.pose.position.z = p_enu[2];
    pose.pose.orientation.w = q_enu[0];
    pose.pose.orientation.x = q_enu[1];
    pose.pose.orientation.y = q_enu[2];
    pose.pose.orientation.z = q_enu[3];
    path_->push_back(pose);
    while (path_->size() > static_cast<size_t>(path_max_)) {
      path_->pop_front();
    }

    const double now = static_cast<double>(this->now().nanoseconds()) / 1e9;
    if (now - last_path_time_ < 1.0 / path_rate_) {
      return;
    }
    last_path_time_ = now;

    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = world_;
    path.poses.assign(path_->begin(), path_->end());
    path_pub_->publish(path);
  }

  // ------------------------------------------------------------------- pose
  void pose_loop()
  {
    const double period = 1.0 / pose_rate_;
    std::unique_ptr<AirSimRpcClient> client;
    while (!stopped_.load()) {
      try {
        if (!client) {
          client = connect();
          prepare(*client, "pose");
          if (path_ && !path_->empty()) {
            path_->clear();
            RCLCPP_INFO(get_logger(), "AirSim reconnected; trajectory cleared");
          }
        }
        msgpack::Value pose = client->call(
          "simGetVehiclePose", {msgpack::Value::make_string(vehicle_)});
        const msgpack::Value * position = pose.find("position");
        const msgpack::Value * orientation = pose.find("orientation");
        if (position == nullptr || orientation == nullptr) {
          throw AirSimRpcError("vehicle pose missing position/orientation");
        }
        const double position_ned[3] = {
          require_double(*position, "x_val"),
          require_double(*position, "y_val"),
          require_double(*position, "z_val"),
        };
        const double orientation_ned[4] = {
          require_double(*orientation, "w_val"),
          require_double(*orientation, "x_val"),
          require_double(*orientation, "y_val"),
          require_double(*orientation, "z_val"),
        };
        double p_enu[3];
        double q_enu[4];
        ned_pose_to_enu(position_ned, orientation_ned, p_enu, q_enu);

        const rclcpp::Time stamp = this->now();

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = world_;
        tf.child_frame_id = body_;
        tf.transform.translation.x = p_enu[0];
        tf.transform.translation.y = p_enu[1];
        tf.transform.translation.z = p_enu[2];
        tf.transform.rotation.w = q_enu[0];
        tf.transform.rotation.x = q_enu[1];
        tf.transform.rotation.y = q_enu[2];
        tf.transform.rotation.z = q_enu[3];
        tf_broadcaster_->sendTransform(tf);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = world_;
        odom.child_frame_id = body_;
        odom.pose.pose.position.x = p_enu[0];
        odom.pose.pose.position.y = p_enu[1];
        odom.pose.pose.position.z = p_enu[2];
        odom.pose.pose.orientation.w = q_enu[0];
        odom.pose.pose.orientation.x = q_enu[1];
        odom.pose.pose.orientation.y = q_enu[2];
        odom.pose.pose.orientation.z = q_enu[3];
        odom_pub_->publish(odom);
        record_pose(p_enu, q_enu, stamp);
      } catch (const std::exception & error) {
        log_waiting("pose", error.what());
        if (client) {
          client->close();
          client.reset();
        }
      }
      wait_period(period);
    }
    if (client) {
      client->close();
    }
  }

  // -------------------------------------------------------------------- rgb
  void rgb_loop()
  {
    const double period = 1.0 / rgb_rate_;
    std::unique_ptr<AirSimRpcClient> client;
    while (!stopped_.load()) {
      try {
        if (!client) {
          client = connect();
          prepare(*client, "camera");
        }
        msgpack::Value png = client->call(
          "simGetImage", {
            msgpack::Value::make_string(camera_),
            msgpack::Value::make_int(kImageScene),
            msgpack::Value::make_string(vehicle_),
            msgpack::Value::make_bool(false),
          });
        if (png.type() == msgpack::Value::Type::Bin && !png.as_string().empty()) {
          const std::string & raw = png.as_string();
          cv::Mat encoded(
            1, static_cast<int>(raw.size()), CV_8U,
            const_cast<char *>(raw.data()));
          const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
          if (!image.empty()) {
            const rclcpp::Time stamp = this->now();
            const int height = image.rows;
            const int width = image.cols;
            sensor_msgs::msg::Image msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = body_;
            msg.height = static_cast<uint32_t>(height);
            msg.width = static_cast<uint32_t>(width);
            msg.encoding = "bgr8";
            msg.is_bigendian = 0;
            msg.step = static_cast<uint32_t>(width * 3);
            const auto bytes = image.total() * image.elemSize();
            msg.data.assign(image.data, image.data + bytes);
            rgb_pub_->publish(msg);
            rgb_info_pub_->publish(camera_info(width, height, hfov(), body_, stamp));
          }
        }
      } catch (const std::exception & error) {
        log_waiting("camera", error.what());
        if (client) {
          client->close();
          client.reset();
        }
      }
      wait_period(period);
    }
    if (client) {
      client->close();
    }
  }

  // ------------------------------------------------------------------ depth
  void depth_loop()
  {
    const double period = 1.0 / depth_rate_;
    std::unique_ptr<AirSimRpcClient> client;
    while (!stopped_.load()) {
      try {
        if (!client) {
          client = connect();
          prepare(*client, "depth");
        }

        msgpack::Value request = msgpack::Value::make_array();
        msgpack::Value options = msgpack::Value::make_map();
        options.set("camera_name", msgpack::Value::make_string(camera_));
        options.set("image_type", msgpack::Value::make_int(kImageDepthPlanar));
        options.set("pixels_as_float", msgpack::Value::make_bool(true));
        options.set("compress", msgpack::Value::make_bool(false));
        request.push(std::move(options));

        msgpack::Value responses = client->call(
          "simGetImages", {
            std::move(request),
            msgpack::Value::make_string(vehicle_),
            msgpack::Value::make_bool(false),
          });
        if (responses.type() != msgpack::Value::Type::Array || responses.size() == 0) {
          throw AirSimRpcError("empty simGetImages response");
        }
        const msgpack::Value & response = responses.at(0);
        const int width = static_cast<int>(require_double(response, "width"));
        const int height = static_cast<int>(require_double(response, "height"));
        const msgpack::Value * raw = response.find("image_data_float");
        if (width <= 0 || height <= 0 || raw == nullptr) {
          throw AirSimRpcError("depth image missing in response");
        }
        const std::vector<float> depth = to_floats(*raw);
        if (depth.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
          throw AirSimRpcError("depth image has an unexpected size");
        }

        const rclcpp::Time stamp = this->now();
        sensor_msgs::msg::Image msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = body_;
        msg.height = static_cast<uint32_t>(height);
        msg.width = static_cast<uint32_t>(width);
        msg.encoding = "32FC1";
        msg.is_bigendian = 0;
        msg.step = static_cast<uint32_t>(width * 4);
        msg.data.resize(depth.size() * sizeof(float));
        std::memcpy(msg.data.data(), depth.data(), msg.data.size());
        depth_pub_->publish(msg);
        depth_info_pub_->publish(camera_info(width, height, hfov(), body_, stamp));

        const std::vector<float> points = depth_to_points(depth, width, height);
        if (!points.empty()) {
          depth_cloud_pub_->publish(point_cloud2(points, body_, stamp));
        }
      } catch (const std::exception & error) {
        log_waiting("depth", error.what());
        if (client) {
          client->close();
          client.reset();
        }
      }
      wait_period(period);
    }
    if (client) {
      client->close();
    }
  }

  std::vector<float> depth_to_points(
    const std::vector<float> & depth, int width, int height)
  {
    const int stride = stride_;
    const double hfov_rad = hfov() * kPi / 180.0;
    const double fx =
      (static_cast<double>(width) * 0.5) / std::tan(hfov_rad * 0.5);
    const double fy = fx;
    const double cx = static_cast<double>(width) * 0.5;
    const double cy = static_cast<double>(height) * 0.5;

    std::vector<float> points;
    points.reserve(
      static_cast<size_t>((width + stride - 1) / stride) *
      static_cast<size_t>((height + stride - 1) / stride) * 3);

    for (int row = 0; row < height; row += stride) {
      const double v = (static_cast<double>(row) - cy) / fy;
      const float * line = depth.data() + static_cast<size_t>(row) * width;
      for (int col = 0; col < width; col += stride) {
        const double z = line[col];
        if (!std::isfinite(z) || z <= min_depth_ || z >= max_depth_) {
          continue;
        }
        const double u = (static_cast<double>(col) - cx) / fx;
        // optical (x right, y down, z forward) -> body FLU (x fwd, y left, z up)
        points.push_back(static_cast<float>(z) + kCameraOffsetFlu[0]);
        points.push_back(static_cast<float>(-u * z) + kCameraOffsetFlu[1]);
        points.push_back(static_cast<float>(-v * z) + kCameraOffsetFlu[2]);
      }
    }
    return points;
  }

  // ------------------------------------------------------------------ lidar
  void lidar_loop()
  {
    const double period = 1.0 / lidar_rate_;
    std::unique_ptr<AirSimRpcClient> client;
    while (!stopped_.load()) {
      try {
        if (!client) {
          client = connect();
          prepare(*client, "lidar");
        }
        msgpack::Value data = client->call(
          "getLidarData", {
            msgpack::Value::make_string(lidar_),
            msgpack::Value::make_string(vehicle_),
          });
        const msgpack::Value * raw = data.find("point_cloud");
        if (raw == nullptr) {
          throw AirSimRpcError("lidar response has no point_cloud");
        }
        const std::vector<float> cloud = to_floats(*raw);
        if (!cloud.empty()) {
          if (cloud.size() % 3 != 0) {
            throw AirSimRpcError("lidar point cloud size is not a multiple of 3");
          }
          std::vector<float> flu(cloud.size());
          for (size_t i = 0; i < cloud.size(); i += 3) {
            // sensor frame is NED (x fwd, y right, z down) -> body FLU
            flu[i] = cloud[i] + kLidarOffsetFlu[0];
            flu[i + 1] = -cloud[i + 1] + kLidarOffsetFlu[1];
            flu[i + 2] = -cloud[i + 2] + kLidarOffsetFlu[2];
          }
          lidar_pub_->publish(point_cloud2(flu, body_, this->now()));
        }
      } catch (const std::exception & error) {
        log_waiting("lidar", error.what());
        if (client) {
          client->close();
          client.reset();
        }
      }
      wait_period(period);
    }
    if (client) {
      client->close();
    }
  }

  // ------------------------------------------------------------- parameters
  std::string ip_;
  int port_ = 41451;
  std::string vehicle_;
  std::string camera_;
  std::string lidar_;
  std::string world_;
  std::string body_;
  double rgb_rate_ = 10.0;
  double depth_rate_ = 5.0;
  double lidar_rate_ = 10.0;
  double pose_rate_ = 20.0;
  double path_rate_ = 2.0;
  int path_max_ = 2000;
  double min_depth_ = 0.3;
  double max_depth_ = 40.0;
  int stride_ = 2;

  // --------------------------------------------------------------- runtime
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<std::deque<geometry_msgs::msg::PoseStamped>> path_;
  double last_path_time_ = 0.0;

  double hfov_ = 90.0;
  std::mutex hfov_mutex_;
  std::atomic<bool> hinted_ready_{false};

  std::atomic<bool> stopped_{false};
  std::mutex wait_mutex_;
  std::condition_variable wait_cv_;
  std::vector<std::thread> threads_;
};

}  // namespace airsim_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<airsim_bridge::AirSimSensors>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception &) {
  }
  node->stop();
  rclcpp::shutdown();
  return 0;
}
