import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("airsim_ros2_bridge")

    args = [
        DeclareLaunchArgument("airsim_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("airsim_port", default_value="41451"),
        DeclareLaunchArgument("vehicle_name", default_value="PX4"),
        DeclareLaunchArgument("camera_name", default_value="front"),
        DeclareLaunchArgument("lidar_name", default_value="Lidar"),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    sensors = Node(
        package="airsim_ros2_bridge",
        executable="airsim_sensors",
        name="airsim_sensors",
        output="screen",
        parameters=[{
            "airsim_ip": LaunchConfiguration("airsim_ip"),
            "airsim_port": LaunchConfiguration("airsim_port"),
            "vehicle_name": LaunchConfiguration("vehicle_name"),
            "camera_name": LaunchConfiguration("camera_name"),
            "lidar_name": LaunchConfiguration("lidar_name"),
            "rgb_rate_hz": 10.0,
            "depth_rate_hz": 5.0,
            "lidar_rate_hz": 10.0,
            "pose_rate_hz": 20.0,
            "path_rate_hz": 2.0,
            "path_max_points": 2000,
            "depth_stride": 2,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(share, "config", "airsim.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(args + [sensors, rviz])
