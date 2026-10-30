from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("altitude", default_value="2.5"),
        DeclareLaunchArgument("square_size", default_value="5.0"),
        DeclareLaunchArgument("leg_seconds", default_value="8.0"),
        DeclareLaunchArgument("offboard_hold_seconds", default_value="2.0"),
        DeclareLaunchArgument("land_timeout", default_value="90.0"),
        Node(
            package="px4_offboard_demo",
            executable="offboard_square",
            name="px4_offboard_square",
            output="screen",
            parameters=[{
                "altitude": LaunchConfiguration("altitude"),
                "square_size": LaunchConfiguration("square_size"),
                "leg_seconds": LaunchConfiguration("leg_seconds"),
                "offboard_hold_seconds": LaunchConfiguration("offboard_hold_seconds"),
                "land_timeout": LaunchConfiguration("land_timeout"),
            }],
        ),
    ])
