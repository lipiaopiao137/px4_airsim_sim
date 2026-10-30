"""Launch the keyboard-driven offboard controller.

The keyboard node is deliberately *not* started here: ``ros2 launch`` gives its
children a pipe instead of the terminal, so a node reading stdin cannot see key
presses.  Run the keyboard node in a terminal with

    ros2 run px4_keyboard_control keyboard_velocity

or use ``scripts/start_keyboard.sh``, which starts this launch file in the
background and the keyboard node in the foreground.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("takeoff_altitude", default_value="1.5"),
        DeclareLaunchArgument("takeoff_climb_rate", default_value="0.5"),
        DeclareLaunchArgument("cmd_timeout", default_value="1.0"),
        Node(
            package="px4_keyboard_control",
            executable="velocity_control",
            name="velocity_control",
            output="screen",
            parameters=[{
                "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
                "takeoff_climb_rate": LaunchConfiguration("takeoff_climb_rate"),
                "cmd_timeout": LaunchConfiguration("cmd_timeout"),
            }],
        ),
    ])
