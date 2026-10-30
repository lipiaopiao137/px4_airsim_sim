from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("rc_control_mode", default_value="USA",
                              description="Stick layout: USA, JAPAN or CHINA"),
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument("control.max_xy_vel", default_value="2.0"),
        DeclareLaunchArgument("control.max_z_vel", default_value="1.5"),
        DeclareLaunchArgument("control.max_yaw_vel", default_value="1.0"),
        DeclareLaunchArgument("control.interval_ms", default_value="50"),
        DeclareLaunchArgument("gamepad.axis_deadzone", default_value="0.1"),
        DeclareLaunchArgument("gamepad.btn_debounce_ms", default_value="200"),
        DeclareLaunchArgument("gamepad.nonlinear_scale", default_value="2.0"),
        DeclareLaunchArgument("axis_mapping.left_x", default_value="0"),
        DeclareLaunchArgument("axis_mapping.left_y", default_value="1"),
        DeclareLaunchArgument("axis_mapping.right_x", default_value="3"),
        DeclareLaunchArgument("axis_mapping.right_y", default_value="4"),
        DeclareLaunchArgument("button.takeoff", default_value="5",
                              description="RB"),
        DeclareLaunchArgument("button.land", default_value="4",
                              description="LB"),
        DeclareLaunchArgument("cmd_timeout", default_value="1.0"),
        DeclareLaunchArgument("takeoff_altitude", default_value="1.5"),
        DeclareLaunchArgument("takeoff_climb_rate", default_value="0.5"),
        DeclareLaunchArgument("command_retry_s", default_value="1.0"),
    ]

    node = Node(
        package="px4_gamepad_control",
        executable="gamepad_control",
        name="gamepad_control",
        output="screen",
        parameters=[{
            "rc_control_mode": LaunchConfiguration("rc_control_mode"),
            "joy_topic": LaunchConfiguration("joy_topic"),
            "control": {
                "max_xy_vel": LaunchConfiguration("control.max_xy_vel"),
                "max_z_vel": LaunchConfiguration("control.max_z_vel"),
                "max_yaw_vel": LaunchConfiguration("control.max_yaw_vel"),
                "interval_ms": LaunchConfiguration("control.interval_ms"),
            },
            "gamepad": {
                "axis_deadzone": LaunchConfiguration("gamepad.axis_deadzone"),
                "btn_debounce_ms": LaunchConfiguration("gamepad.btn_debounce_ms"),
                "nonlinear_scale": LaunchConfiguration("gamepad.nonlinear_scale"),
            },
            "axis_mapping": {
                "left_x": LaunchConfiguration("axis_mapping.left_x"),
                "left_y": LaunchConfiguration("axis_mapping.left_y"),
                "right_x": LaunchConfiguration("axis_mapping.right_x"),
                "right_y": LaunchConfiguration("axis_mapping.right_y"),
            },
            "button": {
                "takeoff": LaunchConfiguration("button.takeoff"),
                "land": LaunchConfiguration("button.land"),
            },
            "cmd_timeout": LaunchConfiguration("cmd_timeout"),
            "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
            "takeoff_climb_rate": LaunchConfiguration("takeoff_climb_rate"),
            "command_retry_s": LaunchConfiguration("command_retry_s"),
        }],
    )

    return LaunchDescription(args + [node])
