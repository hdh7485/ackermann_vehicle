"""Launch the ROS 2 Twist-to-Ackermann command bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("twist_cmd_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument(
            "ackermann_cmd_topic", default_value="/ackermann_cmd"
        ),
        DeclareLaunchArgument("wheelbase", default_value="1.0"),
        DeclareLaunchArgument("frame_id", default_value="odom"),
        Node(
            package="ackermann_vehicle_ros2",
            executable="cmd_vel_to_ackermann_drive",
            name="cmd_vel_to_ackermann_drive",
            output="screen",
            parameters=[{
                "twist_cmd_topic": LaunchConfiguration("twist_cmd_topic"),
                "ackermann_cmd_topic": LaunchConfiguration("ackermann_cmd_topic"),
                "wheelbase": LaunchConfiguration("wheelbase"),
                "frame_id": LaunchConfiguration("frame_id"),
            }],
        ),
    ])
