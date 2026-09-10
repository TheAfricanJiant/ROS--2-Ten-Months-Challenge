"""Tracking camera plus the tracker, and nothing else.

    ros2 launch tracker_launch follow.launch.py
    ros2 launch tracker_launch follow.launch.py target:=person enabled:=false

Leaves camera 2 off, so only one board needs to be plugged in.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("camera1_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("target", default_value="face"),
        DeclareLaunchArgument("enabled", default_value="true"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tracker_launch"), "launch", "bringup.launch.py",
            ])),
            launch_arguments={
                "camera1_port": LaunchConfiguration("camera1_port"),
                "target": LaunchConfiguration("target"),
                "enabled": LaunchConfiguration("enabled"),
                "foxglove": LaunchConfiguration("foxglove"),
                "use_camera2": "false",
            }.items(),
        ),
    ])
