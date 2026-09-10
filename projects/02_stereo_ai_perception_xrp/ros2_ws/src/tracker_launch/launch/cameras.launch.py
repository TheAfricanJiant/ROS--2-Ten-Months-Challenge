"""Just the two cameras and Foxglove - no tracker, nothing moves.

    ros2 launch tracker_launch cameras.launch.py

Use this to check both feeds arrive in Foxglove before letting the robot
drive itself anywhere.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("camera1_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("camera2_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("effect", default_value="ir_heat"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tracker_launch"), "launch", "bringup.launch.py",
            ])),
            launch_arguments={
                "camera1_port": LaunchConfiguration("camera1_port"),
                "camera2_port": LaunchConfiguration("camera2_port"),
                "effect": LaunchConfiguration("effect"),
                "use_tracker": "false",
                "foxglove": "true",
            }.items(),
        ),
    ])
