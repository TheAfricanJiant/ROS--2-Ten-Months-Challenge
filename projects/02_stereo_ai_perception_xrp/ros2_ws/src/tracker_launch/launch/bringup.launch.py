"""Bring up everything: both cameras, the tracker, and the Foxglove bridge.

    ros2 launch tracker_launch bringup.launch.py
    ros2 launch tracker_launch bringup.launch.py target:=person effect:=colour
    ros2 launch tracker_launch bringup.launch.py enabled:=false     # no motion

Camera 1 runs the detection model and drives the robot. Camera 2 runs an image
effect (IR heat palette by default). Both publish for Foxglove::

    /image1              camera 1, plain
    /image1_annotated    camera 1, with detection boxes
    /image2              camera 2, processed

Connect Foxglove Studio on another machine to ws://<pi-address>:8765.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name: str, kind: type) -> ParameterValue:
    """A launch argument as a real int/float/bool, not a string.

    Launch substitutions are always text. Handing "240" to a parameter the
    node declared as an integer fails at startup, so every non-string
    parameter has to say what it really is.
    """
    return ParameterValue(LaunchConfiguration(name), value_type=kind)


def generate_launch_description() -> LaunchDescription:
    args = [
        # --- hardware ---------------------------------------------------
        DeclareLaunchArgument(
            "camera1_port", default_value="/dev/ttyACM0",
            description="Tracking camera. Windows uses COM3-style names."),
        DeclareLaunchArgument(
            "camera2_port", default_value="/dev/ttyACM1",
            description="Effects camera."),
        DeclareLaunchArgument(
            "camera1_key", default_value="ir-3.6mm",
            description="Lens of the tracking camera; sets the projection model."),
        DeclareLaunchArgument(
            "camera2_key", default_value="ir-1.7mm",
            description="Lens of the effects camera."),
        DeclareLaunchArgument(
            "camera1_focal_px", default_value="0.0",
            description="Measured focal length in px. 0 = use the nominal value."),
        DeclareLaunchArgument("width", default_value="240"),
        DeclareLaunchArgument("height", default_value="240"),

        # --- what to track ----------------------------------------------
        DeclareLaunchArgument(
            "target", default_value="face",
            description="Class to follow. Must be one the flashed model knows."),
        DeclareLaunchArgument(
            "object_height_m", default_value="0.22",
            description="Real height of the target, for the distance estimate."),
        DeclareLaunchArgument("desired_distance_m", default_value="0.8"),
        DeclareLaunchArgument(
            "enabled", default_value="true",
            description="false computes everything but publishes zero velocity."),
        DeclareLaunchArgument(
            "drive_forward", default_value="true",
            description="false turns to face the target without driving at it."),
        DeclareLaunchArgument("search_on_lost", default_value="false"),

        # --- camera 2 ---------------------------------------------------
        DeclareLaunchArgument(
            "effect", default_value="ir_heat",
            description="raw | ir_heat | night_vision | enhanced | colour | edges"),
        DeclareLaunchArgument(
            "colour", default_value="red",
            description="Colour to isolate when effect:=colour."),

        # --- optional pieces --------------------------------------------
        DeclareLaunchArgument("use_camera2", default_value="true"),
        DeclareLaunchArgument("use_tracker", default_value="true"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    camera1 = Node(
        package="tracker",
        executable="camera_node",
        name="camera1",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("camera1_port"),
            "camera_key": LaunchConfiguration("camera1_key"),
            "focal_px": typed("camera1_focal_px", float),
            "frame_id": "camera1_optical",
            "detect": True,
            "width": typed("width", int),
            "height": typed("height", int),
        }],
        remappings=[
            ("image", "/image1"),
            ("image_annotated", "/image1_annotated"),
            ("camera_info", "/camera1/camera_info"),
            ("detections", "/camera1/detections"),
        ],
    )

    camera2 = Node(
        package="tracker",
        executable="effects_node",
        name="camera2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_camera2")),
        parameters=[{
            "port": LaunchConfiguration("camera2_port"),
            "frame_id": "camera2_optical",
            "effect": LaunchConfiguration("effect"),
            "colour": LaunchConfiguration("colour"),
            "width": typed("width", int),
            "height": typed("height", int),
        }],
        remappings=[
            ("image", "/image2"),
            ("image_raw", "/image2_raw"),
            ("blob", "/camera2/blob"),
        ],
    )

    tracker = Node(
        package="tracker",
        executable="tracker_node",
        name="object_tracker",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_tracker")),
        parameters=[{
            "target_class": LaunchConfiguration("target"),
            "camera_key": LaunchConfiguration("camera1_key"),
            "focal_px": typed("camera1_focal_px", float),
            "width": typed("width", int),
            "height": typed("height", int),
            "object_height_m": typed("object_height_m", float),
            "desired_distance_m": typed("desired_distance_m", float),
            "enabled": typed("enabled", bool),
            "drive_forward": typed("drive_forward", bool),
            "search_on_lost": typed("search_on_lost", bool),
        }],
        remappings=[
            ("detections", "/camera1/detections"),
            ("cmd_vel", "/cmd_vel"),
        ],
    )

    # foxglove_bridge is what Foxglove Studio connects to over the network.
    # Install with: sudo apt install ros-$ROS_DISTRO-foxglove-bridge
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("foxglove")),
        parameters=[{
            "port": typed("foxglove_port", int),
            "address": "0.0.0.0",          # reachable from another machine
            "send_buffer_limit": 10000000,
        }],
    )

    return LaunchDescription(args + [
        LogInfo(msg=[
            "Foxglove: connect Studio to ws://<this-machine>:",
            LaunchConfiguration("foxglove_port"),
            "  and add Image panels on /image1 and /image2",
        ]),
        GroupAction([camera1, camera2, tracker, foxglove]),
    ])
