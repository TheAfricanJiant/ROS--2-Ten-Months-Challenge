"""Minimal client for Seeed's SSCMA AT protocol (Grove Vision AI V2).

Enough of sscma-micro to pull JPEG frames off the board over USB serial and
show them in OpenCV, so the cameras and the AI modules can be proven out
before any of it is wired into ROS 2.

Layered so the useful parts survive the move into ROS 2 later:

``protocol``  framing and JSON reply parsing, no I/O
``client``    serial transport, AT commands, frame decoding
``cameras``   lens and sensor properties, and the optics that follow
``config``    stereo rig configuration, readable by ROS 2
``stereo``    synchronised capture from two boards, and triangulation
``viewer``    OpenCV presentation (the only part a ROS 2 node would drop)
"""

from .cameras import (
    CAMERAS,
    CUSTOM_CAMERAS_FILE,
    CameraSpec,
    camera_keys,
    distance_from_height,
    focal_px_from_measurement,
    get_camera,
    load_custom_cameras,
    save_custom_camera,
)
from .client import (
    DEFAULT_BAUDRATE,
    Detection,
    Frame,
    PortInfo,
    SSCMAClient,
    SSCMAError,
    available_ports,
    find_port,
    identify,
    open_failure_hint,
    permission_hint,
    port_hint,
)
from .config import CameraConfig, StereoConfig, load_config
from .protocol import Reply, ReplyParser, decode_jpeg
from .stereo import (
    DepthResult,
    StereoCapture,
    StereoPair,
    match_detections,
    triangulate,
)
from .viewer import Viewer

__all__ = [
    # protocol / transport
    "DEFAULT_BAUDRATE",
    "Detection",
    "Frame",
    "PortInfo",
    "Reply",
    "ReplyParser",
    "SSCMAClient",
    "SSCMAError",
    "available_ports",
    "decode_jpeg",
    "find_port",
    "identify",
    "open_failure_hint",
    "permission_hint",
    "port_hint",
    # optics
    "CAMERAS",
    "CUSTOM_CAMERAS_FILE",
    "CameraSpec",
    "camera_keys",
    "distance_from_height",
    "focal_px_from_measurement",
    "get_camera",
    "load_custom_cameras",
    "save_custom_camera",
    # rig
    "CameraConfig",
    "StereoConfig",
    "load_config",
    # stereo
    "DepthResult",
    "StereoCapture",
    "StereoPair",
    "match_detections",
    "triangulate",
    # presentation
    "Viewer",
]
