"""Minimal client for Seeed's SSCMA AT protocol (Grove Vision AI V2).

Enough of sscma-micro to pull JPEG frames off the board over USB serial and
show them in OpenCV, so the cameras and the AI modules can be proven out
before any of it is wired into ROS 2.
"""

from .client import (
    DEFAULT_BAUDRATE,
    Detection,
    Frame,
    PortInfo,
    SSCMAClient,
    SSCMAError,
    available_ports,
    find_port,
)
from .protocol import Reply, ReplyParser, decode_jpeg
from .viewer import Viewer

__all__ = [
    "DEFAULT_BAUDRATE",
    "Detection",
    "Frame",
    "PortInfo",
    "Reply",
    "ReplyParser",
    "SSCMAClient",
    "SSCMAError",
    "Viewer",
    "available_ports",
    "decode_jpeg",
    "find_port",
]
