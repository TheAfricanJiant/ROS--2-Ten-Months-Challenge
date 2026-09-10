"""Follows an object with one camera, publishing /cmd_vel for micro-ROS.

Subscribes to the tracking camera's ``detections``, picks a target, and turns
"where is it" into "how should the robot move":

* **Turning** comes from the horizontal angle to the target, computed with that
  camera's own lens model. A fisheye and an ordinary lens disagree about what a
  pixel offset means, and using the wrong one makes the robot oversteer.
* **Driving** comes from apparent size. One camera cannot measure depth, so
  range is inferred from how tall the target looks, given an assumed real
  height. It is good enough to hold a following distance and no better -
  stereo comes later.

The default target is a human face, matching the SenseCraft *Face Detection*
model. Set ``target_class`` at launch for anything else the model knows.

Safety is deliberately dull: no detection for ``lost_timeout`` seconds and it
publishes zero, and it publishes zero on shutdown too.
"""

from __future__ import annotations

import math
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from .lenses import lens_for

try:
    from sscma import get_camera
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "The 'sscma' package is not importable. Install it with:\n"
        "    pip install -e <repo>/test_camera"
    ) from exc


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class Target:
    """The chosen detection, in the terms the controller needs."""

    def __init__(self, class_id: str, score: float,
                 cx: float, cy: float, width: float, height: float) -> None:
        self.class_id = class_id
        self.score = score
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height


class TrackerNode(Node):
    """Detections in, Twist out."""

    def __init__(self) -> None:
        super().__init__("object_tracker")

        # what to follow
        self.declare_parameter("target_class", "face")
        self.declare_parameter("min_score", 0.4)
        self.declare_parameter("select", "largest")   # largest | best_score | centre

        # the camera doing the tracking
        self.declare_parameter("camera_key", "ir-3.6mm")
        self.declare_parameter("focal_px", 0.0)
        self.declare_parameter("lens_model", "auto")
        self.declare_parameter("width", 240)
        self.declare_parameter("height", 240)
        self.declare_parameter("object_height_m", 0.22)

        # control
        self.declare_parameter("desired_distance_m", 0.8)
        self.declare_parameter("distance_tolerance_m", 0.12)
        self.declare_parameter("angular_gain", 1.6)
        self.declare_parameter("linear_gain", 0.6)
        self.declare_parameter("max_angular", 1.2)
        self.declare_parameter("max_linear", 0.25)
        self.declare_parameter("angular_deadband", 0.06)
        self.declare_parameter("smoothing", 3)
        self.declare_parameter("drive_forward", True)

        # behaviour
        self.declare_parameter("lost_timeout", 0.7)
        self.declare_parameter("search_on_lost", False)
        self.declare_parameter("search_angular", 0.4)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("enabled", True)

        p = self.get_parameter
        self.target_class = str(p("target_class").value).lower()
        self.min_score = float(p("min_score").value)
        self.select = str(p("select").value)
        self.object_height = float(p("object_height_m").value)
        self.desired_distance = float(p("desired_distance_m").value)
        self.distance_tolerance = float(p("distance_tolerance_m").value)
        self.angular_gain = float(p("angular_gain").value)
        self.linear_gain = float(p("linear_gain").value)
        self.max_angular = float(p("max_angular").value)
        self.max_linear = float(p("max_linear").value)
        self.angular_deadband = float(p("angular_deadband").value)
        self.drive_forward = bool(p("drive_forward").value)
        self.lost_timeout = float(p("lost_timeout").value)
        self.search_on_lost = bool(p("search_on_lost").value)
        self.search_angular = float(p("search_angular").value)
        self.enabled = bool(p("enabled").value)

        width = int(p("width").value)
        height = int(p("height").value)
        camera_key = str(p("camera_key").value)

        focal = float(p("focal_px").value)
        if focal <= 0:
            try:
                focal = get_camera(camera_key).focal_px(width)
            except KeyError:
                focal = float(width)   # ~53 deg; a last resort, not a real value
                self.get_logger().error(
                    f"Unknown camera '{camera_key}' and no focal_px given. "
                    "Steering will be wrong until you set one.")
            else:
                self.get_logger().warning(
                    f"Using nominal focal {focal:.1f}px for '{camera_key}'. "
                    "Measure it with camera_info.py --measure.")

        override = str(p("lens_model").value)
        self.lens = lens_for(camera_key, focal, width, height,
                             force_model=None if override == "auto" else override)
        self.get_logger().info(f"Lens -> {self.lens}")

        smoothing = max(1, int(p("smoothing").value))
        self._angle_history: deque[float] = deque(maxlen=smoothing)
        self._range_history: deque[float] = deque(maxlen=smoothing)

        self._target: Target | None = None
        self._last_seen = 0.0
        self._search_direction = 1.0

        self.create_subscription(Detection2DArray, "detections",
                                 self._on_detections,
                                 QoSPresetProfiles.SENSOR_DATA.value)

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "tracker/status", 10)

        rate = max(1.0, float(p("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self._control)

        self.get_logger().info(
            f"Tracking '{self.target_class}', holding "
            f"{self.desired_distance:.2f} m"
            + ("" if self.drive_forward else " (turning only)")
            + ("" if self.enabled else "   [DISABLED - publishing zero]"))

    # -- input -----------------------------------------------------------

    def _on_detections(self, msg: Detection2DArray) -> None:
        candidates = []
        for det in msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda r: r.hypothesis.score)
            class_id = str(best.hypothesis.class_id).lower()
            if self.target_class not in ("", "any") and class_id != self.target_class:
                continue
            if best.hypothesis.score < self.min_score:
                continue
            candidates.append(Target(
                class_id=class_id,
                score=float(best.hypothesis.score),
                cx=float(det.bbox.center.position.x),
                cy=float(det.bbox.center.position.y),
                width=float(det.bbox.size_x),
                height=float(det.bbox.size_y),
            ))

        self._target = self._pick(candidates) if candidates else None

    def _pick(self, candidates: list[Target]) -> Target:
        if self.select == "best_score":
            return max(candidates, key=lambda t: t.score)
        if self.select == "centre":
            return min(candidates, key=lambda t: abs(t.cx - self.lens.cx))
        return max(candidates, key=lambda t: t.width * t.height)

    # -- range -----------------------------------------------------------

    def _range(self, target: Target) -> float | None:
        """Distance from apparent size.

        A single camera cannot measure depth; this works only because the
        target's real height is assumed. Wrong assumption, wrong distance -
        but consistently wrong, which is enough to hold a following gap.
        """
        if target.height <= 1 or self.object_height <= 0:
            return None
        return self.lens.focal_px * self.object_height / target.height

    # -- control ---------------------------------------------------------

    def _control(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        target = self._target

        if target is None:
            self._handle_lost(now)
            return

        self._last_seen = now
        twist = Twist()

        # -- turn --------------------------------------------------------
        error = self.lens.normalised_x(target.cx)
        self._angle_history.append(error)
        error = sum(self._angle_history) / len(self._angle_history)

        if abs(error) < self.angular_deadband:
            error = 0.0
        else:
            # Remember which way it went, so a search spins the right way.
            self._search_direction = 1.0 if error < 0 else -1.0

        # Negative: a target right of centre (+x) needs a clockwise turn,
        # which is negative yaw under REP-103.
        twist.angular.z = clamp(-self.angular_gain * error, self.max_angular)

        # -- drive -------------------------------------------------------
        distance = self._range(target)
        if distance is not None:
            self._range_history.append(distance)
            distance = sorted(self._range_history)[len(self._range_history) // 2]

            if self.drive_forward:
                gap = distance - self.desired_distance
                if abs(gap) >= self.distance_tolerance:
                    twist.linear.x = clamp(self.linear_gain * gap, self.max_linear)

        if not self.enabled:
            twist = Twist()

        self.pub_cmd.publish(twist)

        angle_deg = math.degrees(self.lens.angles(target.cx, target.cy)[0])
        self._publish_status(
            f"tracking {target.class_id} score={target.score:.2f} "
            f"angle={angle_deg:+.1f}deg "
            f"range={f'{distance:.2f}m' if distance else 'unknown'} "
            f"cmd=[{twist.linear.x:+.2f}, {twist.angular.z:+.2f}]")

    def _handle_lost(self, now: float) -> None:
        since = now - self._last_seen
        if since < self.lost_timeout:
            # Ride out a single dropped frame rather than stuttering.
            return

        self._angle_history.clear()
        self._range_history.clear()

        twist = Twist()
        if self.search_on_lost and self.enabled:
            twist.angular.z = self.search_angular * self._search_direction
            self._publish_status(f"lost for {since:.1f}s - searching")
        else:
            self._publish_status(f"lost for {since:.1f}s - stopped")

        self.pub_cmd.publish(twist)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def stop(self) -> None:
        """Publish a zero Twist, so the robot does not coast off."""
        try:
            self.pub_cmd.publish(Twist())
        except Exception:  # noqa: BLE001 - shutting down regardless
            pass


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = TrackerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
