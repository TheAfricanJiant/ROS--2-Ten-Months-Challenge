"""Publishes one Grove Vision AI V2 as a ROS 2 camera, with its detections.

This is the *tracking* camera. It streams frames and model detections off the
board over USB serial and republishes them; `tracker_node` turns those
detections into `/cmd_vel`.

Topics (relative - the launch file remaps `image` to `/image1`)::

    image                 sensor_msgs/CompressedImage   plain frames
    image_annotated       sensor_msgs/CompressedImage   boxes drawn on
    camera_info           sensor_msgs/CameraInfo
    detections            vision_msgs/Detection2DArray

CompressedImage rather than Image because the board already hands us JPEG.
Re-encoding it to raw only to push it over Wi-Fi to Foxglove wastes bandwidth
for no gain - Foxglove's Image panel reads CompressedImage directly.
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

try:
    from sscma import SSCMAClient, SSCMAError, get_camera
except ImportError as exc:  # pragma: no cover - environment, not logic
    raise ImportError(
        "The 'sscma' package is not importable. Install it from this repo:\n"
        "    pip install -e <repo>/test_camera\n"
        "It carries the serial driver and the camera database."
    ) from exc

from .lenses import lens_for

#: Sensor option ids by output width, as reported by AT+SENSORS?.
RESOLUTION_BY_WIDTH = {240: 0, 480: 1, 640: 2}


class CameraNode(Node):
    """One board, published as a camera with detections."""

    def __init__(self) -> None:
        super().__init__("vision_camera")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("camera_key", "ir-3.6mm")
        self.declare_parameter("frame_id", "camera_optical")
        self.declare_parameter("detect", True)
        self.declare_parameter("width", 240)
        self.declare_parameter("height", 240)
        self.declare_parameter("focal_px", 0.0)
        self.declare_parameter("lens_model", "auto")
        self.declare_parameter("box_format", "center")
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("jpeg_quality", 80)

        p = self.get_parameter
        self.camera_key = str(p("camera_key").value)
        self.frame_id = str(p("frame_id").value)
        self.detect = bool(p("detect").value)
        self.width = int(p("width").value)
        self.height = int(p("height").value)
        self.jpeg_quality = int(p("jpeg_quality").value)

        port = str(p("port").value)
        if not port:
            self.get_logger().error("The 'port' parameter is required.")
            raise SystemExit(1)

        # -- optics ------------------------------------------------------
        focal = float(p("focal_px").value)
        if focal <= 0:
            try:
                focal = get_camera(self.camera_key).focal_px(self.width)
            except KeyError:
                self.get_logger().error(
                    f"Unknown camera_key '{self.camera_key}'. Set focal_px "
                    "explicitly, or add the camera with camera_info.py --add.")
                raise SystemExit(1)
            self.get_logger().warning(
                f"Using the NOMINAL focal length {focal:.1f}px for "
                f"{self.camera_key}. Measure it with camera_info.py --measure "
                "before trusting any distance.")

        model_override = str(p("lens_model").value)
        self.lens = lens_for(
            self.camera_key, focal, self.width, self.height,
            force_model=None if model_override == "auto" else model_override,
        )
        self.get_logger().info(f"Lens model -> {self.lens}")

        # -- publishers --------------------------------------------------
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.pub_image = self.create_publisher(CompressedImage, "image", qos)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", qos)
        self.pub_detections = self.create_publisher(Detection2DArray, "detections", qos)
        self.pub_annotated = (
            self.create_publisher(CompressedImage, "image_annotated", qos)
            if bool(p("publish_annotated").value) else None)

        # -- hardware ----------------------------------------------------
        self.client = SSCMAClient(
            port=port,
            baudrate=int(p("baud").value),
            box_format=str(p("box_format").value),
        )
        try:
            self.client.open()
        except SSCMAError as exc:
            self.get_logger().error(str(exc))
            raise SystemExit(1)

        info = self.client.device_info()
        self.get_logger().info(
            f"{info.get('name', 'unknown')} on {port} "
            f"(firmware {info.get('firmware', '?')})")

        option = RESOLUTION_BY_WIDTH.get(self.width)
        if option is not None:
            try:
                self.client.set_resolution(option)
            except SSCMAError as exc:
                self.get_logger().warning(f"Could not set resolution: {exc}")

        if self.detect:
            model = self.client.model_info()
            if model:
                classes = self.client.adopt_model_labels()
                self.get_logger().info(
                    f"Model '{model.get('model_name')}' classes={classes}")
            else:
                self.get_logger().warning(
                    "No model metadata. Load one from SenseCraft AI, or "
                    "detections will always be empty.")

        self._frames = self.client.stream(detect=self.detect)
        self._camera_info = self._build_camera_info()
        self._count = 0

        # The generator blocks on serial, which is what actually paces us;
        # the timer just needs to be faster than the frame rate.
        self.timer = self.create_timer(0.005, self._tick)

    # -- messages --------------------------------------------------------

    def _header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def _build_camera_info(self) -> CameraInfo:
        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        f, cx, cy = self.lens.focal_px, self.lens.cx, self.lens.cy
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Zero until a checkerboard calibration is run. For a fisheye that is
        # a real approximation, not a formality.
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return info

    def _detections_msg(self, frame, header) -> Detection2DArray:
        msg = Detection2DArray()
        msg.header = header

        for det in frame.detections:
            entry = Detection2D()
            entry.header = header

            box = BoundingBox2D()
            box.center.position.x = float(det.x + det.width / 2.0)
            box.center.position.y = float(det.y + det.height / 2.0)
            box.center.theta = 0.0
            box.size_x = float(det.width)
            box.size_y = float(det.height)
            entry.bbox = box

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det.label)
            hypothesis.hypothesis.score = float(det.score) / 100.0
            entry.results.append(hypothesis)

            msg.detections.append(entry)

        return msg

    def _annotate(self, frame):
        import cv2

        canvas = frame.image.copy()
        for det in frame.detections:
            cv2.rectangle(canvas, (det.x, det.y),
                          (det.x + det.width, det.y + det.height),
                          (80, 255, 120), 2)
            caption = f"{det.label} {det.score}%"
            origin = (det.x + 2, max(11, det.y - 4))
            cv2.putText(canvas, caption, origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, caption, origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (80, 255, 120), 1, cv2.LINE_AA)
        return canvas

    def _compressed(self, image, header) -> CompressedImage:
        import cv2

        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        ok, buffer = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        msg.data = buffer.tobytes() if ok else b""
        return msg

    # -- main loop -------------------------------------------------------

    def _tick(self) -> None:
        try:
            frame = next(self._frames)
        except StopIteration:
            self.get_logger().error("The camera stopped streaming.")
            raise SystemExit(1)
        except SSCMAError as exc:
            self.get_logger().error(f"Stream failed: {exc}")
            raise SystemExit(1)

        header = self._header()

        self.pub_image.publish(self._compressed(frame.image, header))

        self._camera_info.header = header
        self.pub_info.publish(self._camera_info)
        self.pub_detections.publish(self._detections_msg(frame, header))

        if self.pub_annotated is not None:
            self.pub_annotated.publish(
                self._compressed(self._annotate(frame), header))

        self._count += 1
        if self._count % 200 == 0:
            self.get_logger().info(
                f"{self._count} frames, {len(frame.detections)} detection(s) "
                "in the last one")

    def destroy_node(self) -> bool:
        try:
            self.client.close()
        except Exception:  # noqa: BLE001 - shutting down regardless
            pass
        return super().destroy_node()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
