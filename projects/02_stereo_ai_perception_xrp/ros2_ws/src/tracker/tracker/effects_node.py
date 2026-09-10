"""The second camera: image processing rather than object detection.

While camera 1 runs the detection model and drives the robot, this one earns
its keep differently. Pick an effect with the ``effect`` parameter:

``raw``          straight through, no processing
``ir_heat``      IR intensity through a heat-style palette
``night_vision`` green phosphor, with a gamma lift and a vignette
``enhanced``     CLAHE - pulls detail out of the shadows
``colour``       isolate one colour and track the biggest blob of it
``edges``        Canny edges over a dimmed original

**On ``ir_heat``: this is not a thermal camera.** These modules see
near-infrared *reflected off things* - the IR LEDs are a torch you cannot see.
It measures brightness, never temperature. A cold white wall under the LEDs
reads "hot". For real temperature you need a thermal sensor such as an
MLX90640.

Topics (relative - the launch file remaps `image` to `/image2`)::

    image        sensor_msgs/CompressedImage   the processed view
    image_raw    sensor_msgs/CompressedImage   unprocessed, if publish_raw
    blob         geometry_msgs/PointStamped    colour blob centre + area (colour effect)
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

try:
    from sscma import SSCMAClient, SSCMAError
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "The 'sscma' package is not importable. Install it with:\n"
        "    pip install -e <repo>/test_camera"
    ) from exc

RESOLUTION_BY_WIDTH = {240: 0, 480: 1, 640: 2}

#: Hue ranges in OpenCV's 0-179 scale. Red straddles the wrap-around, so it
#: needs two ranges.
COLOUR_RANGES = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "orange": [((10, 120, 70), (22, 255, 255))],
    "yellow": [((22, 100, 70), (35, 255, 255))],
    "green": [((35, 80, 60), (85, 255, 255))],
    "cyan": [((85, 80, 60), (100, 255, 255))],
    "blue": [((100, 100, 60), (130, 255, 255))],
    "purple": [((130, 80, 60), (160, 255, 255))],
}

EFFECTS = ("raw", "ir_heat", "night_vision", "enhanced", "colour", "edges")


class EffectsNode(Node):
    """Second camera, doing something other than detection."""

    def __init__(self) -> None:
        super().__init__("vision_effects")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("frame_id", "camera2_optical")
        self.declare_parameter("effect", "ir_heat")
        self.declare_parameter("palette", "INFERNO")
        self.declare_parameter("colour", "red")
        self.declare_parameter("min_blob_area", 120)
        self.declare_parameter("clahe_clip", 2.5)
        self.declare_parameter("width", 240)
        self.declare_parameter("height", 240)
        self.declare_parameter("publish_raw", False)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("overlay_label", True)

        p = self.get_parameter
        self.frame_id = str(p("frame_id").value)
        self.effect = str(p("effect").value).lower()
        self.colour = str(p("colour").value).lower()
        self.min_blob_area = int(p("min_blob_area").value)
        self.clahe_clip = float(p("clahe_clip").value)
        self.width = int(p("width").value)
        self.jpeg_quality = int(p("jpeg_quality").value)
        self.overlay_label = bool(p("overlay_label").value)

        if self.effect not in EFFECTS:
            self.get_logger().error(
                f"Unknown effect '{self.effect}'. Choose from: {', '.join(EFFECTS)}")
            raise SystemExit(1)
        if self.effect == "colour" and self.colour not in COLOUR_RANGES:
            self.get_logger().error(
                f"Unknown colour '{self.colour}'. Choose from: "
                f"{', '.join(COLOUR_RANGES)}")
            raise SystemExit(1)

        port = str(p("port").value)
        if not port:
            self.get_logger().error("The 'port' parameter is required.")
            raise SystemExit(1)

        import cv2

        self._palette = getattr(cv2, f"COLORMAP_{str(p('palette').value).upper()}",
                                cv2.COLORMAP_INFERNO)
        self._clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=(8, 8))

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.pub_image = self.create_publisher(CompressedImage, "image", qos)
        self.pub_raw = (self.create_publisher(CompressedImage, "image_raw", qos)
                        if bool(p("publish_raw").value) else None)
        self.pub_blob = self.create_publisher(PointStamped, "blob", qos)

        self.client = SSCMAClient(port=port, baudrate=int(p("baud").value))
        try:
            self.client.open()
        except SSCMAError as exc:
            self.get_logger().error(str(exc))
            raise SystemExit(1)

        info = self.client.device_info()
        self.get_logger().info(
            f"{info.get('name', 'unknown')} on {port} -> effect '{self.effect}'"
            + (f" ({self.colour})" if self.effect == "colour" else ""))

        option = RESOLUTION_BY_WIDTH.get(self.width)
        if option is not None:
            try:
                self.client.set_resolution(option)
            except SSCMAError as exc:
                self.get_logger().warning(f"Could not set resolution: {exc}")

        # No model needed - this camera never runs inference.
        self._frames = self.client.stream(detect=False)
        self._count = 0
        self.timer = self.create_timer(0.005, self._tick)

    # -- effects ---------------------------------------------------------

    def _ir_heat(self, image):
        import cv2

        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        stretched = cv2.normalize(grey, None, 0, 255, cv2.NORM_MINMAX)
        return cv2.applyColorMap(stretched, self._palette)

    def _night_vision(self, image):
        import cv2
        import numpy as np

        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        lifted = np.power(grey.astype(np.float32) / 255.0, 0.65)
        out = np.zeros_like(image)
        out[:, :, 1] = np.clip(lifted * 255.0, 0, 255).astype(np.uint8)
        out[:, :, 0] = np.clip(lifted * 40.0, 0, 255).astype(np.uint8)

        h, w = out.shape[:2]
        yy, xx = np.ogrid[:h, :w]
        radius = np.sqrt(((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
        mask = np.clip(1.15 - 0.55 * radius, 0, 1).astype(np.float32)
        return (out * mask[:, :, None]).astype(np.uint8)

    def _enhanced(self, image):
        import cv2

        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return cv2.cvtColor(self._clahe.apply(grey), cv2.COLOR_GRAY2BGR)

    def _edges(self, image):
        import cv2

        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(grey, (5, 5), 0), 60, 150)
        canvas = (image * 0.35).astype("uint8")
        canvas[edges > 0] = (80, 255, 120)
        return canvas

    def _colour(self, image, header):
        """Isolate one colour, and report the biggest blob of it.

        The blob centre goes out on `blob` as a PointStamped: x and y in
        pixels, z carrying the area. That is enough for a colour-following
        behaviour later without this node needing to know about it.
        """
        import cv2
        import numpy as np

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = None
        for low, high in COLOUR_RANGES[self.colour]:
            part = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)

        # Open then close: drop speckle, then fill the holes left behind.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        canvas = (image * 0.3).astype("uint8")
        canvas[mask > 0] = image[mask > 0]

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(largest))
            if area >= self.min_blob_area:
                x, y, w, h = cv2.boundingRect(largest)
                cx, cy = x + w / 2.0, y + h / 2.0

                cv2.rectangle(canvas, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.drawMarker(canvas, (int(cx), int(cy)), (255, 255, 255),
                               cv2.MARKER_CROSS, 14, 2)
                cv2.putText(canvas, f"{self.colour} {int(area)}px",
                            (x + 2, max(11, y - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, (255, 255, 255), 1, cv2.LINE_AA)

                point = PointStamped()
                point.header = header
                point.point.x = cx
                point.point.y = cy
                point.point.z = area
                self.pub_blob.publish(point)

        return canvas

    def _apply(self, image, header):
        if self.effect == "ir_heat":
            return self._ir_heat(image)
        if self.effect == "night_vision":
            return self._night_vision(image)
        if self.effect == "enhanced":
            return self._enhanced(image)
        if self.effect == "edges":
            return self._edges(image)
        if self.effect == "colour":
            return self._colour(image, header)
        return image.copy()

    # -- plumbing --------------------------------------------------------

    def _header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def _compressed(self, image, header) -> CompressedImage:
        import cv2

        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        ok, buffer = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        msg.data = buffer.tobytes() if ok else b""
        return msg

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
        processed = self._apply(frame.image, header)

        if self.overlay_label:
            import cv2

            label = self.effect + (f":{self.colour}" if self.effect == "colour" else "")
            cv2.putText(processed, label, (6, 16), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(processed, label, (6, 16), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)

        self.pub_image.publish(self._compressed(processed, header))
        if self.pub_raw is not None:
            self.pub_raw.publish(self._compressed(frame.image, header))

        self._count += 1
        if self._count % 300 == 0:
            self.get_logger().info(f"{self._count} frames ({self.effect})")

    def destroy_node(self) -> bool:
        try:
            self.client.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = None
    try:
        node = EffectsNode()
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
