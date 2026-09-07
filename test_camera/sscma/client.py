"""Serial client for a Grove Vision AI V2 running sscma-micro."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Iterator, Sequence

import serial
from serial.tools import list_ports

from .protocol import REPLY_EVENT, Reply, ReplyParser, decode_jpeg

__all__ = [
    "DEFAULT_BAUDRATE",
    "SSCMAError",
    "Detection",
    "Frame",
    "PortInfo",
    "available_ports",
    "find_port",
    "SSCMAClient",
]

#: sscma-micro ships with the USB CDC link at this rate.
DEFAULT_BAUDRATE = 921600

#: Seeed Studio's USB vendor ID, used to rank candidate ports.
SEEED_VENDOR_ID = 0x2886

#: SSCMA reports boxes as [x, y, w, h, score, target] with x/y at the box
#: *centre*. Firmware forks occasionally emit a top-left origin instead; if
#: overlays sit down-and-right of the object, switch to "corner".
BOX_FORMATS = ("center", "corner")


class SSCMAError(RuntimeError):
    """The device reported a failure, or did not reply in time."""


@dataclass(frozen=True)
class Detection:
    """One bounding box, in pixel coordinates of the frame it came from."""

    x: int
    y: int
    width: int
    height: int
    score: int
    target: int
    label: str

    @property
    def top_left(self) -> tuple[int, int]:
        return self.x, self.y

    @property
    def bottom_right(self) -> tuple[int, int]:
        return self.x + self.width, self.y + self.height


@dataclass
class Frame:
    """A decoded camera frame plus whatever the model inferred about it."""

    image: "object"  # numpy.ndarray (BGR); typed loosely to avoid the import
    index: int
    detections: list[Detection] = field(default_factory=list)
    perf: Sequence[int] = ()

    @property
    def size(self) -> tuple[int, int]:
        height, width = self.image.shape[:2]
        return width, height

    @property
    def inference_ms(self) -> int | None:
        """Model inference time. SSCMA reports [preprocess, inference, post]."""
        return int(self.perf[1]) if len(self.perf) > 1 else None


@dataclass(frozen=True)
class PortInfo:
    device: str
    description: str
    hwid: str
    vid: int | None

    @property
    def is_seeed(self) -> bool:
        return self.vid == SEEED_VENDOR_ID

    def __str__(self) -> str:
        tag = "  <- Seeed device" if self.is_seeed else ""
        return f"{self.device:8} {self.description}{tag}"


def available_ports() -> list[PortInfo]:
    """Every serial port Windows currently exposes, Seeed devices first."""
    ports = [
        PortInfo(
            device=p.device,
            description=p.description or "unknown device",
            hwid=p.hwid or "",
            vid=p.vid,
        )
        for p in list_ports.comports()
    ]
    ports.sort(key=lambda p: (not p.is_seeed, p.device))
    return ports


def find_port() -> str:
    """Best guess at the Vision AI V2's port.

    Prefers a Seeed vendor ID; falls back to the only port present. Raises if
    the choice is ambiguous, so we never stream from the wrong device.
    """
    ports = available_ports()
    if not ports:
        raise SSCMAError(
            "No serial ports found. Check the USB-C cable (a charge-only "
            "cable will power the board but carry no data)."
        )

    seeed = [p for p in ports if p.is_seeed]
    if len(seeed) == 1:
        return seeed[0].device
    if not seeed and len(ports) == 1:
        return ports[0].device

    listing = "\n  ".join(str(p) for p in ports)
    raise SSCMAError(
        f"Could not pick a port automatically. Choose one with --port:\n  {listing}"
    )


class SSCMAClient:
    """Talks AT commands to the board and yields decoded frames.

    Use as a context manager so the device is always told to stop streaming::

        with SSCMAClient("COM5") as client:
            for frame in client.stream():
                ...
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        read_timeout: float = 1.0,
        box_format: str = "center",
    ) -> None:
        if box_format not in BOX_FORMATS:
            raise ValueError(f"box_format must be one of {BOX_FORMATS}")

        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.box_format = box_format
        self.labels: list[str] = []

        self._serial: serial.Serial | None = None
        self._parser = ReplyParser()
        self._frame_index = 0

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> "SSCMAClient":
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.read_timeout,
                write_timeout=2.0,
            )
        except serial.SerialException as exc:
            raise SSCMAError(f"Could not open {self.port}: {exc}") from exc

        # The board may still be streaming from a previous run.
        self.stop()
        self._serial.reset_input_buffer()
        self._parser.reset()
        return self

    def close(self) -> None:
        if self._serial is None:
            return
        try:
            if self._serial.is_open:
                self.stop()
        except (SSCMAError, serial.SerialException):
            pass  # closing anyway
        finally:
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "SSCMAClient":
        return self.open()

    def __exit__(self, *_exc_info) -> None:
        self.close()

    # -- transport ---------------------------------------------------------

    @property
    def _link(self) -> serial.Serial:
        if self._serial is None or not self._serial.is_open:
            raise SSCMAError("Port is not open. Call open() first.")
        return self._serial

    def write(self, command: str) -> None:
        """Send one AT command. ``command`` is given without the AT+ prefix."""
        payload = f"AT+{command}\r\n".encode("ascii")
        try:
            self._link.write(payload)
            self._link.flush()
        except serial.SerialException as exc:
            raise SSCMAError(f"Write to {self.port} failed: {exc}") from exc

    def replies(self, timeout: float | None) -> Iterator[Reply]:
        """Yield replies as they arrive.

        ``timeout`` bounds the wait for the *next* reply, not the whole
        stream. ``None`` waits indefinitely while staying responsive to
        Ctrl-C, because the underlying read still returns every
        ``read_timeout`` seconds.
        """
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            try:
                waiting = self._link.in_waiting
                chunk = self._link.read(waiting if waiting else 1)
            except serial.SerialException as exc:
                raise SSCMAError(f"Read from {self.port} failed: {exc}") from exc

            if chunk:
                got_any = False
                for reply in self._parser.feed(chunk):
                    got_any = True
                    yield reply
                if got_any and deadline is not None:
                    deadline = time.monotonic() + timeout

            if deadline is not None and time.monotonic() > deadline:
                raise SSCMAError(
                    f"{self.port} went quiet for {timeout:.0f}s. Is the board "
                    "running sscma-micro, and is the baud rate right?"
                )

    def command(self, command: str, expect: str, timeout: float = 3.0):
        """Send a command and wait for the device's acknowledgement."""
        self.write(command)
        for reply in self.replies(timeout=timeout):
            if reply.name != expect.upper() or not reply.is_response:
                continue
            if not reply.ok:
                raise SSCMAError(f"AT+{command} rejected (code {reply.code}).")
            return reply.data
        return None

    # -- commands ----------------------------------------------------------

    def stop(self) -> None:
        """Ask the device to end any running SAMPLE or INVOKE loop."""
        self.write("BREAK")
        # BREAK is best-effort: a busy device may not answer promptly, and we
        # do not want teardown to raise on the way out.
        try:
            for reply in self.replies(timeout=1.0):
                if reply.name == "BREAK":
                    return
        except SSCMAError:
            return

    def device_info(self) -> dict[str, str]:
        """Identity strings, for confirming we are talking to the right board."""
        info: dict[str, str] = {}
        for command, key in (("ID?", "id"), ("NAME?", "name"), ("VER?", "version")):
            try:
                data = self.command(command, expect=command.rstrip("?"), timeout=2.0)
            except SSCMAError:
                continue
            if isinstance(data, dict):
                data = next(iter(data.values()), "")
            if data:
                info[key] = str(data)
        return info

    def set_labels(self, labels: Sequence[str]) -> None:
        self.labels = list(labels)

    def stream(self, detect: bool = False, times: int = -1) -> Iterator[Frame]:
        """Yield frames until the caller stops consuming, or ``times`` elapse.

        ``detect=False`` uses ``AT+SAMPLE``, a plain camera feed with no model
        involved - the fastest path to "is this camera working?". ``True``
        uses ``AT+INVOKE`` so each frame arrives with its detections.
        """
        if detect:
            # AT+INVOKE=<times>,<result_only>,<differed>
            #   result_only=0 -> include the JPEG alongside the results
            #   differed=0    -> emit every frame, not only on change
            command, name = f"INVOKE={times},0,0", "INVOKE"
        else:
            command, name = f"SAMPLE={times}", "SAMPLE"

        self._frame_index = 0
        self.write(command)

        for reply in self.replies(timeout=self.read_timeout * 10):
            if reply.name != name:
                continue

            if reply.is_response:
                if not reply.ok:
                    raise SSCMAError(
                        f"AT+{command} rejected (code {reply.code}). "
                        + (
                            "Is a model flashed to the board?"
                            if detect
                            else "Is a camera attached?"
                        )
                    )
                continue

            if not reply.is_event or not isinstance(reply.data, dict):
                continue

            frame = self._to_frame(reply.data)
            if frame is not None:
                yield frame

    # -- decoding ----------------------------------------------------------

    def _label_for(self, target: int) -> str:
        if 0 <= target < len(self.labels):
            return self.labels[target]
        return f"class {target}"

    def _to_detection(self, box: Sequence, image_size: tuple[int, int]) -> Detection | None:
        if len(box) < 6:
            return None

        x, y, w, h, score, target = (int(v) for v in box[:6])

        if self.box_format == "center":
            x -= w // 2
            y -= h // 2

        width, height = image_size
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = max(1, min(w, width - x))
        h = max(1, min(h, height - y))

        return Detection(x, y, w, h, score, target, self._label_for(target))

    def _to_frame(self, data: dict) -> Frame | None:
        image = decode_jpeg(data.get("image", ""))
        if image is None:
            return None

        height, width = image.shape[:2]
        detections = []
        for box in data.get("boxes") or ():
            detection = self._to_detection(box, (width, height))
            if detection is not None:
                detections.append(detection)

        self._frame_index += 1
        return Frame(
            image=image,
            index=self._frame_index,
            detections=detections,
            perf=tuple(data.get("perf") or ()),
        )
