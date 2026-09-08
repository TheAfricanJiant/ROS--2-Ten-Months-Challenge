"""Serial client for a Grove Vision AI V2 running sscma-micro."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from typing import Iterator, Sequence

import serial
from serial.tools import list_ports

from .protocol import REPLY_LOG, Reply, ReplyParser, decode_jpeg

__all__ = [
    "DEFAULT_BAUDRATE",
    "SSCMAError",
    "Detection",
    "Frame",
    "PortInfo",
    "available_ports",
    "find_port",
    "port_hint",
    "permission_hint",
    "open_failure_hint",
    "identify",
    "SSCMAClient",
]

#: sscma-micro ships with the USB CDC link at this rate.
DEFAULT_BAUDRATE = 921600

#: USB vendor IDs of bridges these boards ship behind. The Grove Vision AI V2
#: sits behind a WCH CH343, so matching on Seeed's own VID alone finds nothing.
BRIDGE_VENDOR_IDS = {
    0x1A86: "WCH (CH34x)",
    0x2886: "Seeed",
    0x10C4: "Silicon Labs",
    0x0403: "FTDI",
}

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
    serial_number: str | None

    @property
    def is_candidate(self) -> bool:
        """Sits behind a USB-serial bridge these boards are known to use."""
        return self.vid in BRIDGE_VENDOR_IDS

    def __str__(self) -> str:
        bits = [f"{self.device:6}", self.description]
        if self.serial_number:
            bits.append(f"[SER={self.serial_number}]")
        return "  ".join(bits)


def port_hint() -> str:
    """An example port name for whatever OS we are on."""
    if sys.platform.startswith("win"):
        return "COM3"
    if sys.platform == "darwin":
        return "/dev/cu.wchusbserial1420"
    return "/dev/ttyACM0"


def open_failure_hint(exc: BaseException) -> str:
    """Advice matched to *why* the port would not open.

    A missing port and a busy port are different problems with different
    fixes, and reporting one as the other sends people hunting for a program
    that is not there.
    """
    cause = exc.__cause__ or exc.__context__ or exc
    text = f"{exc} {cause}".lower()

    missing = (
        isinstance(cause, FileNotFoundError)
        or "cannot find the file" in text
        or "no such file" in text
        or getattr(cause, "errno", None) == 2
    )
    if missing:
        return ("That port does not exist. The board may be unplugged, or it "
                "may have come back on a different port - list them with:\n"
                "  python stream_camera.py --list-ports")

    denied = (
        isinstance(cause, PermissionError)
        or "access is denied" in text
        or "permission denied" in text
        or getattr(cause, "errno", None) == 13
    )
    if denied:
        return permission_hint()

    return ("Check the board is plugged in and no other program is using it "
            "(python stream_camera.py --list-ports).")


def permission_hint() -> str:
    """Platform-specific advice when a port exists but will not open."""
    if sys.platform.startswith("win"):
        return ("Another program holds the port - a SenseCraft tab in Chrome "
                "keeps it open even when it fails to connect.")
    if sys.platform == "darwin":
        return ("Another program holds the port, or macOS has not approved the "
                "USB-serial driver (System Settings -> Privacy & Security).")
    return ("Another program holds the port, or your user is not in the "
            "'dialout' group. Add yourself with:\n"
            "  sudo usermod -a -G dialout $USER\n"
            "then log out and back in.")


def available_ports() -> list[PortInfo]:
    """Every serial port the OS exposes, likely boards first."""
    ports = [
        PortInfo(
            device=p.device,
            description=p.description or "unknown device",
            hwid=p.hwid or "",
            vid=p.vid,
            serial_number=p.serial_number,
        )
        for p in list_ports.comports()
    ]
    ports.sort(key=lambda p: (not p.is_candidate, p.device))
    return ports


def identify(port: str, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 1.5) -> str | None:
    """Return the board's reported name, or None if it does not speak SSCMA.

    Two identical boards look the same over USB descriptors, and a board
    running the wrong firmware looks the same as a healthy one. Asking is the
    only reliable way to tell them apart.
    """
    try:
        with SSCMAClient(port=port, baudrate=baudrate, read_timeout=0.3) as client:
            name = client.device_info().get("name")
            return name or None
    except (SSCMAError, serial.SerialException, OSError):
        return None


def find_port(probe: bool = True) -> str:
    """Best guess at the Vision AI V2's port.

    With one candidate, use it. With several, ask each one who it is and pick
    the board that actually answers, so a second board running unrelated
    firmware cannot be selected by accident.
    """
    ports = available_ports()
    if not ports:
        raise SSCMAError(
            "No serial ports found. Check the USB-C cable (a charge-only "
            "cable will power the board but carry no data)."
        )

    candidates = [p for p in ports if p.is_candidate] or ports
    if len(candidates) == 1:
        return candidates[0].device

    if probe:
        responding = [(p, identify(p.device)) for p in candidates]
        alive = [(p, name) for p, name in responding if name]
        if len(alive) == 1:
            port, name = alive[0]
            print(f"Auto-selected {port.device} ({name}); "
                  f"{len(candidates) - 1} other port(s) did not answer.")
            return port.device
        if len(alive) > 1:
            listing = "\n  ".join(f"{p}  -> {name}" for p, name in alive)
            raise SSCMAError(
                "Several boards answered. Pick one with --port (the SER= value "
                f"is stable per board):\n  {listing}"
            )

    listing = "\n  ".join(str(p) for p in ports)
    raise SSCMAError(
        "No port answered as an SSCMA device. Pick one manually with --port, "
        "or check that the board is running sscma-micro firmware:\n  " + listing
    )


class SSCMAClient:
    """Talks AT commands to the board and yields decoded frames.

    Use as a context manager so the device is always told to stop streaming::

        with SSCMAClient(port_hint()) as client:
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
            raise SSCMAError(
                f"Could not open {self.port}: {exc}\n{open_failure_hint(exc)}"
            ) from exc

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

    def command(self, command: str, expect: str | None = None, timeout: float = 3.0):
        """Send a command and wait for the device's acknowledgement.

        ``expect`` defaults to the command's own name. Reply names arrive
        without any trailing '?' or '@tag' (see :mod:`.protocol`), so
        ``AT+VER?`` is matched by ``VER``.
        """
        if expect is None:
            expect = command.split("=", 1)[0].rstrip("?")
        expect = expect.split("=", 1)[0].rstrip("?").upper()

        self.write(command)
        for reply in self.replies(timeout=timeout):
            # Unsupported commands come back as a log line, not a response.
            if reply.type == REPLY_LOG and isinstance(reply.data, str) \
                    and reply.data.lower().startswith("unknown command"):
                raise SSCMAError(f"This firmware does not support AT+{command}.")

            if reply.name != expect or not reply.is_response:
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
        """Identity strings, for confirming we are talking to the right board.

        ``AT+ID?`` and ``AT+NAME?`` answer with a bare string; ``AT+VER?``
        answers with a dict, of which the firmware build date is the useful
        part.
        """
        info: dict[str, str] = {}

        for command, key in (("ID?", "id"), ("NAME?", "name")):
            try:
                data = self.command(command, timeout=2.0)
            except SSCMAError:
                continue
            if isinstance(data, str) and data:
                info[key] = data

        try:
            version = self.command("VER?", timeout=2.0)
        except SSCMAError:
            version = None
        if isinstance(version, dict):
            software = version.get("software")
            if software:
                info["firmware"] = str(software)
        elif isinstance(version, str) and version:
            info["firmware"] = version

        return info

    def sensors(self) -> list[dict]:
        """Cameras the firmware can see. ``state`` 1 means present and ready."""
        data = self.command("SENSORS?", timeout=2.0)
        return data if isinstance(data, list) else []

    def models(self) -> list[dict]:
        """Model slots. A slot with ``size`` 0 holds no model."""
        data = self.command("MODELS?", timeout=2.0)
        return data if isinstance(data, list) else []

    def has_model(self) -> bool:
        """True when a model is actually flashed, so AT+INVOKE can work."""
        return any(int(m.get("size") or 0) > 0 for m in self.models())

    def set_resolution(self, opt_id: int, sensor_id: int = 1) -> None:
        """Select one of the resolutions ``AT+SENSORS?`` advertises."""
        self.command(f"SENSOR={sensor_id},1,{opt_id}", expect="SENSOR", timeout=4.0)

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
