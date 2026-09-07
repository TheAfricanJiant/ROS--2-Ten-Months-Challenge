"""Framing and reply parsing for the SSCMA AT protocol.

The Grove Vision AI V2 runs sscma-micro, which speaks a line-oriented AT
protocol over USB CDC. Commands go out as ``AT+<CMD>\\r\\n``; replies come
back as a single JSON object wrapped in ``\\r`` ... ``\\n``::

    {"type": 0, "name": "SAMPLE", "code": 0, "data": {}}
    {"type": 1, "name": "SAMPLE", "code": 0, "data": {"count": 3,
                                                      "image": "<base64 jpeg>"}}

``type`` distinguishes a direct response to a command from an asynchronous
event emitted while streaming:

===== ==========================================================
type  meaning
===== ==========================================================
0     response - the device acknowledging the command we sent
1     event    - a streamed result (this is where frames arrive)
2     log      - diagnostic chatter, and where an unsupported
               command is reported: ``{"type": 2, "name": "AT",
               "code": 5, "data": "Unknown command: AT+LED?"}``
===== ==========================================================

Base64 payloads only use ``A-Za-z0-9+/=``, so no reply body can contain a
newline. Splitting the stream on ``\\n`` is therefore sufficient framing and
avoids having to brace-match across a 60 kB JPEG.
"""

from __future__ import annotations

import base64
import binascii
import json
from dataclasses import dataclass
from typing import Any, Iterator

__all__ = [
    "REPLY_RESPONSE",
    "REPLY_EVENT",
    "REPLY_LOG",
    "Reply",
    "ReplyParser",
    "decode_jpeg",
]

REPLY_RESPONSE = 0
REPLY_EVENT = 1
REPLY_LOG = 2

#: A single reply must fit in this much memory. A 640x480 JPEG in base64 is
#: well under 200 kB; anything larger means we have lost sync with the device.
MAX_BUFFER_BYTES = 4 * 1024 * 1024


@dataclass(frozen=True)
class Reply:
    """One decoded JSON reply from the device."""

    name: str
    type: int
    code: int
    data: Any

    @property
    def ok(self) -> bool:
        """SSCMA uses ``code == 0`` for success."""
        return self.code == 0

    @property
    def is_event(self) -> bool:
        return self.type == REPLY_EVENT

    @property
    def is_response(self) -> bool:
        return self.type == REPLY_RESPONSE


def _parse_line(line: bytes) -> Reply | None:
    """Decode one framed line, or return None if it is not a reply."""
    text = line.strip().strip(b"\r").decode("utf-8", "replace").strip()
    if not text.startswith("{"):
        return None  # boot banners and stray bytes

    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return None

    if not isinstance(payload, dict) or "name" not in payload:
        return None

    # Normalise the command name so callers can match on one spelling:
    #   "INVOKE@1" -> "INVOKE"   (firmware tags some replies)
    #   "ID?"      -> "ID"       (queries echo the '?', actions do not)
    name = str(payload.get("name", "")).split("@", 1)[0].strip().rstrip("?").upper()

    return Reply(
        name=name,
        type=int(payload.get("type", REPLY_RESPONSE)),
        code=int(payload.get("code", 0)),
        data=payload.get("data"),
    )


class ReplyParser:
    """Turns an arbitrarily chunked byte stream into :class:`Reply` objects."""

    def __init__(self, max_buffer: int = MAX_BUFFER_BYTES) -> None:
        self._buf = bytearray()
        self._max_buffer = max_buffer

    def feed(self, chunk: bytes) -> Iterator[Reply]:
        """Add received bytes and yield every complete reply they finish."""
        if not chunk:
            return

        self._buf += chunk

        while True:
            index = self._buf.find(b"\n")
            if index < 0:
                break
            line = bytes(self._buf[:index])
            del self._buf[: index + 1]
            reply = _parse_line(line)
            if reply is not None:
                yield reply

        # If the device wedges mid-reply we would otherwise grow without
        # bound. Dropping the partial line resynchronises on the next '\n'.
        if len(self._buf) > self._max_buffer:
            self._buf.clear()

    def reset(self) -> None:
        self._buf.clear()


def decode_jpeg(encoded: str):
    """Decode a base64 JPEG from a reply into a BGR image.

    Returns ``None`` when the payload is missing or undecodable, so a single
    corrupt frame drops rather than ending the stream.
    """
    # Imported lazily so protocol parsing stays usable without OpenCV.
    import cv2
    import numpy as np

    if not encoded:
        return None

    try:
        raw = base64.b64decode(encoded, validate=False)
    except (binascii.Error, ValueError):
        return None

    if not raw:
        return None

    return cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
