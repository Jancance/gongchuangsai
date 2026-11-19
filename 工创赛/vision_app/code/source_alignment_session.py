"""MR1 request -> existing A rim geometry -> MA1 reply, no device I/O.

The application owns its ONE camera/serial connection. Feed actual received
bytes and pass a captured frame's boundary/metadata here. This module never
guesses body transforms, frame exposure time, heading or a successful move.
"""
from __future__ import annotations
import math
from metric_alignment_protocol import decode_request
from source_metric_preview import preview_rim
from source_metric_bridge import observation_packet


class SourceAlignmentSession:
    def __init__(self, calibration: dict, context: dict, profile_ids: dict):
        self.calibration = calibration
        self.context = context
        self.profile_ids = dict(profile_ids)
        self.request = None
        self.received_ms = None
        self._line = bytearray()
        self._last_byte_ms = None
        self._overflow = False
        self.last_reason = 'WAIT_REQUEST'

    def invalidate(self):
        self.request = self.received_ms = None
        self.last_reason = 'INVALIDATED'

    def receive(self, data: bytes, now_ms: int):
        """Feed shared UART bytes. Only CRC-valid bounded MR1 lines are used.

        now_ms is the SBC monotonic time; never subtract MCU tick from it.
        A repeated request does not restart its original deadline.
        """
        if not isinstance(data, bytes) or type(now_ms) is not int or now_ms < 0:
            raise ValueError('bytes and actual monotonic milliseconds required')
        if self._last_byte_ms is not None and now_ms-self._last_byte_ms >= 200:
            self._line.clear()
            self._overflow = False
        self._last_byte_ms = now_ms
        for byte in data:
            if byte == 10:
                frame = bytes(self._line) + b'\n'
                self._line.clear()
                if self._overflow:
                    self._overflow = False
                    self.last_reason = 'OVERSIZE'
                    continue
                if not frame.startswith(b'MR1,'):
                    continue
                try:
                    request = decode_request(frame)
                    ids = ('camera_id', 'calibration_id', 'mount_id', 'height_id', 'target_id')
                    if any(self.profile_ids.get(k) != getattr(request, k) for k in ids):
                        raise ValueError('actual calibration IDs differ')
                    if request != self.request:
                        self.request, self.received_ms = request, now_ms
                    self.last_reason = 'REQUEST_ACCEPTED'
                except ValueError:
                    self.invalidate()
                    self.last_reason = 'BAD_REQUEST'
                continue
            if self._overflow:
                continue
            if not self._line and byte != ord('M'):
                # AA/CC mode/observation bytes on the shared link are not an
                # ASCII line prefix and must not poison the next MR1 request.
                continue
            if len(self._line) >= 254 or byte not in (13,) and not 32 <= byte <= 126:
                self._line.clear()
                self._overflow = True
                continue
            self._line.append(byte)

    def observe(self, boundary_px, *, frame_sequence: int, captured_ms: int,
                now_ms: int, stable: bool, confidence: float,
                exposure_time_verified: bool, translation_only_verified: bool):
        """Return (MA1 bytes|None, diagnostic preview) for this actual frame.

        Boundary points are from the current A detector. Captured time must
        come from a verified capture timestamp/age bound, not read() completion.
        A packet is an observation, never an actuator instruction or DONE.
        """
        if self.request is None:
            return None, {'status': 'WAIT_REQUEST', 'motion_allowed': False}
        if any(type(x) is not int or x < 0 for x in (frame_sequence, captured_ms, now_ms)):
            raise ValueError('integer monotonic frame metadata required')
        elapsed = now_ms-self.received_ms
        if elapsed < 0 or elapsed >= self.request.timeout_ms:
            self.invalidate()
            return None, {'status': 'REQUEST_EXPIRED', 'motion_allowed': False}
        if not math.isfinite(confidence):
            raise ValueError('finite confidence required')
        age = now_ms-captured_ms
        after = captured_ms-self.received_ms
        preview = preview_rim(boundary_px, self.calibration, frame_context=self.context,
                              stable=stable, confidence=confidence, frame_age_ms=age,
                              timing_verified=exposure_time_verified)
        if after < 0 or age < 0:
            self.last_reason = 'PRE_REQUEST_OR_FUTURE_FRAME'
            return None, preview
        try:
            packet = observation_packet(preview, self.request, profile_ids=self.profile_ids,
                frame_sequence=frame_sequence, capture_after_request_ms=after,
                frame_age_ms=age, confidence=confidence, timing_verified=exposure_time_verified,
                translation_only_verified=translation_only_verified)
        except ValueError as exc:
            self.last_reason = str(exc)
            return None, preview
        self.last_reason = 'METRIC_OBSERVATION_READY'
        return packet, preview
