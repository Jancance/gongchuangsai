from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from enum import IntEnum
import json
import struct
from typing import Any

HEADER = b"\xA5\x5A"
TAIL = b"\x0D\x0A"
VERSION = 1
MAX_PAYLOAD = 512


class Command(IntEnum):
    HEARTBEAT = 0x01
    STATUS = 0x02
    SET_VELOCITY = 0x10
    EXECUTE_PLAN = 0x11
    SAFE_STOP = 0x12
    ACTION_TARGET = 0x13
    ROBOT_STATE = 0x20
    ACCEPTED = 0x80
    DONE = 0x81
    ERROR = 0xFF


@dataclass(frozen=True)
class Packet:
    sequence: int
    command: int
    payload: bytes = b""
    version: int = VERSION


def encode_json_payload(value: dict[str, Any]) -> bytes:
    payload = json.dumps(value, ensure_ascii=True, separators=(",", ":"), sort_keys=True).encode("utf-8")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("JSON payload too large")
    return payload


def decode_json_payload(payload: bytes) -> dict[str, Any]:
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid JSON payload") from exc
    if not isinstance(value, dict):
        raise ValueError("JSON payload root must be an object")
    return value


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_packet(packet: Packet) -> bytes:
    if not 0 <= packet.sequence <= 0xFFFF:
        raise ValueError("sequence must fit uint16")
    if len(packet.payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = struct.pack(">BHHB", packet.version, len(packet.payload), packet.sequence, int(packet.command)) + packet.payload
    return HEADER + body + struct.pack(">H", crc16_ccitt_false(body)) + TAIL


class StreamParser:
    """Noise-tolerant parser for partial and concatenated UART frames."""

    _MIN_FRAME = len(HEADER) + 6 + 2 + len(TAIL)

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.length_errors = 0

    def feed(self, data: bytes) -> list[Packet]:
        self._buffer.extend(data)
        packets: list[Packet] = []
        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer[-1:] == HEADER[:1] else b""
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < self._MIN_FRAME:
                break
            version, length, sequence, command = struct.unpack(">BHHB", self._buffer[2:8])
            if length > MAX_PAYLOAD:
                self.length_errors += 1
                del self._buffer[0]
                continue
            frame_length = 2 + 6 + length + 2 + 2
            if len(self._buffer) < frame_length:
                break
            candidate = bytes(self._buffer[:frame_length])
            if candidate[-2:] != TAIL:
                del self._buffer[0]
                continue
            body = candidate[2 : 8 + length]
            expected_crc = struct.unpack(">H", candidate[8 + length : 10 + length])[0]
            if crc16_ccitt_false(body) != expected_crc:
                self.crc_errors += 1
                del self._buffer[0]
                continue
            packets.append(Packet(sequence, command, candidate[8 : 8 + length], version))
            del self._buffer[:frame_length]
        return packets


class IdempotencyCache:
    """Caches final MCU responses so retries cannot repeat physical motion."""

    def __init__(self, capacity: int = 128) -> None:
        self.capacity = capacity
        self._items: OrderedDict[int, Packet] = OrderedDict()

    def remember(self, request_sequence: int, response: Packet) -> None:
        self._items[request_sequence] = response
        self._items.move_to_end(request_sequence)
        while len(self._items) > self.capacity:
            self._items.popitem(last=False)

    def lookup(self, request_sequence: int) -> Packet | None:
        return self._items.get(request_sequence)
