from __future__ import annotations

import logging
import time

import serial

from .protocol import Command, Packet, StreamParser, encode_packet

LOG = logging.getLogger(__name__)


class SerialTransport:
    def __init__(self, device: str, baud: int = 115200, timeout: float = 0.05) -> None:
        self.serial = serial.Serial(device, baud, timeout=timeout, write_timeout=timeout)
        self.parser = StreamParser()
        self._sequence = 0

    def close(self) -> None:
        self.serial.close()

    def send(self, command: Command, payload: bytes = b"") -> int:
        self._sequence = (self._sequence + 1) & 0xFFFF
        self.serial.write(encode_packet(Packet(self._sequence, command, payload)))
        self.serial.flush()
        return self._sequence

    def receive(self, deadline_s: float) -> list[Packet]:
        packets: list[Packet] = []
        deadline = time.monotonic() + deadline_s
        while time.monotonic() < deadline:
            data = self.serial.read(1024)
            if data:
                packets.extend(self.parser.feed(data))
                if packets:
                    return packets
        return packets

    def safe_stop(self) -> None:
        try:
            self.send(Command.SAFE_STOP)
        except serial.SerialException:
            LOG.exception("failed to send SAFE_STOP; STM32 hardware timeout must take over")

