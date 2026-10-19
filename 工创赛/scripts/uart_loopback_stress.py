#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

import serial

from smart_carrier.protocol import Command, Packet, StreamParser, encode_packet


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify a physical 3.3V UART TX-RX loopback")
    parser.add_argument("--device", default="/dev/ttyS6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--frames", type=int, default=100000)
    args = parser.parse_args()
    port = serial.Serial(args.device, args.baud, timeout=0.2, write_timeout=0.2)
    stream = StreamParser()
    received = 0
    started = time.monotonic()
    try:
        for sequence in range(args.frames):
            expected = sequence & 0xFFFF
            port.write(encode_packet(Packet(expected, Command.HEARTBEAT, expected.to_bytes(2, "big"))))
            deadline = time.monotonic() + 0.5
            found = False
            while time.monotonic() < deadline:
                for packet in stream.feed(port.read(1024)):
                    if packet.sequence == expected and packet.payload == expected.to_bytes(2, "big"):
                        received += 1
                        found = True
                        break
                if found:
                    break
            if not found:
                break
    finally:
        port.close()
    report = {"sent": args.frames, "received": received, "crc_errors": stream.crc_errors, "seconds": time.monotonic() - started}
    print(json.dumps(report))
    return 0 if received == args.frames and stream.crc_errors == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

