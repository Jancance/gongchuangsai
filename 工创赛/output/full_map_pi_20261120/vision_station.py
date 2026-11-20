"""Full-map station identity and camera frame admission."""

from binascii import crc_hqx
from dataclasses import dataclass
import math

from mcu_cycle_protocol import frame


@dataclass(frozen=True)
class Station:
    run: int
    request: int
    batch: int
    mode: int
    pose: int
    ring: int
    color: int

    @classmethod
    def parse(cls, line):
        if not line.endswith(b"\r") or len(line) > 100:
            raise ValueError("VS1 framing")
        body, crc = line[:-1].rsplit(b",", 1)
        if crc != f"{crc_hqx(body + b',', 0xffff):04X}".encode("ascii"):
            raise ValueError("VS1 CRC")
        prefix, *fields = body.split(b",")
        if prefix != b"VS1" or len(fields) != 7:
            raise ValueError("VS1 fields")
        values = tuple(map(int, fields))
        if any(str(value).encode("ascii") != field or not 0 <= value <= 0xFFFFFFFF
               for value, field in zip(values, fields)):
            raise ValueError("VS1 integer")
        station = cls(*values)
        if station.mode == 0:
            if station.batch or station.ring or station.color:
                raise ValueError("VS1 transit")
        elif (not station.run or not station.request or station.batch not in (1, 2)
              or station.mode not in (1, 2, 3, 4) or station.ring > 3 or station.color > 6
              or (station.mode == 3 and station.batch != 1)
              or (station.mode == 4 and station.batch != 2)):
            raise ValueError("VS1 station")
        return station

    def observation(self, sequence, occupied, age):
        return frame("VO1", (self.run, self.request, self.batch, self.mode, self.pose,
                            sequence, self.ring, self.color, 1, occupied, 1, age))


class StationGate:
    def __init__(self):
        self.station = None
        self.last_envelope_at = None
        self.last_status_at = None
        self.paired = False
        self.status_context = None
        self.fence = None
        self.previous = None
        self.first_valid = False
        self.activated_at = None
        self.last_active_station = None

    def announce(self, station, now):
        old = self.station
        last = self.last_active_station
        if last and last.run == station.run and station.mode:
            advance = (station.request - last.request) & 0xFFFFFFFF
            if advance >= 0x80000000 or (not advance and
                    (old.mode, old.batch) != (station.mode, station.batch)):
                self.paired = False
                return False
            if not advance and (station.pose - last.pose) & 0xFFFFFFFF >= 0x80000000:
                self.paired = False
                return False
        elapsed = self.last_envelope_at is None or now - self.last_envelope_at > .2
        changed = elapsed or station != old
        if changed:
            self.status_context = None
            self.last_status_at = None
            self.previous = None
            self.fence = now
            if old is None or (station.run, station.request, station.mode) != (old.run, old.request, old.mode):
                self.first_valid = False
                self.activated_at = now
        self.station = station
        if station.mode:self.last_active_station = station
        self.last_envelope_at = now
        self.paired = bool(station.mode)
        return changed

    def accept_status(self, source, run, pose, state, now):
        station = self.station
        paired, self.paired = self.paired, False
        if (not paired or now - self.last_envelope_at > .2
                or source != (station.mode == 1)
                or (run, pose) != (station.run, station.pose)):
            return False
        if state != self.status_context or self.last_status_at is None or now - self.last_status_at > .2:
            self.status_context = state
            self.fence = now
            self.previous = None
        self.last_status_at = now
        return True

    def active(self, now):
        return self.last_status_at is not None and 0 <= now - self.last_status_at <= .2

    def acquisition(self, metadata, now):
        if not self.active(now) or metadata is None:
            self.previous = None
            return None
        if (type(metadata["sequence"]) is not int or metadata["sequence"] < 1
                or not isinstance(metadata["completed"], (int, float))
                or not math.isfinite(metadata["completed"])):
            self.previous = None
            return None
        previous = self.previous
        self.previous = dict(metadata)
        if (previous is None or metadata["generation"] != previous["generation"]
                or metadata["sequence"] <= previous["sequence"]):
            return None
        start, end = previous["completed"], metadata["completed"]
        if self.fence < start < end <= now and math.ceil((now - start) * 1000) <= 200:
            return start
        return None
