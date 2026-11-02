import binascii
import unittest
from p1_serial_motion import parse_odometry_telemetry, parse_telemetry

def wire(values):
    p=("OD1,"+",".join(map(str,values))+",").encode()
    return p+f"{binascii.crc_hqx(p,0xFFFF):04X}\r\n".encode()

class OdometryTelemetryTests(unittest.TestCase):
    def test_valid_observation_never_becomes_control_telemetry(self):
        f=wire([200,180,3,1,0,0,1,100,-20,90000,20])
        self.assertEqual(parse_odometry_telemetry(f)["y_mm"],-20)
        self.assertIsNone(parse_telemetry(f))
    def test_loss_and_unknown_age_are_preserved(self):
        r=parse_odometry_telemetry(wire([200,0,0,0,10,1,1,0,0,0,0xFFFFFFFF]))
        self.assertEqual(r["age_ms"],0xFFFFFFFF)
        self.assertFalse(r["valid"])
    def test_crc_length_and_validity_rejected(self):
        with self.assertRaises(ValueError):parse_odometry_telemetry(b'OD1,1,0000')
        for v in [[1],[1,1,1,1,8,1,1,0,0,0,0],[-1,1,1,1,0,0,1,0,0,0,0]]:
            with self.assertRaises(ValueError):parse_odometry_telemetry(wire(v))
    def test_other_frames_ignored(self):
        self.assertIsNone(parse_odometry_telemetry(b'TM1,...'))

if __name__=='__main__':unittest.main()
