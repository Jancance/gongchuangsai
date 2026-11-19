import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from protocol import crc16, frame, decode, parse_status, observation, Session
from stage3 import Pipeline, matrix


def status(run=1, pose=1, state=2, ready=1):
    return parse_status(frame(('S31', run, state, pose, 0, 2, 1, 5, 0, ready)))


class ProtocolTest(unittest.TestCase):
    def test_crc_and_roundtrip(self):
        self.assertEqual(crc16(b'123456789'), 0x29B1)
        s = status()
        self.assertEqual((s.run, s.pose, s.state), (1, 1, 2))
        o = decode(observation(1, 2, 3, True, (1.234, -2.345, 1.), 20))
        self.assertEqual(o, ['O31','1','2','3','1','1234','-2345','1000','0','0','20'])
        for yaw in (-180., -1., 0., 1., 180.):
            self.assertEqual(int(decode(observation(1,1,1,True,(0,0,yaw),0))[7]), yaw*1000)

    def test_crc_bad_and_fields(self):
        data = bytearray(frame(('S31',1,2,3,0,2,1,5,0,1)))
        data[5] ^= 1
        with self.assertRaises(ValueError):
            parse_status(bytes(data))
        with self.assertRaises(ValueError):
            parse_status(frame(('S31',1,99,3,0,2,1,5,0,1)))
        with self.assertRaises(ValueError):
            observation(1,1,1,True,(float('nan'),0,0),0)

    def test_epoch_newframe_and_state(self):
        s = Session(); self.assertIsNone(s.encode(0, 1, 1, 1, True, (1,2,3)))
        s.accept(status(), 10.)
        self.assertEqual(decode(s.encode(1, 9.99,10.01,10.02,True,(1,2,3)))[4], '0')
        self.assertEqual(decode(s.encode(2,10.01,10.02,10.03,True,(1,2,3)))[4], '1')
        self.assertIsNone(s.encode(2,10.01,10.02,10.03,True,(1,2,3)))
        s.accept(status(pose=2),10.04)
        self.assertEqual(decode(s.encode(3,10.03,10.05,10.06,True,(1,2,3)))[4], '0')
        s.accept(status(state=3),10.07)
        self.assertEqual(decode(s.encode(4,10.08,10.09,10.1,True,(1,2,3)))[4], '0')

    def test_total_age_200_201_and_unverified(self):
        for duration, expected in ((.200,'1'),(.201,'0')):
            s=Session(); s.accept(status(),10.)
            # Fresh MCU heartbeat cannot make an old captured image fresh.
            s.accept(status(),10.19)
            out = decode(s.encode(1,10.001,10.002,10.001+duration,True,(1,2,3)))
            self.assertEqual(out[4],expected)
        s=Session(); s.accept(status(),10.)
        out=decode(s.encode(1,10.01,10.02,10.03,False,(1,2,3)))
        self.assertEqual(out[4:8],['0','0','0','0'])
        s=Session(); s.accept(status(ready=0),10.)
        out=decode(s.encode(1,10.01,10.02,10.03,True,(1,2,3),(1.99,-.9)))
        self.assertEqual(out[4],'1')
        self.assertEqual(out[8:10],['1990','-900'])

    def test_unconfigured_profile_never_verified(self):
        p = Pipeline(ROOT/'config/stage3.json', (640,480))
        self.assertFalse(p.verified())
        with self.assertRaises(ValueError):
            matrix([[0,0],[0,0]], (2,2))


if __name__ == '__main__':
    unittest.main()
