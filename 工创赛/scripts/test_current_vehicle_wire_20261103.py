"""用本次真实串口样本验证TMQ/HF3解析及0.1RPM单位。"""
import binascii
from pathlib import Path
import unittest

import p1_serial_motion as protocol
import vehicle_uart_capture as capture


def frame(tag, values):
    body = (tag + ',' + ','.join(map(str, values)) + ',').encode()
    return body + f'{binascii.crc_hqx(body, 0xffff):04X}\r\n'.encode()


class CurrentWire(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'evidence/integration/2026-11-03/no-motion/before-uart/uart1.raw'
        cls.lines = path.read_bytes().splitlines()
        cls.tmq = next(x for x in cls.lines if x.startswith(b'TMQ,'))
        cls.hf3 = next(x for x in cls.lines if x.startswith(b'HF3,'))

    def test_real_no_motion_evidence(self):
        r = protocol.parse_telemetry(self.tmq)
        self.assertEqual((r['phase'], r['trip'], r['backend_result']), (4, 5, 9))
        self.assertEqual((r['sequence_index'], r['sequence_completed']), (0, 0))
        self.assertEqual((r['source_valid'], r['can_fault'], r['rtos_fault'], r['safety_fault']), (1, 0, 0, 0))
        self.assertEqual(capture.decode(self.tmq)[0], 'TMQ')
        self.assertEqual(capture.decode(self.hf3)[0], 'HF3')
        f = protocol.parse_fault_telemetry(self.hf3)
        self.assertEqual((f['trip'], f['phase'], f['backend_session']), (5, 1, 0))

    def test_current_rpm_encoding_is_converted_once(self):
        values = list(map(int, self.tmq.rsplit(b',', 1)[0].split(b',')[1:]))
        values[12] = 20
        values[13:17] = [992, -992, 1, -1]
        r = protocol.parse_telemetry(frame('TMQ', values))
        self.assertEqual(r['correction_rpm'], 2.0)
        self.assertEqual([r[f'rpm{i}'] for i in range(1, 5)], [99.2, -99.2, 0.1, -0.1])
        self.assertEqual(r['base_rpm'], 51)

    def test_crc_and_schema_are_not_bypassed(self):
        with self.assertRaises(ValueError):
            protocol.parse_telemetry(self.tmq[:-1] + b'Z')
        values = list(map(int, self.tmq.rsplit(b',', 1)[0].split(b',')[1:]))
        with self.assertRaises(ValueError):
            protocol.parse_telemetry(frame('TMQ', values[:-1]))
        with self.assertRaises(ValueError):
            protocol.parse_fault_telemetry(self.hf3[:-1] + b'Z')

    def test_direction_fault_signed_values(self):
        values = list(map(int, self.hf3.rsplit(b',', 1)[0].split(b',')[1:]))
        values[29] = 10
        values[38:40] = [0xfffffff6, 20]
        f = protocol.parse_fault_telemetry(frame('HF3', values))
        self.assertEqual(f['backend_target_decirpm'], -10)
        self.assertEqual(f['backend_measured_decirpm'], 20)
        values[29] = 13
        with self.assertRaises(ValueError):
            protocol.parse_fault_telemetry(frame('HF3', values))


if __name__ == '__main__':
    unittest.main()
