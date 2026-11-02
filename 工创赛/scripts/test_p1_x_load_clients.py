import binascii
import contextlib
import io
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import p1_x_load_diag_20261101 as pd
import p1_x_load_integral_20261101 as pi
from test_p1_fine_serial_motion import FineUnitsTests, frame


class LoadClientTests(unittest.TestCase):
    def test_client_firmware_identity_and_fractional_feedback(self):
        names=pd.FIELDS+pd.SEQUENCE_FIELDS+pd.CAN_DIAG_FIELDS+pd.ACTION_FIELDS+pd.DISPLAY_FIELDS+pd.DISTANCE_FIELDS+["distance_fault_reason","can_stack_words","debug_plan_id"]
        r=FineUnitsTests().record();r["rpm1"]=801;r["correction_rpm"]=3
        for module,schema,other in [(pd,"TM7",pi),(pi,"TM8",pd)]:
            wire=frame(schema,[r[k] for k in names])
            q=module.parse_telemetry(wire)
            self.assertEqual(q["rpm1"],80.1)
            self.assertEqual(q["correction_rpm"],.3)
            self.assertIsNone(other.parse_telemetry(wire))
            self.assertIsNone(module.parse_telemetry(frame("TM6",[r[k] for k in names])))

    def test_only_requested_210_route_can_open_motion_client(self):
        for module in (pd,pi):
            for extra in ([],["--mode","3","--plan","forward210"],
                          ["--mode","3","--plan","default","--turn-trial"]):
                with patch.object(sys,"argv",["test","--action","run-once"]+extra),patch.object(module,"WindowsSerial") as serial,contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):module.main()
                    serial.assert_not_called()

    def test_xd_history_alone_cannot_start_or_renew_motion(self):
        value=(100,90,0,10,10,500,24000,0x33,1,2|(6<<3),3)
        payload=struct.pack("<IIIhhHHBBBB",*value).hex().encode()
        prefix=b"XD1,7,0,1,1,0,"+payload+b","
        wire=prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()
        class Serial:
            def __init__(self):self.sent=[];self.wire=wire
            def read(self):
                result=self.wire;self.wire=b"";return result
            def write(self,data):self.sent.append(data)
            def close(self):pass
        for module in (pd,pi):
            port=Serial();clock=[0.0]
            def monotonic():clock[0]+=.05;return clock[0]
            with tempfile.TemporaryDirectory() as out,patch.object(sys,"argv",["test","--action","run-once","--mode","3","--plan","default","--route-trial","--seconds","1","--output",out]),patch.object(module,"WindowsSerial",return_value=port),patch.object(module.time,"monotonic",side_effect=monotonic),contextlib.redirect_stdout(io.StringIO()):
                module.main()
                self.assertFalse(any(b",ARM," in x or b",START," in x or b",KEEP," in x or b",PLAN" in x for x in port.sent))
                self.assertTrue(list(Path(out).glob("*.summary.json")))


if __name__=="__main__":unittest.main(verbosity=2)
