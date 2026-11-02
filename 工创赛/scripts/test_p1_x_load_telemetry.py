import binascii
import struct
import unittest
from p1_x_load_telemetry import parse_load_telemetry, parse_integral_telemetry, LoadAccumulator, analyze_load


def packet(records, first=0, total=None, dropped=0):
    total = len(records) if total is None else total
    wire = b"".join(struct.pack("<IIIhhHHBBBB", *r) for r in records)
    prefix = f"XD1,7,{first},{len(records)},{total},{dropped},".encode()+wire.hex().encode()+b","
    return prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}".encode()


def sample(tick=100, address=2, target=-200, actual=-180, current=2800):
    return (tick, 90, 1234, target, actual, current, 24000, 0x30|address, 1, 2|(6<<3)|1, 3)


class LoadTelemetryTests(unittest.TestCase):
    def test_integral_metadata_units_and_crc(self):
        prefix=b"IG1,7,30000,1,30,300,123,-55,2000,1500,"
        wire=prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}".encode()
        p=parse_integral_telemetry(wire)
        self.assertEqual((p["ki"],p["peak_integral_rpm"],p["last_integral_rpm"]),(.03,.123,-.055))
        with self.assertRaises(ValueError):parse_integral_telemetry(wire[:-1]+b"Z")

    def test_units_direction_faults_and_crc(self):
        r = parse_load_telemetry(packet([sample()]))["records"][0]
        self.assertEqual((r["target_rpm"],r["actual_rpm"],r["phase_current_ma"]),(-20,-18,2800))
        self.assertEqual(r["position_deg"],-123.4)
        self.assertTrue(r["encoder_ready"] and r["calibration_ready"])
        self.assertEqual(r["target_ack_age_ms"],10)
        bad = bytearray(packet([sample()])); bad[-1] ^= 1
        with self.assertRaises(ValueError): parse_load_telemetry(bytes(bad))

    def test_out_of_order_dedup_and_completeness(self):
        a=LoadAccumulator(); a.add(parse_load_telemetry(packet([sample(200)],1,2)))
        self.assertFalse(a.complete)
        p=parse_load_telemetry(packet([sample()],0,2));a.add(p);a.add(p)
        self.assertTrue(a.complete)
        with self.assertRaises(ValueError):a.add(parse_load_telemetry(packet([sample(current=2)],0,2)))

    def test_overflow_never_claims_valid_analysis(self):
        a=LoadAccumulator();a.add(parse_load_telemetry(packet([sample()],dropped=1)))
        self.assertFalse(a.complete)
        with self.assertRaises(ValueError):analyze_load(a,[])

    def test_changed_session_and_bounds_rejected(self):
        a=LoadAccumulator();a.add(parse_load_telemetry(packet([sample()])))
        p=parse_load_telemetry(packet([sample()]));p["session"]=8
        with self.assertRaises(ValueError):a.add(p)
        bad=list(sample());bad[7]=0x35
        with self.assertRaises(ValueError):parse_load_telemetry(packet([tuple(bad)]))

    def test_wrap_and_logical_right_wheel_shortfall(self):
        a=LoadAccumulator()
        for i,t in enumerate([100,200,300]):
            a.add(parse_load_telemetry(packet([sample(t)],i,3)))
        live=[dict(tick_ms=50,distance_mode=1,backend_active=1,source_valid=1,distance_base_rpm=20)]
        q=analyze_load(a,live)["phase_statistics"][0]
        self.assertEqual(q["mean_shortfall_rpm"],2)
        self.assertEqual(q["near_current_limit_fraction"],1)
        r=list(sample());r[0]=10;r[1]=0xFFFFFFF0
        self.assertEqual(parse_load_telemetry(packet([tuple(r)]))["records"][0]["target_ack_age_ms"],26)

    def test_missing_target_and_clipped_feedback_excluded(self):
        a=LoadAccumulator();r=list(sample());r[9]=0
        q=list(sample(200));q[9]|=4
        a.add(parse_load_telemetry(packet([tuple(r),tuple(q)])))
        result=analyze_load(a,[])
        self.assertEqual(result["phase_statistics"],[])
        self.assertEqual(result["excluded"]["no_confirmed_target"],1)
        self.assertEqual(result["excluded"]["clipped"],1)


if __name__=="__main__":unittest.main(verbosity=2)
