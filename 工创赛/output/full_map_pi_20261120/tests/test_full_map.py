import json
import importlib.util
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "stage3_candidate")]
from mcu_cycle_protocol import frame
from protocol import parse_status
from vision_station import Station, StationGate


class TestGate(unittest.TestCase):
    def test_envelope_and_frame(self):
        gate = StationGate()
        wire = frame("VS1", (4, 7, 1, 1, 3, 0, 0))
        station = Station.parse(wire[:-1])
        with self.assertRaises(ValueError):
            Station.parse(wire.replace(b"4,7", b"5,7")[:-1])
        self.assertTrue(gate.announce(station, 10))
        self.assertFalse(gate.accept_status(False, 4, 3, 2, 10.002))
        self.assertFalse(gate.announce(station, 10.003))
        self.assertFalse(gate.accept_status(True, 4, 2, 2, 10.004))
        gate.announce(station, 10.005)
        self.assertTrue(gate.accept_status(True, 4, 3, 2, 10.006))
        meta = {"generation": 1, "sequence": 2, "completed": 10.01}
        self.assertIsNone(gate.acquisition(meta, 10.02))
        self.assertIsNone(gate.acquisition(meta, 10.03))
        meta.update(sequence=4, completed=10.04)
        self.assertEqual(gate.acquisition(meta, 10.05), 10.01)
        meta.update(sequence=5, completed=10.06, generation=2)
        self.assertIsNone(gate.acquisition(meta, 10.07))
        self.assertFalse(gate.active(10.3))
        self.assertTrue(gate.announce(Station(4, 0, 0, 0, 0, 0, 0), 10.4))
        self.assertFalse(gate.announce(station, 10.41))
        self.assertFalse(gate.accept_status(True, 4, 3, 2, 10.42))

    def test_real_hsv_save_reload_independent_blue_masks(self):
        tool = ROOT.parents[1] / "vision_app" / "code" / "颜色标定.py"
        spec = importlib.util.spec_from_file_location("hsv_calibrator_fixture", tool)
        calibrator = importlib.util.module_from_spec(spec)

        def in_range(image, low, high):
            return np.all((image >= low) & (image <= high), axis=-1).astype(np.uint8) * 255

        cv2 = stub("cv2", COLOR_BGR2HSV=0, cvtColor=lambda image, _: image,
                   bitwise_or=np.bitwise_or, inRange=in_range)
        camera = stub("project_camera", DEFAULT_DEVICE="fixture", open_project_camera=None,
                      calibration_maps=None)
        with patch.dict(sys.modules, {"cv2": cv2, "project_camera": camera,
                                     spec.name: calibrator}):
            spec.loader.exec_module(calibrator)
            config = calibrator.build_default_color_config("fixture", "fixture-camera.yaml")
            for color in config["colors"]:
                if color["color_id"] == 3:
                    color["ranges"] = calibrator.bounds_to_ranges((117, 121, 160, 255, 80, 230))
                if color["color_id"] == 6:
                    color["ranges"] = calibrator.bounds_to_ranges((84, 88, 130, 255, 100, 255))
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "hsv.json"
                calibrator.save_color_config(path, config)
                loaded = calibrator.load_color_config(path)
            pixels = np.array([[[119, 180, 170], [86, 180, 170], [110, 180, 170]]],
                              dtype=np.uint8)
            masks = calibrator.make_all_color_masks(pixels, loaded)
            self.assertEqual(set(masks), set(range(1, 7)))
            np.testing.assert_array_equal(masks[3], [[255, 0, 0]])
            np.testing.assert_array_equal(masks[6], [[0, 255, 0]])


def stub(name, **members):
    module = ModuleType(name)
    vars(module).update(members)
    return module


class TestEntry(unittest.TestCase):
    def test_one_serial_with_actual_entry_and_capture(self):
        import source_fast_cycle_view_20261115 as entry
        clock = [20.0]
        camera = SimpleNamespace(_worker=SimpleNamespace(pid=1), _sequence=0,
                                 last_read_completed_time=None)
        packets = []
        stats = dict(open=0, close=0, read=0, detection=0, storage=0, camera=0)
        intervals = []
        current_color = []
        hsv_seen = []
        data = []
        for request, batch, mode, pose, ring, color in [
                (0, 0, 0, 0, 0, 0), (1, 1, 1, 3, 0, 0),
                (2, 1, 2, 4, 0, 0), (2, 1, 2, 4, 1, 6), (3, 1, 3, 5, 1, 6),
                (3, 0, 0, 0, 0, 0), (4, 2, 1, 6, 0, 0),
                (5, 2, 2, 7, 0, 0), (5, 2, 2, 7, 2, 3), (6, 2, 4, 8, 2, 3),
                (6, 0, 0, 0, 0, 0)]:
            begin = len(data)
            vs = frame("VS1", (9, request, batch, mode, pose, ring, color))
            if mode in (0, 1):
                message = frame("SC2", (9, 2, pose, 1, 0, 1, 5, 6, 0, 0, 0, 0, 0))
            else:
                message = frame("S31", (9, 5 if ring else 2, pose, 0, 1, 5, 6, 0, 1))
            data.extend([(vs + message, mode, color)] * 8)
            intervals.append((begin, len(data), mode, ring))
        no_op = lambda *a, **kw: None

        class Serial:
            def open(self):
                stats["open"] += 1

            def close(self):
                stats["close"] += 1

            @property
            def in_waiting(self):
                return len(data[camera._sequence - 1][0])

            def read(self, amount):
                stats["read"] += 1
                return data[camera._sequence - 1][0]

            def write(self, body):
                packets.append((camera._sequence - 1, body))
                return len(body)

        class Tracking:
            reset = pause = register_rim = no_op
            diagnostic = lambda self: {}

            def __init__(self, *a, **kw):
                pass

        def body(*a):
            stats["detection"] += 1
            return dict(right_forward_mm=(1, 2), error_norm_mm=0.5, center_px=(10, 10), radius_px=8)

        class Region:
            def __init__(self, config):
                self.config = config
                self.pipe = SimpleNamespace(detector=SimpleNamespace(reset=no_op))
                self.session = SimpleNamespace(status=None)
                self.last_result = SimpleNamespace(detected=True, stable=True)
                self.last_image = None

            warm = reset = no_op
            source_received = no_op

            def receive(self, wire, at):
                self.session.status = parse_status(wire+b"\n")

            def process(self, image, metadata, **kw):
                self.last_image = image
                state = self.session.status
                wire = (frame("O31", (state.run, state.pose, camera._sequence, 1, 0, 0, 0, 0, 0, 50))
                        if state.state == 2 else None)
                return dict(_wire=wire, metric_candidate_valid=wire is not None,
                            calibration_verified=False, alignment_response_ready=True), image

        class DisabledSerialLink:
            close = no_op

        def initialize(args):
            stats["camera"] += 1
            app.camera = camera
            app.serial_link = DisabledSerialLink()
            app.undistort_enabled = False

        def read_camera():
            if camera._sequence >= len(data):
                raise KeyboardInterrupt
            clock[0] += .05
            camera._sequence += 1
            camera.last_read_completed_time = clock[0] - .001
            current_color[:] = [data[camera._sequence - 1][2]]
            return np.zeros((24, 32, 3), np.uint8)

        def parse_args():
            import argparse
            parser = argparse.ArgumentParser()
            parser.add_argument("--width", type=int)
            parser.add_argument("--height", type=int)
            for key in ("camera-processing-config", "turntable-config", "turntable-material-seed-config",
                        "material-pickup-config", "material-color-config"):
                parser.add_argument("--" + key, type=Path, default=ROOT/"body-profile.json")
            args = parser.parse_args()
            assert (args.width, args.height) == (1920, 1080)
            load_hsv(args.material_color_config)
            return args

        def load_hsv(path):
            hsv_seen.append(json.loads(path.read_text()))
            return hsv_seen[-1]

        def classify(self, station, *args):
            stats["storage"] += 1
            assert self.colors is hsv_seen[0]
            return current_color[0] == station.color

        app = stub("main", camera=None, __file__=str(ROOT/"test-app"),
                   parse_args=parse_args, initialize=initialize, read_camera=read_camera, close_camera=no_op)
        modules = dict(
            cv2=stub("cv2", circle=no_op, drawMarker=no_op, putText=no_op, imwrite=no_op,
                     MARKER_CROSS=0, FONT_HERSHEY_SIMPLEX=0,
                     imencode=lambda *a: (True, np.zeros(1, np.uint8))),
            serial=stub("serial", Serial=Serial), main=app,
            project_camera=stub("project_camera", DisabledSerialLink=DisabledSerialLink),
            source_body_registration=stub("source_body_registration",
                                          load_profile=lambda p: dict(reference_center_px=(10, 10)),
                                          estimate=body),
            latched_material_stream_20261115=stub("latched_material_stream_20261115", RemainingStopped=Tracking),
            continuous_turntable=stub("continuous_turntable", rim_observation_valid=lambda *a: True),
            direct_frame_trigger_20261115=stub("direct_frame_trigger_20261115", DirectFrameTrigger=Tracking),
            each_observation_rotation=stub("each_observation_rotation", EachObservationRotation=Tracking),
            source_cycle_dashboard_20261115=stub("source_cycle_dashboard_20261115", compose=lambda a, *x, **y: a),
            adaptive_source_rim_20261115=stub("adaptive_source_rim_20261115",
                                             AdaptiveSourceRim=lambda *a: SimpleNamespace(estimate=body, reset=no_op)),
            region_desktop=stub("region_desktop", RegionDesktop=Region),
            source_metric_preview=stub("source_metric_preview", preview_rim=no_op),
            **{"颜色标定": stub("颜色标定", load_color_config=load_hsv, make_all_color_masks=no_op)})
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            hsv = folder / "HSV.json"
            hsv.write_text('{"blue": [99], "light_blue": [81]}')
            storage = folder / "storage.json"
            ring = dict(offset_px=[0, 0], radius_px=3, empty_color_max=.1,
                        occupied_color_min=.6, empty_background_min=.7,
                        empty_hsv_low=[0, 0, 220], empty_hsv_high=[179, 20, 255],
                        top_height_separates_base=True)
            storage.write_text(json.dumps(dict(modes={
                str(mode): dict(verified=True, rings=[ring]*3) for mode in (2, 3, 4)})))
            session = folder / "run"
            args = ["entry", "--full-map", "--headless",
                    "--camera-size", "1920", "1080",
                    "--body-profile", str(ROOT/"body-profile.json"), "--session-dir", str(session),
                    "--material-pickup-config", str(ROOT/"orbit-config.json"),
                    "--material-color-config", str(hsv), "--storage-config", str(ROOT/"storage-align.json"),
                    "--storage-observation-config", str(storage), "--orbit-config", str(ROOT/"orbit-config.json"),
                    "--rim-detector-config", str(ROOT/"rim-detector.json")]
            with patch.dict(sys.modules, modules), patch.object(sys, "argv", args), \
                    patch.object(time, "monotonic", lambda: clock[0]):
                from storage_observation import StorageObservation
                with patch.object(StorageObservation, "classify", classify):
                    self.assertEqual(entry.main(), 0)
            records = [json.loads(line) for line in (session/"observations.jsonl").read_text().splitlines()]
            self.assertEqual(len(records), len(data))
            for record, item in zip(records, data):
                if item[1]==0:
                    self.assertEqual(record["region"], "transit")
                    self.assertFalse(record["recognition_enabled"])
        self.assertEqual((stats["open"], stats["close"], stats["camera"]), (1, 1, 1))
        self.assertEqual(stats["read"], len(data))
        self.assertGreater(stats["detection"], 0)
        self.assertGreater(stats["storage"], 0)
        expected = {0: b"", 1: b"OC1", 2: b"O31", 3: b"VO1", 4: b"VO1"}
        for begin, end, mode, ring in intervals:
            wires = [(i, body) for i, body in packets if begin <= i < end]
            if mode == 0:
                self.assertFalse(wires)
                continue
            self.assertTrue(wires)
            for i, body in wires:
                self.assertGreaterEqual(i, begin + 2)
                self.assertEqual(body.split(b",")[0], b"VO1" if ring else expected[mode])
        self.assertEqual(hsv_seen, [{"blue": [99], "light_blue": [81]}] * 2)


if __name__ == "__main__":
    unittest.main()
