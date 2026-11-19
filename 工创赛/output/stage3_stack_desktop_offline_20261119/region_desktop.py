"""Explicit material-stack observation branch; original source/ring UI retained."""
from pathlib import Path
import json
import time
import cv2
from dataclasses import asdict
from _stack_base_region_desktop import RegionDesktop as BaseDesktop, compose_stage3
from stage3_stack_material_pipeline import StackMaterialPipeline, OfflineStackBridge

class RegionDesktop(BaseDesktop):
    def __init__(self, config):
        super().__init__(config)
        self.stack_mode = json.loads(Path(config).read_text(encoding='utf-8-sig')).get('observation_mode') == 'stack_base_materials'
        self.stack_bridge = None

    def process(self, frame, metadata, preview_only):
        if not self.stack_mode:
            return super().process(frame, metadata, preview_only)
        self.frame_id += 1
        self.snapshot_count += 1
        if self.pipe is None or self.pipe.size != tuple(frame.shape[1::-1]):
            self.pipe = StackMaterialPipeline(self.config, tuple(frame.shape[1::-1]))
            self.stack_bridge = OfflineStackBridge(self.pipe)
        self.stack_bridge.session = self.session
        status = self.session.status
        now = time.monotonic()
        completed = metadata.get('completed') if metadata else None
        eligible = self.acquired_after is not None and completed is not None and self.new_camera_frame
        if eligible:
            row = self.stack_bridge.process(frame, self.frame_id, self.acquired_after, completed, now)
        else:
            self.pipe.reset()
            row = dict(detected=False, stable=False, centers_px=(), reason='WAIT_CONTINUOUS_CAPTURE_METADATA',
                       alignment_response_ready=self.pipe.alignment_ready(), wire=None, observation_valid=False)
        wire = row.pop('wire')
        wire = wire.encode('ascii') if wire and not preview_only else None
        left = frame.copy()
        for center in row['centers_px']:
            cv2.drawMarker(left, tuple(round(v) for v in center), (0,255,0), cv2.MARKER_CROSS, 20, 2)
        active = status is not None and 0 <= now-self.session.last_status_at <= .2
        views = compose_stage3(left, frame, status, active, False, row['observation_valid'], False)
        cv2.putText(views, 'STACK BASE MATERIALS / OFFLINE CANDIDATE', (20,78), cv2.FONT_HERSHEY_SIMPLEX,.5,(0,220,255),1)
        return dict(**row, region='stage3', mcu_status=asdict(status) if status else None,
                    calibration_verified=False, metric_candidate_valid=bool(wire and row['observation_valid']),
                    capture_start_lower_bound=self.acquired_after, exposure_timestamp_verified=False,
                    motor_commands_sent=False, observation_wire=wire.decode('ascii').strip() if wire else None,
                    preview_only=preview_only, _snapshot=self.snapshot_count==2, _wire=wire), views
