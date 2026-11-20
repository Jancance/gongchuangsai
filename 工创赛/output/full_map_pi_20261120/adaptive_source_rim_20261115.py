"""Use the existing whole-frame rim detector and its latest unsmoothed fit.

No colour/material seeder, no added temporal filter, no actuator API. Pixel
coordinates are converted to the MATLAB1-based calibration before subtraction.
"""
from pathlib import Path
import hashlib
import numpy as np
from turntable_detector import TurntableDetector,load_turntable_detector_config

class AdaptiveSourceRim:
    def __init__(self,profile,config_path):
        self.profile=profile;self.config_path=Path(config_path)
        self.config_sha256=hashlib.sha256(self.config_path.read_bytes()).hexdigest()
        self.detector=TurntableDetector(load_turntable_detector_config(self.config_path))

    def reset(self):
        self.detector.reset()

    def estimate(self,frame):
        if frame is None or frame.shape!=(1080,1920,3):raise ValueError('盘缘输入必须是当前1920x1080原图')
        result=self.detector.detect(frame)
        if not result.detected or not self.detector.samples:raise ValueError(result.detail)
        # The existing detector keeps the CURRENT raw CircleFit in samples[-1].
        # Its public median coordinates/stable flag are deliberately not used.
        fit=self.detector.samples[-1]
        center=np.asarray([fit.center_x_px+1,fit.center_y_px+1],dtype=float)
        delta=center-np.asarray(self.profile['reference_center_px'],dtype=float)
        correction=np.linalg.solve(np.asarray(self.profile['response_px_per_command_mm'],dtype=float),-delta)
        error=delta*float(self.profile['scale_mm_per_px']);norm=float(np.linalg.norm(error))
        return dict(center_px=center.tolist(),radius_px=fit.radius_px,fit_rms_px=fit.fit_rms_px,
            delta_px=delta.tolist(),error_mm=error.tolist(),error_norm_mm=norm,
            right_forward_mm=correction.tolist(),within_tolerance=norm<=self.profile['position_tolerance_mm'],
            motion_allowed=False,source='existing_global_and_local_rim',profile_id=self.profile['profile_id'],
            detector_config=str(self.config_path),detector_config_sha256=self.config_sha256,
            tracking_mode=result.tracking_mode,raw_current_frame=True,temporal_filter_used=False)
