"""CYQM original R detector + measured vehicle response; no hardware ownership."""
from pathlib import Path
import hashlib
import json
import math
import numpy as np
from stage3 import Pipeline, RingStationDetector, angle_error_in_plane, matrix, cv2


def vehicle_edge_angle(frame, roi):
    """Fit the observed dark chassis top edge in its calibrated image region."""
    x0,y0,x1,y1=map(int,roi)
    if not (0<=x0<x1<=frame.shape[1] and 0<=y0<y1<=frame.shape[0]):
        raise ValueError('Body-edge region outside image')
    gray=cv2.cvtColor(frame[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY).astype(float)
    gradient=np.diff(gray,axis=0)
    columns=np.arange(gradient.shape[1])
    peaks=np.argmin(gradient,axis=0)
    xs=columns+x0
    ys=peaks+y0+.5
    selected=-gradient[peaks,columns]>8
    if np.count_nonzero(selected)<2:raise ValueError('Insufficient observed edge columns')
    slope,intercept=np.polyfit(xs[selected],ys[selected],1)
    selected=selected & (np.abs(ys-(slope*xs+intercept))<=2)
    if np.count_nonzero(selected)<2:raise ValueError('Insufficient line support')
    slope,intercept=np.polyfit(xs[selected],ys[selected],1)
    angle=math.degrees(math.atan(slope))
    return dict(angle_deg=angle,columns=int(selected.sum()),
                rms_px=float(np.sqrt(np.mean((ys[selected]-slope*xs[selected]-intercept)**2))),
                segment=((x0,round(slope*x0+intercept)),(x1-1,round(slope*(x1-1)+intercept))))


class RingRowPipeline:
    def __init__(self, config, size):
        self.base=Pipeline(config,size)
        self.profile=self.base.profile
        self.size=size
        self.path=Path(config)
        self.digest=hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.detector=RingStationDetector(self.base.ring)
        c=self.profile['ring_row_alignment']
        self.response=matrix(c['response_pixel_per_right_forward_ccw'],(3,3))
        self.planar=matrix(self.response[:2,:2],(2,2))
        self.affine=matrix(c['nominal_image_to_station_affine'],(2,2))
        self.target=np.asarray(c['target_center_px'],dtype=float)
        if self.target.shape!=(2,) or not np.isfinite(self.target).all():
            raise ValueError('Need finite target center')
        self.target_yaw=float(c['target_row_yaw_deg'])
        if not math.isfinite(self.target_yaw) or self.response[2,2]==0:
            raise ValueError('Need finite target yaw and measured yaw response')
        self.evidence=c
        self.edge=None

    def verified(self):
        # Engineering command response is separate from physical calibration.
        return False

    def alignment_ready(self):
        c=self.evidence
        return (hashlib.sha256(self.path.read_bytes()).hexdigest()==self.digest
                and c.get('enabled') is True and c.get('target_confirmed_by_user') is True
                and c.get('response_measured_on_vehicle') is True
                and self.profile['camera']['width']==self.size[0]
                and self.profile['camera']['height']==self.size[1])

    def detect(self, raw):
        corrected=self.base.corrected(raw)
        self.edge=None
        if self.evidence.get('yaw_reference')=='vehicle_edge':
            try:self.edge=vehicle_edge_angle(corrected,self.evidence['vehicle_edge_roi_xyxy'])
            except (ValueError,np.linalg.LinAlgError):
                self.detector.reset()
                return corrected,None
            self.target_yaw=self.edge['angle_deg']
        result=self.detector.detect(corrected)
        if self.edge is not None:cv2.line(corrected,*self.edge['segment'],(255,180,0),2)
        return corrected,result

    def measured_motion(self, result):
        # Same R outputs as upstream main.send_ring_if_ready: center[1], row yaw.
        pixel=np.asarray(result.centers_px[1])-self.target
        row=(float(result.station_yaw_deg)-self.target_yaw+90)%180-90
        yaw=angle_error_in_plane(self.affine,float(result.station_yaw_deg),self.target_yaw)
        metric=self.affine@pixel
        if abs(yaw)>self.base.ring.maximum_angle_error_deg:
            motion=(0.,0.,-row/self.response[2,2])
        else:
            planar=-np.linalg.solve(self.planar,pixel)
            motion=(float(planar[0]),float(planar[1]),0.)
        return motion,(float(np.linalg.norm(metric)),float(yaw))
