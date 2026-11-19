"""One correction at a time; each step consumes a new observation revision."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Correction:
    kind: str
    direction: int | None
    amount: float
    unit: str


class AlignmentPlanner:
    def __init__(self, *, step_cap_mm=20.0, minimum_step_mm=1.0,
                 minimum_turn_deg=5.0, maximum_turn_deg=10.0,
                 position_tolerance_mm=2.0, yaw_tolerance_deg=1.0):
        if not (0 < minimum_step_mm <= step_cap_mm and 0 < minimum_turn_deg <= maximum_turn_deg):
            raise ValueError('执行器能力范围无效')
        self.step_cap_mm=step_cap_mm;self.minimum_step_mm=minimum_step_mm
        self.minimum_turn_deg=minimum_turn_deg;self.maximum_turn_deg=maximum_turn_deg
        self.position_tolerance_mm=position_tolerance_mm;self.yaw_tolerance_deg=yaw_tolerance_deg
        self.last_revision=None;self.reason=''

    @classmethod
    def from_parameters(cls, parameters, capabilities):
        t=parameters.tuning
        return cls(step_cap_mm=min(t['recommended_body_step_mm'],capabilities['max_step_mm']),
                   minimum_step_mm=capabilities['min_step_mm'],
                   minimum_turn_deg=capabilities['min_turn_deg'],maximum_turn_deg=capabilities['max_turn_deg'],
                   position_tolerance_mm=t['alignment_position_tolerance_mm'],
                   yaw_tolerance_deg=t['alignment_yaw_tolerance_deg'])

    def next(self, revision, dx_mm, dy_mm, yaw_deg):
        if not all(math.isfinite(x) for x in (dx_mm, dy_mm, yaw_deg)):
            self.reason='等待有效纠偏观测';return None
        if revision == self.last_revision:
            self.reason='动作后等待新图，不按旧残差连续发车';return None
        if abs(yaw_deg)>self.yaw_tolerance_deg:
            angle=min(self.maximum_turn_deg,max(self.minimum_turn_deg,abs(yaw_deg)))
            # Do not oscillate forever where the backend's coarse minimum
            # cannot improve the current residual. Expose the limitation.
            if angle-abs(yaw_deg)>self.yaw_tolerance_deg:
                self.reason='当前转角能力过粗，等待更细转向接口或调整现场容差';return None
            self.last_revision=revision;self.reason='先转向，再重新观测平移'
            return Correction('turn',None,math.copysign(angle,yaw_deg),'deg')
        if max(abs(dx_mm),abs(dy_mm))<=self.position_tolerance_mm:
            self.reason='当前观测在设定容差内';return None
        axis=0 if abs(dx_mm)>=abs(dy_mm) else 1;value=dx_mm if axis==0 else dy_mm
        amount=min(abs(value),self.step_cap_mm)
        if amount<self.minimum_step_mm:
            self.reason='当前执行器最小步长不足，不伪称已对准';return None
        direction=(1 if value<0 else 2) if axis==0 else (3 if value<0 else 0)
        self.last_revision=revision;self.reason='只提交一个主轴小步，完成后重观测'
        return Correction('axis',direction,amount,'mm')
