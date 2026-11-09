"""实时像素状态门：新鲜、同方法连续窗口；不允许毫米/动作输出。"""
from collections import deque
from copy import deepcopy
import math

import numpy as np


class LivePixelGate:
    def __init__(self, window=3,jitter_px=1.5,maximum_age_ms=150):
        if window<2 or not math.isfinite(jitter_px) or jitter_px<=0 or not math.isfinite(maximum_age_ms) or maximum_age_ms<=0:
            raise ValueError('Invalid pixel gate configuration')
        self.window=window;self.jitter=jitter_px;self.age=maximum_age_ms
        self.history=deque(maxlen=window)
        self.last_stamp=None;self.last_method=None
        self.status=self.empty('WAITING','NO_FRAME')

    def empty(self,state,reason,stamp=None):
        return dict(state=state,reason=reason,host_read_time=stamp,result_host_age_ms=None,
                    candidate_centers_px=[],stable_centers_px=[],pixel_ready=False,
                    stationary_verified=False,metric_valid=False,motion_allowed=False,serial='DISABLED',units='px')

    def reset_window(self):
        self.history.clear();self.last_method=None

    def observe(self,result,shape,stamp,now):
        increasing=isinstance(stamp,(float,int)) and not isinstance(stamp,bool) and math.isfinite(stamp) and (self.last_stamp is None or stamp>self.last_stamp)
        age=(now-stamp)*1000 if increasing else None
        if not increasing or not math.isfinite(age) or not 0<=age<=self.age:
            self.reset_window()
            self.status=self.empty('STALE','READ_TIMESTAMP_OR_RESULT_AGE',stamp if increasing else None)
            if increasing:self.last_stamp=stamp
            return self.current(now)
        if self.last_stamp is not None and (stamp-self.last_stamp)*1000>self.age:
            self.reset_window()
        self.last_stamp=stamp
        points=result.get('centers_px',[])
        valid=bool(result.get('accepted')) and len(points)==3
        if valid:
            values=np.asarray(points,dtype=float)
            valid=values.shape==(3,2) and np.isfinite(values).all() and np.all(values>=0) and np.all(values[:,0]<shape[1]) and np.all(values[:,1]<shape[0])
        if not valid:
            self.reset_window()
            self.status=self.empty('LOST',result.get('reason','NO_THREE_CENTERS'),stamp)
            return self.current(now)
        method=result['method']
        if method!=self.last_method:
            self.reset_window()
        self.last_method=method
        self.history.append(values)
        stack=np.array(self.history)
        median=np.median(stack,axis=0)
        jitter=float(np.max(np.linalg.norm(stack-median,axis=2)))
        stable=len(self.history)==self.window and jitter<=self.jitter
        self.status=self.empty('WINDOW_STABLE' if stable else 'CANDIDATE',
                               'FRESH_SHORT_WINDOW' if stable else 'WINDOW_OR_JITTER',stamp)
        self.status.update(candidate_centers_px=values.tolist(),stable_centers_px=median.tolist() if stable else [],
                           pixel_ready=stable,method=method,window_frames=len(self.history),jitter_px=jitter)
        return self.current(now)

    def current(self,now):
        stamp=self.status['host_read_time']
        if stamp is not None:
            age=(now-stamp)*1000
            if not math.isfinite(age) or not 0<=age<=self.age:
                self.reset_window()
                self.status=self.empty('STALE','DISPLAY_OR_RESULT_AGE',stamp)
            self.status['result_host_age_ms']=age if math.isfinite(age) else None
        return deepcopy(self.status)

    def fault(self,reason):
        self.reset_window();self.status=self.empty('FAULT',reason)
