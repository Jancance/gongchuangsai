"""Learn one real orbit at the fixed view, then report future crossing times.

No actuator API. Constant angular speed and the calibrated view orientation are
explicit model assumptions. A learned model is cleared for every new MCU run.
"""
import math
import numpy as np

def rim_observation_valid(body,maximum_rms_px=3.0):
    """User-approved gate: wait for a valid image; never issues a STOP/fault."""
    value=float(body.get('fit_rms_px',math.inf))
    return math.isfinite(value) and value<=maximum_rms_px


class ContinuousTurntable:
    def __init__(self,target_angles_rad,lead_ms_by_region,maximum_frame_interval_s=.25):
        if len(target_angles_rad)!=3 or len(lead_ms_by_region)!=3:raise ValueError('three calibrated regions')
        self.targets=np.asarray(target_angles_rad,dtype=float)
        self.leads=np.asarray(lead_ms_by_region,dtype=float)/1000
        self.maximum_frame_interval_s=maximum_frame_interval_s
        if not np.isfinite(self.targets).all() or not np.isfinite(self.leads).all() or (self.leads<=0).any():
            raise ValueError('finite angles and positive measured timing required')
        self.reset()

    def reset(self):
        self.rows=[];self.geometry=None;self.omega=None;self.learned=False;self.recent=[]

    def pause(self):
        # The geometry belongs to the same fixed180/65 view. Moving-camera
        # samples never train velocity, and temporal tracks restart on return.
        self.recent=[]
        if not self.learned:self.rows=[];self.geometry=None;self.omega=None

    @staticmethod
    def _ellipse(points):
        if len(points)<6:return None
        origin=points.mean(axis=0);scale=float(np.std(points,axis=0).max())
        if scale==0 or not math.isfinite(scale):return None
        q=(points-origin)/scale;x,y=q.T
        design=np.c_[x*x,x*y,y*y,x,y]
        coefficients,_,rank,_=np.linalg.lstsq(design,np.ones(len(q)),rcond=None)
        if rank<5:return None
        a,b,c,d,e=coefficients;quadratic=np.array([[a,b/2],[b/2,c]])
        eigenvalues,vectors=np.linalg.eigh(quadratic)
        if (eigenvalues<=0).any():return None
        center=-.5*np.linalg.solve(quadratic,[d,e]);k=1+center@quadratic@center
        if k<=0:return None
        whitening=(vectors@np.diag(np.sqrt(eigenvalues/k))@vectors.T)/scale
        center=origin+scale*center
        return center,whitening

    @staticmethod
    def _phase(point,geometry):
        center,whitening=geometry
        q=whitening@(np.asarray(point)-center)
        return math.atan2(q[1],q[0])

    def update(self,regions,observed_s,now_s,target_color,remaining_colors):
        positions={}
        for item in regions:
            color=item['color_id']
            if color in positions or color not in remaining_colors:return None
            try:point=(float(item['center_x_px']),float(item['center_y_px']))
            except (TypeError,ValueError):return None
            if not all(map(math.isfinite,point)):return None
            positions[color]=point
        if not positions or not math.isfinite(observed_s) or not math.isfinite(now_s) or now_s<observed_s:
            self.pause();return None
        if self.recent and observed_s<=self.recent[-1][0]:return None
        if self.recent and observed_s-self.recent[-1][0]>self.maximum_frame_interval_s:self.pause()
        self.recent.append((observed_s,positions));self.recent=self.recent[-2:]
        if not self.learned:
            self.rows.extend((observed_s,color,*point) for color,point in positions.items())
            array=np.asarray(self.rows,dtype=float)
            geometry=self._ellipse(array[:,2:4])
            if geometry is None:return None
            colors=sorted(set(array[:,1].astype(int)));matrix=[];values=[];full_turn=False
            for column,color in enumerate(colors):
                selected=array[array[:,1]==color]
                phase=np.unwrap([self._phase(point,geometry) for point in selected[:,2:4]])
                time=selected[:,0]-array[0,0];onehot=np.zeros((len(time),len(colors)));onehot[:,column]=1
                matrix.extend(np.c_[time,onehot]);values.extend(phase)
                full_turn=full_turn or abs(phase[-1]-phase[0])>=2*math.pi
            coefficients,_,rank,_=np.linalg.lstsq(matrix,values,rcond=None)
            if rank<len(colors)+1 or not full_turn:return None
            omega=float(coefficients[0])
            if not math.isfinite(omega) or omega==0:return None
            self.geometry=geometry;self.omega=omega;self.learned=True
            self.rows=[]
        if len(self.recent)<2 or any(target_color not in row[1] for row in self.recent):return None
        phase=[]
        for stamp,points in self.recent:
            angle=self._phase(points[target_color],self.geometry)+self.omega*(now_s-stamp)
            phase.append(complex(math.cos(angle),math.sin(angle)))
        mean=sum(phase)
        if mean==0:return None
        theta=math.atan2(mean.imag,mean.real);speed=abs(self.omega);period=2*math.pi/speed
        angle=(self.targets-theta)%(2*math.pi) if self.omega>0 else (theta-self.targets)%(2*math.pi)
        waits=angle/speed
        for i in range(3):
            if waits[i]<self.leads[i]:waits[i]+=math.ceil((self.leads[i]-waits[i])/period)*period
        delays=(waits-self.leads)*1000
        if not np.isfinite(delays).all() or (delays>0x7fffffff).any():return None
        return dict(target_color=target_color,start_delay_ms=delays.tolist(),computed_s=now_s,
            period_ms=period*1000,angular_speed_rad_s=self.omega,learned_one_turn=True,
            center_px=self.geometry[0].tolist(),actual_grasp_verified=False)

    @staticmethod
    def delays_at_send(prediction,now_s,wire_bytes=128,baud=115200):
        # Relative start times refer to the send instant, with wire time removed.
        elapsed=(now_s-prediction['computed_s'])*1000+wire_bytes*10*1000/baud
        period=prediction['period_ms']
        return [int(round((value-elapsed)%period)) for value in prediction['start_delay_ms']]
