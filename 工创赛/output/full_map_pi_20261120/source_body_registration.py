"""Fixed turntable-rim translation at the saved180deg/65mm/top camera pose.

This is a position observation, never an actuator API. It does not use moving
materials as a vehicle reference. Sampling matches the preceding local MATLAB.
"""
from pathlib import Path
import json
import math
import numpy as np


def load_profile(path):
    value=json.loads(Path(path).read_text(encoding='utf-8'))
    if value.get('image_size')!=[1920,1080] or value.get('camera_pose')!={
            'yaw_deg':180,'extension_mm':65,'lift_depth_pulses':600}:
        raise ValueError('盘缘配置不对应当前已标定观察姿态')
    return value


def _sample(image,x,y):
    # MATLAB pixel coordinates are1-based; the frame array is0-based.
    x=x-1;y=y-1
    valid=(x>=0)&(y>=0)&(x<image.shape[1]-1)&(y<image.shape[0]-1)
    result=np.full(x.shape,np.nan,dtype=float)
    xx=x[valid];yy=y[valid];ix=np.floor(xx).astype(int);iy=np.floor(yy).astype(int)
    dx=xx-ix;dy=yy-iy
    result[valid]=(image[iy,ix]*(1-dx)*(1-dy)+image[iy,ix+1]*dx*(1-dy)+
                   image[iy+1,ix]*(1-dx)*dy+image[iy+1,ix+1]*dx*dy)
    return result


def estimate(frame_bgr,profile):
    if frame_bgr is None or frame_bgr.shape!=(1080,1920,3):
        raise ValueError('盘缘图像须为当前1920x1080原图')
    image=frame_bgr.astype(float)
    gray=.299*image[:,:,2]+.587*image[:,:,1]+.114*image[:,:,0]
    origin=np.asarray(profile['sampling_origin_px'],dtype=float)
    radius=float(profile['sampling_radius_px']);rr=np.arange(radius-100,radius+100.01,.5)
    angles=np.deg2rad(np.r_[np.arange(0,71,2),np.arange(110,181,2)])
    points=[]
    for angle in angles:
        x=origin[0]+rr*np.cos(angle);y=origin[1]+rr*np.sin(angle)
        z=_sample(gray,x,y);score=(z[:-16]+z[16:])/2-z[8:-8]
        score[~np.isfinite(score)]=-np.inf;index=int(np.argmax(score))
        if math.isfinite(float(score[index])):
            points.append((x[index+8],y[index+8]))
    if len(points)<=20:
        raise ValueError('固定盘缘有效采样不足；没有生成纠偏指令')
    points=np.asarray(points);keep=np.ones(len(points),dtype=bool)
    for _ in range(8):
        o=points[keep].mean(axis=0);q=points[keep]-o
        v=np.linalg.lstsq(np.c_[2*q,np.ones(len(q))],(q*q).sum(axis=1),rcond=None)[0]
        center=v[:2]+o;radius2=v[2]+np.dot(v[:2],v[:2])
        if radius2<=0 or not np.isfinite(radius2):raise ValueError('盘缘几何无有效解')
        radius=math.sqrt(radius2);error=np.linalg.norm(points-center,axis=1)-radius
        median=np.median(error[keep]);mad=np.median(np.abs(error[keep]-median))
        keep=np.abs(error-median)<max(1.5,3*1.4826*mad)
        if np.count_nonzero(keep)<3:raise ValueError('盘缘几何无有效解')
    delta=center-np.asarray(profile['reference_center_px'],dtype=float)
    correction=np.linalg.solve(np.asarray(profile['response_px_per_command_mm']),-delta)
    scale=float(profile['scale_mm_per_px']);norm=float(np.linalg.norm(delta*scale))
    return dict(center_px=center.tolist(),radius_px=radius,
                fit_rms_px=float(np.sqrt(np.mean(error[keep]**2))),
                delta_px=delta.tolist(),error_mm=(delta*scale).tolist(),error_norm_mm=norm,
                right_forward_mm=correction.tolist(),
                within_tolerance=norm<=profile['position_tolerance_mm'],
                motion_allowed=False,source='fixed_disc_rim',
                profile_id=profile['profile_id'])


def next_existing_bounded_step(result,profile):
    """Reuse the existing2mm tolerance and1..20mm body-step entry."""
    if result['within_tolerance']:return None
    u=np.asarray(result['right_forward_mm'],dtype=float)
    axis=int(np.argmax(np.abs(u)));distance=max(1,min(profile['max_step_mm'],int(round(abs(u[axis])))))
    direction=(2 if u[axis]>0 else 1) if axis==0 else (0 if u[axis]>0 else 3)
    return dict(direction=direction,distance_mm=distance)
