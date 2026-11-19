"""Validated planar/polar maths; never treat a side/top centroid as a base."""
import math
import numpy as np


def arm_from_source_xy(point_mm, retracted_radius_mm):
    p=np.asarray(point_mm,dtype=float)
    if p.shape!=(2,) or not np.isfinite(p).all() or not math.isfinite(retracted_radius_mm) or retracted_radius_mm<=0:
        raise ValueError('需要有效轴心坐标与已测收回半径')
    return dict(yaw_deg=(180+math.degrees(math.atan2(p[1],p[0])))%360,
                extension_mm=float(np.linalg.norm(p)-retracted_radius_mm))


def arm_from_base_pixel(point_px, image_to_plane, plane_to_pivot, retracted_radius_mm, *, point_plane):
    if point_plane!='contact':
        raise ValueError('必须提供接触平面中心；顶面/侧面中心需另做高度投影')
    H=np.asarray(image_to_plane,dtype=float);T=np.asarray(plane_to_pivot,dtype=float)
    p=np.asarray([*point_px,1.],dtype=float)
    if H.shape!=(3,3) or T.shape!=(3,3) or p.shape!=(3,) or not all(np.isfinite(x).all() for x in (H,T,p)):
        raise ValueError('平面/机构变换无效')
    q=T@(H@p)
    if abs(q[2])<1e-10:raise ValueError('投影分母为零')
    return arm_from_source_xy(q[:2]/q[2],retracted_radius_mm)
