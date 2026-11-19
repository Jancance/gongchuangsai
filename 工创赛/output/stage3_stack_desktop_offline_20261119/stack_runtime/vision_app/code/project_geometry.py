"""未标定时只显示实测像素候选，阻止原参考矩阵形成有效毫米纠偏。"""
from pathlib import Path
import json
import cv2

PROFILE=Path(__file__).resolve().parent/'config/project_profile.json'
def metric_verified(kind):
    p=json.loads(PROFILE.read_text(encoding='utf-8'))
    return p.get(kind+'_geometry_verified') is True and p.get('geometry_verified') is True

def draw_pixels(frame,result,kind):
    centers=getattr(result,'centers_px',())
    if not centers and getattr(result,'middle_center_px',None) is not None:
        centers=[result.middle_center_px]
    for i,center in enumerate(centers):
        x,y=map(lambda v:int(round(v)),center)
        cv2.drawMarker(frame,(x,y),(0,255,0),cv2.MARKER_CROSS,20,2)
        cv2.putText(frame,f'circle {i+1}: ({x},{y})px',(x+12,y),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
    cv2.putText(frame,kind.upper()+' PIXEL PREVIEW - METRIC CALIBRATION REQUIRED',
                (10,60),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
    return frame
