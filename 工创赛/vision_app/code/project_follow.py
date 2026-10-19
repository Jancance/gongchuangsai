"""持续像素跟随预览；接触时延/毫米坐标未验收，ready始终False。"""
import math
import time
import cv2
from project_detector import analyze
from project_tracker import TabletopTracker

class MaterialFollower:
    def __init__(self,color_config=None,lookahead_s=.1):
        self.color_config=color_config;self.lookahead_s=lookahead_s
        self.tracker=TabletopTracker();self.history={}
    def reset(self):
        self.tracker.clear();self.history.clear()
    def detect(self,frame,timestamp=None):
        now=time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(now):self.reset();raise ValueError('时间戳必须有限')
        if timestamp is not None and not 0<=time.monotonic()-now<=.15:
            self.reset()
            return {'state':'STALE_FRAME','targets':[],'unknown_candidates':0,
                    'ready':False,'motion_allowed':False,'units':'pixels',
                    'timestamp_source':'host_read_completion','timestamp':now,
                    'retired_ids':[],'detector_regions':[]}
        # 仅作图像搜索提示；超过内部暂存期的旧框不再使用，也不输出旧目标。
        hints=[dict(t) for t in self.tracker.tracks.values() if 0<=now-t['read_time']<=.5]
        detection,_=analyze(frame,self.color_config,prior_regions=hints,draw_overlay=False)
        if time.monotonic()-now>.15:
            self.reset()
            return {'state':'STALE_RESULT','targets':[],'unknown_candidates':0,
                    'ready':False,'motion_allowed':False,'units':'pixels',
                    'timestamp_source':'host_read_completion','timestamp':now,
                    'retired_ids':[],'detector_regions':detection['regions'],
                    'circle_diagnostics':detection.get('circle_diagnostics',[])}
        tracked=self.tracker.update(detection['regions'],now)
        targets=[];history={}
        for t in tracked['observed']:
            identity=t['id'];p=t['center_px'];previous=self.history.get(identity)
            source=next((r.get('proposal_source','combined_silhouette') for r in detection['regions']
                         if r['label']==t['color']),'combined_silhouette')
            velocity=None;prediction=None
            if previous and previous[2]==source and not t.get('identity_continuity_unverified',False):
                dt=now-previous[0]
                if .02<=dt<=.25:
                    velocity=[(p[i]-previous[1][i])/dt for i in range(2)]
                    prediction=[p[i]+velocity[i]*self.lookahead_s for i in range(2)]
            history[identity]=(now,p,source)
            targets.append(dict(t,proposal_source=source,velocity_px_s=velocity,predicted_px=prediction,
                                prediction_horizon_s=self.lookahead_s,
                                identity_valid_for_prediction=not t.get('identity_continuity_unverified',False)))
        self.history=history
        return {'state':'FOLLOWING_PIXEL' if targets else 'SEARCHING',
                'targets':targets,'unknown_candidates':sum(r['label']=='UNKNOWN' for r in detection['regions']),
                'ready':False,'motion_allowed':False,'units':'pixels',
                'note':'当前为原始像素诊断，速度未补偿相机自身运动，禁止用于抓取指令',
                'timestamp_source':'host_read_completion','timestamp':now,
                'retired_ids':tracked['retired_ids'],'detector_regions':detection['regions'],
                'circle_diagnostics':detection.get('circle_diagnostics',[])}

def draw_follow(frame,result):
    for t in result['targets']:
        x,y,w,h=t['bbox_px'];cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.putText(frame,t['id']+' '+t['color'],(x,max(90,y-8)),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0),2)
        if t['predicted_px'] is not None:
            cx,cy=map(lambda v:int(round(v)),t['predicted_px'])
            if 0<=cx<frame.shape[1] and 0<=cy<frame.shape[0]:
                cv2.drawMarker(frame,(cx,cy),(0,255,255),cv2.MARKER_CROSS,18,2)
    cv2.putText(frame,'FOLLOW: PIXEL GUIDANCE ONLY / NO GRASP',(10,60),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
    return frame
