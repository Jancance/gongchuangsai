"""Region UI adapter: owns neither camera nor serial; only O31 observations."""
from pathlib import Path
import sys
import time
import json
import math
from dataclasses import asdict
import cv2
import numpy as np
from PIL import Image, ImageDraw
from source_cycle_dashboard_20261115 import font

THIRD=Path(__file__).resolve().parent/'stage3_candidate'
sys.path.insert(0,str(THIRD))
from stage3 import Pipeline, measured_motion
from protocol import Session, parse_status, decode

def resume_wire_sequence(folder,run,pose,current):
    """Continue actual archived O31 sequence for the same MCU run/pose."""
    paths=sorted(Path(folder).glob('run-*/observations.jsonl'),key=lambda p:p.stat().st_mtime,reverse=True)[:12]
    for path in paths:
        try:
            with path.open('rb') as stream:
                stream.seek(max(0,path.stat().st_size-65536));lines=stream.read().splitlines()
            for line in reversed(lines):
                try:
                    record=json.loads(line);wire=record.get('observation_wire')
                    if not wire:continue
                    fields=decode(wire.rstrip().encode('ascii')+b'\r\n')
                    if len(fields)!=11 or fields[0]!='O31' or int(fields[1])!=run or int(fields[2])!=pose:continue
                    seq=int(fields[3]);delta=(seq-current)&0xffffffff
                    if 0<seq<=0xffffffff and 0<delta<0x80000000:current=seq
                    break
                except (ValueError,TypeError,UnicodeError):continue
        except OSError:continue
    return current

LABELS={0:'等待启动',1:'机构准备中',2:'视觉纠偏中',3:'底盘微调中',4:'纠偏后机构准备',5:'抓取与放置中',6:'本轮软件动作完成',7:'等待配置或机械示教',9:'已停止',10:'人工装盘准备',11:'等待搬至第三段'}
COLORS={0:'未知',1:'红',2:'黄',3:'蓝',4:'绿',5:'黑',6:'浅蓝'}

def compose_stage3(left,right,status,active,verified,valid,engineering=False):
    canvas=np.full((400,960,3),(32,28,25),dtype=np.uint8)
    for x,frame in ((12,left),(486,right)):
        canvas[82:342,x:x+462]=cv2.resize(frame,(462,260),interpolation=cv2.INTER_AREA)
    image=Image.fromarray(cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB));d=ImageDraw.Draw(image)
    white=(235,239,245);muted=(160,175,190);accent=(86,205,190)
    title=LABELS.get(status.state,'等待机器人') if active else '等待第三段状态'
    d.text((15,10),'第三段 · '+title,font=font(23),fill=white)
    d.text((16,49),'视觉纠偏',font=font(20),fill=accent)
    d.text((490,49),'物料放置',font=font(20),fill=accent)
    d.text((700,15),f'主控 item {status.item}' if status else '动作尚未启动',font=font(18),fill=muted)
    metric='标定已确认' if verified else '未标定 · 仅预览，不发送有效纠偏'
    if engineering:metric='三环对位响应已启用 · 实际落点待验'
    d.text((16,356),metric,font=font(16),fill=accent if valid else muted)
    if status:
        colors=(status.color0,status.color1,status.color2)
        d.text((490,356),'车载槽颜色 '+ '/'.join(COLORS.get(c,str(c)) for c in colors),font=font(16),fill=muted)
        # S31 publishes slot colors and item, not positions/first/count. Never infer a target ring or success.
        d.text((490,325),'目标圆环：按主控任务映射',font=font(14),fill=white)
    return cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)

class RegionDesktop:
    def __init__(self,config):
        self.config=config;self.region='source';self.session=Session();self.pipe=None
        self.previous=None;self.acquired_after=None;self.frame_id=0;self.snapshot_count=0
        self.new_camera_frame=False
        self.last_result=None
        self.last_image=None

    def warm(self,size):
        if self.pipe is not None and self.pipe.size==size:return
        profile=json.loads(Path(self.config).read_text())
        if profile.get('observation_mode')=='cyqm_ring_station':
            from ring_row_alignment import RingRowPipeline
            self.pipe=RingRowPipeline(self.config,size)
        else:self.pipe=Pipeline(self.config,size)

    def reset(self):
        self.session=Session();self.previous=None;self.snapshot_count=0
        self.last_result=None;self.last_image=None
        if self.pipe is not None:self.pipe.detector.reset()

    def receive(self,line,now):
        try:status=parse_status(line.rstrip(b'\r\n')+b'\r\n')
        except (ValueError,UnicodeError):return False
        if self.region!='stage3':
            self.session=Session();self.previous=None
            if self.pipe is not None:self.pipe.detector.reset()
        self.region='stage3'
        if self.session.status!=status:print('MCU_STAGE3_STATUS '+str(asdict(status)),flush=True)
        changed=self.session.accept(status,now)
        if changed:
            self.session.sequence=resume_wire_sequence(THIRD.parent/'captures',status.run,status.pose,self.session.sequence)
        if changed:self.snapshot_count=0
        if changed and self.pipe is not None:self.pipe.detector.reset()
        return True

    def source_received(self):
        if self.region!='source':
            self.region='source';self.session=Session();self.previous=None
            if self.pipe is not None:self.pipe.detector.reset()

    def capture_boundary(self,metadata):
        self.acquired_after=None
        self.new_camera_frame=False
        if metadata is None:
            self.previous=None;return
        old=self.previous
        if (old is not None and old['generation']==metadata['generation']
                and type(metadata['sequence']) is int and type(old['sequence']) is int
                and metadata['sequence']>old['sequence']
                and isinstance(old['completed'],(int,float))
                and isinstance(metadata['completed'],(int,float))
                and metadata['completed']>old['completed']):
            # Sequential worker's previous completion is a conservative read-start
            # lower bound, never an exposure timestamp or a driver backlog claim.
            self.acquired_after=old['completed']
            self.new_camera_frame=True
        elif self.pipe is not None:
            self.pipe.detector.reset()
        self.previous=dict(metadata)

    def process(self,frame,metadata,preview_only,acquired_after=None,full_map=False,storage=False,headless=False):
        self.frame_id+=1;self.snapshot_count+=1;now=time.monotonic()
        status=self.session.status
        active=status is not None and 0<=now-self.session.last_status_at<=.2
        self.warm(tuple(frame.shape[1::-1]))
        if full_map:self.acquired_after=acquired_after
        ring_row=hasattr(self.pipe,'alignment_ready')
        engineering=ring_row and self.pipe.alignment_ready()
        eligible=bool(active and status.state==2 and self.acquired_after is not None
                      and self.acquired_after>self.session.fence)
        track=eligible or (storage and active and status.state==5 and
                           self.acquired_after is not None and self.acquired_after>self.session.fence)
        if engineering and not track:self.pipe.detector.reset()
        left,result=self.pipe.detect(frame)
        self.last_result=result
        self.last_image=left.copy()
        if engineering and not track:self.pipe.detector.reset()
        verified=self.pipe.verified()
        valid=bool(eligible and (verified or engineering) and result and result.detected and result.stable)
        motion=(0.,0.,0.);errors=(0.,0.)
        if valid:
            motion,errors=(self.pipe.measured_motion(result) if engineering else measured_motion(self.pipe.profile,result))
        if result is not None:
            center=(result.centers_px[1] if ring_row and result.detected else getattr(result,'middle_center_px',None))
            if center:cv2.drawMarker(left,tuple(round(v) for v in center),(0,255,0),cv2.MARKER_CROSS,20,2)
            if ring_row and result.detected:
                cv2.line(left,tuple(round(v) for v in result.centers_px[0]),tuple(round(v) for v in result.centers_px[2]),(0,255,0),2)
                cv2.drawMarker(left,tuple(round(v) for v in self.pipe.target),(0,0,255),cv2.MARKER_CROSS,24,2)
            else:
                for a,b in getattr(result,'line_segments_px',()):cv2.line(left,a,b,(0,255,0),2)
        right=frame.copy();wire=None
        completed=metadata.get('completed') if metadata else None
        send_at=time.monotonic()
        if full_map and valid:
            valid=(0<=send_at-self.session.last_status_at<=.2
                   and math.ceil((send_at-self.acquired_after)*1000)<=200)
        if (not preview_only and self.acquired_after is not None and completed is not None
                and (not full_map or (active and status.state==2))):
            wire=self.session.encode(self.frame_id,self.acquired_after,completed,send_at,valid,motion,errors)
        views=right if headless else compose_stage3(left,right,status,active,verified,valid,engineering)
        return dict(region='stage3',mcu_status=asdict(status) if status else None,
                    calibration_verified=verified,detected=bool(result and result.detected),
                    alignment_response_ready=engineering,
                    actual_ring_centers_px=list(result.centers_px) if ring_row and result and result.detected else None,
                    actual_row_yaw_deg=result.station_yaw_deg if ring_row and result and result.detected else None,
                    vehicle_edge=getattr(self.pipe,'edge',None),
                    row_to_vehicle_edge_deg=((result.station_yaw_deg-self.pipe.target_yaw+90)%180-90
                                             if ring_row and result and result.detected else None),
                    nominal_alignment_errors_mm_deg=list(errors),
                    metric_candidate_valid=valid,motion_right_forward_yaw=list(motion),
                    capture_start_lower_bound=self.acquired_after,
                    exposure_timestamp_verified=False,motor_commands_sent=False,
                    observation_wire=wire.decode('ascii').strip() if wire else None,
                    preview_only=preview_only,_snapshot=self.snapshot_count==2,_wire=wire),views
