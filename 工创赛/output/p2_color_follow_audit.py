"""指定颜色的桌面跟踪取证；复用已安装V4记录器，不改变检测参数。"""
import argparse
import copy
from datetime import datetime
import hashlib
import importlib
from pathlib import Path
import sys
import time

from p2_integrated_follow_audit import FollowRecorder, json_value
import p2_integrated_follow_audit as diagnostic_base
import json

COLORS=('red','yellow','blue','green','black','lightblue')


class ColorRecorder(FollowRecorder):
    def __init__(self,folder,watch_color):
        if watch_color not in COLORS:raise ValueError('Unknown watch color')
        super().__init__(folder)
        self.watch_color=watch_color

    def write_json(self,path,value):
        if path.name=='run.json':
            value=dict(value,watch_color=self.watch_color)
            value['longest_watched_loss_s']=value.pop('longest_red_loss_s',0.)
        super().write_json(path,value)

    def save_event(self,name,previous,current,metadata):
        if name=='longest-red-loss':name='longest-'+self.watch_color+'-loss'
        super().save_event(name,previous,current,metadata)

    def observe(self,frame,result,started,finished,tracker=None):
        for future in self.pending:
            if future.done():future.result()
        row=copy.deepcopy(result)
        self.frames+=1
        row.update(frame_index=self.frames,watch_color=self.watch_color,
            detector_call_ms=(finished-started)*1000,
            result_host_age_ms=(finished-result['timestamp'])*1000,
            recorded_monotonic=finished,last_watched_ids_before_frame=self.last_red_ids)
        if tracker is not None:
            row['internal_tracks_not_actionable']=copy.deepcopy(list(tracker.tracks.values()))
        self.states[row['state']]+=1
        ids=tuple(sorted(t['id'] for t in row['targets'] if t['color']==self.watch_color))
        old=tuple(sorted(t['id'] for t in (self.previous or {}).get('targets',[]) if t['color']==self.watch_color))
        # 即使第一帧一直错分成别的颜色，也留下原图，而不是只留下计数。
        if self.frames==1:self.event('first-frame',frame,row)
        if any(t['color']!=self.watch_color for t in row['targets']):
            self.event('first-other-color-candidate',frame,row)
        # 按异色分别保存，避免手部红候选抢占首张，漏掉蓝/浅蓝切换原图。
        for other in sorted({t['color'] for t in row['targets'] if t['color']!=self.watch_color}):
            self.event('first-'+self.watch_color+'-with-'+other,frame,row)
        prefix='first-'+self.watch_color
        if ids and self.last_red_ids is None:self.event(prefix+'-visible',frame,row)
        if old and not ids:
            self.event(prefix+'-loss',frame,row)
            self.lost=True;self.loss_started=row['timestamp']
        if ids:
            if self.lost:
                self.event(prefix+'-reappearance',frame,row)
                duration=row['timestamp']-self.loss_started
                if duration>self.longest_loss_s:
                    self.longest_loss_s=duration
                    metadata=copy.deepcopy({'event':'longest-'+self.watch_color+'-loss',
                        'previous':self.previous,'current':row,'loss_duration_s':duration,
                        'note':'Previous raw is last missing frame; current raw is reappearance.'})
                    self.longest_evidence=(self.previous_frame.copy(),frame.copy(),metadata)
                self.lost=False
            if self.last_red_ids is not None and ids!=self.last_red_ids:
                self.event(prefix+'-id-change',frame,row)
            self.last_red_ids=ids
        row['diagnostic_io_ms']=(time.monotonic()-finished)*1000
        self.log.write(json.dumps(row,ensure_ascii=False,default=json_value)+'\n')
        if self.frames%12==0:self.log.flush()
        self.previous=row;self.previous_frame=frame.copy()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch-color',choices=COLORS,default='blue')
    parser.add_argument('--app',type=Path,default=Path('/home/orangepi/p2-integrated-20261003/vision_app'))
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    code=args.app.resolve()/'code'
    if not (code/'main.py').is_file():raise RuntimeError('Integrated app not found: '+str(code))
    sys.path.insert(0,str(code))
    workbench=importlib.import_module('main')
    sys.argv=[str(code/'main.py')]+(['--check'] if args.check else [])
    if args.check:return workbench.main()
    folder=(args.output_dir or args.app.parent/'color-follow-audits')/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    recorder=ColorRecorder(folder,args.watch_color)
    hashes={name:hashlib.sha256((code/name).read_bytes()).hexdigest()
            for name in ('project_detector.py','project_follow.py','project_tracker.py','project_camera.py')}
    hashes['diagnostic_entry']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    hashes['diagnostic_base']=hashlib.sha256(Path(diagnostic_base.__file__).read_bytes()).hexdigest()
    recorder.write_json(folder/'module_source_hashes.json',hashes)
    for name in ('material_color_thresholds.json','project_profile.json','camera_processing.json'):
        source=code/'config'/name;data=source.read_bytes()
        (folder/name).write_bytes(data)
        recorder.write_json(folder/(name+'.source.json'),{'file':str(source),'sha256':hashlib.sha256(data).hexdigest()})
    follower=workbench.MaterialFollower;original=follower.detect
    def audited(instance,frame,timestamp=None):
        started=time.monotonic();result=original(instance,frame,timestamp=timestamp)
        recorder.observe(frame,result,started,time.monotonic(),instance.tracker)
        return result
    follower.detect=audited
    status=None;error=None
    print('Diagnostic folder:',folder,flush=True)
    print('Watch color:',args.watch_color,'; keep F mode; Q quits. No robot control.',flush=True)
    try:
        status=workbench.main();return status
    except BaseException as exc:
        error=type(exc).__name__+': '+str(exc);raise
    finally:
        follower.detect=original;recorder.close(status,error)


if __name__=='__main__':raise SystemExit(main())
