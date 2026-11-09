"""以模拟采集/GUI验证发布入口的自动结束、提前退出、异常及停顿归档。"""
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import ModuleType
from unittest.mock import patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
VERSION=2 if '--v2' in sys.argv else 1
BUNDLE=ROOT/('output/p2_partial_ring_live%d_20261027'%VERSION)
sys.path.insert(0,str(BUNDLE))
sys.path.insert(0,str(BUNDLE/'vision_app/code'))
spec=importlib.util.spec_from_file_location('live_test_entry',BUNDLE/'p2_partial_ring_live_audit.py')
entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
raw=cv2.imdecode(np.fromfile(ROOT/'evidence/p2-camera/2026-10-27/pixel1-material-occlusion/run-20261027-205050-650002/first-frame-raw.png',np.uint8),1)


class FakeCamera:
    events=[{'event':'SIMULATED_CAPTURE_NOT_HARDWARE'}]
    def read(self):
        time.sleep(.025)
        self.last_read_completed_time=time.monotonic()
        return True,raw
    def release(self):pass


class FailedCamera(FakeCamera):
    def read(self):raise RuntimeError('SIMULATED_READ_FAILURE')


class AgedCamera(FakeCamera):
    def read(self):
        ok,frame=super().read()
        self.last_read_completed_time-=.02
        return ok,frame


def main():
    destination=ROOT/('tmp/partial-live%d-validation'%VERSION)
    destination.mkdir(parents=True,exist_ok=True)
    report={}
    cases=[('duration',FakeCamera,-1),('early-quit',FakeCamera,ord('q')),('read-failure',FailedCamera,-1),('processing-stall',FakeCamera,-1)]
    if VERSION==2:cases.append(('aged-input',AgedCamera,-1))
    for name,camera,key in cases:
        out=destination/name;out.mkdir(exist_ok=True)
        old=set(out.iterdir())
        fake=ModuleType('project_camera');fake.LatestProjectCamera=camera
        import p2_partial_ring_candidate
        original=p2_partial_ring_candidate.detect_candidate
        def stall(*args):
            time.sleep(1)
            return original(*args)
        target=stall if name=='processing-stall' else original
        status=0
        with patch.dict(sys.modules,{'project_camera':fake}), patch.object(sys,'argv',['live','--seconds','3','--output-dir',str(out)]), patch.object(p2_partial_ring_candidate,'detect_candidate',target), patch.object(cv2,'namedWindow'), patch.object(cv2,'resizeWindow'), patch.object(cv2,'imshow'), patch.object(cv2,'destroyAllWindows'), patch.object(cv2,'waitKey',return_value=key):
            try:entry.main()
            except SystemExit as exc:status=exc.code
        folder=next(p for p in set(out.iterdir())-old if p.is_dir())
        summary=json.loads((folder/'run.json').read_text(encoding='utf8'))
        rows=[json.loads(s) for s in (folder/'observations.jsonl').read_text(encoding='utf8').splitlines()]
        if name=='duration':
            assert status==0 and summary['completed_duration'] and summary['frames']>0
            assert 3<=summary['sampling_span_s']<4
        elif name=='early-quit':
            assert status==0 and not summary['completed_duration']
        elif name=='read-failure':
            assert status==1 and 'SIMULATED_READ_FAILURE' in summary['error']
        else:
            assert status==1 and summary['error']=='NO_PROCESSED_RESULT_500MS'
            if name=='aged-input':
                assert summary['input_decisions']['SKIP_INPUT_AGE_OVER_12MS']>0
                assert 'PROCESS' not in summary['input_decisions']
        for row in rows:
            state=row['status']
            if state['pixel_ready']:assert 0<=state['result_host_age_ms']<=150
            if state['state'] in ('LOST','STALE','FAULT'):assert not state['stable_centers_px'] and not state['candidate_centers_px']
            assert not state['metric_valid'] and not state['motion_allowed']
        assert folder.with_suffix('.zip').exists()
        report[name]=dict(passed=True,frames=len(rows),exit_code=status,summary=summary)
    (destination/'review.json').write_text(json.dumps(dict(cases=report,hardware_acceptance=False,simulation_only=True),indent=2),encoding='utf8')
    print('SIMULATED_ENTRY_CHECKS=%d/%d; NO_REAL_CAMERA_OR_GUI_OPENED'%(len(report),len(cases)))


if __name__=='__main__':main()
