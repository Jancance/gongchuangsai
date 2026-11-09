"""用已归档原图检查像素状态UI；不把离线渲染当本次板端验收。"""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import time

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'output/p2_vision_generic135_pixel1_20261027/vision_app'
OUT=ROOT/'output/pixel1-validation-20261027'


def main():
    spec=importlib.util.spec_from_file_location('pixel1_view',APP/'code/project_ring_pixel.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    base=ROOT/'evidence/p2-camera/2026-10-27'
    raw_folder=base/'ring-rotation/run-20261027-193422-279783'
    empty_folder=base/'ring-empty/run-20261027-194859-020849'
    stable_row=json.loads((raw_folder/'first-stable-reappearance.json').read_text(encoding='utf8'))
    empty_row=json.loads((empty_folder/'first-frame.json').read_text(encoding='utf8'))
    OUT.mkdir(exist_ok=True)
    cases=(('stable',raw_folder/'first-stable-reappearance-raw.png',stable_row,True,0.),
           ('candidate-ui-simulation',raw_folder/'first-stable-reappearance-raw.png',stable_row,False,0.),
           ('expired-ui-simulation',raw_folder/'first-stable-reappearance-raw.png',stable_row,True,.3),
           ('empty',empty_folder/'first-frame-raw.png',empty_row,False,0.))
    report={}
    for name,path,row,stable,delay in cases:
        frame=cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)
        gate=module.RingPixelPresenter(paper_labels=(3,2,1))
        result=SimpleNamespace(detected=row['detected'],stable=stable,confidence=row['confidence'],centers_px=row['centers_px'])
        finish=row['host_read_time']+row['result_host_age_ms']/1000
        gate.observe(result,frame.shape,row['host_read_time'],finish)
        status=gate.current(finish+delay)
        started=time.perf_counter();module.draw_ring_pixels(frame,status);elapsed=(time.perf_counter()-started)*1000
        ok,data=cv2.imencode('.png',frame);assert ok
        data.tofile(OUT/(name+'.png'))
        report[name]=dict(state=status['state'],ready=status['pixel_ready'],centers=status['stable_centers_px'],
                          single_offline_draw_ms=elapsed,source=str(path.relative_to(ROOT)),
                          scope='Offline UI only. Candidate/expired states are simulated; no new live acceptance.')
    assert report['stable']['state']=='STABLE'
    assert report['candidate-ui-simulation']['state']=='CANDIDATE'
    assert report['expired-ui-simulation']['state']=='STALE'
    assert report['empty']['state']=='LOST'
    (OUT/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
