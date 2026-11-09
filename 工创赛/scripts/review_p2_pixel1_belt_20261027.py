"""复核用户粘贴的普通R终端采样；不当作逐帧日志或全程录像。"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'evidence/p2-camera/2026-10-27/pixel1-belt-distractor'


def main():
    log=BASE/'console.txt'
    text=log.read_text(encoding='utf8')
    decoder=json.JSONDecoder();rows=[]
    for line in text.splitlines():
        if not line.startswith('Mode=RING_STATION '):continue
        def value(key):return decoder.raw_decode(line.split(' '+key+'=',1)[1])[0]
        row=dict(state=re.search(r'\bState=(\w+)',line).group(1),
                 fps=float(re.search(r'\bFPS=([0-9.]+)',line).group(1)),
                 centers_px=value('Centers'),candidate_centers_px=value('CandidateCenters'),
                 paper_labels=value('PaperIDs'),pixel_ready='PixelStable=True' in line)
        assert row['pixel_ready']==(row['state']=='STABLE')
        assert row['centers_px']==(row['candidate_centers_px'] if row['pixel_ready'] else [])
        assert row['paper_labels']==[3,2,1]
        assert 'LabelSource=MANUAL_FIXED_LAYOUT_NOT_OCR' in line
        assert 'Serial=DISABLED' in line and 'Metric=UNVERIFIED' in line
        if row['pixel_ready']:
            assert len(row['centers_px'])==3 and np.isfinite(row['centers_px']).all()
            assert all(0<=x<1920 and 0<=y<1080 for x,y in row['centers_px'])
        rows.append(row)
    assert rows
    stable=[r for r in rows if r['pixel_ready']]
    a=np.array([r['centers_px'] for r in stable]);fps=[r['fps'] for r in stable]
    report=dict(scene='One timing belt loop above the complete three printed rings; user substitution for proposed tape roll.',
        source='console.txt',screenshot='board-preview.png',
        log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
        screenshot_sha256=hashlib.sha256((BASE/'board-preview.png').read_bytes()).hexdigest(),
        status_samples=len(rows),states=dict(Counter(r['state'] for r in rows)),
        first_state=rows[0]['state'],remaining_samples_all_stable=all(r['pixel_ready'] for r in rows[1:]),
        median_stable_xy_px=np.median(a,axis=0).tolist(),range_stable_xy_px=np.ptp(a,axis=0).tolist(),
        reported_loop_fps_min_max=[min(fps),max(fps)],
        error_markers=[s for s in ('Traceback','ModuleNotFoundError','Camera unavailable','错误：') if s in text],
        manual_labels=[3,2,1],label_source='MANUAL_FIXED_LAYOUT_NOT_OCR',
        visual_review='Belt is visibly above the paper; three green crosses remain at printed rings; no cross on belt.',
        acceptance='Current stationary single-belt placement, as shown in screenshot and status samples, does not replace a printed-ring center with the distractor.',
        limits=['Timing belt is the actual tested object, not tape. Do not require a tape retest for this gate.',
                'Status output is about once per second, with no per-line timestamps or per-frame telemetry; no all-frame claim.',
                'Only one distractor/placement. No conclusion on arbitrary shapes, positions, multiple distractors or target occlusion.',
                'Cross-run center differences cannot be assigned solely to belt without matched pose and reference truth.',
                'Loop FPS is not exposure FPS or end-to-end latency; no sustained performance acceptance.'],
        metric_valid=False,motion_allowed=False,serial='DISABLED')
    (BASE/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
