"""Read saved A evidence and prepare measured inputs for local MATLAB."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'evidence/integration/2026-11-06/source-vision-new-pose'
SESSION=BASE/'source-finalstop-A-20261106'
OUT=BASE/'finalstop-A-review'


def read_image(path):
    return cv2.imdecode(np.frombuffer(path.read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)


def main():
    OUT.mkdir(exist_ok=True)
    rows=[json.loads(s) for s in (SESSION/'observations.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
    session=json.loads((SESSION/'session.json').read_text(encoding='utf-8'))
    detections=[r['detection'] for r in rows]
    summary={
        'schema_version':1,'frames':len(rows),'session_frames':session['frames'],
        'error':session['error'],'serial':session['serial'],'motion_allowed':False,
        'detected_frames':sum(d['detected'] for d in detections),
        'stable_frames':sum(d['stable'] for d in detections),
        'unstable_frame_sequences':[r['frame_sequence'] for r in rows if not r['detection']['stable']],
        'tracking_counts':dict(Counter(d['tracking_mode'] for d in detections)),
        'metric_status_counts':dict(Counter(r['metric_preview']['status'] for r in rows)),
        'frame_sequence_contiguous': [r['frame_sequence'] for r in rows]==list(range(1,len(rows)+1)),
        'sensor_exposure_age_verified':False,
        'camera_config_hashes_unchanged': session['configs']==json.loads((BASE/'source-paper-finalstop-20261106/session.json').read_text(encoding='utf-8'))['configs'],
        'images':{n:hashlib.sha256((SESSION/n).read_bytes()).hexdigest() for n in ['first-raw.png','last-raw.png','last-overlay.png']},
    }
    for key in ['center_x_px','center_y_px','radius_px','fit_rms_px','visible_arc_deg','confidence','processing_ms']:
        a=np.array([d[key] for d in detections if d['detected']],float)
        summary[key]={'minimum':float(a.min()),'median':float(np.median(a)),'maximum':float(a.max()),'peak_to_peak':float(np.ptp(a))}
    summary['all_frames_motion_disabled']=all(not r['motion_allowed'] and not r['replay'] for r in rows)
    paper=cv2.cvtColor(read_image(BASE/'source-paper-finalstop-20261106/last-raw.png'),cv2.COLOR_BGR2GRAY)
    plain=cv2.cvtColor(read_image(SESSION/'last-raw.png'),cv2.COLOR_BGR2GRAY)
    points=[(1115,216),(975,245),(1045,275),(1056,347),(1011,485),(998,396),(927,365),(915,293),(831,260),(1355,40),(1400,232),(965,830),(650,196),(580,148)]
    patches=[]
    for x,y in points:
        template=paper[y-15:y+16,x-15:x+16]
        search=plain[y-21:y+22,x-21:x+22]
        response=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED)
        _,score,_,where=cv2.minMaxLoc(response)
        patches.append({'paper_px':[x,y],'correlation':score,'integer_shift_px':[where[0]-6,where[1]-6]})
    summary['fixed_surface_patches']=patches
    accepted=[p for p in patches if p['correlation']>=.9]
    summary['accepted_patch_count']=len(accepted)
    summary['max_accepted_integer_patch_shift_px']=max(float(np.linalg.norm(p['integer_shift_px'])) for p in accepted)
    data={'current_calibration_path':'calibration/source_paper_finalstop_20261106.json','summary':summary,
          'frame_boundary_points_px':[r['boundary_points_px'] for r in rows],
          'paper_to_plain_patches':patches}
    (OUT/'measurement-input.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    (OUT/'pixel-summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ['frames','detected_frames','stable_frames','tracking_counts','accepted_patch_count','max_accepted_integer_patch_shift_px','center_x_px','center_y_px','visible_arc_deg','fit_rms_px']},indent=2))


if __name__=='__main__':
    main()
