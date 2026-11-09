from pathlib import Path
import json,zipfile,collections,hashlib,numpy as np
R=Path(__file__).resolve().parents[1];E=R/'evidence/p2-camera/2026-11-05/blue-perimeter-board';p=E/'20261105-144531-087584.zip';z=zipfile.ZipFile(p);assert z.testzip() is None
r=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()];fresh=[x for x in r if x['state']!='STALE_RESULT'];stale=[x for x in r if x['state']=='STALE_RESULT']
assert len(fresh)==227 and len(stale)==39
assert all(sorted(t['color'] for t in x['targets'])==['blue','red','yellow'] for x in fresh)
assert all(not x['targets'] and x['result_host_age_ms']>=150 for x in stale)
manifest=json.loads(z.read('module_source_hashes.json'));A=R/'output/blue-perimeter-trial-20261105/vision_app/code'
for n in ['project_detector.py','project_follow.py','project_tracker.py','project_camera.py']:assert hashlib.sha256((A/n).read_bytes()).hexdigest()==manifest[n],n
run=json.loads(z.read('run.json'));report=dict(zip_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),frames=len(r),valid_rgb_frames=len(fresh),stale_empty_frames=len(stale),stale_fraction=len(stale)/len(r),all_valid_frames_exact_red_yellow_blue=True,missing_color_on_valid_frame=0,first_loss_events=run['transition_counts']['first-blue-loss'],identity_changes=run['transition_counts']['first-blue-id-change'],longest_blue_absence_seconds=run['longest_watched_loss_s'],age_ms_percentiles=np.percentile([x['result_host_age_ms'] for x in r],[50,95,100]).tolist(),detector_ms_percentiles=np.percentile([x['detector_call_ms'] for x in r],[50,95,100]).tolist(),exact_package_module_hashes=True,serial='DISABLED',motion_allowed=False,physical_acceptance='geometry observed on valid frames; continuous freshness and identity not accepted')
(E/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,indent=2))
