"""Prepare a MATLAB-only causal arrival-estimator replay; no device imports."""
from pathlib import Path
import json,hashlib,subprocess
O=Path(__file__).resolve().parent;R=O.parents[4];PREV=O.parent/'four-direction-short-analysis';DAY=O.parent.parent/'2026-11-16'
C=json.loads((PREV/'contexts.json').read_text('utf-8-sig'));S=json.loads((PREV/'summary.json').read_text('utf-8-sig'))
h={};out=[]
for c,r in zip(C,S['runs']):
 c={k:v for k,v in c.items() if k not in ['pm_positions','measurement']};c['profile_trace']=str(DAY/'all-directions-short-prep'/f"mode{dict(forward=0,left=1,right=2,backward=3)[c['direction']]}_{c['target_mm']}_profile_trace.csv")
 c['raw_series']=str(PREV/(c['name']+'_series.csv'));c['last_progress_age_ms']=r['progress_age_at_STOP_ms'];out.append(c)
 for p in [Path(c['decoded']),Path(c['profile_trace']),Path(c['raw_series'])]:h[str(p.relative_to(R))]=hashlib.sha256(p.read_bytes()).hexdigest()
for name in ['3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c','3.stm32f407工程/CHASSIS/USER/app_fusion_capture.c','3.stm32f407工程/CHASSIS/USER/app_entry.c','3.stm32f407工程/CHASSIS/USER/app_serial_runtime.c','3.stm32f407工程/CHASSIS/USER/app_heading_runtime.c','工创赛/docs/competition_requirements_2027.md']:
 h[name]=hashlib.sha256((R/name).read_bytes()).hexdigest()
for name,obj in [('contexts.json',out),('input_sha256.json',h),('parameters.json',json.loads((PREV/'parameters.json').read_text('utf-8-sig')))]:
 (O/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
dirty=subprocess.check_output(['git','diff','--name-only','-z']).decode('utf-8').strip('\0').split('\0');(O/'original_dirty_sha256.json').write_text(json.dumps({s:hashlib.sha256((R/s).read_bytes()).hexdigest() for s in dirty},ensure_ascii=False,indent=2),encoding='utf-8')
print('Prepared',len(out),'trials;',len(h),'hashed sources; dirty',len(dirty))
