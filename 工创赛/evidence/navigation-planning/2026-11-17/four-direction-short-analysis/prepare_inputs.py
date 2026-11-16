import hashlib,json,subprocess
from pathlib import Path
O=Path('工创赛/evidence/navigation-planning/2026-11-17/four-direction-short-analysis').resolve()
R=Path.cwd(); D=O.parent.parent/'2026-11-16'
def read(p): return json.loads(p.read_text('utf-8-sig'))
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
C=[]; H={}
for direction in ['forward','backward','left','right']:
 for cm in [150,30]:
  for root in sorted(D.glob(f'{direction}{cm}_40-field-run*')):
   if not (root/'operator_measurement.json').exists():continue
   d=read(root/'capture/decoded.json');r=read(root/'result_summary.json');m=read(root/'operator_measurement.json');rc=read(root/'capture/read_receipt.json');ex=read(root/'field_execution_summary.json')
   logs=list((root/'serial').glob('*.jsonl'));assert len(logs)==1
   rows=[json.loads(s) for s in logs[0].read_text('utf-8-sig').splitlines() if s.strip()];tr=d['header']['trigger_ms']
   tm=[a for a in rows if a.get('wire_tag')=='TMQ' and a['tick_ms']>=tr]
   done=next(a for a in tm if a['backend_active']==0 and a['backend_result']==1)
   zero=next(a for i,a in enumerate(tm) if all(all(z.get(f'rpm{j}',1)==0 for j in range(1,5)) for z in tm[i:]))
   assert r['status']=='software_complete' and rc['accepted']
   ep=[a for a in r['endpoints'] if a['session']==d['header']['session']];assert len(ep)==1
   cross=m.get('left_mm',-m.get('right_mm',0)) if direction in ['forward','backward'] else m.get('forward_mm',-m.get('backward_mm',0))
   C.append(dict(name=root.name,direction=direction,target_mm=cm*10,primary=not(direction=='right' and cm==150 and root.name[-1] in '123'),axf_sha256=rc['axf_sha256'],decoded=str(root/'capture/decoded.json'),measurement=m,ground_main_mm=m[direction+'_mm'],ground_cross_mm=cross,STOP_s=(d['header']['stop_ms']-tr)/1000,terminal_s=(done['tick_ms']-tr)/1000,persistent_zero_s=(zero['tick_ms']-tr)/1000,frozen_progress_mm=ex['frozen_progress_mm'],endpoint=ep[0], pm_positions=[a for a in rows if a.get('frame_type')=='PM1' and a.get('session')==d['header']['session']], first_reached_s=next((a['tick_ms']-tr)/1000 for a in tm if a['distance_reached'])))
   for p in [root/'capture/decoded.json',root/'capture/read_receipt.json',root/'result_summary.json',root/'operator_measurement.json',root/'field_execution_summary.json',logs[0]]:H[str(p.relative_to(R))]=sha(p)
assert len(C)==22,len(C)
p=D/'backward-three-run-analysis/analysis_parameters.json';H[str(p.relative_to(R))]=sha(p)
for s in ['CHASSIS/USER/chassis_kinematics.c','CHASSIS/USER/chassis_kinematics.h','CHASSIS/USER/chassis_odometry.c','USER/vehicle_parameters.h']:
 p=R/'3.stm32f407工程'/s;H[str(p.relative_to(R))]=sha(p)
for extra in ['3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c','工创赛/docs/competition_requirements_2027.md']:H[extra]=sha(R/extra)
for name,obj in [('contexts.json',C),('parameters.json',read(D/'backward-three-run-analysis/analysis_parameters.json')),('input_sha256.json',H)]: (O/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
dirty=subprocess.check_output(['git','diff','--name-only','-z']).decode('utf-8').strip('\0').split('\0')
(O/'original_dirty_sha256.json').write_text(json.dumps({s:sha(R/s) for s in dirty},ensure_ascii=False,indent=2),encoding='utf-8')
print('Prepared',len(C),'trials',len(H),'hashes;',len(dirty),'dirty files protected')
