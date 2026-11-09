"""Apply the explicitly requested37deg slot and200ms release-to-yaw wait."""
from pathlib import Path
import hashlib,json,shutil
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'integration13-slot37-release200-matlab-20261109.json').read_text())['passed']
manifest=E/'stage2-fast-candidate-20261109.json';m=json.loads(manifest.read_text())
prior=json.loads((E/'integration13-three-immediate-once-20261109-210430.json').read_text(encoding='utf-8'))
assert prior['complete'] and prior['sha256']==m['images']['Vehicle_LiftRuler']['sha256']
m.setdefault('previous_deployed_images',[]).append({'images':m['images'],'flash_log':m.get('flash_log'),
 'verified_run':'evidence/integration/2026-11-07/stage2-mainline/integration13-three-immediate-once-20261109-210430.json',
 'three_release_command_s':18.791,'three_zero_s':19.788,
 'operator_observation':'\u4e09\u69fd\u5404\u4e00\u4ef6\u4e14\u653e\u7a33\uff0c\u65e0\u78b0\u64e6\uff0c\u5df2\u505c\u7a33'})
p=P/'integration10_complete.inc';data=p.read_bytes();old=b'it_command.goal.dm_deg=38.380798f;';new=b'it_command.goal.dm_deg=37.0f;'
assert data.count(old)==1;p.write_bytes(data.replace(old,new,1))
s=R/'software_interfaces/stage2_fast_cycle/integration13_immediate.inc';shutil.copy2(s,P/s.name)
for p in (p,P/s.name):m['changed'][p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
m['images']={};m['deployed']=False;m['hardware_motion']=False
m['slot_yaw_deg']=37;m['immediate_chain']['fixed_release_delay_ms']=200
m['release200_applies_to_final_zero']=True
m['lead5']['field_verified']=True
m['three_direct']['right_early_retract_field_verified']=True
m['empty_test_entry']['historical_only']=True
m['loaded_test_entry']['historical_only']=True
manifest.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print('Prepared37deg common slot and200ms after release, previous successful source/AXFs preserved.')
