"""Align the actual retraction-complete and rotation admission windows to 2deg."""
import hashlib,json
from pathlib import Path
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'integration13-zero2-matlab-20261109.json').read_text())['passed']
changes={
 'vehicle_source_parallel.inc':[(b'(zero_return?1.0f:45.0f)',b'(zero_return?2.0f:45.0f)')],
 'integration10_complete.inc':[
  (b'fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>1.0f',
   b'fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>2.0f'),
  (b'fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=1.0f',
   b'fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=2.0f')],
 'stage2_fast_cycle.inc':[
  (b'fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=1.0f',
   b'fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=2.0f'),
  (b'fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>1.0f',
   b'fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>2.0f')]
}
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text())
m.setdefault('previous_deployed_images',[]).append({'images':m['images'],'flash_log':m.get('flash_log')})
for name, pairs in changes.items():
 p=P/name;data=p.read_bytes()
 for old,new in pairs:
  assert old in data,(name,old);data=data.replace(old,new)
 p.write_bytes(data);m['changed'][name]=hashlib.sha256(data).hexdigest()
 if name=='stage2_fast_cycle.inc':(R/'software_interfaces/stage2_fast_cycle'/name).write_bytes(data)
m['images']={};m['deployed']=False;m['hardware_motion']=False
m['zero_window_motor_deg']=2;m['zero_window_nominal_mm']=2/(315.5*360/3200)
f.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')
print('Zero-return completion and yaw admission now share 2deg; other protections unchanged.')
