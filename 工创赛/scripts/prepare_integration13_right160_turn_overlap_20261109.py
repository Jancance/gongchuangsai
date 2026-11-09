"""Right160mm and user-authorized yaw/retract overlap at last5mm height."""
from pathlib import Path
import hashlib,json
from prepare_stage2_fast_candidate_20261109 import replace
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'integration13-right160-turn-overlap-matlab-20261109.json').read_text())['passed']
p=P/'integration10_complete.inc';d=p.read_bytes()
d=replace(d,'  !(g_it_extension_mm>=0.0f && g_it_extension_mm<=156.0f) ||\r\n',
 '  !((g_it_dm_deg==180.0f && g_it_extension_mm==0.0f) ||\r\n'
 '    (g_it_dm_deg==195.0f && g_it_extension_mm==156.0f) ||\r\n'
 '    (g_it_dm_deg==165.0f && g_it_extension_mm==160.0f)) ||\r\n')
d=replace(d,'  !(g_it_extension_mm==0.0f || g_it_extension_mm==156.0f) ||\r\n',
 '  !((g_it_dm_deg==180.0f && g_it_extension_mm==0.0f) ||\r\n'
 '    (g_it_dm_deg==195.0f && g_it_extension_mm==156.0f) ||\r\n'
 '    (g_it_dm_deg==165.0f && g_it_extension_mm==160.0f)) ||\r\n')
d=replace(d,' if(depth>=0.0f && depth<=15200.0f &&\r\n'
 '  (!(vsp_group.command.axes_mask&SPF_EXT) || ((vsp_group.done&SPF_EXT) &&\r\n'
 '   vsp_axis[0].samples>=3U && fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=2.0f)))\r\n',
 ' /* User-authorized carry overlap: at the last5mm height, yaw may start\r\n'
 '  * while retraction finishes. Release still waits for ALL axes. */\r\n'
 ' if(depth>=0.0f && depth<=15200.0f)\r\n')
p.write_bytes(d)
a=P/'integration13_immediate.inc';b=a.read_bytes()
b=replace(b,'  g_it_extension_mm=g_it_auto_item?156.0f:0.0f;g_it_lift_pulses=118800.0f;',
 '  g_it_extension_mm=g_it_auto_item==2U?160.0f:(g_it_auto_item==1U?156.0f:0.0f);g_it_lift_pulses=118800.0f;')
a.write_bytes(b);(R/'software_interfaces/stage2_fast_cycle'/a.name).write_bytes(b)
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text(encoding='utf-8'))
m.setdefault('previous_deployed_images',[]).append({'images':m['images'],'flash_log':m.get('flash_log'),
 'verified_run':'evidence/integration/2026-11-07/stage2-mainline/integration13-three-immediate-once-20261109-211735.json',
 'release_command_s':20.180,'zero_s':21.535,
 'operator_observation':'three slots each loaded and stable, no contact, stopped'})
for p in (p,a):m['changed'][p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
m['images']={};m['deployed']=False;m['hardware_motion']=False
m['three_direct']['extension_mm']=[0,156,160]
m['three_direct']['right_early_retract_field_verified']=False
m['lead5']['extension_zero_before_yaw']=False
m['lead5']['field_verified']=False
m['right160_yaw_retract_overlap']={'user_authorized':True,'yaw_max_depth_pulses':15200,
 'all_axes_complete_before_release':True,'field_verified':False}
f.write_text(json.dumps(m,indent=2)+'\n',encoding='utf-8')
print('Prepared left156/right160 and yaw+remaining-retraction overlap; not flashed.')
