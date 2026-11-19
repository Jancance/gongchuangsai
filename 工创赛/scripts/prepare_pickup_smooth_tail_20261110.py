"""Extend bounded settling only to the measured low-speed tail."""
from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];B=R/'tmp/pickup_smooth_handoff_20261110/project'
P=R/'tmp/pickup_smooth_tail_20261110/project';E=R/'evidence/integration/2026-11-10/smooth-tail'
PREV=R/'evidence/integration/2026-11-10/smooth-handoff'
assert json.loads((E/'matlab.json').read_text(encoding='utf-8'))['passed'];assert not P.exists()
m=json.loads((PREV/'candidate.json').read_text(encoding='utf-8'))
for rel,h in m['changed'].items():assert hashlib.sha256((B/rel).read_bytes()).hexdigest()==h,rel
shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
p=P/'CHASSIS/USER/vehicle_source_parallel.inc';b=p.read_bytes();old=b' if(zero_return)return 1U;'
new=b''' if(zero_return)return 1U;
 if(address==6U && g_pick_trial_profile>=2U && g_pick_trial_profile<=3U &&
  g_pick_extension_segment==2U && vsp_axis[0].goal<vsp_axis[0].before &&
  (vsp_group.command.goal.extension_mm==156.0f || vsp_group.command.goal.extension_mm==160.0f) &&
  vsp_axis[0].move[2]==2U && vsp_axis[0].move[3]==88U &&
  vsp_axis[0].move[4]==2U && vsp_axis[0].move[5]==88U &&
  vsp_axis[0].move[6]==4U && vsp_axis[0].move[7]==36U)return 1U;'''
assert b.count(old)==1;p.write_bytes(b.replace(old,new,1))
m.update(base='tmp/pickup_smooth_handoff_20261110/project',parent_deployed_sha256=m['images']['Vehicle_LiftRuler']['sha256'],
 images={},revision='smooth_tail_bounded_settle',hardware_access=False,deployed=False,actual_tests_run=0,field_verified=False,
 motion_change='Only add90deg/<150ms settling for actual106RPM/600RPMs outward tail, never high-speed segment. 50ms handoff and staging unchanged.')
m['changed']={rel:hashlib.sha256((P/rel).read_bytes()).hexdigest() for rel in m['changed']}
(E/'candidate.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for name in ('timing-model.json','integration13-three-direct-matlab-20261109.json'):
 shutil.copy2(PREV/name,E/name)
for name in ('build_pickup_smooth_handoff_20261110.py','check_pickup_smooth_handoff_20261110.py',
             'check_pickup_smooth_owner_20261110.py','pickup_smooth_handoff_live_20261110.py'):
 s=(R/'scripts'/name).read_text(encoding='utf-8').replace('pickup_smooth_handoff_20261110','pickup_smooth_tail_20261110').replace('smooth-handoff','smooth-tail')
 dest=name.replace('smooth_handoff','smooth_tail').replace('smooth_owner','smooth_tail_owner')
 (R/'scripts'/dest).write_text(s,encoding='utf-8')
print('Prepared low-speed-tail-only revision, no hardware access.')
