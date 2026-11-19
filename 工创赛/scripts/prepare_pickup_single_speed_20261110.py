"""User correction: remove staged extension; keep 50ms and grip handoff."""
from pathlib import Path
import json,shutil,hashlib
R=Path(__file__).resolve().parents[1];B=R/'tmp/pickup_trial1_settle_20261110/project'
SM=R/'tmp/pickup_smooth_handoff_20261110/project';P=R/'tmp/pickup_single_speed_20261110/project'
E=R/'evidence/integration/2026-11-10/single-speed';E.mkdir(exist_ok=True)
PREV=R/'evidence/integration/2026-11-10/speed-trials-settle'
assert not P.exists()
assert json.loads((R/'evidence/integration/2026-11-10/smooth-handoff/matlab.json').read_text(encoding='utf-8'))['passed']
shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
for rel in ('CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/integration13_immediate.inc','HARDWARE/DM_CAN2/dm_can2_motion.c'):
 shutil.copy2(SM/rel,P/rel)
p=P/'CHASSIS/USER/integration10_complete.inc';b=p.read_bytes()
old=b''' if(g_pick_trial_profile>=2U && g_it_extension_mm>0.0f){
  vsp_group.delayed_mask=SPF_LIFT;vsp_group.release_mask=SPF_EXT;
 }'''
assert b.count(old)==1;b=b.replace(old,b'',1)
i=b.index(b'static void it_empty_overlap_gates(void)');j=b.index(b'static void it_empty_overlap_publish(void)',i);b=b[:i]+b[j:]
old=b' it_empty_overlap_gates();\r\n';assert b.count(old)==1;b=b.replace(old,b'',1)
old=b'return g_pick_extension_segment!=1U && x->samples>=1U';assert b.count(old)==1;b=b.replace(old,b'return x->samples>=1U',1)
assert b'g_pick_extension_segment' not in b;p.write_bytes(b)
assert (P/'CHASSIS/USER/vehicle_source_parallel.inc').read_bytes()==(B/'CHASSIS/USER/vehicle_source_parallel.inc').read_bytes()
m=json.loads((PREV/'candidate.json').read_text(encoding='utf-8'))
for k in ('field_runs','field_review','latest_run_complete','mcu_three_complete','field_verified_profiles'):m.pop(k,None)
m.update(images={},revision='single_speed_release50',hardware_access=False,deployed=False,actual_tests_run=0,
 field_verified=False,physical_placement_confirmation_pending=True,base='tmp/pickup_trial1_settle_20261110/project',
 release_to_next_request_ms=50,extension_staged=False,extension_and_descent_start_together=True,
 motion_change='User explicitly cancelled staged speed. One native FD target per extension leg at200/300/400; keep first-profile600/400 braking and bounded settling, high profiles1200/2000 accel/decel. Tight DM3-frame and early-grip/50ms handoff retained.',
 high_speed_overrun_resolved=False)
m['changed']={}
for p in P.rglob('*'):
 if p.is_file():
  rel=p.relative_to(P);base=R/'handoff/pzq/project'/rel
  if not base.exists() or base.read_bytes()!=p.read_bytes():m['changed'][rel.as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
(E/'candidate.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for name in ('timing-model.json','integration13-three-direct-matlab-20261109.json'):shutil.copy2(PREV/name,E/name)
for name in ('build_pickup_trial1_settle_20261110.py','check_pickup_trial1_settle_20261110.py'):
 s=(R/'scripts'/name).read_text(encoding='utf-8').replace('pickup_trial1_settle_20261110','pickup_single_speed_20261110').replace('speed-trials-settle','single-speed').replace('.read_text()',".read_text(encoding='utf-8')")
 (R/'scripts'/name.replace('pickup_trial1_settle','pickup_single_speed')).write_text(s,encoding='utf-8')
for name in ('check_pickup_smooth_owner_20261110.py','pickup_smooth_handoff_live_20261110.py'):
 s=(R/'scripts'/name).read_text(encoding='utf-8').replace('pickup_smooth_handoff_20261110','pickup_single_speed_20261110').replace('smooth-handoff','single-speed')
 dest=name.replace('pickup_smooth_owner','pickup_single_speed_owner').replace('pickup_smooth_handoff','pickup_single_speed')
 (R/'scripts'/dest).write_text(s,encoding='utf-8')
print('Prepared single-speed candidate. Native extension backend restored byte-for-byte; no device access.')
