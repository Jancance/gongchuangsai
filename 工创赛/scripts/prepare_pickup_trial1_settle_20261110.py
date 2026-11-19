"""Prepare one bounded-settling candidate, without accessing hardware."""
from pathlib import Path
import json, shutil, hashlib
R=Path(__file__).resolve().parents[1]
B=R/'tmp/pickup_trial1_braking_20261110/project'
P=R/'tmp/pickup_trial1_settle_20261110/project'
E=R/'evidence/integration/2026-11-10/speed-trials-settle'
PREV=R/'evidence/integration/2026-11-10/speed-trials-braking'

def main():
    assert json.loads((E/'matlab.json').read_text())['passed']
    assert not P.exists()
    m=json.loads((PREV/'candidate.json').read_text())
    for rel,digest in m['changed'].items():
        assert hashlib.sha256((B/rel).read_bytes()).hexdigest()==digest,rel
    shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    p=P/'CHASSIS/USER/vehicle_source_parallel.inc';b=p.read_bytes()
    def rep(old,new):
        nonlocal b
        old=old.encode();assert b.count(old)==1
        b=b.replace(old,new.encode(),1)
    rep('static uint32_t VSP_PositionWindow(VSP_Axis *a,uint32_t zero_return)',
        '''/* First-profile extension only: bounded terminal braking transient.
 * Position completion still requires the original window and zero-speed samples. */
static uint32_t VSP_SettleAllowed(uint8_t address,uint32_t zero_return)
{
 if(zero_return)return 1U;
 return address==6U && g_pick_trial_profile==1U && PickupTrial_ExtDecel()==400U &&
  vsp_group.command.extension_rpm==200.0f && vsp_group.command.stepper_accel_rpm_s==600U &&
  (vsp_group.command.goal.extension_mm==156.0f || vsp_group.command.goal.extension_mm==160.0f) &&
  vsp_axis[0].goal<vsp_axis[0].before;
}
static uint32_t VSP_PositionWindow(VSP_Axis *a,uint32_t bounded_settle)''')
    rep(' /* Only a demonstrated zero-return stop transient gets a bounded settling\r\n  * interval. Severe excursion and every other axis/goal keep the old exit. */',
        ' /* Existing zero-return or first-profile outward braking transient only.\r\n  * Hard90deg/150ms remains a fault; this does not declare the axis complete. */')
    rep(' if(!zero_return || a->current<lo-90.0f || a->current>hi+90.0f)return 0U;',
        ' if(!bounded_settle || a->current<lo-90.0f || a->current>hi+90.0f)return 0U;')
    rep('  if(!VSP_PositionWindow(a,zero_return))return 2;',
        '  if(!VSP_PositionWindow(a,VSP_SettleAllowed(address,zero_return)))return 2;')
    p.write_bytes(b)
    m['parent_deployed_sha256']=m['images']['Vehicle_LiftRuler']['sha256']
    m['base']='tmp/pickup_trial1_braking_20261110/project';m['images']={}
    m['revision']='trial1_decel400_bounded_settle90deg150ms'
    m['hardware_access']=m['deployed']=m['field_verified']=False;m['actual_tests_run']=0
    for k in ('mcu_three_complete','physical_placement_confirmation_pending','field_review'):m.pop(k,None)
    m['motion_change']='First-profile node6 156/160mm outward move may settle within90 virtual deg for <150ms; final45deg/3 zero-speed samples and drive protection unchanged.'
    m['changed']={rel:hashlib.sha256((P/rel).read_bytes()).hexdigest() for rel in m['changed']}
    (E/'candidate.json').write_text(json.dumps(m,indent=2)+'\n')
    for name in ('timing-model.json','integration13-three-direct-matlab-20261109.json'):
        shutil.copy2(PREV/name,E/name)
    for kind in ('build','check','live'):
        name=f'{kind}_pickup_trial1_braking_20261110.py' if kind!='live' else 'pickup_trial1_braking_live_20261110.py'
        text=(R/'scripts'/name).read_text(encoding='utf-8').replace('pickup_trial1_braking_20261110','pickup_trial1_settle_20261110').replace('speed-trials-braking','speed-trials-settle')
        dest=name.replace('braking','settle');(R/'scripts'/dest).write_text(text,encoding='utf-8')
    print('Prepared bounded-settling candidate. Only vehicle_source_parallel.inc changed versus braking revision; no hardware access.')

if __name__=='__main__':main()
