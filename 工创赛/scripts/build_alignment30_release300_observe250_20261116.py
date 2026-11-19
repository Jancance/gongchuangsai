"""Real request5/6 dispatch and MCU preparation/owner regressions; dual build."""
import hashlib
import json
import shutil
from pathlib import Path
import build_integration18_ext150_20261115 as build
import build_integration23_sync_20261115 as prior
from build_stopped5_fastalign_20261116 import block
from prepare_alignment30_release300_observe250_20261116 import R,BASE,P,E


def request_fixture():
    integration=(P/'CHASSIS/USER/integration10_complete.inc').read_text(encoding='utf-8')
    actual=block(integration,'if(request==5U || request==6U)')
    return r'''
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
static unsigned checks,request,g_it_rejected,g_it_state,calls,accepted;
static float written_angle;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t jiazhua(float angle){calls++;written_angle=angle;return accepted;}
static void real_dispatch(void){
'''+actual+r'''
}
int main(void){unsigned i,before;
 for(request=5;request<=6;request++)for(i=0;i<2;i++){
  g_it_state=2;g_it_rejected=0;accepted=i;before=calls;real_dispatch();
  CHECK(calls==before+1);CHECK(written_angle==(request==5?140:250));
  if(accepted)CHECK(g_it_state==1 && g_it_rejected==0);
  else CHECK(g_it_state==2 && g_it_rejected==7);
 }
 request=4;before=calls;real_dispatch();CHECK(calls==before);
 printf("PASS actual integration observation dispatch: %u checks; request5 close140/request6 open250; PWM rejection7/state retained\n",checks);return 0;
}
'''


def owner_fixture(text):
    replacements=[
        ('static uint32_t vsp_fast_lift,vsp_pick_lead_ms,vsp_pick_sync;',
         'static uint32_t vsp_fast_lift,vsp_pick_lead_ms,vsp_pick_sync;\nstatic unsigned observations,observation_attempts,reject_observation;static float last_gripper_angle;'),
        ('CHECK(angle==140 || angle==170 || angle==210);',
         'CHECK(angle==140 || angle==170 || angle==210 || angle==250);\n if(angle==210)CHECK(g_dm_can2_logical_position_deg==37 && extension==0);\n if(angle==250){CHECK(!owned && g_dm_can2_logical_position_deg==180 && extension==65 && depth==600);observation_attempts++;if(reject_observation)return 0;observations++;}\n last_gripper_angle=angle;'),
        ('owned=body=failures=grips=releases=hold_calls=idle_reports=0;platform=reference=1;',
         'owned=body=failures=grips=releases=hold_calls=idle_reports=0;platform=reference=1;\n observations=observation_attempts=reject_observation=0;last_gripper_angle=0;'),
        ('CHECK(grips==1 && releases==1 && extension==(slot==3?0:65)',
         'CHECK(observations==(slot<3?1U:0U) && observation_attempts==observations && last_gripper_angle==(slot<3?250:210));\n CHECK(grips==1 && releases==1 && extension==(slot==3?0:65)')]
    for before,after in replacements:
        assert text.count(before)==1,before
        text=text.replace(before,after,1)
    return text


def return_failure_fixture():
    return r'''
#define main previous_owner_main
#include "all-regions-owner.c"
#undef main
int main(void){unsigned slot,loops,before;
 for(slot=1;slot<=3;slot++){
  start_region(0,slot);loops=0;
  while(g_low_phase!=7 && ++loops<300){
   if(g_low_phase==6){tick=pulse_apply_at+50;Low_Service(tick);tick+=300;Low_Service(tick);}
   else if(owned)finish_group();else{tick+=10;Low_Service(tick);}
  }
  CHECK(loops<300 && Low_Busy() && owned && observation_attempts==0);
  before=recorded;Low_Service(tick);CHECK(observation_attempts==0 && recorded==before && !g_low_done_sequence);
  reject_observation=1;finish_group();
  if(slot<3){CHECK(g_low_state==9 && g_low_fault==7 && failures==1 && g_it_state==9);CHECK(!g_low_done_sequence && observation_attempts==1 && last_gripper_angle==210);}
  else{CHECK(g_low_state==2 && g_low_done_sequence==1 && !failures);CHECK(observation_attempts==0 && last_gripper_angle==210);}
 }
 printf("PASS actual returned-observation PWM failure: %u checks; only completed slot1/2 reopen250, failure7 prevents DONE, slot3 parks210\n",checks);return 0;
}
'''


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed'] and proof['local_matlab'] and proof['all_source_observation_visits_250']
    previous=R/'evidence/integration/2026-11-16/source-alignment30-release300'
    for file in previous.iterdir():
        if file.suffix in ('.c','.h','.inc','.txt') or file.name=='parameter-and-poll-checks.json':
            shutil.copyfile(file,E/file.name)
    for name in ('all-regions-owner.c','base-commit-fixture.c'):
        path=E/name;path.write_text(owner_fixture(path.read_text(encoding='utf-8')),encoding='utf-8')
    path=E/'final-park-owner.c';text=path.read_text(encoding='utf-8')
    text=text.replace('if(item<2)CHECK(', 'if(item<2){CHECK(last_gripper_angle==250 && observations==item+1);}\n  else CHECK(last_gripper_angle==210 && observations==2);\n  if(item<2)CHECK(',1)
    path.write_text(text,encoding='utf-8')
    (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(
        b'#include "low_readonly_diag.inc"',b'/* separately verified diagnostic */'))
    prior.P=P;prior.E=E
    prior.run('actual-observe250-dispatch',request_fixture())
    prior.run('actual-return-observe250-failure',return_failure_fixture())
    for name in ('final-park-owner','owner-faults'):
        prior.run(name,(E/(name+'.c')).read_text(encoding='utf-8'))
    preserved={}
    for name in ('CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/source_body_step.inc',
                 'CHASSIS/USER/chassis_auto_test.c',
                 'USER/vehicle_parameters.h'):
        data=(P/name).read_bytes();assert data==(BASE/name).read_bytes()
        preserved[name]=hashlib.sha256(data).hexdigest()
    owner=(P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes()
    inserted=b'  if(low_slot<3U && jiazhua(250.0f)!=1U){Low_Fail(7U);return;}\r\n'
    assert owner.count(inserted)==1
    assert owner.replace(b'if(low_slot<3U && jiazhua(250.0f)',b'if(low_slot<3U && jiazhua(230.0f)',1)==(BASE/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes()
    mcu=(P/'CHASSIS/USER/source_mcu_cycle.inc').read_text(encoding='utf-8')
    assert 'prep[6]={1U,2U,7U,6U,3U,3U}' in mcu
    driver=Path(build.__file__).read_text(encoding='utf-8').replace('.read_text()',".read_text(encoding='utf-8')")
    driver=driver.replace("str(ROOT/'tests'/source)","str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope={'__name__':'observe250_build','__file__':build.__file__}
    exec(compile(driver,build.__file__,'exec'),scope)
    scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()
    for name in ('checks.json','candidate.json'):
        path=E/name;data=json.loads(path.read_text(encoding='utf-8'))
        data.update(observation_open_deg=250,observation_nominal_pulse_us=2352,
            preclose_deg=170,close_deg=140,carrier_open_deg=210,carrier_release_ms=300,
            source_step_max_mm=30,source_alignment_speed_mm_s=45,source_extra_wait_ms=0,
            pi_changed=False,unchanged_source_cap_release_and_protection_sha256=preserved,
            all_source_observation_visits_250=True,slot3_park_angle_unchanged=True,
            physical_observation_angle_and_sweep_validated=False)
        if name=='checks.json':
            data['actual_request_and_owner_regressions']={n:(E/(n+'.log')).read_text(encoding='utf-8').strip()
                for n in ('actual-observe250-dispatch','actual-return-observe250-failure','final-park-owner','owner-faults')}
            data['prior_30mm_300ms_evidence']={n:hashlib.sha256((previous/n).read_bytes()).hexdigest()
                for n in ('actual-source-step-bounds.log','actual-source-clip30.log','owner.log','checks.json')}
        path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
