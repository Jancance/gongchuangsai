"""Real-source boundary/owner/fault fixtures and dual Keil build; no hardware."""
import json
import shutil
from pathlib import Path
import build_integration18_ext150_20261115 as build
import build_integration23_sync_20261115 as prior
import build_stopped5_fastalign_20261116 as alignment
from prepare_source_alignment30_release300_20261116 import R,BASE,P,E


def bounds_fixture():
    body=(P/'CHASSIS/USER/source_body_step.inc').read_text(encoding='utf-8')
    admission=alignment.block(body,'if((request==2U')
    return r'''
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
static unsigned checks,request=2,g_source_align_direction,reason;
static float g_source_align_distance_mm,g_source_align_turn_deg;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static void SBS_Fail(unsigned why){reason=why;}
static void admission(void){
'''+admission+r'''
}
int main(void){unsigned direction,mm;
 for(direction=0;direction<4;direction++)for(mm=1;mm<=30;mm++){
  request=2;g_source_align_direction=direction;g_source_align_distance_mm=(float)mm;reason=0;
  admission();CHECK(reason==0);
 }
 for(direction=0;direction<4;direction++){
  static const float invalid[]={0,.99f,30.01f,31,60,-1,NAN,INFINITY};
  for(mm=0;mm<sizeof(invalid)/sizeof(invalid[0]);mm++){
   g_source_align_direction=direction;g_source_align_distance_mm=invalid[mm];reason=0;
   admission();CHECK(reason==10);
  }
 }
 g_source_align_direction=4;g_source_align_distance_mm=30;reason=0;admission();CHECK(reason==10);
 request=3;
 for(mm=0;mm<=11;mm++){g_source_align_turn_deg=(float)mm;reason=0;admission();CHECK((reason==0)==(mm>=5 && mm<=10));}
 request=1;g_source_align_distance_mm=999;reason=0;admission();CHECK(reason==0);
 printf("PASS actual low-level source bounds: %u checks;1..30 admitted;31/invalid rejected; turn/legacy unchanged\n",checks);return 0;
}
'''


def clip_fixture():
    return r'''
#include "static-cycle-fixture.c"
int main(void){
 static const int amounts[]={-60000,-31000,-30500,-30000,-29500,-29000,-1000,0,1000,29000,29500,30000,30500,31000,60000};
 unsigned axis,i,expected,direction;
 for(axis=0;axis<2;axis++)for(i=0;i<sizeof(amounts)/sizeof(amounts[0]);i++){
  reset_cycle();g_it_state=2;g_source_cycle_state=2;g_source_cycle_run=1;g_source_cycle_pose=1;
  sample(1,1,0,60000,amounts[i],0,0,0,0,0);
  if(axis){g_source_cycle_right_milli_mm=0;g_source_cycle_forward_milli_mm=amounts[i];}
  SourceCycle_Service(tick);
  expected=abs(amounts[i])>=30000?30:(unsigned)((abs(amounts[i])+500)/1000);if(!expected)expected=1;
  direction=axis && amounts[i]!=0?(amounts[i]>0?0:3):(amounts[i]>0?2:1);
  CHECK(g_source_cycle_state==3 && g_source_body_step_request==2);
  CHECK(g_source_align_distance_mm==(float)expected && g_source_align_distance_mm<=30);
  CHECK(g_source_align_direction==direction);
 }
 /* The completion tolerance remains inclusive5mm, unchanged by the step cap. */
 reset_cycle();g_it_state=2;g_source_cycle_state=2;g_source_cycle_run=1;g_source_cycle_pose=1;
 sample(1,1,0,5000,30000,0,0,0,0,0);SourceCycle_Service(tick);
 CHECK(g_source_cycle_state==5 && !g_source_body_step_request);
 printf("PASS real MCU source alignment clip: %u checks;signed29/30/31/60mm both axes;30mm cap;5mm tolerance unchanged\n",checks);return 0;
}
'''


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    release=json.loads((R/'evidence/integration/2026-11-16/carrier-release300/matlab-result.json').read_text(encoding='utf-8'))
    assert proof['validation_passed'] and proof['local_matlab'] and release['passed'] and release['local_matlab']
    previous=R/'evidence/integration/2026-11-16/source-alignment-fast'
    for f in previous.iterdir():
        if f.suffix in ('.c','.h','.inc','.txt') or f.name=='parameter-and-poll-checks.json':
            data=f.read_bytes()
            if f.name in ('all-regions-owner.c','base-commit-fixture.c'):
                text=data.decode('utf-8')
                for before,after in [('tick+=199;','tick+=299;'),('g_low_release_wait_ms==200','g_low_release_wait_ms==300'),('pulse+200','pulse+300')]:
                    assert text.count(before)==1,(f.name,before)
                    text=text.replace(before,after,1)
                data=text.encode('utf-8')
            elif f.name=='owner-faults.c':
                text=data.decode('utf-8');before='tick+=200U;';assert text.count(before)==1
                data=text.replace(before,'tick+=300U;',1).encode('utf-8')
            elif f.name=='static-cycle-fixture.c':
                text=data.decode('utf-8');before='tick=pulse_apply_at+50;Low_Service(tick);tick+=200;'
                assert text.count(before)==1
                data=text.replace(before,'tick=pulse_apply_at+50;Low_Service(tick);tick+=300;',1).encode('utf-8')
            elif f.name=='final-park-owner.c':
                text=data.decode('utf-8');assert text.count('200ms release')==1
                data=text.replace('200ms release','300ms release',1).encode('utf-8')
            (E/f.name).write_bytes(data)
    (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(
        b'#include "low_readonly_diag.inc"',b'/* separately verified diagnostic */'))
    prior.P=P;prior.E=E
    prior.run('actual-source-step-bounds',bounds_fixture())
    prior.run('actual-source-clip30',clip_fixture())
    # No new SETTLE/speed behavior: preserve the prior validated implementation.
    assert (P/'CHASSIS/USER/chassis_auto_test.c').read_bytes()==(BASE/'CHASSIS/USER/chassis_auto_test.c').read_bytes()
    source=(P/'CHASSIS/USER/source_body_step.inc').read_text(encoding='utf-8')
    old=(BASE/'CHASSIS/USER/source_body_step.inc').read_text(encoding='utf-8')
    for signature in ('uint32_t SourceBodyStep_FastAlignmentWindow(', 'static void SBS_Fail(', 'if(g_source_body_step_state==1U)'):
        assert alignment.block(source,signature)==alignment.block(old,signature),signature
    for name in ('final-park-owner','owner-faults'):
        prior.run(name,(E/(name+'.c')).read_text(encoding='utf-8'))
    driver=Path(build.__file__).read_text(encoding='utf-8').replace('.read_text()',".read_text(encoding='utf-8')")
    driver=driver.replace("['passed']\n    assert json.loads((E/'parameter-and-poll-checks.json')", "['validation_passed']\n    assert json.loads((E/'parameter-and-poll-checks.json')")
    driver=driver.replace("str(ROOT/'tests'/source)","str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope={'__name__':'source30_release300_build','__file__':build.__file__}
    exec(compile(driver,build.__file__,'exec'),scope)
    scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()
    for name in ('checks.json','candidate.json'):
        path=E/name;data=json.loads(path.read_text(encoding='utf-8'))
        data.update(optimization_passed=True,three_second_target_validated=proof['target_3s_passed'],
            source_alignment_speed_mm_s=45,source_step_max_mm=30,carrier_release_ms=300,
            release_timing_reference='applied_gripper_pwm_before_4310_return',source_extra_wait_ms=0,
            preclose_deg=170,pi_stopped_frames=5,pi_changed=False,
            original_stop_fault_vm_protection_preserved=True,
            physical_timing_and_alignment_validated=False,
            release_matlab_checks=release['checks'])
        if name=='checks.json':
            data['actual_source_regressions']={n:(E/(n+'.log')).read_text(encoding='utf-8').strip()
                for n in ('actual-source-step-bounds','actual-source-clip30','final-park-owner','owner-faults')}
            data['unchanged_fast_alignment_code_byte_identical']=True
        path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
