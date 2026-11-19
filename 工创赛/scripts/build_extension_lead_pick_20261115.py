"""Regression and dual build for original-speed extension-leading pickup."""
from pathlib import Path
import json
import subprocess
import build_continuous_arm_pick_20261115 as prior

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/extension-lead-pick'
P=R/'tmp/source_extension_lead_pick_20261115/project'


def main():
    proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed']
    prior.P=P;prior.E=E
    # Reuse original owner's three-region/three-slot and fault fixtures, but
    # exclude all obsolete variable-speed math/packet tests.
    text=(R/'scripts/build_continuous_arm_pick_20261115.py').read_text()
    start=text.index('    # Compare C timing');end=text.index('    # Existing driver/poll/STOP checks',start)
    text=text[:start]+text[end:]
    old="fixture = fixture.replace('records[16]', 'records[32]').replace('recorded<16', 'recorded<32')"
    assert text.count(old)==1
    text=text.replace(old,old+"\n    fixture = fixture.replace('static uint32_t vsp_fast_lift;', 'static uint32_t vsp_fast_lift,vsp_pick_lead_ms;\\n#include \\\"source_pick_extension_lead_20261115.h\\\"')",1)
    scope={'__name__':'extension_lead_build','__file__':str(R/'scripts/build_continuous_arm_pick_20261115.py')}
    exec(compile(text,scope['__file__'],'exec'),scope)
    scope.update(P=P,E=E,BASE=R/'tmp/source_mcu_rimpark_20261115/project')
    # Before building, prove actual VSP prepare/packet construction is byte
    # identical to the deployed baseline: speed and acceleration unchanged.
    old=(scope['BASE']/'CHASSIS/USER/vehicle_source_parallel.inc').read_text()
    new=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_text()
    def prepare(s):return s[s.index('static uint32_t VSP_Prepare('):s.index('static uint32_t VSP_PositionWindow(')]
    assert prepare(old)==prepare(new)
    scope['main']()
    # Test actual timed release together with actual generic group handling.
    helper=new[new.index('static void VSP_ReleasePickDescent('):new.index('static void VehicleSourceParallel_Service(')]
    fixture=r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "source_parallel_flow.h"
#include "source_pick_extension_lead_20261115.h"
static unsigned checks,tick,started,stops,bad;
static SPF_Group vsp_group;static uint32_t vsp_pick_lead_ms;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t prep(void*a,uint32_t bit,const SPF_Command*c){(void)a;(void)bit;(void)c;return 1;}
static uint32_t start(void*a,uint32_t bit,const SPF_Command*c){(void)a;CHECK(c->lift_rpm==1846.4f && c->extension_rpm==150.0f && c->stepper_accel_rpm_s==600);started|=bit;return 1;}
static uint32_t poll(void*a,uint32_t bit){(void)a;(void)bit;return bad?2:0;}
static uint32_t stop(void*a,uint32_t bit){(void)a;stops|=bit;return 1;}
static uint32_t clk(void*a){(void)a;return tick;}
'''+helper+r'''
static void setup(unsigned region){SPF_Command c={0};SPF_GroupIO io={0,prep,start,poll,stop,clk};
 memset(&vsp_group,0,sizeof(vsp_group));started=stops=bad=0;tick=100;
 c.axes_mask=6;c.kind=SPF_CMD_PICK;c.goal.lift_pulses=29700;c.goal.extension_mm=162;
 c.dm_deg_s=300;c.extension_rpm=150;c.lift_rpm=1846.4f;c.stepper_accel_rpm_s=600;c.keep_vm_on=1;
 CHECK(SPF_GroupBegin(&vsp_group,&c,&io,1,90,12000));
 vsp_pick_lead_ms=SourcePick_ExtensionLeadMs(region);vsp_group.delayed_mask=4;vsp_group.release_mask=2;
 SPF_GroupService(&vsp_group);SPF_GroupService(&vsp_group);CHECK(started==2 && !stops);
}
int main(void){unsigned region;
 for(region=0;region<3;region++){
  setup(region);tick=vsp_group.first_start_ms+vsp_pick_lead_ms-1;
  VSP_ReleasePickDescent(tick);SPF_GroupService(&vsp_group);CHECK(started==2);
  tick++;VSP_ReleasePickDescent(tick);SPF_GroupService(&vsp_group);SPF_GroupService(&vsp_group);
  CHECK(started==6 && !stops);
  setup(region);SPF_GroupCancel(&vsp_group,2);VSP_ReleasePickDescent(tick+2000);SPF_GroupService(&vsp_group);
  CHECK(started==2 && stops==6 && vsp_group.state==4);
  setup(region);bad=1;SPF_GroupService(&vsp_group);SPF_GroupService(&vsp_group);
  CHECK(started==2 && stops==6 && vsp_group.state==4);
 }
 printf("PASS actual timed-release/group: %u checks; original speed bytes, left/right/near lead, no Z before due, STOP/fault cancels queued Z\n",checks);return 0;}
'''
    file=E/'actual-release-group.c';file.write_text(fixture,encoding='utf-8');exe=E/'actual-release-group.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation',
        '-I'+str(P/'CHASSIS/USER'),str(file),str(P/'CHASSIS/USER/source_parallel_flow.c'),'-lm','-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    (E/'actual-release-group.log').write_text(result.stdout+result.stderr,encoding='utf-8');print(result.stdout+result.stderr)
    assert result.returncode==0
    checks=json.loads((E/'checks.json').read_text());checks.update(original_vsp_prepare_byte_identical=True,
        actual_timed_release_and_stop_test_passed=True,extension_lead_ms_by_region=proof['config']['extension_lead_ms_by_region'],
        flashed=False,hardware_access=False)
    (E/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')


if __name__=='__main__':main()
