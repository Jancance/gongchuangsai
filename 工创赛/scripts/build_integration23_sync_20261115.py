"""Actual-owner, actual queued-start/group and dual-target verification; no IO."""
from pathlib import Path
import json
import shutil
import subprocess
import build_integration18_ext150_20261115 as build
from prepare_integration23_sync_20261115 import R,BASE,P,E

def run(name,source,extra=()):
    f=E/(name+'.c');f.write_text(source,encoding='utf-8');exe=E/(name+'.exe')
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function',
        '-Wno-misleading-indentation','-I'+str(E),'-I'+str(R/'tests'),
        '-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(f),*map(str,extra),'-lm','-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],cwd=E,capture_output=True,text=True)
    (E/(name+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
    print(result.stdout+result.stderr,flush=True);assert result.returncode==0

def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    prior=R/'evidence/integration/2026-11-15/release100-overlap'
    for f in prior.iterdir():
        if f.suffix in ('.c','.h','.inc','.txt') or f.name=='parameter-and-poll-checks.json':
            shutil.copyfile(f,E/f.name)
    for name in ('all-regions-owner.c','base-commit-fixture.c'):
        f=E/name;s=f.read_text(encoding='utf-8')
        s=s.replace('vsp_fast_lift,vsp_pick_lead_ms;','vsp_fast_lift,vsp_pick_lead_ms,vsp_pick_sync;')
        s=s.replace('if(g_low_phase==4)CHECK(vsp_group.command.axes_mask==6 && vsp_group.command.goal.lift_pulses==29700);',
            'if(g_low_phase==4){CHECK(vsp_group.command.axes_mask==6 && vsp_group.command.goal.lift_pulses==29700);'
            'CHECK(region<2?(vsp_pick_sync && !vsp_group.delayed_mask):(!vsp_pick_sync && vsp_group.delayed_mask==SPF_LIFT));}')
        # Real backend resets the opt-in at each group; mirror that in the fixture.
        s=s.replace('owned=1;g_source_parallel_state=1;', 'vsp_pick_sync=0;owned=1;g_source_parallel_state=1;')
        f.write_text(s,encoding='utf-8')
    (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(
        b'#include "low_readonly_diag.inc"',b'/* separately verified diagnostic */'))
    for name in ('final-park-owner','owner-faults'):
        run(name,(prior/(name+'.c')).read_text(encoding='utf-8'))
    backend=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8')
    baseline=(BASE/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8')
    def prep(s):return s[s.index('static uint32_t VSP_Prepare('):s.index('static uint32_t VSP_PositionWindow(')]
    assert prep(backend)==prep(baseline),'Do not change speed/acceleration/physical goals'
    start=backend[backend.index('static uint32_t VSP_StartQueuedPick('):backend.index('static uint32_t VSP_Poll(')]
    policy=backend[backend.index('uint32_t VehicleArm_SourceParallelCanAllowed('):]
    axis=backend[backend.index('typedef struct {uint32_t phase'):backend.index('static VSP_Axis')]
    prefix=r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#define APP_MOTOR_OFFLINE_ONLY 0
static unsigned checks,tick,queued,moving,stops,calls,fail,trigger_count;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
'''+axis+r'''
static VSP_Axis vsp_axis[2];static SPF_Group vsp_group;
static unsigned vsp_pick_sync=1,vsp_pick_queued,vsp_owned=1,g_source_parallel_sync_tick,g_dm_parallel_group_go;
static struct{unsigned char Data[8];}s_rx;
static unsigned xTaskGetTickCount(void){return tick;}
static unsigned VehicleArm_Exchange(unsigned char addr,const unsigned char*q,unsigned n,unsigned rn,void*out){
 (void)out;CHECK(n==15 && rn==3 && q[13]==1 && q[14]==0x6b && (addr==5 || addr==6));
 CHECK(moving==0);calls++;tick+=80;if(fail==calls)return 4;
 queued|=addr==6?2:4;s_rx.Data[1]=2;return 1;
}
static unsigned CAN1_StartQueuedSourceAxes(void){
 CHECK(queued==6 && vsp_group.prepared==6 && vsp_group.started==6);
 if(fail==3)return 4;trigger_count++;moving=queued;return 1;
}
'''
    suffix=r'''
static unsigned prep(void*p,unsigned b,const SPF_Command*c){(void)p;(void)b;CHECK(c->extension_rpm==150 && c->lift_rpm==1846.4f && c->stepper_accel_rpm_s==600);return 1;}
static unsigned poll(void*p,unsigned b){(void)p;(void)b;return 1;}
static unsigned stop(void*p,unsigned b){(void)p;stops|=b;return 1;}
static unsigned clk(void*p){(void)p;return tick;}
static void setup(void){
 SPF_Command c={0};SPF_GroupIO io={0,prep,VSP_Start,poll,stop,clk};
 tick=queued=moving=stops=calls=trigger_count=vsp_pick_queued=0;memset(&vsp_group,0,sizeof(vsp_group));
 memset(vsp_axis,0,sizeof(vsp_axis));vsp_axis[0].goal=5000;vsp_axis[1].goal=3000;
 vsp_axis[0].move[14]=vsp_axis[1].move[14]=0x6b;
 c.axes_mask=6;c.kind=SPF_CMD_PICK;c.goal.extension_mm=162;c.goal.lift_pulses=29700;
 c.dm_deg_s=SPF_DM_DEG_S;c.extension_rpm=150;c.lift_rpm=1846.4f;c.stepper_accel_rpm_s=600;c.keep_vm_on=1;
 CHECK(SPF_GroupBegin(&vsp_group,&c,&io,1,90,12000));
}
int main(void){unsigned i;unsigned char q[3]={0xff,0x66,0x6b};
 for(fail=0;fail<=3;fail++){
  setup();for(i=0;i<8 && vsp_group.state<3;i++)SPF_GroupService(&vsp_group);
  if(!fail)CHECK(vsp_group.state==3 && moving==6 && trigger_count==1);
  else{SPF_GroupService(&vsp_group);CHECK(vsp_group.state==4 && stops==6 && !moving && !trigger_count);}
 }
 fail=0;setup();SPF_GroupService(&vsp_group);SPF_GroupCancel(&vsp_group,2);SPF_GroupService(&vsp_group);
 CHECK(stops==6 && !moving && !calls);
 CHECK(!VehicleArm_SourceParallelCanAllowed(0,q,3));
 vsp_group.state=1;vsp_group.started=6;vsp_pick_queued=6;
 CHECK(VehicleArm_SourceParallelCanAllowed(0,q,3));
 vsp_pick_sync=0;CHECK(!VehicleArm_SourceParallelCanAllowed(0,q,3));vsp_pick_sync=1;
 q[1]=0;CHECK(!VehicleArm_SourceParallelCanAllowed(0,q,3));
 printf("PASS actual queued-start/SPF group: %u checks; no movement before one broadcast; queue/trigger failure and STOP; scoped policy\n",checks);return 0;
}
'''
    run('actual-sync-group',prefix+start+policy+suffix,[P/'CHASSIS/USER/source_parallel_flow.c'])
    # Reuse the existing verified owner/wheel/group checks and Keil build driver.
    source=Path(build.__file__).read_text(encoding='utf-8').replace('.read_text()',".read_text(encoding='utf-8')")
    source=source.replace("str(ROOT/'tests'/source)","str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope={'__name__':'integration23_build','__file__':build.__file__}
    exec(compile(source,build.__file__,'exec'),scope)
    scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()
    f=E/'checks.json';d=json.loads(f.read_text());d.update(
        original_prepare_byte_identical=True,synchronized_regions=[0,1],near_unchanged=True,
        carrier_release_ms=100,source_pick_release_delay_added=False,
        physical_sweep_validated=False,physical_start_skew_validated=False,cycle25_validated=False)
    f.write_text(json.dumps(d,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':main()
