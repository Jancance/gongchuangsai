"""Actual owner and group checks, dual Keil rebuild, no flash."""
import json,hashlib,shutil,subprocess
from pathlib import Path
from prepare_loaded_retract30_20261116 import R,BASE,P,E
import build_integration18_ext150_20261115 as build
import build_integration23_sync_20261115 as prior

GATE_TEST=r'''
#define main old_owner_main
#include "all-regions-owner.c"
#undef main
static unsigned prepared,started,stopped,poll_fault,complete_axes;
static uint32_t prep(void *p,uint32_t bit,const SPF_Command *c){
 (void)p;CHECK(c->goal.dm_deg==36.5f && c->goal.extension_mm==0 && c->goal.lift_pulses==1800);
 if(bit==SPF_DM)CHECK(vsp_group.release_mask&SPF_DM);
 prepared|=bit;return 1;
}
static uint32_t go(void *p,uint32_t bit,const SPF_Command *c){(void)p;(void)c;started|=bit;return 1;}
static uint32_t poll(void *p,uint32_t bit){(void)p;if(poll_fault&bit)return 2;return (complete_axes&bit)?1:0;}
static uint32_t stop(void *p,uint32_t bit){(void)p;stopped|=bit;return 1;}
static uint32_t clk(void *p){(void)p;return tick;}
static void sample(float mm,float pulses,unsigned age){
 extension=mm;depth=pulses;
 vsp_axis[0].current=-mm*VEHICLE_EXTENSION_PULSES_PER_MM*360/3200;
 vsp_axis[1].current=pulses*360/3200;
 vsp_axis[0].position_cycle=vsp_axis[1].position_cycle=1;
 vsp_axis[0].position_tick=vsp_axis[1].position_tick=tick-age;
}
static void group(unsigned ext){
 SPF_Command c;SPF_GroupIO io={0,prep,go,poll,stop,clk};unsigned i;
 reset();memset(&vsp_group,0,sizeof(vsp_group));memset(&c,0,sizeof(c));
 c.sequence=99;c.kind=SPF_CMD_TRANSFER;c.axes_mask=SPF_DM|SPF_LIFT|(ext?SPF_EXT:0);
 c.goal.dm_deg=36.5f;c.goal.lift_pulses=1800;c.dm_deg_s=SPF_DM_DEG_S;
 c.extension_rpm=SPF_EXT_RPM;c.lift_rpm=SPF_LIFT_RPM;c.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;c.keep_vm_on=1;
 CHECK(SPF_GroupBegin(&vsp_group,&c,&io,1,90,12000));
 vsp_group.delayed_mask=SPF_DM;vsp_group.release_mask=c.axes_mask&~SPF_DM;
 owned=1;low_group_sequence=99;g_low_state=1;g_low_phase=5;g_low_actual_pick_depth_pulses=29700;
 prepared=started=stopped=poll_fault=complete_axes=fail_position=reads_x=reads_z=0;
 for(i=0;i<5;i++)SPF_GroupService(&vsp_group);
 CHECK(vsp_group.state==2 && !(prepared&SPF_DM) && !(started&SPF_DM));
 sample(ext?80:0,1800,0);
}
static void gate(void){Low_ReleaseLoadedTransfer(tick);}
int main(void){unsigned i,ext;float xs[]={-30.001f,-30,-29.999f,0,29.999f,30,30.001f,NAN,INFINITY};
 for(i=0;i<sizeof(xs)/sizeof(xs[0]);i++){
  group(1);vsp_group.done=SPF_LIFT;sample(xs[i],1800,0);gate();
  CHECK(!!(vsp_group.release_mask&SPF_DM)==(isfinite(xs[i]) && fabsf(xs[i])<=30));
 }
 group(1);sample(20,1800,0);gate();CHECK(!(vsp_group.release_mask&SPF_DM)); /* lift not done */
 group(1);vsp_group.done=SPF_LIFT;sample(20,2000.1f,0);gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT;sample(20,1599.9f,0);gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT;sample(20,NAN,0);gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT;sample(20,1800,151);fail_position=1;gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT;sample(20,1800,0);vsp_axis[0].position_tick=tick-151;gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT;sample(20,1800,0);vsp_axis[0].position_cycle=0;gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT;sample(20,1800,0);low_group_sequence++;gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT;sample(20,1800,0);reference=0;gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT;sample(20,1800,0);g_low_actual_pick_depth_pulses=27800;gate();CHECK(!(vsp_group.release_mask&SPF_DM));
 /* Already-done Z stale counterexample is fixed by an actual owner read. */
 group(1);vsp_group.done=SPF_LIFT;sample(20,1800,0);vsp_axis[1].position_tick=tick-501;gate();
 CHECK(reads_z==1 && reads_x==0 && (vsp_group.release_mask&SPF_DM));
 CHECK(!(vsp_group.done&SPF_EXT) && (vsp_group.started&SPF_EXT));
 for(i=0;i<3;i++)SPF_GroupService(&vsp_group);
 CHECK((prepared&SPF_DM) && (started&SPF_DM) && vsp_group.state==2 && owned);
 complete_axes=SPF_DM;SPF_GroupService(&vsp_group);CHECK(vsp_group.state==2); /* X continues */
 complete_axes=SPF_DM|SPF_EXT;SPF_GroupService(&vsp_group);CHECK(vsp_group.state==3);
 /* Near-side no-EXT path must read actual X rather than assume home. */
 group(0);vsp_group.done=SPF_LIFT;sample(0,1800,0);vsp_axis[0].position_cycle=0;extension=31;gate();
 CHECK(reads_x==1 && !(vsp_group.release_mask&SPF_DM));
 tick+=151;extension=0;gate();CHECK(reads_x==2 && (vsp_group.release_mask&SPF_DM));
 /* Done X also gets refreshed, and zero/failed response cannot grant gate. */
 group(1);vsp_group.done=SPF_LIFT|SPF_EXT;sample(0,1800,151);gate();CHECK(reads_x==1 && reads_z==1 && (vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT|SPF_EXT;sample(0,1800,150);gate();CHECK(!reads_x && !reads_z && (vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT|SPF_EXT;sample(0,1800,151);fail_position=1;gate();CHECK(reads_z==1 && !reads_x && !(vsp_group.release_mask&SPF_DM));
 group(1);vsp_group.done=SPF_LIFT|SPF_EXT;sample(0,1800,0);vsp_axis[0].position_tick=tick-151;fail_position=1;gate();CHECK(reads_x==1 && !reads_z && !(vsp_group.release_mask&SPF_DM));
 for(ext=0;ext<2;ext++)for(i=0;i<2;i++){
  group(ext);if(i){vsp_group.done=SPF_LIFT;sample(20,1800,0);gate();SPF_GroupService(&vsp_group);}
  SPF_GroupCancel(&vsp_group,2);SPF_GroupService(&vsp_group);CHECK(stopped==vsp_group.command.axes_mask && vsp_group.state==4);
  prepared=0;gate();SPF_GroupService(&vsp_group);CHECK(!prepared);
 }
 group(1);poll_fault=SPF_EXT;SPF_GroupService(&vsp_group);CHECK(vsp_group.state==4);SPF_GroupService(&vsp_group);CHECK(stopped==7);
 printf("PASS actual owner gate + actual SPF group: %u checks; fresh30 boundary, fresh current-group samples, doneZ refresh, noEXT read, held DM never prepared, same movingX, all axes done, STOP/fault\n",checks);return 0;
}
'''

def main():
 previous=R/'evidence/integration/2026-11-16/observe210-tray36p5'
 for file in previous.iterdir():
  if file.suffix in ('.c','.h','.inc','.txt') or file.name=='parameter-and-poll-checks.json':shutil.copyfile(file,E/file.name)
 for name in ('all-regions-owner.c','base-commit-fixture.c'):
  path=E/name;t=path.read_text(encoding='utf-8')
  t=t.replace('CHECK(!(vsp_group.command.axes_mask&SPF_DM));','CHECK((vsp_group.command.axes_mask&SPF_DM) && vsp_group.delayed_mask==SPF_DM);')
  t=t.replace('CHECK(g_low_group_count==recorded);','CHECK(g_low_group_count==recorded && recorded==(region==2?3U:4U));\n CHECK(g_low_lift_retract_axes==(SPF_DM|SPF_LIFT|(region==2?0U:SPF_EXT)));')
  t=t.replace('static uint32_t VSP_Position(uint8_t address,VSP_Axis *a){','static unsigned fail_position,reads_x,reads_z;\nstatic uint32_t VSP_Position(uint8_t address,VSP_Axis *a){\n if(address==5)reads_z++;else reads_x++;if(fail_position)return 0;')
  path.write_text(t,encoding='utf-8')
 for name in ('owner-faults.c','continuous-owner-faults.c'):
  path=E/name;t=path.read_text(encoding='utf-8').replace('axes_mask==6U &&','axes_mask==7U &&');path.write_text(t,encoding='utf-8')
 (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(b'#include "low_readonly_diag.inc"',b'/* separately verified diagnostic */'))
 prior.P=P;prior.E=E
 # Existing owner regressions retain physical release/slot/300ms and fault assertions.
 for name in ('owner-faults','final-park-owner','actual-return-observe210-failure','actual-observe210-dispatch','actual-source-step-bounds','actual-source-clip30','actual-alignment-fast'):
  prior.run(name,(E/(name+'.c')).read_text(encoding='utf-8'))
 (E/'loaded-gate-and-group.c').write_text(GATE_TEST,encoding='utf-8')
 subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Wno-unused-function','-Wno-misleading-indentation','-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(E/'loaded-gate-and-group.c'),str(P/'CHASSIS/USER/source_parallel_flow.c'),'-lm','-o',str(E/'loaded-gate-and-group.exe')],check=True)
 r=subprocess.run([str(E/'loaded-gate-and-group.exe')],text=True,capture_output=True,check=True);print(r.stdout,flush=True);(E/'loaded-gate-and-group.log').write_text(r.stdout+r.stderr,encoding='utf-8')
 driver=Path(build.__file__).read_text(encoding='utf-8').replace('.read_text()',".read_text(encoding='utf-8')").replace("str(ROOT/'tests'/source)","str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
 scope={'__name__':'loaded_retract30_build','__file__':build.__file__};exec(compile(driver,build.__file__,'exec'),scope)
 scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()
 preserved={}
 for name in ('CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/source_body_step.inc','CHASSIS/USER/chassis_auto_test.c','CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/vehicle_source_parallel.inc','CHASSIS/USER/source_parallel_flow.c','USER/vehicle_parameters.h'):
  assert (P/name).read_bytes()==(BASE/name).read_bytes();preserved[name]=hashlib.sha256((P/name).read_bytes()).hexdigest()
 for name in ('candidate.json','checks.json'):
  path=E/name;data=json.loads(path.read_text(encoding='utf-8'));data.update(unchanged_sha256=preserved,observation_open_deg=210,preclose_deg=170,close_deg=140,carrier_open_deg=210,carrier_yaw_deg=36.5,carrier_release_ms=300,source_step_max_mm=30,source_alignment_speed_mm_s=45,group_count_per_piece={'side':4,'near':3},loaded_extension_gate_mm=30,safe_lift_target_pulses=1800,physical_sweep_validated=False,time_saving_validated=False)
  if name=='checks.json':data['actual_loaded_gate_and_group']=(E/'loaded-gate-and-group.log').read_text(encoding='utf-8').strip()
  path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
