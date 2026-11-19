"""Actual profile/planner/VSP/frame checks; all device I/O is simulated."""
from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parents[1];U=R/'tmp/pickup_speed_trials_20261110/project/CHASSIS/USER'
E=R/'evidence/integration/2026-11-10/speed-trials'
def run(name,text,extra=()):
 src=E/(name+'.c');exe=E/(name+'.exe');src.write_text(text,encoding='utf-8')
 cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
  '-Wno-unused-function','-Wno-unused-variable','-I',str(U),str(src),*map(str,extra),'-lm','-o',str(exe)]
 p=subprocess.run(cmd,capture_output=True,text=True);log=p.stdout+p.stderr
 if p.returncode==0:
  p=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15);log+=p.stdout+p.stderr
 (E/(name+'.log')).write_text(log,encoding='utf-8');print(log,flush=True);assert p.returncode==0,name
 return log.strip()

def main():
 m=json.loads((E/'timing-model.json').read_text());assert m['passed']
 test=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pickup_s_curve.h"
#include "source_parallel_flow.h"
volatile uint32_t g_pick_trial_profile,g_pick_trial_requested,g_pick_trial_scurve;
#define CK(x) do{if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t tick;
static uint32_t prepare(void*p,uint32_t bit,const SPF_Command*c){(void)p;(void)bit;(void)c;return 1;}
static uint32_t begin(void*p,uint32_t bit,const SPF_Command*c){(void)p;(void)bit;(void)c;return 1;}
static uint32_t poll(void*p,uint32_t bit){(void)p;(void)bit;return 1;}
static uint32_t stop(void*p,uint32_t bit){(void)p;(void)bit;return 1;}
static uint32_t clock_ms(void*p){(void)p;return tick;}
int main(void){PickupSCurve plan;float x,v,last;SPF_Command c={0};SPF_Group g={0};
 SPF_GroupIO io={0,prepare,begin,poll,stop,clock_ms};
 for(uint32_t i=0;i<4;i++){
  float z[4]={1846.4f,2100,2500,2900},e[4]={106,200,300,400};g_pick_trial_profile=i;
  CK(fabsf(PickupTrial_LiftRpm()-z[i])<.01f && PickupTrial_ExtRpm()==e[i]);
  c.axes_mask=4;c.goal.lift_pulses=118800;c.dm_deg_s=300;c.extension_rpm=e[i];c.lift_rpm=z[i];
  c.stepper_accel_rpm_s=PickupTrial_ExtAccel();c.keep_vm_on=1;g.state=0;
  CK(SPF_GroupBegin(&g,&c,&io,1,90,12000));g.state=0;c.lift_rpm+=1;
  CK(!SPF_GroupBegin(&g,&c,&io,1,90,12000));
 }
 g_pick_trial_profile=4;CK(!SPF_GroupBegin(&g,&c,&io,1,90,12000));
 CK(!PickupSCurve_Begin(&plan,10,3001,45055,.02f));
 CK(!PickupSCurve_Begin(&plan,10,2900,0,.02f));
 CK(!PickupSCurve_Begin(&plan,NAN,2900,45055,.02f));
'''
 for fixture in m['s_curve_fixtures']:
  test+=' CK(PickupSCurve_Begin(&plan,%.12ff,%.1ff,45055,.02f));\n'%(fixture['distance_rev'],fixture['rpm'])
  for t,x,v in zip(fixture['time_s'],fixture['position_rev'],fixture['velocity_rev_s']):
   test+=' PickupSCurve_Eval(&plan,%.12ff,&x,&v);CK(fabsf(x-%.12ff)<.0002f && fabsf(v-%.12ff)<.002f);\n'%(t,x,v)
  test+=r'''
 last=0;
 for(uint32_t i=0;i<=1000;i++){
  PickupSCurve_Eval(&plan,plan.total*(float)i/1000,&x,&v);
  CK(x>=last-.0001f && x<=plan.distance+.0001f && v*60<=plan.rpm+.05f);last=x;
 }
 PickupSCurve_Eval(&plan,plan.total,&x,&v);CK(v==0 && fabsf(x-plan.distance)<.0001f);
'''
 test+=' puts("PASS: actual C profiles andS-curve planner versus MATLAB75 points, monotone bounds, endpoint zero-speed, exact paired speed admission. SIMULATED GROUP.");return 0;}\n'
 logs={'planner':run('profiles-planner',test,[U/'source_parallel_flow.c'])}
 # Reuse the accepted port fixture, exercising the exact new prepare/FB code.
 source=(R/'scripts/check_stage2_fast_port_20261109.py').read_text(encoding='utf-8')
 source=source[:source.index("src = OUT / 'stage2-fast-port-test.c'")]
 ns={'__file__':str(R/'scripts/check_stage2_fast_port_20261109.py')}
 source=source.replace("P = R / 'tmp/stage2_three_overlap_20261109/project/CHASSIS/USER'", "P = R / 'tmp/pickup_speed_trials_20261110/project/CHASSIS/USER'")
 # Existing script uses ROOT/P names; inspect and replace the actual owning path.
 source=source.replace("P = ROOT / 'tmp/stage2_sub20_20261109/project/CHASSIS/USER'", "P = ROOT / 'tmp/pickup_speed_trials_20261110/project/CHASSIS/USER'")
 exec(compile(source,'port-fixture','exec'),ns);unit=ns['unit']
 vsp=(U/'vehicle_source_parallel.inc').read_text(encoding='utf-8',errors='replace')
 whitelist=vsp[vsp.index('uint32_t VehicleArm_SourceParallelCanAllowed'):]
 unit=unit.replace('int main(void){',whitelist+'\nint main(void){',1)
 unit=unit.replace('static uint32_t tick=50',
  'volatile uint32_t g_pick_trial_profile,g_pick_trial_requested,g_pick_trial_scurve;\nstatic uint32_t tick=50')
 unit=unit.replace(' if(q[0]==0xf3 || q[0]==0xfe)s_rx.Data[1]=2;',
  ' if(q[0]==0xf3 || q[0]==0xfe || q[0]==0xfb)s_rx.Data[1]=2;')
 extra=r'''
 for(uint32_t k=1;k<=3;k++){
  g_pick_trial_profile=k;g_pick_trial_scurve=0;memset(vsp_axis,0,sizeof vsp_axis);
  c.lift_rpm=PickupTrial_LiftRpm();c.extension_rpm=PickupTrial_ExtRpm();c.goal.lift_pulses=6400;c.stepper_accel_rpm_s=PickupTrial_ExtAccel();
  options=128;raw=36000;sign=0;g_lift_ruler_controller.top_scale=100;vsp_fast_lift=1;
  assert(!VSP_Prepare(0,4,&c));assert(!VSP_Prepare(0,4,&c));
  assert(u16(vsp_axis[1].move+6)==(uint32_t)(c.lift_rpm*10+.5f) && u16(vsp_axis[1].move+2)==45055);
  memset(vsp_axis,0,sizeof vsp_axis);raw=0;
  assert(!VSP_Prepare(0,2,&c));assert(!VSP_Prepare(0,2,&c));
  assert(u16(vsp_axis[0].move+6)==(uint32_t)(c.extension_rpm*10+.5f) && u16(vsp_axis[0].move+2)==PickupTrial_ExtAccel());
 }
 g_pick_trial_profile=3;g_pick_trial_scurve=1;tick=1000;vsp_group.state=2;vsp_group.done=0;
 vsp_group.command.lift_rpm=2900;vsp_axis[1].before=270;vsp_axis[1].goal=13365;vsp_axis[1].options=128;
 assert(VSP_SCurveStart(tick));assert(vsp_scurve_frame[0]==0xfb && u16(vsp_scurve_frame+2)==29000);
 assert(vsp_scurve_frame[8]==1 && vsp_scurve_frame[10]==0x6b);
 tick+=20;assert(VSP_SCurveService() && g_pick_scurve_reference_deg>270);
 tick=1000+(uint32_t)(vsp_scurve.total*1000)+2;
 assert(VSP_SCurveService() && fabsf(g_pick_scurve_reference_deg-13365)<.05f);
 vsp_owned=1;vsp_group.command.axes_mask=4;vsp_group.started=4;
 assert(VehicleArm_SourceParallelCanAllowed(5,vsp_scurve_frame,11));
 uint8_t bad[11];memcpy(bad,vsp_scurve_frame,11);bad[6]^=1;
 assert(!VehicleArm_SourceParallelCanAllowed(5,bad,11));
 assert(!VehicleArm_SourceParallelCanAllowed(6,vsp_scurve_frame,11));
 vsp_group.state=4;uint32_t frames=g_pick_scurve_frames;tick+=40;
 assert(VSP_SCurveService() && frames==g_pick_scurve_frames);
 assert(!VehicleArm_SourceParallelCanAllowed(5,vsp_scurve_frame,11));
 memset(vsp_axis,0,sizeof vsp_axis);options=4;raw=0;
 assert(VSP_Prepare(0,2,&c)==2); /* Scale changed: reject the taught trial */
'''
 unit=unit.replace(' puts("PASS: actual VSP',extra+'\n puts("PASS: actual VSP')
 logs['port']=run('port-frames',unit)
 (E/'host-checks.json').write_text(json.dumps({'passed':True,'results':logs,'hardware_access':False,
  'real_motor_following_verified':False},indent=2)+'\n')
if __name__=='__main__':main()
