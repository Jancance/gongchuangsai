"""Actual profile/planner/VSP/frame checks; all device I/O is simulated."""
from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parents[1];U=R/'tmp/pickup_smooth_tail_20261110/project/CHASSIS/USER'
E=R/'evidence/integration/2026-11-10/smooth-tail'
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
 m=json.loads((E/'timing-model.json').read_text(encoding='utf-8'));assert m['passed']
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
  uint32_t dec[4]={600,400,1200,2000};CK(PickupTrial_ExtDecel()==dec[i]);
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
 source=source.replace("P = R / 'tmp/stage2_three_overlap_20261109/project/CHASSIS/USER'", "P = R / 'tmp/pickup_smooth_tail_20261110/project/CHASSIS/USER'")
 # Existing script uses ROOT/P names; inspect and replace the actual owning path.
 source=source.replace("P = ROOT / 'tmp/stage2_sub20_20261109/project/CHASSIS/USER'", "P = ROOT / 'tmp/pickup_smooth_tail_20261110/project/CHASSIS/USER'")
 exec(compile(source,'port-fixture','exec'),ns);unit=ns['unit']
 vsp=(U/'vehicle_source_parallel.inc').read_text(encoding='utf-8',errors='replace')
 whitelist=vsp[vsp.index('uint32_t VehicleArm_SourceParallelCanAllowed'):]
 poll=vsp[vsp.index('static uint32_t VSP_Start'):vsp.index('static uint32_t VSP_Stop')]
 poll+=vsp[vsp.index('static uint32_t VSP_Clock'):vsp.index('static uint32_t VehicleSourceParallel_Owns')]
 unit=unit.replace('static uint32_t VSP_Stop',poll+'\nstatic uint32_t VSP_Stop',1)
 unit=unit.replace('static uint32_t mock_speed,','static uint32_t mock_flags=1;\nstatic uint32_t mock_speed,',1)
 unit=unit.replace(' if(q[0]==0x1a)', ' if(q[0]==0x3a)s_rx.Data[1]=(uint8_t)mock_flags;\n if(q[0]==0x1a)',1)
 unit=unit.replace('int main(void){',whitelist+'\nint main(void){',1)
 unit=unit.replace('static uint32_t it_empty_overlap_begin', 'static uint32_t VehicleSourceParallel_Owns(void);\nstatic uint32_t it_empty_overlap_begin',1)
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
  assert(u16(vsp_axis[1].move+4)==45055U);
  memset(vsp_axis,0,sizeof vsp_axis);raw=0;
  assert(!VSP_Prepare(0,2,&c));assert(!VSP_Prepare(0,2,&c));
  assert(u16(vsp_axis[0].move+6)==(uint32_t)(c.extension_rpm*10+.5f) && u16(vsp_axis[0].move+2)==PickupTrial_ExtAccel());
  assert(u16(vsp_axis[0].move+4)==PickupTrial_ExtDecel());
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

 # Test the exact outbound whitelist and unchanged overrun decision.
 extra+=r"""
 g_pick_trial_profile=1;g_pick_trial_scurve=0;vsp_fast_lift=1;
 memset(vsp_axis,0,sizeof vsp_axis);options=132;raw=0;sign=0;
 c.extension_rpm=200;c.lift_rpm=2100;c.stepper_accel_rpm_s=600;c.goal.extension_mm=156;
 assert(!VSP_Prepare(0,2,&c));assert(!VSP_Prepare(0,2,&c));
 assert(u16(vsp_axis[0].move+2)==600 && u16(vsp_axis[0].move+4)==400 && u16(vsp_axis[0].move+6)==2000);
 vsp_owned=1;vsp_group.state=1;vsp_group.started=2;vsp_group.command.axes_mask=2;
 assert(VehicleArm_SourceParallelCanAllowed(6,vsp_axis[0].move,15));
 uint8_t wrong[15];memcpy(wrong,vsp_axis[0].move,15);wrong[5]^=1;
 assert(!VehicleArm_SourceParallelCanAllowed(6,wrong,15));
 vsp_axis[0].before=0;vsp_axis[0].goal=-5537.02490234375f;vsp_axis[0].current=-5610.39990234375f;
 assert(!VSP_PositionWindow(&vsp_axis[0],0));
 vsp_axis[0].current=vsp_axis[0].goal-59.9f;assert(VSP_PositionWindow(&vsp_axis[0],0));
 assert(VSP_Stop(0,SPF_EXT));
 """

 extra+=r"""
 g_pick_trial_profile=1;vsp_group.command.extension_rpm=200;
 vsp_group.command.stepper_accel_rpm_s=600;vsp_group.command.goal.extension_mm=156;
 vsp_axis[0].before=-1;vsp_axis[0].goal=-5537.02490234375f;
 assert(VSP_SettleAllowed(6,0));assert(!VSP_SettleAllowed(5,0));
 for(uint32_t profile=0;profile<4;profile++){
  g_pick_trial_profile=profile;assert(VSP_SettleAllowed(6,0)==(profile==1));
 }
 g_pick_trial_profile=1;vsp_group.command.goal.extension_mm=100;
 assert(!VSP_SettleAllowed(6,0));vsp_group.command.goal.extension_mm=156;
 vsp_axis[0].phase=1;vsp_axis[0].options=132;vsp_axis[0].flags=1;vsp_axis[0].overshoot_active=0;
 sign=1;raw=55979;tick=1000;
 assert(!VSP_Poll(0,SPF_EXT) && vsp_axis[0].overshoot_active && vsp_axis[0].phase==2);
 vsp_axis[0].phase=1;tick=1149;assert(!VSP_Poll(0,SPF_EXT));
 vsp_axis[0].phase=1;tick=1150;assert(VSP_Poll(0,SPF_EXT)==2);
 vsp_axis[0].overshoot_active=0;vsp_axis[0].phase=1;raw=56272;tick=1200;
 assert(VSP_Poll(0,SPF_EXT)==2); /* >90 deg is immediate STOP */
 g_pick_trial_profile=2;vsp_axis[0].phase=1;raw=55979;
 assert(VSP_Poll(0,SPF_EXT)==2); /* no wider window for untested fast profiles */
 g_pick_trial_profile=1;vsp_axis[0].phase=0;mock_flags=9;
 assert(VSP_Poll(0,SPF_EXT)==2); /* drive protection still immediate */
 vsp_axis[1].phase=1;vsp_axis[1].options=132;vsp_axis[1].before=810;vsp_axis[1].goal=13365;raw=1342600;sign=0;
 assert(VSP_Poll(0,SPF_LIFT)==2); /* lift window unchanged */
 /* Return inside45deg still needs three fresh zero-speed/done samples. */
 sign=1;raw=55370;tick=1300;mock_flags=3;mock_speed=mock_speed_sign=mock_speed_miss=0;
 vsp_axis[0].phase=0;vsp_axis[0].samples=0;vsp_axis[0].overshoot_active=1;
 for(uint32_t sample=0;sample<3;sample++){
  assert(!VSP_Poll(0,SPF_EXT));assert(!VSP_Poll(0,SPF_EXT));
  assert(VSP_Poll(0,SPF_EXT)==(sample==2));
 }
 assert(!vsp_axis[0].overshoot_active && vsp_axis[0].samples==3);
 assert(VSP_Stop(0,SPF_EXT));
 """

 extra+=r"""
 /* Both high gears, both directions, and both taught reaches. */
 for(uint32_t gear=2;gear<=3;gear++)for(uint32_t side=0;side<2;side++)for(uint32_t returning=0;returning<2;returning++){
  float mm=side?160.0f:156.0f;float reach=mm*315.5f*360.0f/3200.0f;
  g_pick_trial_profile=gear;g_pick_trial_scurve=0;g_extension_home_reference_deg=0;
  c.extension_rpm=PickupTrial_ExtRpm();c.lift_rpm=PickupTrial_LiftRpm();c.stepper_accel_rpm_s=PickupTrial_ExtAccel();c.goal.extension_mm=returning?0.0f:mm;
  memset(vsp_axis,0,sizeof vsp_axis);options=132;sign=returning;raw=returning?(uint32_t)(reach*10+.5f):0;
  assert(!VSP_Prepare(0,SPF_EXT,&c));assert(!VSP_Prepare(0,SPF_EXT,&c));
  assert(g_pick_extension_segment==1 && fabsf(g_pick_extension_waypoint_deg-vsp_axis[0].goal)>.99f*VSP_EXT_TAIL_DEG);
  assert(u16(vsp_axis[0].move+6)==(uint32_t)(c.extension_rpm*10+.5f));
  vsp_owned=1;vsp_group.state=2;vsp_group.command=c;vsp_group.command.axes_mask=2;vsp_group.started=2;vsp_group.done=0;
  vsp_axis[0].phase=0;mock_flags=3;mock_speed=mock_speed_sign=mock_speed_miss=0;
  raw=(uint32_t)(fabsf(g_pick_extension_waypoint_deg)*10+.5f);sign=g_pick_extension_waypoint_deg<0;
  for(uint32_t sample=0;sample<3;sample++){
   assert(!VSP_Poll(0,SPF_EXT));assert(!VSP_Poll(0,SPF_EXT));assert(!VSP_Poll(0,SPF_EXT));
  }
  assert(vsp_axis[0].phase==3 && g_pick_extension_segment==1 && !vsp_group.done);
  assert(!VSP_Poll(0,SPF_EXT)); /* tail command; still not DONE */
  assert(g_pick_extension_segment==2 && !vsp_axis[0].samples && vsp_axis[0].phase==0);
  assert(u16(vsp_axis[0].move+6)==1060 && u16(vsp_axis[0].move+2)==600 && u16(vsp_axis[0].move+4)==600);
  assert(vsp_axis[0].move[1]==(!returning));
  raw=(uint32_t)(fabsf(vsp_axis[0].goal)*10+.5f);sign=vsp_axis[0].goal<0;
  for(uint32_t sample=0;sample<3;sample++){
   assert(!VSP_Poll(0,SPF_EXT));assert(!VSP_Poll(0,SPF_EXT));assert(VSP_Poll(0,SPF_EXT)==(sample==2));
  }
 }
 /* Boundary remains the full taught endpoint, not the intermediate point. */
 g_pick_trial_profile=2;g_pick_extension_segment=1;vsp_group.command.goal.extension_mm=156;
 vsp_axis[0].before=0;vsp_axis[0].goal=-5537.025f;vsp_axis[0].phase=1;raw=57366;sign=1;
 assert(VSP_Poll(0,SPF_EXT)==2); /* actual second-gear199deg overrun still rejected */
 g_it_state=1;g_it_action=14;vsp_group.delayed_mask=SPF_LIFT;vsp_group.release_mask=SPF_EXT;vsp_group.started=SPF_EXT;
 it_empty_overlap_gates();assert(vsp_group.release_mask==SPF_EXT);
 g_pick_extension_segment=2;it_empty_overlap_gates();assert(vsp_group.release_mask==(SPF_EXT|SPF_LIFT));
 /* First fresh zero-speed full-target sample may close; old/stage1 may not. */
 it_command.axes_mask=6;vsp_group.done=0;vsp_axis[0].samples=1;vsp_axis[0].flags=3;
 vsp_axis[0].current=vsp_axis[0].goal;vsp_axis[0].position_cycle=1;vsp_axis[0].position_tick=tick;
 assert(it_grip_extension_ready(tick));assert(!it_grip_extension_ready(tick+101));
 g_pick_extension_segment=1;assert(!it_grip_extension_ready(tick));g_pick_extension_segment=2;
 vsp_axis[0].samples=0;assert(!it_grip_extension_ready(tick));vsp_axis[0].samples=1;
 vsp_axis[0].flags=9;assert(!it_grip_extension_ready(tick));vsp_axis[0].flags=3;
 vsp_axis[0].current= vsp_axis[0].goal+45.1f;assert(!it_grip_extension_ready(tick));
 """
 unit=unit.replace(' puts("PASS: actual VSP',extra+'\n puts("PASS: actual VSP')

 unit=unit.replace('static uint32_t mock_flags=1;', 'static uint32_t mock_device_mode,mock_ext_moves,mock_lift_moves;\nstatic int32_t mock_ext_raw,mock_lift_raw;\nstatic uint32_t mock_flags=1;')
 unit=unit.replace(' if(q[0]==0x36){',r'''
 if(mock_device_mode && q[0]==0x36){int32_t v=addr==6?mock_ext_raw:mock_lift_raw;sign=v<0;raw=(uint32_t)(v<0?-v:v);}
 if(mock_device_mode && q[0]==0xfd){
  uint32_t u=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  int32_t delta=q[1]?-(int32_t)u:(int32_t)u;
  if(addr==6){mock_ext_raw+=delta;mock_ext_moves++;}
  else{assert(mock_ext_moves==2);mock_lift_raw+=delta;mock_lift_moves++;}
  s_rx.Data[1]=2;
 }
 if(q[0]==0x36){''',1)
 pipeline=r"""
 {
  SPF_GroupIO io={0,VSP_Prepare,VSP_Start,VSP_Poll,VSP_Stop,VSP_Clock};
  memset(&vsp_group,0,sizeof vsp_group);memset(vsp_axis,0,sizeof vsp_axis);
  g_pick_trial_profile=2;g_pick_trial_scurve=0;g_pick_extension_segment=0;
  c.axes_mask=6;c.goal.extension_mm=156;c.goal.lift_pulses=118800;c.goal.dm_deg=195;
  c.extension_rpm=300;c.lift_rpm=2500;c.dm_deg_s=300;c.stepper_accel_rpm_s=1200;c.keep_vm_on=1;
  mock_device_mode=1;mock_ext_moves=mock_lift_moves=0;mock_ext_raw=0;mock_lift_raw=27000;
  mock_flags=3;mock_speed=mock_speed_sign=mock_speed_miss=0;options=132;vsp_fast_lift=1;g_lift_ruler_controller.top_scale=100;
  assert(SPF_GroupBegin(&vsp_group,&c,&io,1,90,12000));
  vsp_group.delayed_mask=SPF_LIFT;vsp_group.release_mask=SPF_EXT;vsp_owned=1;g_it_state=1;g_it_action=14;
  for(uint32_t k=0;k<500 && vsp_group.state!=3 && vsp_group.state!=4;k++){
   tick+=5;it_empty_overlap_gates();SPF_GroupService(&vsp_group);
   if(mock_ext_moves<2)assert(!mock_lift_moves && !(vsp_group.started&SPF_LIFT));
  }
  assert(vsp_group.state==3 && vsp_group.done==6 && mock_ext_moves==2 && mock_lift_moves==1);
  assert(fabsf(vsp_axis[0].current-vsp_axis[0].goal)<.1f && vsp_axis[0].samples==3);
  mock_device_mode=0;
 }
 """
 unit=unit.replace(' puts("PASS: actual VSP',pipeline+'\n puts("PASS: actual VSP')

 tailcheck=r"""
 g_pick_trial_profile=2;g_pick_extension_segment=2;vsp_group.command.goal.extension_mm=156;
 vsp_group.state=2;vsp_group.started=2;vsp_group.done=0;
 vsp_axis[0].before=0;vsp_axis[0].goal=-5537.02490234375f;vsp_axis[0].phase=1;vsp_axis[0].options=132;
 vsp_axis[0].move[2]=vsp_axis[0].move[4]=2;vsp_axis[0].move[3]=vsp_axis[0].move[5]=88;
 vsp_axis[0].move[6]=4;vsp_axis[0].move[7]=36;
 assert(VSP_SettleAllowed(6,0));g_pick_extension_segment=1;assert(!VSP_SettleAllowed(6,0));g_pick_extension_segment=2;
 vsp_axis[0].move[6]=11;vsp_axis[0].move[7]=184;assert(!VSP_SettleAllowed(6,0));
 vsp_axis[0].move[6]=4;vsp_axis[0].move[7]=36;vsp_axis[0].overshoot_active=0;
 raw=55984;sign=1;tick=1000;assert(!VSP_Poll(0,SPF_EXT) && vsp_axis[0].overshoot_active);
 tick=1149;vsp_axis[0].phase=1;assert(!VSP_Poll(0,SPF_EXT));
 tick=1150;vsp_axis[0].phase=1;assert(VSP_Poll(0,SPF_EXT)==2);
 vsp_axis[0].overshoot_active=0;vsp_axis[0].phase=1;raw=57366;assert(VSP_Poll(0,SPF_EXT)==2);
 """
 unit=unit.replace(' puts("PASS: actual VSP',tailcheck+'\n puts("PASS: actual VSP')
 logs['port']=run('port-frames',unit,[U/'source_parallel_flow.c'])
 (E/'host-checks.json').write_text(json.dumps({'passed':True,'results':logs,'hardware_access':False,
  'real_motor_following_verified':False},indent=2)+'\n')
if __name__=='__main__':main()
