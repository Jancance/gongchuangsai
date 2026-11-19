"""Execute the full actual Low_Service with motion-callback trajectories."""
from pathlib import Path
import json,subprocess,shutil
from prepare_three_frame_grip_lead20_20261118 import R,P,E,BASE

def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 old=R/'evidence/integration/2026-11-18/three-frame-initial5'
 shutil.copy2(old/'can.h',E/'can.h')
 actual=(P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(b'#include "low_readonly_diag.inc"',b'/* Hardware diagnostics are outside this owner fixture. */')
 (E/'tested-commit-owner.inc').write_bytes(actual)
 s=(old/'base-commit-fixture.c').read_text(encoding='utf-8')
 s=s.replace('static uint32_t tick,platform=', 'static unsigned fail_grip;static uint32_t tick,platform=',1)
 s=s.replace('if(angle==140)grips++;','if(angle==140){grips++;if(fail_grip)return 0;}',1)
 s=s.replace('static void reset(void){','static void reset(void){fail_grip=0;',1)
 tests=r'''
static void live_depth(float d,unsigned age,unsigned cycle){
 tick+=20;depth=d;vsp_axis[1].current=d*360.0f/3200.0f;
 vsp_axis[1].position_tick=tick-age;vsp_axis[1].position_cycle=cycle;
 vsp_group.started=vsp_group.command.axes_mask;vsp_group.state=2;g_source_parallel_state=2;
 Low_Service(tick);
}
static void done_pose(unsigned stale){
 SPF_Command c=vsp_group.command;CHECK(owned);tick+=100;
 depth=c.goal.lift_pulses;extension=c.goal.extension_mm;g_dm_can2_logical_position_deg=c.goal.dm_deg;
 vsp_axis[1].current=depth*360.0f/3200.0f;vsp_axis[1].position_tick=tick-(stale?151:0);vsp_axis[1].position_cycle=1;
 vsp_axis[0].current=STARTUP_EXTENSION_EXTEND_SIGN*extension*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;vsp_axis[0].position_tick=tick;vsp_axis[0].position_cycle=1;
 g_vehicle_lift_current_deg=vsp_axis[1].current;
 vsp_group.state=3;g_source_parallel_state=3;g_source_parallel_done_mask=c.axes_mask;owned=0;
 Low_Service(tick);
}
int main(void){unsigned region,slot,k,timestamp,pwm_tick,before_tick;float target,threshold;
 CHECK(SOURCE_GRIP_LEAD_MM==20.0f && SOURCE_LIFT_PULSES_PER_MM==400.0f);
 for(region=0;region<3;region++)for(slot=1;slot<=3;slot++){
  start_region(region,slot);target=region<2?25700:29700;threshold=region<2?17700:21700;
  CHECK(low_pick_depth==target && vsp_group.command.goal.lift_pulses==target);
  live_depth(threshold-1,0,1);CHECK(!grips && g_low_phase==4 && owned && recorded==1);
  live_depth(threshold,150,1);CHECK(grips==1 && g_it_lead5_grip_done && g_low_phase==4 && owned && recorded==1);
  timestamp=g_low_grip_command_ms;pwm_tick=pulse_apply_at;
  for(k=0;k<3;k++){live_depth(threshold+1000+k*1000,0,1);CHECK(grips==1 && g_low_grip_command_ms==timestamp && pulse_apply_at==pwm_tick && recorded==1);}
  before_tick=tick;done_pose(0);CHECK(grips==1 && g_low_grip_command_ms==timestamp && pulse_apply_at==pwm_tick);
  CHECK(g_low_phase==5 && recorded==2 && records[1].tick==before_tick+100 && records[1].command.goal.lift_pulses==SOURCE_RETURN_PULSES);
  CHECK(g_low_actual_pick_depth_pulses>target-1 && g_low_actual_pick_depth_pulses<target+1);
 }
 /* All early samples absent or stale: original DONE fallback closes once,
  * before submitting rise in that same call; no new dwell or PWM wait. */
 for(region=0;region<3;region++)for(k=0;k<2;k++){
  start_region(region,1);target=low_pick_depth;threshold=target-8000;
  live_depth(threshold+1,k?151:0,k?1:0);CHECK(!grips && g_low_phase==4 && recorded==1);
  before_tick=tick;done_pose(1);CHECK(grips==1 && g_it_lead5_grip_done && g_low_phase==5 && recorded==2);
  CHECK(records[1].tick==before_tick+100 && low_begin+g_low_grip_command_ms==records[1].tick);
 }
 /* Other sequence / LIFT not present / not started / other phase cannot
  * reuse the sample as a moving close. None adds a new fault or wait. */
 for(k=0;k<4;k++){
  start_region(0,1);vsp_group.started=7;g_source_parallel_state=2;
  tick+=20;vsp_axis[1].current=(17701.0f)*360/3200;vsp_axis[1].position_tick=tick;vsp_axis[1].position_cycle=1;
  if(k==0)vsp_group.command.sequence++;
  if(k==1)vsp_group.command.axes_mask&=~SPF_LIFT;
  if(k==2)vsp_group.started&=~SPF_LIFT;
  if(k==3)g_low_phase=1;
  Low_Service(tick);CHECK(!grips && !g_low_fault && recorded==1);
 }
 for(k=0;k<4;k++){
  start_region(0,1);vsp_group.started=7;g_source_parallel_state=2;tick+=20;
  vsp_axis[1].current=17701.0f*360/3200;vsp_axis[1].position_tick=tick;vsp_axis[1].position_cycle=1;
  if(k==0)g_low_request=9;
  if(k==1)platform=0;
  if(k==2)g_source_parallel_state=4;
  if(k==3)fail_grip=1;
  Low_Service(tick);CHECK(g_low_state==9 && g_it_state==9 && recorded==1 && failures==1);
  CHECK(grips==(k==3?1U:0U));
 }
 /* Original no-live-sample whole-piece path remains valid in all nine slots. */
 for(region=0;region<3;region++)for(slot=1;slot<=3;slot++)piece(region,slot);
 printf("PASS complete actual Low_Service %u checks:9 moving trajectories side17700/bottom21700, final depths25700/29700, fresh150 once before fullDONE, missing/stale immediate fallback, no repeated PWM/time, STOP/failures,9 full-piece paths\n",checks);return 0;}
'''
 p=E/'actual-owner-lead20-trajectories.c';p.write_text(s+tests,encoding='utf-8');exe=p.with_suffix('.exe')
 args=['gcc','-std=c99','-O1','-Wall','-Wextra','-Wno-unused-function','-Wno-unused-variable','-Wno-misleading-indentation','-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(p),'-lm','-o',str(exe)]
 b=subprocess.run(args,capture_output=True,text=True);(E/'actual-owner-build.log').write_text(b.stdout+b.stderr);assert not b.returncode,b.stderr
 z=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-owner.log').write_text(z.stdout+z.stderr);assert not z.returncode,z.stdout+z.stderr
 print(z.stdout.strip());(E/'actual-c-result.json').write_text(json.dumps(dict(passed=True,result=z.stdout.strip(),full_actual_Low_Service=True,hardware_access=False),indent=2)+'\n')
if __name__=='__main__':main()
