"""编译实际候选C动作组并模拟唯一任务仲裁；不发送硬件命令。"""
from pathlib import Path
import hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[1]
S=ROOT/'tmp/arm_fixed_replay_20261101/project';H=ROOT/'tmp/arm_fixed_replay_20261101/host-check'
EV=ROOT/'evidence/p2-arm/2026-11-01/fixed-replay'
H.mkdir(parents=True,exist_ok=True)
(H/'dm_can2.h').write_text('''#include <stdint.h>
extern volatile uint32_t g_dm_jog_request,g_dm_jog_state,g_dm_jog_result,g_dm_jog_stop_result,g_dm_point_enable,g_dm_posvel_holding;
extern volatile float g_dm_pick_target_deg,g_dm_place_target_deg,g_dm_point_latched_deg;
uint32_t DM_CAN2_JogIsBusy(void);
''')
base=ROOT/'tmp/arm_top_start_20261031/host-check'
s=(base/'test.c').read_text();s=s[:s.index('int main(')]
s=s.replace('static void grip(void){start();until(2);g_arm_grip_confirm=1;tick();}', '')
s=s.replace('#include "gripper_task.h"','#include "gripper_task.h"\n#include "dm_can2.h"')
s=s.replace('unsigned g_dm_jog_request,g_carrier_request,now;', '''unsigned g_carrier_request,now;
volatile uint32_t g_dm_jog_request,g_dm_jog_state,g_dm_jog_result,g_dm_jog_stop_result,g_dm_point_enable,g_dm_posvel_holding;
volatile float g_dm_pick_target_deg,g_dm_place_target_deg,g_dm_point_latched_deg;
unsigned dm_ticks,dm_starts,dm_stops,dm_stall,dm_fail;''')
s=s.replace('directions[4],pulses[4]','directions[16],pulses[16]').replace('angles[4]','angles[8]')
s=s.replace('unsigned DM_CAN2_JogIsBusy(void){return other_busy;}',
            'unsigned DM_CAN2_JogIsBusy(void){return g_dm_jog_state>=1&&g_dm_jog_state<=6;}')
main=(S/'USER/main.c').read_bytes().decode('latin1')
a=main.index('static uint32_t App_ArmSequenceReady(void)');b=main.index('static void App_CarrierStep(void)',a)
c=s.index('static uint32_t App_ArmSequenceReady(void)');d=s.index('static void tick(void)',c)
s=s[:c]+main[a:b]+s[d:]
s=s.replace(' ArmSequence_Service(App_ArmSequenceReady(),carrier_busy||other_busy||',r'''
 if(g_dm_jog_request==3){g_dm_jog_request=0;dm_ticks=0;dm_stops++;g_dm_posvel_holding=0;g_dm_jog_state=8;g_dm_jog_result=2;g_dm_jog_stop_result=1;}
 else if(g_dm_jog_request==4||g_dm_jog_request==5){
  unsigned request=g_dm_jog_request;g_dm_jog_request=0;
  if(!dm_stall){assert(g_dm_point_enable==1);dm_starts++;g_dm_posvel_holding=0;g_dm_jog_state=1;dm_ticks=4;
  g_dm_point_latched_deg=request==4?g_dm_pick_target_deg:g_dm_place_target_deg;}
 }else if(dm_ticks){dm_ticks--;if(!dm_ticks){g_dm_jog_state=dm_fail?8:7;g_dm_jog_result=dm_fail?7:1;g_dm_posvel_holding=dm_fail?0:1;}}
 ArmSequence_Service(App_ArmSequenceReady(),carrier_busy||other_busy||(DM_CAN2_JogIsBusy()&&!ArmSequence_OwnsDM())||''')
s+=r'''
int main(int argc,char**argv){const char*c=argv[1];(void)argc;
 config();g_arm_sequence_mode=3;g_arm_pick_down_pulses=5000;
 if(is(c,"disabled")){g_arm_sequence_request=1;tick();assert(!moves&&!commands&&!dm_starts&&g_arm_sequence_result==3);}
 else if(is(c,"zero_source")||is(c,"large_source")||is(c,"nan_angle")){
  if(is(c,"zero_source"))g_arm_pick_down_pulses=0;
  if(is(c,"large_source"))g_arm_pick_down_pulses=50001;
  if(is(c,"nan_angle"))g_arm_replay_place_deg=0.0f/0.0f;
  start();assert(g_arm_sequence_result==4&&!moves&&!commands&&!dm_starts);
 }else if(is(c,"complete")||is(c,"snapshot")){
  start();assert(!g_arm_top_start_ready);
  if(is(c,"snapshot")){g_arm_pick_down_pulses=999;g_arm_replay_pick_deg=45;g_arm_replay_place_deg=12;g_arm_sequence_mode=0;}
  until(7);assert(g_arm_sequence_result==1&&!ArmSequence_IsBusy()&&!g_arm_sequence_enable);
  assert(moves==7&&dm_starts==2&&commands==3&&angles[0]==180&&angles[1]==140&&angles[2]==180);
  assert(directions[0]==1&&pulses[0]==5000&&directions[1]==2&&pulses[1]==5000);
  assert(directions[2]==1&&pulses[2]==16000&&pulses[3]==8000&&pulses[4]==4000);
  assert(directions[5]==2&&pulses[5]==16000&&pulses[6]==12000);
  assert(g_dm_pick_target_deg==180.0f&&g_dm_place_target_deg==35.1079407f);
  assert(g_lift_up_pulses==8123&&g_lift_down_pulses==40000);
  for(unsigned i=0;i<100;i++)tick();assert(moves==7&&commands==3&&dm_starts==2);
 }else if(is(c,"stale_dm")){
  dm_stall=1;g_dm_jog_state=7;g_dm_jog_result=1;g_dm_posvel_holding=1;g_dm_point_latched_deg=180;
  start();until(21);for(unsigned i=0;i<100;i++)tick();assert(!moves&&g_arm_sequence_state==21);
 }else if(is(c,"lift_failure")||is(c,"gripper_failure")||is(c,"rotation_failure")){
  if(is(c,"lift_failure"))lift_fail=1;
  if(is(c,"gripper_failure"))command_fail=1;
  if(is(c,"rotation_failure"))dm_fail=1;
  start();until(9);unsigned count=commands;for(unsigned i=0;i<10;i++)tick();assert(commands==count&&ArmSequence_IsBusy());
  assert(g_arm_sequence_result==(is(c,"lift_failure")?6:is(c,"gripper_failure")?5:9));
 }else if(is(c,"abort_down")||is(c,"abort_rotate")||is(c,"external_stop")||is(c,"lost_ready")||is(c,"hold_loss")||is(c,"gap")){
  start();until(is(c,"abort_rotate")?21:22);unsigned count=commands;
  if(is(c,"external_stop"))g_dm_jog_request=3;
  else if(is(c,"lost_ready"))g_vm_test_request=0;
  else if(is(c,"hold_loss"))g_dm_posvel_holding=0;
  else if(is(c,"gap"))now+=501;
  else g_arm_sequence_request=2;
  tick();assert((g_arm_sequence_state==8||g_arm_sequence_state==9)&&commands==count&&ArmSequence_IsBusy());
  for(unsigned i=0;i<10;i++)tick();assert(!g_dm_posvel_holding&&dm_stops&&commands==count);
  g_arm_sequence_request=3;tick();assert(!ArmSequence_IsBusy()&&!g_arm_top_start_ready);
 }else if(is(c,"legacy")){
  g_arm_sequence_mode=0;start();until(2);g_arm_grip_confirm=1;tick();until(7);
  assert(moves==2&&commands==2&&!dm_starts&&g_arm_sequence_result==1);
 }else assert(!"unknown case");
 puts(c);return 0;}
'''
(H/'test.c').write_text(s)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(H),'-I'+str(base),
     '-I'+str(ROOT/'tmp/arm_sequence_fast_20261031/host-check'),'-I'+str(S/'USER'),
     '-I'+str(S/'HARDWARE/LIFT'),str(H/'test.c'),str(S/'USER/arm_sequence.c'),'-o',str(H/'test.exe')]
r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
cases='disabled zero_source large_source nan_angle complete snapshot stale_dm lift_failure gripper_failure rotation_failure abort_down abort_rotate external_stop lost_ready hold_loss gap legacy'.split()
logs=[]
for case in cases:
    r=subprocess.run([str(H/'test.exe'),case],capture_output=True,text=True)
    logs.append(case+': '+str(r.returncode)+' '+r.stdout+r.stderr)
    if r.returncode:print('\n'.join(logs));raise SystemExit(1)
(EV/'host-check.log').write_text('\n'.join(logs))
(EV/'host-review.json').write_text(json.dumps(dict(passed=True,cases=cases,hardware_stubs=True,
    actual_main_guard_extracted=True,source_sha256={n:hashlib.sha256((S/n).read_bytes()).hexdigest()
    for n in ['USER/arm_sequence.c','USER/arm_sequence.h','USER/main.c']}),indent=2))
print(len(cases),'actual-C/arbitration scenarios passed; hardware stubs only')
