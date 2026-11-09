"""顶部起步模式2新增路径：提取候选main仲裁，用主机桩检查状态机。"""
from pathlib import Path
import subprocess,json,hashlib,shutil
ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tmp/arm_top_start_20261031/project'
EV=ROOT/'evidence/p2-arm/2026-10-31/arm-top-start'
HOST=ROOT/'tmp/arm_top_start_20261031/host-check';HOST.mkdir(parents=True,exist_ok=True)
for n in ['lift.h','lift_home.h']:shutil.copyfile(ROOT/'tmp/arm_top_cycle_20261031/host-check'/n,HOST/n)
s=(ROOT/'tmp/arm_top_cycle_20261031/host-check/test.c').read_text();s=s[:s.index('int main(')]
main=(STAGE/'USER/main.c').read_bytes().decode('latin1')
a=main.index('static uint32_t App_ArmSequenceReady(void)');b=main.index('static void App_CarrierStep(void)',a)
c=s.index('static uint32_t App_ArmSequenceReady(void)');d=s.index('static void tick(void)',c)
s=s[:c]+main[a:b]+s[d:]
s=s.replace('static void wait_grip(void){for(unsigned i=0;i<15&&g_arm_sequence_state!=2;i++)tick();assert(g_arm_sequence_state==2);}', '')
s+=r'''
static void config(void){g_arm_sequence_mode=2;g_arm_pick_down_pulses=32000;g_arm_top_approach_pulses=27000;g_arm_return_down_pulses=32000;g_arm_top_start_ready=1;}
static void until(unsigned state){for(unsigned i=0;i<300&&g_arm_sequence_state!=state&&g_arm_sequence_state!=9;i++)tick();assert(g_arm_sequence_state==state);}
static void grip(void){start();until(2);g_arm_grip_confirm=1;tick();}
int main(int argc,char**argv){const char*s=argv[1];(void)argc;
 assert(!g_arm_pick_down_pulses&&!g_arm_top_approach_pulses&&!g_arm_top_start_ready&&!g_arm_place_confirm);
 config();
 if(is(s,"disabled")){g_arm_sequence_request=1;tick();assert(!moves&&!commands&&g_arm_sequence_result==3);}
 else if(is(s,"missing_top")){g_arm_top_start_ready=0;start();assert(!moves&&!commands&&g_arm_sequence_result==3);}
 else if(is(s,"bad_pick")||is(s,"zero_pick")||is(s,"bad_up")||is(s,"zero_up")||is(s,"bad_return")||is(s,"bad_mode")){
  if(is(s,"bad_pick"))g_arm_pick_down_pulses=50001;
  if(is(s,"zero_pick"))g_arm_pick_down_pulses=0;
  if(is(s,"bad_up"))g_arm_top_approach_pulses=32000;
  if(is(s,"zero_up"))g_arm_top_approach_pulses=0;
  if(is(s,"bad_return"))g_arm_return_down_pulses=50001;
  if(is(s,"bad_mode"))g_arm_sequence_mode=3;
  start();assert(!moves&&!commands&&g_arm_sequence_result==4);
 }else if(is(s,"complete")||is(s,"snapshot")){
  start();assert(!g_arm_top_start_ready);
  if(is(s,"snapshot")){g_arm_sequence_mode=0;g_arm_pick_down_pulses=50001;g_arm_top_approach_pulses=50001;g_arm_return_down_pulses=50001;}
  until(2);assert(commands==2&&moves==1&&directions[0]==1&&pulses[0]==32000);
  g_arm_grip_confirm=1;tick();until(12);assert(commands==2&&moves==3&&home_starts==1);
  assert(directions[1]==2&&pulses[1]==27000&&directions[2]==1&&pulses[2]==32000);
  assert(angles[0]==180&&angles[1]==140&&!g_arm_place_confirm);
  g_arm_place_confirm=1;tick();until(7);assert(commands==3&&angles[2]==180&&!ArmSequence_IsBusy());
  assert(g_arm_sequence_result==1&&!g_arm_top_start_ready&&!g_arm_grip_height_ready&&!g_arm_sequence_enable);
  assert(g_lift_up_pulses==8123&&g_lift_down_pulses==40000);
  for(unsigned i=0;i<100;i++){tick();}assert(moves==3&&commands==3);
  g_arm_sequence_mode=2;g_arm_sequence_enable=1;g_arm_sequence_request=1;tick();assert(moves==3&&g_arm_sequence_result==3);
 }else if(is(s,"fresh_place")||is(s,"place_timeout")){
  g_arm_place_confirm=1;grip();g_arm_place_confirm=1;until(12);
  for(unsigned i=0;i<30;i++){tick();}assert(commands==2&&!g_arm_place_confirm&&g_arm_sequence_state==12);
  if(is(s,"place_timeout")){for(unsigned i=0;i<760;i++){tick();}assert(g_arm_sequence_state==9&&commands==2&&ArmSequence_IsBusy());}
 }else if(is(s,"initial_open_failure")){
  command_fail=1;start();until(9);assert(!moves&&commands==1&&g_arm_sequence_result==5);
 }else if(is(s,"initial_down_failure")){
  lift_fail=1;start();until(9);assert(moves==1&&commands==1);
 }else if(is(s,"return_failure")){
  grip();until(4);lift_fail=1;until(9);assert(commands==2&&moves==3);
 }else if(is(s,"home_failure")){
  home_fail=1;grip();until(9);assert(commands==2&&moves==2);
 }else if(is(s,"abort_initial")||is(s,"abort_up")||is(s,"abort_home")||is(s,"external_stop_place")){
  start();
  if(is(s,"abort_initial"))until(10);
  else {until(2);g_arm_grip_confirm=1;tick();if(is(s,"abort_home"))until(11);if(is(s,"external_stop_place"))until(12);}
  unsigned cmds=commands;
  if(is(s,"external_stop_place"))g_lift_motion_request=3;else g_arm_sequence_request=2;
  tick();assert(g_arm_sequence_state==8&&ArmSequence_IsBusy()&&commands==cmds&&!g_arm_top_start_ready);
  for(unsigned i=0;i<4;i++){tick();}g_arm_sequence_request=3;tick();assert(!ArmSequence_IsBusy());
 }else if(is(s,"stale_home")){
  home_stall=1;g_lift_home_state=8;g_lift_home_result=g_lift_home_reported=1;
  grip();until(11);for(unsigned i=0;i<40;i++){tick();}assert(moves==2&&commands==2&&g_arm_sequence_state==11);
 }else if(is(s,"power_loss")){
  grip();g_vm_test_request=0;tick();assert(g_arm_sequence_state==9&&commands==2&&ArmSequence_IsBusy());
 }else if(is(s,"guard_new_phases")){
  start();until(10);g_carrier_request=1;g_extension_motion_request=2;g_dm_jog_request=4;App_ArmSequenceGuard();assert(!g_carrier_request&&!g_extension_motion_request&&!g_dm_jog_request);
  until(2);g_arm_grip_confirm=1;tick();until(11);App_ArmSequenceGuard();assert(g_lift_home_request==1);
 }else if(is(s,"legacy0")||is(s,"legacy1")){
  g_arm_sequence_mode=is(s,"legacy0")?0:1;grip();until(7);assert(commands==2&&angles[0]==140&&angles[1]==180);
  if(is(s,"legacy0"))assert(moves==2&&pulses[0]==5000&&pulses[1]==5000&&!home_starts);
  else assert(moves==1&&pulses[0]==32000&&home_starts==1);
 }else assert(!"unknown case");
 puts(s);return 0;}
'''
(HOST/'test.c').write_text(s)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(HOST),'-I'+str(ROOT/'tmp/arm_sequence_fast_20261031/host-check'),'-I'+str(STAGE/'USER'),'-I'+str(STAGE/'HARDWARE/LIFT'),str(HOST/'test.c'),str(STAGE/'USER/arm_sequence.c'),'-o',str(HOST/'test.exe')]
r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
cases='disabled missing_top bad_pick zero_pick bad_up zero_up bad_return bad_mode complete snapshot fresh_place place_timeout initial_open_failure initial_down_failure return_failure home_failure abort_initial abort_up abort_home external_stop_place stale_home power_loss guard_new_phases legacy0 legacy1'.split()
logs=[]
for c in cases:
    r=subprocess.run([str(HOST/'test.exe'),c],capture_output=True,text=True);logs.append(c+': '+str(r.returncode)+' '+r.stdout+r.stderr)
    if r.returncode: print(''.join(logs));raise SystemExit(1)
(EV/'host-check.log').write_text('\n'.join(logs))
(EV/'host-review.json').write_text(json.dumps({'passed':True,'cases':cases,'actual_main_guard_extracted':True,'hardware_stubs':True,'source_sha256':{n:hashlib.sha256((STAGE/n).read_bytes()).hexdigest() for n in ['USER/arm_sequence.c','USER/arm_sequence.h','USER/main.c']}},indent=2))
print(len(cases),'targeted software scenarios passed; no hardware commands.')
