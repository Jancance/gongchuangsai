"""验证模式4只发一条105000下降，保留错误/停止及旧模式路径。"""
from pathlib import Path
import hashlib,json,shutil,subprocess
ROOT=Path(__file__).resolve().parents[1]
S=ROOT/'tmp/arm_source_single_20261102/project'
H=ROOT/'tmp/arm_source_single_20261102/host-check'
EV=ROOT/'evidence/p2-arm/2026-11-02/source-pick-single'
OLD=ROOT/'tmp/arm_fixed_replay_20261101/host-check'
H.mkdir(parents=True,exist_ok=True)
shutil.copyfile(OLD/'dm_can2.h',H/'dm_can2.h')
s=(OLD/'test.c').read_text();s=s[:s.index('int main(')]
main=(S/'USER/main.c').read_bytes().decode('latin1')
a=main.index('static uint32_t App_ArmSequenceReady(void)');b=main.index('static void App_CarrierStep(void)',a)
c=s.index('static uint32_t App_ArmSequenceReady(void)');d=s.index('static void tick(void)',c)
assert ''.join(s[c:d].split())==''.join(main[a:b].split())
s=s[:c]+main[a:b].replace('\r\n','\n')+s[d:]
s=s.replace('static void tick(void){','static unsigned test_stall;\nstatic void tick(void){')
s=s.replace(' if(g_lift_motion_request==3){',
            ' if(test_stall && (g_lift_motion_request==1 || g_lift_motion_request==2))g_lift_motion_request=0;\n if(g_lift_motion_request==3){')
s+=r'''
int main(int argc,char**argv){const char*c=argv[1];(void)argc;
 config();g_arm_sequence_mode=4;
 if(is(c,"disabled")||is(c,"missing_top")){
  if(is(c,"missing_top")){g_arm_top_start_ready=0;start();}
  else {g_arm_sequence_request=1;tick();}
  assert(g_arm_sequence_result==3&&!moves&&!commands&&!dm_starts);
 }else if(is(c,"complete")||is(c,"snapshot")||is(c,"duplicate_start")){
  start();assert(!g_arm_top_start_ready);
  if(is(c,"snapshot")){g_arm_sequence_mode=0;g_arm_pick_down_pulses=1;}
  if(is(c,"duplicate_start")){g_arm_sequence_request=1;tick();}
  until(7);
  const unsigned expected[1]={105000};
  unsigned sum=0;assert(moves==1&&commands==1&&angles[0]==140);
  for(unsigned i=0;i<1;i++){assert(directions[i]==1&&pulses[i]==expected[i]&&pulses[i]<=105000);sum+=pulses[i];}
  assert(sum==105000&&g_arm_source_down_done_pulses==105000);
  assert(!home_starts&&!dm_starts&&!dm_stops&&!g_dm_point_enable);
  assert(!ArmSequence_IsBusy()&&!g_arm_sequence_enable&&!g_arm_top_start_ready&&g_arm_sequence_result==1);
  assert(g_lift_up_pulses==8123&&g_lift_down_pulses==40000);
  for(unsigned i=0;i<100;i++){tick();}assert(moves==1&&commands==1);
  g_arm_sequence_mode=4;g_arm_sequence_enable=1;g_arm_sequence_request=1;tick();
  assert(g_arm_sequence_result==3&&moves==1&&commands==1);
 }else if(is(c,"stale_lift_done")){
  test_stall=1;g_lift_motion_state=7;g_lift_motion_result=1;start();
  for(unsigned i=0;i<100;i++)tick();
  assert(g_arm_sequence_state==30&&!g_arm_source_down_done_pulses&&!commands&&!moves);
 }else if(strncmp(c,"lift_fail",9)==0){
  unsigned segment=(unsigned)(c[9]-'0');assert(segment>=1&&segment<=6);
  start();for(unsigned i=0;i<150&&moves<segment;i++)tick();assert(moves==segment);
  lift_fail=1;until(9);assert(g_arm_sequence_result==6&&!commands&&moves==segment);
  for(unsigned i=0;i<10;i++){tick();}assert(!commands&&moves==segment&&ArmSequence_IsBusy());
 }else if(is(c,"gripper_fail")){
  command_fail=1;start();until(9);
  assert(g_arm_sequence_result==5&&moves==1&&commands==1&&angles[0]==140);
 }else if(is(c,"abort_down")||is(c,"abort_close")||is(c,"external_stop")||is(c,"lost_ready")||is(c,"other_busy")||is(c,"gap")||is(c,"gripper_cancel")){
  start();
  if(is(c,"abort_close"))until(31);else {for(unsigned i=0;i<2;i++)tick();}
  unsigned count=commands;
  if(is(c,"external_stop"))g_lift_motion_request=3;
  else if(is(c,"lost_ready"))g_vm_test_request=0;
  else if(is(c,"other_busy"))other_busy=1;
  else if(is(c,"gap"))now+=501;
  else if(is(c,"gripper_cancel"))g_gripper_signal_stop_count++;
  else g_arm_sequence_request=2;
  tick();assert((g_arm_sequence_state==8||g_arm_sequence_state==9)&&commands==count&&ArmSequence_IsBusy());
  for(unsigned i=0;i<10;i++){tick();}assert(commands==count&&!dm_starts&&!dm_stops&&!home_starts);
  g_arm_sequence_request=3;tick();assert(!ArmSequence_IsBusy()&&!g_arm_top_start_ready);
 }else if(is(c,"legacy0")){
  g_arm_sequence_mode=0;start();until(2);g_arm_grip_confirm=1;tick();until(7);
  assert(moves==2&&commands==2&&!dm_starts&&g_arm_sequence_result==1);
 }else if(is(c,"legacy3")){
  g_arm_sequence_mode=3;g_arm_pick_down_pulses=5000;start();until(7);
  assert(moves==7&&commands==3&&dm_starts==2&&g_arm_sequence_result==1);
 }else assert(!"unknown case");
 puts(c);return 0;}
'''
(H/'test.c').write_text(s)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(H),
     '-I'+str(ROOT/'tmp/arm_top_start_20261031/host-check'),
     '-I'+str(ROOT/'tmp/arm_sequence_fast_20261031/host-check'),
     '-I'+str(S/'USER'),'-I'+str(S/'HARDWARE/LIFT'),str(H/'test.c'),str(S/'USER/arm_sequence.c'),
     '-o',str(H/'test.exe')]
r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
cases='disabled missing_top complete snapshot duplicate_start stale_lift_done lift_fail1 gripper_fail abort_down abort_close external_stop lost_ready other_busy gap gripper_cancel legacy0 legacy3'.split()
logs=[]
for case in cases:
    r=subprocess.run([str(H/'test.exe'),case],capture_output=True,text=True)
    logs.append(case+': '+str(r.returncode)+' '+r.stdout+r.stderr)
    if r.returncode:print('\n'.join(logs));raise SystemExit(1)
(EV/'host-check.log').write_text('\n'.join(logs))
(EV/'host-review.json').write_text(json.dumps(dict(passed=True,cases=cases,hardware_stubs=True,
    actual_main_guard_unchanged_and_matched=True,source_sha256={n:hashlib.sha256((S/n).read_bytes()).hexdigest()
    for n in ['USER/arm_sequence.c','USER/arm_sequence.h','HARDWARE/LIFT/lift_config.h','HARDWARE/LIFT/lift.h']}),indent=2))
print(len(cases),'actual-C source-pick scenarios passed; no hardware commands')
