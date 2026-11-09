"""实码验证完整单循环、单段车载下降与最终回零完成门禁。"""
from pathlib import Path
import hashlib,json,shutil,subprocess
R=Path(__file__).resolve().parents[1]
S=R/'tmp/arm_full_cycle_20261102/project';H=R/'tmp/arm_full_cycle_20261102/host-check'
E=R/'evidence/p2-arm/2026-11-02/full-cycle'
B=R/'tmp/arm_source_single_20261102/host-check'
H.mkdir(parents=True,exist_ok=True);shutil.copyfile(B/'dm_can2.h',H/'dm_can2.h')
s=(B/'test.c').read_text();s=s[:s.index('int main(')]
main=(S/'USER/main.c').read_bytes().decode('latin1')
a=main.index('static uint32_t App_ArmSequenceReady(void)');b=main.index('static void App_CarrierStep(void)',a)
c=s.index('static uint32_t App_ArmSequenceReady(void)');d=s.index('static unsigned test_stall;',c)
assert ''.join(s[c:d].split())==''.join(main[a:b].split())
s=s[:c]+main[a:b].replace('\r\n','\n')+s[d:]
s+=r'''
int main(int argc,char**argv){const char*c=argv[1];(void)argc;
 config();g_arm_sequence_mode=5;
 if(is(c,"disabled")||is(c,"missing_height")){
  g_arm_sequence_request=1;if(is(c,"missing_height"))g_arm_sequence_enable=1;
  tick();assert(g_arm_sequence_result==3&&!moves&&!home_starts&&!commands);
 }else if(is(c,"complete")||is(c,"snapshot")||is(c,"duplicate_start")){
  start();assert(!g_arm_grip_height_ready);
  if(is(c,"snapshot")){g_arm_sequence_mode=0;g_arm_return_down_pulses=9;}
  if(is(c,"duplicate_start")){g_arm_sequence_request=1;tick();}
  until(7);assert(moves==1&&directions[0]==2&&pulses[0]==102000&&home_starts==1);
  assert(g_lift_home_state==8&&g_lift_home_result==1&&g_lift_home_reported==1);
  assert(!commands&&!dm_starts&&!dm_stops&&!ArmSequence_IsBusy()&&g_arm_sequence_result==1);
  assert(g_lift_up_pulses==8123&&g_lift_down_pulses==40000&&!g_arm_sequence_enable);
  for(unsigned i=0;i<100;i++){tick();}assert(moves==1&&home_starts==1&&!commands);
  g_arm_sequence_mode=5;g_arm_sequence_enable=1;g_arm_sequence_request=1;tick();assert(g_arm_sequence_result==3);
 }else if(is(c,"stale_home")||is(c,"home_wait")||is(c,"home_fail")){
  if(is(c,"home_fail"))home_fail=1;
  else {home_stall=1;g_lift_home_state=8;g_lift_home_result=g_lift_home_reported=1;}
  start();
  if(is(c,"home_fail")){until(9);assert(g_arm_sequence_result==6&&!commands&&!dm_starts);}
  else {until(33);for(unsigned i=0;i<1000;i++){tick();}assert(g_arm_sequence_state==33&&ArmSequence_IsBusy()&&moves==1&&!commands);}
 }else if(is(c,"lift_fail")){
  lift_fail=1;start();until(9);assert(g_arm_sequence_result==6&&moves==1&&!home_starts&&!commands);
 }else if(is(c,"abort_up")||is(c,"abort_home")||is(c,"external_stop")||is(c,"lost_ready")||is(c,"other_busy")||is(c,"gap")||is(c,"gripper_cancel")){
  start();
  if(is(c,"abort_home")){until(33);tick();assert(LiftHome_IsBusy()&&ArmSequence_OwnsHome());}
  else {tick();}
  if(is(c,"external_stop"))g_lift_motion_request=3;
  else if(is(c,"lost_ready"))g_vm_test_request=0;
  else if(is(c,"other_busy"))other_busy=1;
  else if(is(c,"gap"))now+=501;
  else if(is(c,"gripper_cancel"))g_gripper_signal_stop_count++;
  else g_arm_sequence_request=2;
  tick();assert((g_arm_sequence_state==8||g_arm_sequence_state==9)&&ArmSequence_IsBusy());
  for(unsigned i=0;i<10;i++){tick();}assert(!commands&&!dm_starts&&!dm_stops);
  if(is(c,"abort_home"))assert(home_abort&&g_lift_home_stop_result==1);
  g_arm_sequence_request=3;tick();assert(!ArmSequence_IsBusy()&&!g_arm_grip_height_ready);
 }else if(is(c,"mode3_complete")||is(c,"mode3_home_fail")||is(c,"mode3_stale_home")||is(c,"mode3_final_home_fail")||is(c,"mode3_final_home_stale")||is(c,"mode3_abort_final_home")||is(c,"mode3_carrier_down_fail")||is(c,"mode3_carrier_up_fail")){
  g_arm_sequence_mode=3;g_arm_pick_down_pulses=105000;
  if(is(c,"mode3_home_fail"))home_fail=1;
  if(is(c,"mode3_stale_home")){home_stall=1;g_lift_home_state=8;g_lift_home_result=g_lift_home_reported=1;}
  start();until(29);
  assert(moves==2&&directions[0]==1&&pulses[0]==105000&&directions[1]==2&&pulses[1]==102000);
  assert(commands==2&&dm_starts==1&&ArmSequence_OwnsHome());
  if(is(c,"mode3_home_fail")){until(9);assert(g_arm_sequence_result==6&&commands==2&&dm_starts==1);}
  else if(is(c,"mode3_stale_home")){for(unsigned i=0;i<100;i++){tick();}assert(g_arm_sequence_state==29&&dm_starts==1&&commands==2);}
  else {
   until(26);assert(moves==2&&dm_starts==2&&commands==2);
   if(is(c,"mode3_carrier_down_fail")){lift_fail=1;until(9);assert(moves==3&&commands==2&&g_arm_sequence_result==6);}
   else {
    until(28);assert(moves==3&&pulses[2]==28000&&directions[2]==1&&commands==3&&angles[2]==180);
    if(is(c,"mode3_carrier_up_fail")){lift_fail=1;until(9);assert(moves==4&&g_arm_sequence_result==6);}
    else {
     until(34);assert(moves==4&&pulses[3]==25000&&directions[3]==2&&ArmSequence_OwnsHome());
     if(is(c,"mode3_final_home_fail")){home_fail=1;until(9);assert(g_arm_sequence_result==6&&ArmSequence_IsBusy());}
     else if(is(c,"mode3_final_home_stale")){home_stall=1;for(unsigned i=0;i<100;i++){tick();}assert(g_arm_sequence_state==34&&ArmSequence_IsBusy());}
     else if(is(c,"mode3_abort_final_home")){tick();g_arm_sequence_request=2;tick();for(unsigned i=0;i<8;i++){tick();}assert(g_arm_sequence_state==8&&home_abort&&!g_dm_posvel_holding);}
     else {
      until(7);assert(home_starts==2&&moves==4&&commands==3&&dm_starts==2&&g_arm_sequence_result==1);
      assert(!ArmSequence_IsBusy()&&!g_arm_sequence_enable&&g_dm_posvel_holding&&g_dm_point_latched_deg==35.1079407f);
      for(unsigned i=0;i<100;i++){tick();}assert(moves==4&&home_starts==2&&commands==3);
     }
    }
   }
  }
 }else if(is(c,"mode4_unchanged")){
  g_arm_sequence_mode=4;start();until(7);assert(moves==1&&pulses[0]==105000&&directions[0]==1&&commands==1&&angles[0]==140&&!home_starts&&!dm_starts);
 }else if(is(c,"legacy0")){
  g_arm_sequence_mode=0;start();until(2);g_arm_grip_confirm=1;tick();until(7);assert(moves==2&&commands==2&&!home_starts&&!dm_starts);
 }else assert(!"unknown case");
 puts(c);return 0;}
'''
(H/'test.c').write_text(s)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(H),
     '-I'+str(R/'tmp/arm_top_start_20261031/host-check'),'-I'+str(R/'tmp/arm_sequence_fast_20261031/host-check'),
     '-I'+str(S/'USER'),'-I'+str(S/'HARDWARE/LIFT'),str(H/'test.c'),str(S/'USER/arm_sequence.c'),'-o',str(H/'test.exe')]
r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
cases='disabled missing_height complete snapshot duplicate_start stale_home home_wait home_fail lift_fail abort_up abort_home external_stop lost_ready other_busy gap gripper_cancel mode3_complete mode3_home_fail mode3_stale_home mode3_final_home_fail mode3_final_home_stale mode3_abort_final_home mode3_carrier_down_fail mode3_carrier_up_fail mode4_unchanged legacy0'.split()
logs=[]
for case in cases:
    r=subprocess.run([str(H/'test.exe'),case],capture_output=True,text=True)
    logs.append(case+': '+str(r.returncode)+' '+r.stdout+r.stderr)
    if r.returncode:print('\n'.join(logs));raise SystemExit(1)
(E/'host-check.log').write_text('\n'.join(logs))
(E/'host-review.json').write_text(json.dumps(dict(passed=True,cases=cases,hardware_stubs=True,
    actual_main_guard_matched=True,source_sha256={n:hashlib.sha256((S/n).read_bytes()).hexdigest()
    for n in ['USER/arm_sequence.c','USER/arm_sequence.h']}),indent=2))
print(len(cases),'actual-C prehome/ownership scenarios passed; no hardware commands')
