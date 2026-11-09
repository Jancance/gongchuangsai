"""只覆盖本次确认窗口和阶段诊断改动，不重复旧机构套件。"""
from pathlib import Path
import subprocess,json,hashlib,shutil
ROOT=Path(__file__).resolve().parents[1];STAGE=ROOT/'tmp/arm_confirm_wait_20261031/project'
EV=ROOT/'evidence/p2-arm/2026-10-31/arm-confirm-wait';HOST=ROOT/'tmp/arm_confirm_wait_20261031/host-check';HOST.mkdir(parents=True,exist_ok=True)
for n in ['lift.h','lift_home.h']:shutil.copyfile(ROOT/'tmp/arm_top_start_20261031/host-check'/n,HOST/n)
s=(ROOT/'tmp/arm_top_start_20261031/host-check/test.c').read_text();s=s[:s.index('int main(')]
# main为本次保护文件；额外确认桩采用的仲裁与候选main一致。
main=(STAGE/'USER/main.c').read_bytes().decode('latin1');a=main.index('static uint32_t App_ArmSequenceReady(void)');b=main.index('static void App_CarrierStep(void)',a)
c=s.index('static uint32_t App_ArmSequenceReady(void)');d=s.index('static void tick(void)',c);assert s[c:d]==main[a:b]
s+=r'''
int main(int argc,char**argv){const char*s=argv[1];(void)argc;config();
 if(is(s,"grip_wait_resume")){
  start();until(2);for(unsigned i=0;i<1000;i++){tick();}
  assert(g_arm_sequence_state==2&&g_arm_sequence_phase_elapsed_ms==20000&&!g_arm_sequence_fault_state&&moves==1);
  g_arm_grip_confirm=1;tick();until(12);g_arm_place_confirm=1;tick();until(7);assert(commands==3);
 }else if(is(s,"grip_timeout")){
  start();until(2);for(unsigned i=0;i<6002;i++){tick();}
  assert(g_arm_sequence_state==9&&g_arm_sequence_result==7&&g_arm_sequence_fault_state==2&&commands==2&&moves==1);
  assert(g_arm_sequence_phase_elapsed_ms>120000);
  g_arm_sequence_request=2;tick();assert(g_arm_sequence_fault_state==2);
  g_arm_sequence_request=3;tick();assert(!g_arm_sequence_fault_state&&!g_arm_sequence_phase_elapsed_ms&&!ArmSequence_IsBusy());
 }else if(is(s,"place_wait_resume")){
  grip();until(12);for(unsigned i=0;i<1000;i++){tick();}
  assert(g_arm_sequence_state==12&&commands==2&&g_arm_sequence_phase_elapsed_ms==20000);
  g_arm_place_confirm=1;tick();until(7);assert(commands==3);
 }else if(is(s,"place_timeout")){
  grip();until(12);for(unsigned i=0;i<6002;i++){tick();}
  assert(g_arm_sequence_state==9&&g_arm_sequence_result==7&&g_arm_sequence_fault_state==12&&commands==2&&ArmSequence_IsBusy());
 }else if(is(s,"legacy_timeout")){
  g_arm_sequence_mode=0;start();until(2);for(unsigned i=0;i<760;i++){tick();}
  assert(g_arm_sequence_state==9&&g_arm_sequence_result==7&&g_arm_sequence_fault_state==2&&g_arm_sequence_phase_elapsed_ms<16000);
 }else if(is(s,"service_gap")){
  start();until(2);now+=501;tick();assert(g_arm_sequence_state==9&&g_arm_sequence_result==8&&g_arm_sequence_fault_state==2);
 }else if(is(s,"abort_phase")){
  start();until(10);g_arm_sequence_request=2;tick();assert(g_arm_sequence_state==8&&g_arm_sequence_fault_state==10);
  unsigned elapsed=g_arm_sequence_phase_elapsed_ms;for(unsigned i=0;i<5;i++){tick();}assert(g_arm_sequence_phase_elapsed_ms==elapsed);
 }else if(is(s,"phase_clock")){
  start();until(2);for(unsigned i=0;i<10;i++){tick();}assert(g_arm_sequence_phase_elapsed_ms==200);
  g_vm_test_request=0;tick();assert(g_arm_sequence_fault_state==2);unsigned elapsed=g_arm_sequence_phase_elapsed_ms;
  for(unsigned i=0;i<10;i++){tick();}assert(g_arm_sequence_phase_elapsed_ms==elapsed);
  g_arm_sequence_request=3;tick();g_vm_test_request=1;config();start();assert(!g_arm_sequence_fault_state&&!g_arm_sequence_phase_elapsed_ms);
 }else if(is(s,"home_envelope")){
  home_stall=1;g_lift_home_state=8;g_lift_home_result=g_lift_home_reported=1;
  grip();until(11);for(unsigned i=0;i<800;i++){tick();}
  assert(g_arm_sequence_state==9&&g_arm_sequence_result==7&&g_arm_sequence_fault_state==11&&commands==2);
  assert(g_arm_sequence_phase_elapsed_ms<16000);
 }else assert(!"unknown case");
 puts(s);return 0;}
'''
(HOST/'test.c').write_text(s)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(HOST),'-I'+str(ROOT/'tmp/arm_sequence_fast_20261031/host-check'),'-I'+str(STAGE/'USER'),'-I'+str(STAGE/'HARDWARE/LIFT'),str(HOST/'test.c'),str(STAGE/'USER/arm_sequence.c'),'-o',str(HOST/'test.exe')]
r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
cases='grip_wait_resume grip_timeout place_wait_resume place_timeout legacy_timeout service_gap abort_phase phase_clock home_envelope'.split();logs=[]
for c in cases:
    r=subprocess.run([str(HOST/'test.exe'),c],capture_output=True,text=True);logs.append(c+': '+str(r.returncode)+' '+r.stdout+r.stderr)
    if r.returncode:print(''.join(logs));raise SystemExit(1)
(EV/'host-check.log').write_text('\n'.join(logs))
(EV/'host-review.json').write_text(json.dumps({'passed':True,'cases':cases,'actual_main_guard_unchanged_and_matched':True,'hardware_stubs':True,'source_sha256':{n:hashlib.sha256((STAGE/n).read_bytes()).hexdigest() for n in ['USER/arm_sequence.c','USER/arm_sequence.h']}},indent=2))
print(len(cases),'targeted wait/diagnostic scenarios passed; no hardware commands.')
