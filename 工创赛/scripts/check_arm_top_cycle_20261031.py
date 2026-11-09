"""只检查新增顶部往返及共享停止路径，硬件由桩替代。"""
from pathlib import Path
import subprocess,json,hashlib
ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tmp/arm_top_cycle_20261031/project'
EV=ROOT/'evidence/p2-arm/2026-10-31/arm-top-cycle'
HOST=ROOT/'tmp/arm_top_cycle_20261031/host-check'
HOST.mkdir(parents=True,exist_ok=True)
(HOST/'lift.h').write_text('#include <stdint.h>\nextern volatile uint32_t g_lift_state,g_lift_motion_state,g_lift_motion_result,g_lift_stop_result,g_lift_motion_request,g_lift_up_pulses,g_lift_down_pulses;\n')
(HOST/'lift_home.h').write_text('#include <stdint.h>\nextern volatile uint32_t g_lift_home_request,g_lift_home_state,g_lift_home_result,g_lift_home_reported,g_lift_home_abort_result,g_lift_home_stop_result;\nuint32_t LiftHome_IsBusy(void);\n')
source=(ROOT/'tmp/arm_sequence_fast_20261031/host-check/sequence-test.c').read_text()
source=source.replace('unsigned g_lift_home_request,g_lift_home_state,g_lift_read_request,g_lift_home_read_request;', '''volatile uint32_t g_lift_home_request,g_lift_home_state,g_lift_home_result,g_lift_home_reported,g_lift_home_abort_result,g_lift_home_stop_result;
unsigned g_lift_read_request,g_lift_home_read_request;
unsigned home_ticks,home_starts,home_abort,home_stall,home_fail,home_stop_fail;''')
source=source.replace('unsigned LiftHome_IsBusy(void){return 0;}', 'unsigned LiftHome_IsBusy(void){return g_lift_home_state>=1&&g_lift_home_state<=7;}')
main=(STAGE/'USER/main.c').read_bytes().decode('latin1')
start=main.index('static uint32_t App_ArmSequenceReady(void)');end=main.index('static void App_CarrierStep(void)',start)
a=source.index('static uint32_t App_ArmSequenceReady(void)');b=source.index('static void tick(void)',a)
source=source[:a]+main[start:end]+source[b:]
source=source.replace(' if(g_lift_home_request==2)g_lift_home_request=0;', ''' if(g_lift_home_request==2){g_lift_home_request=0;home_ticks=0;home_abort++;g_lift_home_state=home_stop_fail?9:10;g_lift_home_abort_result=g_lift_home_stop_result=home_stop_fail?0:1;}
 else if(g_lift_home_request==1){g_lift_home_request=0;home_starts++;if(!home_stall){g_lift_home_state=1;g_lift_home_reported=0;home_ticks=4;}}
 else if(home_ticks){home_ticks--;if(home_ticks)g_lift_home_state++;else {g_lift_home_state=home_fail?9:8;g_lift_home_result=home_fail?5:1;g_lift_home_reported=home_fail?0:1;}}''')
source=source.replace('ArmSequence_Service(App_ArmSequenceReady(),carrier_busy||other_busy,now);', 'ArmSequence_Service(App_ArmSequenceReady(),carrier_busy||other_busy||(LiftHome_IsBusy()&&!ArmSequence_OwnsHome())||(g_lift_home_state==9&&!ArmSequence_OwnsHome()),now);')
source=source[:source.index('int main(')]+r'''
int main(int argc,char**argv){const char*s=argv[1];(void)argc;
 g_arm_sequence_mode=1;g_arm_return_down_pulses=40000;
 if(is(s,"default")){assert(g_arm_sequence_enable==0&&g_arm_grip_height_ready==0);g_arm_sequence_request=1;tick();assert(!commands&&!moves&&!home_starts);}
 else if(is(s,"invalid_return")||is(s,"invalid_mode")||is(s,"zero_return")){
  if(is(s,"invalid_mode"))g_arm_sequence_mode=2;
  if(is(s,"invalid_return"))g_arm_return_down_pulses=50001;
  if(is(s,"zero_return"))g_arm_return_down_pulses=0;
  start();assert(g_arm_sequence_result==4&&!commands&&!home_starts);
 }else if(is(s,"complete")||is(s,"snapshot")||is(s,"legacy")){
  if(is(s,"legacy"))g_arm_sequence_mode=0;
  start();wait_grip();
  if(is(s,"snapshot")){g_arm_sequence_mode=0;g_arm_return_down_pulses=50001;}
  g_arm_grip_confirm=1;
  for(unsigned i=0;i<150&&g_arm_sequence_state!=7;i++)tick();
  assert(g_arm_sequence_state==7&&g_arm_sequence_result==1&&!ArmSequence_IsBusy());
  assert(commands==2&&angles[0]==140&&angles[1]==180&&g_lift_up_pulses==8123&&g_lift_down_pulses==40000);
  if(is(s,"legacy"))assert(home_starts==0&&moves==2&&pulses[0]==5000&&pulses[1]==5000);
  else assert(home_starts==1&&moves==1&&directions[0]==1&&pulses[0]==40000&&g_arm_grip_height_ready==0);
  for(unsigned i=0;i<100;i++){tick();}assert(commands==2);
 }else if(is(s,"fresh_confirm")){
  g_arm_grip_confirm=1;start();wait_grip();for(unsigned i=0;i<30;i++)tick();assert(!home_starts&&!moves&&g_arm_grip_confirm==0);
 }else if(is(s,"stale_home")){
  home_stall=1;g_lift_home_state=8;g_lift_home_result=g_lift_home_reported=1;
  start();wait_grip();g_arm_grip_confirm=1;for(unsigned i=0;i<30;i++)tick();
  assert(g_arm_sequence_state==3&&commands==1&&!moves);
 }else if(is(s,"abort_pending")||is(s,"abort_active")||is(s,"external_stop")||is(s,"stop_unknown")){
  start();wait_grip();g_arm_grip_confirm=1;tick();assert(g_lift_home_request==1&&!home_starts);
  if(!is(s,"abort_pending"))tick();
  if(is(s,"stop_unknown"))home_stop_fail=1;
  if(is(s,"external_stop"))g_lift_motion_request=3;else g_arm_sequence_request=2;
  tick();assert(g_arm_sequence_state==8&&home_abort==1&&commands==1&&!moves&&g_arm_grip_height_ready==0);
  if(is(s,"abort_pending"))assert(!home_starts);
  g_arm_sequence_request=3;tick();
  if(is(s,"stop_unknown")){assert(ArmSequence_IsBusy());home_stop_fail=0;g_lift_home_request=2;tick();g_arm_sequence_request=3;tick();}
  assert(!ArmSequence_IsBusy());
 }else if(is(s,"home_fail")||is(s,"home_timeout")||is(s,"power_loss")){
  if(is(s,"home_fail"))home_fail=1;
  if(is(s,"home_timeout"))home_stall=1;
  start();wait_grip();g_arm_grip_confirm=1;tick();tick();
  if(is(s,"power_loss"))g_vm_test_request=0;
  unsigned count=is(s,"home_timeout")?800:15;for(unsigned i=0;i<count;i++)tick();
  assert(g_arm_sequence_state==9&&ArmSequence_IsBusy()&&commands==1&&!moves&&g_arm_grip_height_ready==0);
 }else if(is(s,"return_fail")){
  lift_fail=1;start();wait_grip();g_arm_grip_confirm=1;for(unsigned i=0;i<100;i++)tick();
  assert(g_arm_sequence_state==9&&commands==1&&moves==1);
 }else if(is(s,"guard")){
  start();g_lift_home_request=1;g_dm_jog_request=4;g_extension_motion_request=2;g_carrier_request=1;App_ArmSequenceGuard();
  assert(!g_lift_home_request&&!g_dm_jog_request&&!g_extension_motion_request&&!g_carrier_request);
  wait_grip();g_arm_grip_confirm=1;tick();assert(g_lift_home_request==1);App_ArmSequenceGuard();assert(g_lift_home_request==1);
 }else assert(!"unknown case");
 puts(s);return 0;}
'''
(HOST/'test.c').write_text(source)
cmd=['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(HOST),'-I'+str(ROOT/'tmp/arm_sequence_fast_20261031/host-check'),'-I'+str(STAGE/'USER'),'-I'+str(STAGE/'HARDWARE/LIFT'),str(HOST/'test.c'),str(STAGE/'USER/arm_sequence.c'),'-o',str(HOST/'test.exe')]
r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
cases='default invalid_return invalid_mode zero_return complete snapshot legacy fresh_confirm stale_home abort_pending abort_active external_stop stop_unknown home_fail home_timeout power_loss return_fail guard'.split()
logs=[]
for c in cases:
    r=subprocess.run([str(HOST/'test.exe'),c],capture_output=True,text=True);logs.append(c+': '+str(r.returncode)+' '+r.stdout+r.stderr)
    if r.returncode: print(''.join(logs));raise SystemExit(1)
(EV/'host-check.log').write_text('\n'.join(logs))
(EV/'host-review.json').write_text(json.dumps({'passed':True,'cases':cases,'actual_main_guard_extracted':True,'hardware_stubs':True,'source_sha256':{n:hashlib.sha256((STAGE/n).read_bytes()).hexdigest() for n in ['USER/arm_sequence.c','USER/arm_sequence.h','USER/main.c']}},indent=2))
print(len(cases),'targeted software cases passed; no hardware commands.')
