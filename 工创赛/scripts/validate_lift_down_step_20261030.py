"""验证实际升降状态机、方向帧以及节点5传输边界；无硬件。"""
from pathlib import Path
import json,hashlib,shutil,subprocess
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/lift_down_step_20261030';STAGE=BASE/'project';HOST=BASE/'host-check'
EV=ROOT/'evidence/p2-arm/2026-10-30/lift-down-step'
HOST.mkdir(parents=True,exist_ok=True)
OLD=ROOT/'tmp/lift_sensorless_20261025/host-check'
for n in ['stm32f4xx.h','stm32f4xx_can.h','FreeRTOS.h','task.h','can.h']:
    shutil.copyfile(OLD/n,HOST/n)
can=(STAGE/'SYSTEM/CAN/can.c').read_text(encoding='utf-8',errors='replace')
node=can[can.index('uint32_t CAN1_LiftExchange('):]
guard=node[node.index('    speed_rpm ='):node.index('    memset(reply,')]
MODEL=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "lift.h"
#include "lift_home.h"
#include "lift_protocol.h"
#include "lift_config.h"
#include "FreeRTOS.h"
CAN_TypeDef fake_can={0};
static unsigned tick,position_reads,moves,stops,last_pulses,last_rpm,last_direction;
static int32_t pos;
static unsigned moving,emm=1,enabled=1,status_fault,speed_fail,move_fail,stop_fail,home_busy,no_delta;
volatile uint32_t g_lift_home_request,g_lift_home_state,g_lift_home_result;
unsigned LiftHome_IsBusy(void){return home_busy;}
void LiftHome_Service(uint32_t r){(void)r;}
TickType_t xTaskGetTickCount(void){return tick;}
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg*r){assert(a==5);memset(r,0,sizeof(*r));return 1;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){assert(a==5);r->Data[1]=(enabled?1:0)|(moving?0:2)|(status_fault?4:0);return 1;}
uint32_t CAN1_ReadMotorOptions(uint8_t a,CanRxMsg*r){assert(a==5);r->Data[2]=emm?6:4;return 1;}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*s,uint32_t*v){assert(a==5);(void)r;position_reads++;*s=pos<0;*v=(uint32_t)(pos<0?-pos:pos);return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v){assert(a==5);(void)r;*v=moving?1:0;return speed_fail?5:1;}
uint32_t CAN1_ReadLiftHomeParameters(uint8_t*p,CanRxMsg*r){(void)p;(void)r;return 5;}
static unsigned validate(const uint8_t*request,uint8_t length){uint32_t pulses,speed_rpm;
@@GUARD@@
return 1;}
uint32_t CAN1_LiftExchange(const uint8_t*q,uint8_t n,CanRxMsg*r){
 memset(r,0,sizeof(*r));r->Data[1]=2;
 if(q[0]==0xFE){stops++;moving=0;return stop_fail?5:1;}
 if(q[0]==0xF3){enabled=1;return 1;}
 assert(validate(q,n)==1);moves++;last_direction=q[1];
 last_pulses=((uint32_t)q[5]<<24)|((uint32_t)q[6]<<16)|((uint32_t)q[7]<<8)|q[8];
 last_rpm=((uint32_t)q[2]<<8)|q[3];
 if(!no_delta)pos+=q[1]?-(int32_t)last_pulses:(int32_t)last_pulses;
 return move_fail?5:1;
}
static void step(unsigned count){while(count--){tick+=10;Lift_Service(1);}}
static int is(const char*a,const char*b){return strcmp(a,b)==0;}
int main(int argc,char**argv){const char*s=argv[1];uint8_t q[12];(void)argc;
 step(120);assert(g_lift_state==4&&moves==0&&g_lift_down_pulses==5000&&g_lift_up_pulses==5000);
 if(is(s,"boot")){step(300);assert(moves==0);}
 else if(is(s,"down_default")||is(s,"down40000")||is(s,"down50000")){
  if(is(s,"down40000"))g_lift_down_pulses=40000;
  if(is(s,"down50000"))g_lift_down_pulses=50000;
  g_lift_motion_request=1;step(80);
  assert(g_lift_motion_state==7&&g_lift_motion_result==1&&last_direction==0&&last_rpm==300);
  assert(last_pulses==(is(s,"down_default")?5000:is(s,"down40000")?40000:50000));
  assert(g_lift_delta_raw==(int64_t)last_pulses&&g_lift_speed_rpm==0);
 }
 else if(is(s,"down0")||is(s,"down50001")||is(s,"hexmistake")){
  g_lift_down_pulses=is(s,"down0")?0:is(s,"down50001")?50001:0x40000;
  g_lift_motion_request=1;step(1);assert(moves==0&&g_lift_motion_state==9&&g_lift_motion_result==6);
 }
 else if(is(s,"up233333")){
  g_lift_up_pulses=233333;g_lift_down_pulses=50000;g_lift_motion_request=2;step(80);
  assert(last_pulses==233333&&last_rpm==3000&&last_direction==1&&g_lift_motion_state==7);
 }
 else if(is(s,"up0")||is(s,"up233334")){
  g_lift_up_pulses=is(s,"up0")?0:233334;g_lift_motion_request=2;step(1);
  assert(moves==0&&g_lift_motion_state==9&&g_lift_motion_result==6);
 }
 else if(is(s,"snapshot")||is(s,"busy")||is(s,"no_repeat")){
  g_lift_down_pulses=40000;g_lift_motion_request=1;step(1);
  g_lift_down_pulses=1;
  if(is(s,"busy"))g_lift_motion_request=1;
  step(80);assert(moves==1&&last_pulses==40000&&g_lift_motion_state==7);
  step(300);assert(moves==1);
 }
 else if(is(s,"stop")||is(s,"stop_fail")){
  moving=1;g_lift_down_pulses=40000;g_lift_motion_request=1;step(10);
  if(is(s,"stop_fail"))stop_fail=1;
  g_lift_motion_request=3;step(1);
  assert(stops==1&&g_lift_motion_state==(is(s,"stop")?8:9));
 }
 else if(is(s,"timeout")||is(s,"zero_delta")||is(s,"speed_fail")||is(s,"drive_fault")||is(s,"move_fail")){
  if(is(s,"timeout"))moving=1;
  if(is(s,"zero_delta"))no_delta=1;
  if(is(s,"speed_fail"))speed_fail=1;
  if(is(s,"drive_fault"))status_fault=1;
  if(is(s,"move_fail"))move_fail=1;
  g_lift_down_pulses=40000;g_lift_motion_request=1;step(1250);
  assert(g_lift_motion_state==9&&stops>=1);
 }
 else if(is(s,"bad_options")){
  emm=0;g_lift_motion_request=1;step(1);assert(moves==0&&g_lift_motion_state==9);
 }
 else if(is(s,"home_busy")){
  home_busy=1;g_lift_motion_request=1;step(20);assert(moves==0);
 }
 else if(is(s,"latch_recover")){
  g_lift_down_pulses=50001;g_lift_motion_request=1;step(1);
  g_lift_down_pulses=40000;g_lift_motion_request=1;step(80);assert(moves==0&&g_lift_motion_state==9);
  g_lift_motion_request=3;step(1);assert(g_lift_motion_state==8);
  g_lift_motion_request=1;step(80);assert(moves==1&&g_lift_motion_state==7);
 }
 else if(is(s,"protocol")){
  assert(LiftProtocol_Move(0,50000,q)&&q[2]==1&&q[3]==44&&q[7]==0xC3&&q[8]==0x50&&q[9]==2);
  assert(!LiftProtocol_Move(0,50001,q)&&!LiftProtocol_Move(0,0,q));
  assert(LiftProtocol_Move(1,233333,q)&&q[2]==11&&q[3]==184);
  assert(!LiftProtocol_Move(1,233334,q)&&!LiftProtocol_Move(2,1,q)&&!LiftProtocol_Move(0,1,0));
 }
 else if(is(s,"guard")){
  assert(LiftProtocol_Move(0,50000,q)&&validate(q,12)==1);
  q[8]++;assert(validate(q,12)==7);q[8]--;
  q[3]=45;assert(validate(q,12)==7);q[3]=44;
  q[2]=0;q[3]=0;assert(validate(q,12)==7);
  assert(LiftProtocol_Move(1,233333,q)&&validate(q,12)==1);
  q[8]++;assert(validate(q,12)==7);q[8]--;
  q[2]=11;q[3]=185;assert(validate(q,12)==7);
  assert(LiftProtocol_Move(0,40000,q));q[9]=0;assert(validate(q,12)==7);q[9]=2;
  q[10]=1;assert(validate(q,12)==7);q[10]=0;
  q[11]=0;assert(validate(q,12)==7);q[11]=0x6B;
  q[4]=0;assert(validate(q,12)==7);q[4]=250;
  q[1]=2;assert(validate(q,12)==7);q[1]=0;
  assert(validate(q,11)==7&&validate(q,0)==7);
 }
 else assert(!"unknown scenario");
 puts(s);return 0;
}
'''
(HOST/'test.c').write_text(MODEL.replace('@@GUARD@@',guard),encoding='utf-8')
def run(args):
    r=subprocess.run(args,capture_output=True,text=True,timeout=30)
    assert r.returncode==0,(args,r.stdout,r.stderr)
    return r.stdout.strip()
exe=HOST/'check.exe';module=STAGE/'HARDWARE/LIFT'
run(['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror',
     '-I'+str(HOST),'-I'+str(module),str(module/'lift.c'),str(module/'lift_protocol.c'),
     str(HOST/'test.c'),'-o',str(exe)])
cases=['boot','down_default','down40000','down50000','down0','down50001','hexmistake',
       'up233333','up0','up233334','snapshot','busy','no_repeat','stop','stop_fail',
       'timeout','zero_delta','speed_fail','drive_fault','move_fail','bad_options',
       'home_busy','latch_recover','protocol','guard']
outputs=[run([str(exe),c]) for c in cases]
m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
for n,h in m['protected'].items():assert hashlib.sha256((STAGE/n).read_bytes()).hexdigest()==h,n
report={'passed':True,'runtime_scenarios':len(cases),'cases':cases,
        'actual_state_machine_protocol_and_node5_guard_tested':True,
        'source_sha256':{e['file']:hashlib.sha256((STAGE/e['file']).read_bytes()).hexdigest() for e in m['files']},
        'protected_files':len(m['protected']),'physical_millimetre_calibration_or_move_verified':False}
(EV/'host-review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(EV/'host-check.log').write_text('\n'.join(outputs)+'\n',encoding='utf-8')
print('PASS:',len(cases),'C scenarios; direction limits/speeds, snapshot, failure/stop and actual node5 guard.')
