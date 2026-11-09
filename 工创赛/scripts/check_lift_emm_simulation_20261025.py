"""Native fake-bus checks exercise the actual staged lift state machine."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'tmp/lift_emm_jog_20261025/host-check'
MODULE=ROOT/'tmp/lift_emm_jog_20261025/project/HARDWARE/LIFT'
FILES={
'stm32f4xx.h':'''#ifndef FAKE_STM_H
#define FAKE_STM_H
#include <stdint.h>
typedef struct { uint32_t ESR; } CAN_TypeDef;
extern CAN_TypeDef fake_can;
#define CAN1 (&fake_can)
#endif
''',
'stm32f4xx_can.h':'''#ifndef FAKE_CAN_H
#define FAKE_CAN_H
#include <stdint.h>
typedef struct { uint32_t StdId,ExtId; uint8_t IDE,RTR,DLC,Data[8],FMI; } CanRxMsg;
#endif
''',
'FreeRTOS.h':'''#ifndef FAKE_RTOS_H
#define FAKE_RTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#endif
''',
'task.h':'#include "FreeRTOS.h"\nTickType_t xTaskGetTickCount(void);\n',
'can.h':'''#include "stm32f4xx_can.h"
uint32_t CAN1_ReadVersion(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadStatus(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadPosition(uint8_t,CanRxMsg*,uint8_t*,uint32_t*);
uint32_t CAN1_ReadSpeed(uint8_t,CanRxMsg*,int32_t*);
uint32_t CAN1_ReadMotorOptions(uint8_t,CanRxMsg*);
uint32_t CAN1_LiftExchange(const uint8_t*,uint8_t,CanRxMsg*);
''',
'test.c':'''#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "lift.h"
#include "lift_protocol.h"
#include "FreeRTOS.h"

CAN_TypeDef fake_can={0};
static TickType_t ticks=0;
static int32_t position=0;
static unsigned enabled=1, moving=0, fail_speed=0, emm=1, moves=0, stops=0, enables=0;
TickType_t xTaskGetTickCount(void) { return ticks; }
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg*r) { assert(a==5); memset(r,0,sizeof(*r));return 1; }
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r) { assert(a==5);r->Data[1]=(uint8_t)(enabled|(moving?0:2));return 1; }
uint32_t CAN1_ReadMotorOptions(uint8_t a,CanRxMsg*r) { assert(a==5);r->Data[1]=(uint8_t)(emm?2:0);return 1; }
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*s,uint32_t*v)
{ assert(a==5);(void)r;*s=position<0;*v=(uint32_t)(position<0?-position:position);return 1; }
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v)
{ assert(a==5);(void)r;*v=moving?2:0;return fail_speed?5:1; }
uint32_t CAN1_LiftExchange(const uint8_t*q,uint8_t n,CanRxMsg*r)
{
    memset(r,0,sizeof(*r));r->Data[1]=2;
    if(q[0]==0xF3){assert(n==5 && q[2]==1);enabled=1;enables++;}
    else if(q[0]==0xFE){assert(n==4);stops++;moving=0;}
    else {assert(q[0]==0xFD && n==12 && q[9]==2 && q[8]==16);position+=q[1]?-16:16;moves++;}
    return 1;
}
static void step(unsigned count)
{ while(count--){ ticks+=10;Lift_Service(1); } }
static void ready(void)
{ Lift_Service(0);moving=0;fail_speed=0;emm=1;step(110);assert(g_lift_state==4); }
int main(void)
{
    uint8_t q[12];
    const uint8_t expected[12]={0xFD,0,0,5,20,0,0,0,16,2,0,0x6B};
    assert(LiftProtocol_Move(0,16,q) && memcmp(q,expected,12)==0);
    assert(!LiftProtocol_Move(2,16,q));assert(!LiftProtocol_Move(0,0,q));
    assert(!LiftProtocol_Move(0,17,q));
    ready();assert(moves==0);g_lift_motion_request=1;step(80);
    assert(g_lift_motion_state==7 && g_lift_motion_result==1 && g_lift_delta_raw==16);
    assert(moves==1);
    g_lift_motion_request=2;step(80);
    assert(g_lift_motion_state==7 && g_lift_delta_raw==-16 && moves==2);
    ready();enabled=0;g_lift_motion_request=1;step(80);
    assert(enables==1 && g_lift_motion_state==7);
    ready();fail_speed=1;g_lift_motion_request=1;step(80);
    assert(g_lift_motion_state==9 && g_lift_motion_result==4 && stops>0);
    ready();emm=0;{unsigned before=moves;g_lift_motion_request=1;step(20);
    assert(g_lift_motion_state==9 && moves==before);}
    ready();moving=1;g_lift_motion_request=1;step(15);
    {unsigned i;for(i=0;i<310;i++){g_lift_motion_request=1;step(1);}}
    assert(g_lift_motion_state==9 && g_lift_motion_result==3);
    ready();g_lift_motion_request=1;step(6);g_lift_motion_request=3;step(1);
    assert(g_lift_motion_state==8 && g_lift_motion_result==2);
    ready();g_lift_motion_request=1;step(6);Lift_Service(0);
    assert(g_lift_motion_state==9 && g_lift_motion_result==6);
    puts("PASS: packet, input guards, startup idle, both directions, enable-only, feedback failure stop, firmware mismatch, busy timeout, cancel, power loss");
    return 0;
}
'''}

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    for name,content in FILES.items():(OUT/name).write_text(content,encoding='utf-8')
    cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror',
         '-I'+str(OUT),'-I'+str(MODULE),str(MODULE/'lift.c'),
         str(MODULE/'lift_protocol.c'),str(OUT/'test.c'),'-o',str(OUT/'test.exe')]
    p=subprocess.run(cmd,capture_output=True,text=True,encoding='utf-8',errors='replace')
    if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    p=subprocess.run([str(OUT/'test.exe')],capture_output=True,text=True)
    log=ROOT/'evidence/p2-arm/2026-10-25/lift-emm-jog/host-check.log'
    log.write_text(p.stdout+p.stderr,encoding='utf-8')
    print(p.stdout+p.stderr)
    if p.returncode:raise RuntimeError('Fake bus state machine checks failed')
if __name__=='__main__':main()
