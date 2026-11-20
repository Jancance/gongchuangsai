from pathlib import Path
import re,subprocess,sys,struct,json
R=Path.cwd();T=R/'工创赛/tmp/radar_zone2_20261111/observer_tests';T.mkdir(exist_ok=True);E=R/'工创赛/evidence/navigation-planning/2026-11-11/radar-zone2';B=Path('D:/Keil_v5/ARM/arm5_compiler/bin');P=R/'3.stm32f407工程/CHASSIS/HARDWARE/RADAR_V6'
for n in ['radar_client.h','radar_uart_port.h','radar_observer.h']:(T/n).write_bytes((P/n).read_bytes())
(T/'radar_lift.h').write_bytes((P.parent/'RADAR_SERVO/radar_lift.h').read_bytes())
(T/'radar_servo.h').write_text('#include <stdint.h>\nuint32_t RadarServo_SetAngle(uint32_t,float);\n')
(T/'stm32f4xx.h').write_text('')
(T/'FreeRTOS.h').write_text('#include <stdint.h>\n#define portTICK_PERIOD_MS 1U\n#define pdMS_TO_TICKS(x) (x)\n')
(T/'task.h').write_text('#include <stdint.h>\nuint32_t xTaskGetTickCount(void);\nvoid vTaskDelay(uint32_t);\n')
product=(P/'radar_observer.c').read_text('utf8');a=product.index('static void configure_uart(void)');b=product.index('bool RadarObserver_Configure',a);product=product[:a]+'static void configure_uart(void) {}\n'+product[b:]
source=r'''
#include <stdint.h>
#include <string.h>
#include "radar_client.h"
#include "radar_lift.h"
volatile unsigned failed,assertions,cases;
static uint32_t clock_ms,outputs,up_calls,scan_calls,plan_calls,cal_calls,stop_calls,safe=1U,busy;
static RadarLiftSnapshot fixture_lift;
static radar_client_callbacks_t cb;
#define CHECK(x) do{assertions++;if(!(x)){failed=__LINE__;return;}}while(0)
uint32_t xTaskGetTickCount(void){return clock_ms;}void vTaskDelay(uint32_t t){clock_ms+=t;}
uint32_t RadarUartPort_EnterCritical(void){return 0;}void RadarUartPort_ExitCritical(uint32_t key){(void)key;}
uint32_t RadarServo_SetAngle(uint32_t id,float a){(void)id;(void)a;outputs++;return 1;}
uint32_t RadarLift_Available(void){return 1U;}
uint32_t RadarLift_SafeIdle(uint32_t t){(void)t;return safe;}
uint32_t RadarLift_Begin(uint32_t n,uint32_t d,uint32_t *seq){(void)n;outputs++;if(d)up_calls++;fixture_lift.sequence++;*seq=fixture_lift.sequence;fixture_lift.state=1;fixture_lift.result=1;fixture_lift.done=0;return 1;}
void RadarLift_Cancel(void){stop_calls++;fixture_lift.state=3;}
void RadarLift_GetSnapshot(RadarLiftSnapshot *s){*s=fixture_lift;}
void RadarClient_Init(const radar_client_callbacks_t *c,uint8_t node){(void)node;cb=*c;}
void RadarClient_Poll(void){}bool RadarClient_IsReady(void){return true;}bool RadarClient_IsBusy(void){return busy!=0;}
bool RadarClient_GetState(radar_client_state_t *s){memset(s,0,sizeof(*s));s->ready=true;s->selected_zone=2;return true;}
radar_client_request_result_t RadarClient_RequestSelectZone(uint8_t z){(void)z;return RADAR_CLIENT_REQUEST_ACCEPTED;}
radar_client_request_result_t RadarClient_RequestStop(void){stop_calls++;busy=0;return RADAR_CLIENT_REQUEST_ACCEPTED;}
radar_client_request_result_t RadarClient_RequestGridScan(void){scan_calls++;busy=1;return RADAR_CLIENT_REQUEST_ACCEPTED;}
radar_client_request_result_t RadarClient_RequestCalibrate(uint8_t z){(void)z;cal_calls++;busy=1;return RADAR_CLIENT_REQUEST_ACCEPTED;}
radar_client_request_result_t RadarClient_RequestPlanPath(uint8_t a,uint8_t b){(void)a;(void)b;plan_calls++;busy=1;return RADAR_CLIENT_REQUEST_ACCEPTED;}
'''+product+r'''
static void reset(uint32_t wrap){
 memset(&config,0,sizeof(config));memset(&pending_config,0,sizeof(pending_config));memset(&status,0,sizeof(status));memset(&published_status,0,sizeof(published_status));
 command=configure_pending=cancel_pending=owner_busy=initialized=0;clock_ms=wrap;outputs=up_calls=scan_calls=plan_calls=cal_calls=stop_calls=busy=0;safe=1;
}
static void configure(void){RadarZone2Config c={7,0,8,5,5,5,100,1};CHECK(RadarObserver_Configure(&c));RadarObserver_Service();}
static void until_scan(void){
 CHECK(RadarObserver_StartZone2(0,3,true));CHECK(!RadarObserver_StartZone2(1,4,true));RadarObserver_Service();CHECK(status.stage==1 && outputs==1);
 clock_ms+=4;RadarObserver_Service();CHECK(status.stage==1 && outputs==1);
 clock_ms++;RadarObserver_Service();CHECK(status.stage==2 && outputs==2);
 RadarObserver_Service();CHECK(status.stage==2);
 fixture_lift.state=2;fixture_lift.done=8;RadarObserver_Service();CHECK(status.stage==3);
 clock_ms+=4;RadarObserver_Service();CHECK(status.stage==3);
 clock_ms++;RadarObserver_Service();CHECK(status.stage==4 && scan_calls==1);
}
static void normal(void){
 radar_client_grid_result_t grid={10,0,{0}};radar_client_path_result_t p;RadarObserverPath result;
 reset(0);RadarObserver_Service();CHECK(outputs==0 && !RadarObserver_StartZone2(0,3,true));configure();until_scan();
 cb.grid(&grid);RadarObserver_Service();CHECK(status.stage==4 && up_calls==0);busy=0;RadarObserver_Service();CHECK(status.stage==5 && up_calls==1);
 RadarObserver_Service();CHECK(status.stage==5);fixture_lift.state=2;fixture_lift.done=8;RadarObserver_Service();CHECK(status.stage==6);
 clock_ms+=5;RadarObserver_Service();CHECK(status.stage==7 && plan_calls==1);
 memset(&p,0,sizeof(p));p.goal=3;p.waypoint_count=1;p.waypoints[0]=3;cb.path(&p);RadarObserver_Service();CHECK(!RadarObserver_GetPath(&result));busy=0;RadarObserver_Service();CHECK(RadarObserver_GetPath(&result)&&status.stage==8 && result.scan_sequence==10);cases++;
}
static void failures(void){unsigned stage_id,kind;
 for(stage_id=1;stage_id<=7;stage_id++)for(kind=0;kind<3;kind++){
  reset(0xfffffff0);configure();status.stage=stage_id;owner_busy=1;entered=clock_ms;
  if(kind==0)RadarObserver_Cancel();if(kind==1)safe=0;if(kind==2)clock_ms+=101;
  RadarObserver_Service();CHECK(status.stage==99 && up_calls==0 && !status.path_valid);RadarObserver_Service();CHECK(up_calls==0);cases++;
 }
 reset(0xfffffff0);configure();until_scan();CHECK(status.stage==4);cases++;
 reset(0);configure();status.stage=4;owner_busy=1;status.error=3;RadarObserver_Service();CHECK(status.stage==99 && up_calls==0);cases++;
}
static void calibration_case(void){radar_client_calibration_t c;reset(0);configure();CHECK(RadarCalibration_StartZone2());RadarObserver_Service();clock_ms+=5;RadarObserver_Service();fixture_lift.state=2;fixture_lift.done=8;RadarObserver_Service();clock_ms+=5;RadarObserver_Service();CHECK(cal_calls==1&&status.stage==4);memset(&c,0,sizeof(c));c.pose.zone=2;cb.calibration(&c);RadarObserver_Service();CHECK(status.stage==4);busy=0;RadarObserver_Service();CHECK(status.calibration_complete&&status.stage==5&&cal_calls==1);cases++;}

static void protocol_error(void){radar_client_error_t e;reset(0);configure();status.stage=4;owner_busy=1;memset(&e,0,sizeof(e));cb.error(&e);RadarObserver_Service();CHECK(status.stage==99&&status.error==3);cases++;}
static void queued_cancel(void){unsigned prior,calmode;
 for(prior=0;prior<2;prior++)for(calmode=0;calmode<2;calmode++){
  reset(0);configure();status.stage=prior?8:0;status.path_valid=1;status.map_valid=1;status.calibration_complete=1;published_status=status;
  if(calmode)CHECK(RadarCalibration_StartZone2());else CHECK(RadarObserver_StartZone2(0,3,true));
  RadarObserver_Cancel();RadarObserver_Service();CHECK(status.stage==99&&status.error==4&&status.sequence==1&&!status.path_valid&&!status.map_valid&&!status.calibration_complete&&outputs==0);cases++;
 }
}
unsigned test_entry(void){normal();if(failed)return failed;failures();if(failed)return failed;calibration_case();if(failed)return failed;queued_cancel();if(failed)return failed;protocol_error();return failed;}
'''
(T/'unit.c').write_text(source,'utf8');(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }')
commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I.','-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
from radar_zone2_arm_runner_20261111 import run
result=run(R,T,B,source,commands)
(E/'observer_arm_result.json').write_text(json.dumps({**result,'external_client_lift_servo_stub':True,'configure_uart_body_stub':True,'hardware':False},indent=2),'utf8')
