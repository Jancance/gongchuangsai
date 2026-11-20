from pathlib import Path
import json
from radar_zone2_arm_runner_20261111 import run
R=Path.cwd();E=R/'工创赛/evidence/navigation-planning/2026-11-12/radar-zone2/us733';B=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
p=R/'3.stm32f407工程/CHASSIS/USER/main.c';s=p.read_bytes().decode('gbk');a=s.index('static void App_RadarTask(void *argument)\r\n{') if '\r\n' in s else s.index('static void App_RadarTask(void *argument)\n{');b=s.index('{',a);depth=1;i=b+1
while depth:
 if s[i]=='{':depth+=1
 if s[i]=='}':depth-=1
 i+=1
body=s[a:i].replace('for (;;)','for(test_iteration=0;test_iteration<test_loops;test_iteration++)')
report={}
for restored in [False,True]:
 T=R/('工创赛/tmp/radar_us733_20261112/main_tests_'+str(int(restored)));T.mkdir(parents=True,exist_ok=True)
 for n in ['radar_observer.h','radar_client.h','radar_uart_port.h']:(T/n).write_bytes((R/'3.stm32f407工程/CHASSIS/HARDWARE/RADAR_V6'/n).read_bytes())
 product=body
 if restored:product=product.replace('/* g_radar_app_calibration_result = RadarCalibration_StartZone2() ? 1U : 2U; */','g_radar_app_calibration_result = RadarCalibration_StartZone2() ? 1U : 2U;')
 source=r'''
#include "radar_observer.h"
#include "radar_client.h"
volatile unsigned failed,assertions,cases;
#define CHECK(x) do{assertions++;if(!(x)){failed=__LINE__;return;}}while(0)
#define pdMS_TO_TICKS(x) (x)
static uint32_t g_radar_app_config_result,g_radar_app_calibration_result,g_radar_app_scan_result;
static uint32_t test_iteration,test_loops=8,mode,scan_calls,cal_calls,scan_iteration,configured_steps,first_scan;
static RadarObserverStatus fixture;
bool RadarObserver_Configure(const RadarZone2Config *c){configured_steps=c->steps;return true;}
void RadarObserver_Service(void){fixture.stage=0;fixture.error=0;fixture.calibration_complete=0;
 if(mode==2){fixture.stage=99;return;}if(mode==3){fixture.error=3;return;}
 if(test_iteration>=5){fixture.stage=8;fixture.calibration_complete=1;}
 if(mode==4)fixture.calibration_complete=0;
}
void RadarObserver_GetStatus(RadarObserverStatus *s){*s=fixture;}
bool RadarClient_IsReady(void){return mode!=5 && test_iteration>=2;}
bool RadarCalibration_StartZone2(void){cal_calls++;return mode!=1;}
bool RadarObserver_StartZone2(uint8_t start,uint8_t goal,bool ref){scan_calls++;scan_iteration=test_iteration;first_scan=start==0&&goal==24&&ref;return true;}
static void vTaskDelay(uint32_t t){(void)t;}
'''+product+r'''
unsigned test_entry(void){uint32_t k;
 for(k=0;k<6;k++){
  mode=k;g_radar_app_calibration_result=g_radar_app_config_result=g_radar_app_scan_result=0;scan_calls=cal_calls=first_scan=0;
  App_RadarTask(0);CHECK(configured_steps==8559);
''' + (r'''
  if(k==0){CHECK(cal_calls==1&&scan_calls==1&&scan_iteration==5&&first_scan&&g_radar_app_scan_result==1);}
  if(k==1){CHECK(cal_calls==1&&g_radar_app_calibration_result==2&&!scan_calls);}
  if(k==2||k==3||k==5){CHECK(!scan_calls&&!cal_calls);}
  if(k==4){CHECK(cal_calls==1&&!scan_calls);}
''' if restored else r'''
  CHECK(cal_calls==0);
  if(k==0||k==1||k==4){CHECK(scan_calls==1&&scan_iteration==3&&first_scan&&g_radar_app_scan_result==1);}
  if(k==2||k==3||k==5){CHECK(!scan_calls);}
''')+r'''
  cases++;
 }
 return failed;
}
'''
 (T/'unit.c').write_text(source,'utf8');(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }')
 commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I.','-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
 report['restored_calibration' if restored else 'default_commented_calibration']=run(R,T,B,source,commands)
report.update(actual_main_task_body=True,only_infinite_loop_bounded_in_fixture=True,external_observer_client_stubs=True,hardware=False)
(E/'main_arm_result.json').write_text(json.dumps(report,indent=2),'utf8')
