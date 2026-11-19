"""Verify actual wire frames and the real buzzer implementation, no hardware."""
from pathlib import Path
import json
import subprocess
import sys

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/source-mcu-cycle'
P=R/'tmp/source_mcu_cycle_20261115/project'

def main():
    sys.path.insert(0,str(R/'src'))
    from smart_carrier.vision_manipulation.mcu_cycle_protocol import frame,decode_status,observation
    wire=frame('SC1',[1,6,8,6,1,2,1,5,1,1,10]);parsed=decode_status(wire)
    assert parsed['colors']==[2,1,5] and parsed['state']==6 and parsed['item']==1
    rejected=0
    for i in range(len(wire)-2):
        bad=bytearray(wire);bad[i]^=1
        try:decode_status(bytes(bad))
        except (ValueError,UnicodeError):rejected+=1
        else:raise AssertionError(('corrupted status accepted',i))
    assert observation(1,2,1,True,12,0,10,False,[0,0,0],0)==(E/'python-observation.txt').read_bytes()
    for file in (R/'output/source_mcu_cycle_view_20261115.py',R/'src/smart_carrier/vision_manipulation/mcu_cycle_protocol.py'):
        compile(file.read_text(encoding='utf-8'),str(file),'exec')
    source=(P/'CHASSIS/HARDWARE/BUZZER/buzzer.c').read_bytes()
    source=b'\n'.join(x for x in source.splitlines() if not x.startswith(b'#include'))
    prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define APP_BEEP_ON_MS 200U
#define APP_BEEP_GAP_MS 200U
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define GPIOB 1U
#define GPIO_Pin_0 1U
#define RCC_AHB1Periph_GPIOB 1U
#define ENABLE 1U
#define GPIO_Mode_OUT 1U
#define GPIO_OType_PP 1U
#define GPIO_Speed_2MHz 1U
#define GPIO_PuPd_NOPULL 1U
typedef uint32_t TickType_t;
typedef struct{uint32_t GPIO_Pin,GPIO_Mode,GPIO_OType,GPIO_Speed,GPIO_PuPd;}GPIO_InitTypeDef;
static uint32_t tick,on,rises,falls,last_rise,last_fall;
static void RCC_AHB1PeriphClockCmd(unsigned a,unsigned b){(void)a;(void)b;}
static void GPIO_StructInit(GPIO_InitTypeDef*p){(void)p;}
static void GPIO_Init(unsigned a,GPIO_InitTypeDef*p){(void)a;(void)p;}
static void GPIO_ResetBits(unsigned a,unsigned b){(void)a;(void)b;if(on){falls++;last_fall=tick;}on=0;}
static void GPIO_SetBits(unsigned a,unsigned b){(void)a;(void)b;if(!on){rises++;last_rise=tick;}on=1;}
static uint32_t xTaskGetTickCount(void){return tick;}
void Buzzer_StartTimed(uint32_t,uint32_t,uint32_t);
'''
    tests=r'''
int main(void){Buzzer_Init();Buzzer_StartTimed(2,80,80);
 for(tick=0;tick<=300;tick+=10)Buzzer_Update();
 if(rises!=2 || falls!=2 || on || Buzzer_IsBusy() || last_rise!=160 || last_fall!=240)return 1;
 Buzzer_Start(1);tick=499;Buzzer_Update();if(!Buzzer_IsBusy())return 2;
 tick=510;Buzzer_Update();if(Buzzer_IsBusy())return 3;
 printf("PASS actual buzzer: two80ms pulses,80ms gap,240ms finish; existing200ms default preserved\n");return 0;}
'''
    file=E/'actual-buzzer.c';file.write_bytes(prefix.encode()+source+tests.encode())
    exe=E/'actual-buzzer.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(file),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True,check=True)
    (E/'buzzer-check.log').write_text(result.stdout,encoding='utf-8')
    summary=dict(passed=True,status_corruption_cases=rejected,python_status_and_observation_wire=True,
        actual_buzzer=result.stdout.strip(),hardware_access=False)
    (E/'python-and-buzzer-checks.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary))

if __name__=='__main__':main()
