from pathlib import Path
import json
R=Path.cwd();T=R/'工创赛/tmp/radar_speed80_20261111/lift_tests';T.mkdir(exist_ok=True)
E=R/'工创赛/evidence/navigation-planning/2026-11-11/radar-zone2/speed80';B=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
P=R/'3.stm32f407工程/CHASSIS/HARDWARE/RADAR_SERVO'
(T/'radar_lift.h').write_bytes((P/'radar_lift.h').read_bytes())
(T/'zdt_emm_config.h').write_text('#define APP_MOTOR_OFFLINE_ONLY 0\n')
(T/'app_entry_config.h').write_text('#define APP_IS_DEBUG 1\n')
(T/'stm32f4xx.h').write_text(r"""
#include <stdint.h>
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
typedef struct { uint16_t BSRRL,BSRRH; } GPIO_TypeDef;
static GPIO_TypeDef bank_b,bank_d;
#define GPIOB (&bank_b)
#define GPIOD (&bank_d)
typedef struct { uint32_t GPIO_Pin,GPIO_Mode,GPIO_OType,GPIO_PuPd,GPIO_Speed; } GPIO_InitTypeDef;
#define RCC_AHB1Periph_GPIOB 1U
#define RCC_AHB1Periph_GPIOD 2U
#define ENABLE 1U
#define GPIO_Pin_8 256U
#define GPIO_Pin_10 1024U
#define GPIO_Pin_11 2048U
#define GPIO_Pin_12 4096U
#define GPIO_Mode_OUT 1U
#define GPIO_OType_PP 0U
#define GPIO_PuPd_DOWN 0U
#define GPIO_Speed_2MHz 0U
static void RCC_AHB1PeriphClockCmd(uint32_t a,uint32_t b){(void)a;(void)b;}
static void GPIO_ResetBits(GPIO_TypeDef *a,uint16_t b){(void)a;(void)b;}
static void GPIO_StructInit(GPIO_InitTypeDef *a){a->GPIO_Pin=0U;}
static void GPIO_Init(GPIO_TypeDef *a,GPIO_InitTypeDef *b){(void)a;(void)b;}
""")
product=(P/'radar_lift.c').read_text('utf8')
source=product+r"""
volatile unsigned failed,assertions,cases;
#define CHECK(x) do{assertions++;if(!(x)){failed=__LINE__;return;}}while(0)
static void reset(void){initialized=0;phase=7;last_tick=start_tick=total=direction=active_budget=0;pending_long=pending_steps=pending_direction=pending_budget=pending_interval=claimed=0;g_radar_lift_interval_ms=4;g_radar_lift_enable=g_radar_lift_request=g_radar_lift_state=g_radar_lift_result=g_radar_lift_sequence=g_radar_lift_done=g_radar_lift_phase_bits=0;g_radar_lift_steps=32;g_radar_lift_direction=0;}
static void limits(void){uint32_t s;
 reset();CHECK(!RadarLift_Begin(1,0,&s)&&!RadarLift_BeginBounded(65,0,1000,&s));g_radar_lift_enable=1;
 CHECK(!RadarLift_Begin(65,0,&s));CHECK(!RadarLift_BeginBounded(0,0,1000,&s));CHECK(!RadarLift_BeginBounded(65536,0,1000000,&s));CHECK(!RadarLift_BeginBounded(65,2,1000,&s));CHECK(!RadarLift_BeginBounded(65,0,259,&s));CHECK(!RadarLift_BeginBounded(65,0,0x80000000U,&s));
 g_radar_lift_steps=65;g_radar_lift_request=1;RadarLift_Service(0,1);CHECK(g_radar_lift_result==3&&g_radar_lift_state==0);cases++;
 CHECK(RadarLift_Begin(64,0,&s));RadarLift_Service(0,1);for(s=4;s<=256;s+=4)RadarLift_Service(s,1);CHECK(g_radar_lift_state==2&&g_radar_lift_done==64);cases++;
}
static void queue_and_lock(void){uint32_t s,i;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_BeginBounded(65,0,1000,&s));CHECK(!RadarLift_Begin(1,0,&s));CHECK(!RadarLift_BeginBounded(0,0,0,&s));CHECK(!RadarLift_BeginBounded(100,1,1000,&s));
 RadarLift_Service(0,1);CHECK(g_radar_lift_state==1&&total==65&&active_budget==1000&&direction==0);
 g_radar_lift_steps=1;g_radar_lift_direction=1;CHECK(!RadarLift_BeginBounded(2,1,8,&s));
 for(i=4;i<=260;i+=4)RadarLift_Service(i,1);CHECK(g_radar_lift_state==2&&g_radar_lift_done==65&&direction==0);cases++;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_BeginBounded(65535,1,262140,&s));RadarLift_Cancel();RadarLift_Service(0,1);CHECK(!pending_long&&g_radar_lift_done==0&&g_radar_lift_result==5);g_radar_lift_steps=65;g_radar_lift_request=1;RadarLift_Service(1,1);CHECK(g_radar_lift_result==3&&g_radar_lift_done==0);cases++;
}
static void long_delay_and_wrap(void){uint32_t s,i,start=0xffffff00U;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_BeginBounded(16384,0,90000,&s));RadarLift_Service(0,1);for(i=5;i<=81920;i+=5)RadarLift_Service(i,1);CHECK(g_radar_lift_state==2&&g_radar_lift_done==16384);cases++;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_BeginBounded(65,1,1000,&s));RadarLift_Service(start,1);for(i=4;i<=260;i+=4)RadarLift_Service(start+i,1);CHECK(g_radar_lift_state==2&&g_radar_lift_done==65);cases++;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_BeginBounded(65,0,1000,&s));RadarLift_Service(0,1);RadarLift_Service(100,1);CHECK(g_radar_lift_done==1);RadarLift_Service(101,1);CHECK(g_radar_lift_done==1);cases++;
}
static void stops(void){uint32_t s,bits;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_BeginBounded(65,0,260,&s));RadarLift_Service(0,1);RadarLift_Service(4,1);bits=g_radar_lift_phase_bits;RadarLift_Service(261,1);CHECK(g_radar_lift_state==3&&g_radar_lift_done==1&&g_radar_lift_phase_bits==bits);cases++;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_BeginBounded(65,0,1000,&s));RadarLift_Service(0,1);RadarLift_Service(4,1);bits=g_radar_lift_phase_bits;RadarLift_Cancel();RadarLift_Service(5,1);CHECK(g_radar_lift_state==3&&g_radar_lift_result==5&&g_radar_lift_done==1&&g_radar_lift_phase_bits==bits);cases++;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_BeginBounded(65,0,1000,&s));RadarLift_Service(0,1);RadarLift_Service(4,1);bits=g_radar_lift_phase_bits;RadarLift_Service(5,0);CHECK(g_radar_lift_state==3&&g_radar_lift_result==2&&g_radar_lift_done==1&&g_radar_lift_phase_bits==bits);cases++;
}
static void speed_release(void){uint32_t seq,i,bits;
 for(i=1;i<=4;i++){
  reset();g_radar_lift_enable=1;g_radar_lift_interval_ms=i;
  CHECK(!RadarLift_BeginBounded(9781,1,9781*i-1,&seq));
  CHECK(RadarLift_BeginBounded(9781,1,9781*i+50,&seq));
  CHECK(!RadarLift_Release(seq));g_radar_lift_interval_ms=9;RadarLift_Service(0,1);
  CHECK(active_interval==i);RadarLift_Service(i-1,1);CHECK(g_radar_lift_done==0);
  RadarLift_Service(i,1);CHECK(g_radar_lift_done==1);
  for(bits=2;bits<=9781;bits++)RadarLift_Service(bits*i,1);
  CHECK(g_radar_lift_state==2&&g_radar_lift_done==9781);
  CHECK(!RadarLift_Release(seq-1));CHECK(RadarLift_Release(seq));CHECK(!g_radar_lift_enable&&!g_radar_lift_phase_bits);
  CHECK(RadarLift_Release(seq));cases++;
 }
 reset();g_radar_lift_enable=1;CHECK(RadarLift_Begin(1,0,&seq));RadarLift_Service(0,1);RadarLift_Service(4,1);
 CHECK(RadarLift_Begin(1,0,&i));CHECK(!RadarLift_Release(seq));RadarLift_Service(4,1);CHECK(!RadarLift_Release(i));RadarLift_Service(8,1);CHECK(RadarLift_Release(i));cases++;
 reset();g_radar_lift_enable=1;g_radar_lift_interval_ms=0;CHECK(!RadarLift_Begin(1,0,&seq));g_radar_lift_interval_ms=5;CHECK(!RadarLift_BeginBounded(1,0,10,&seq));cases++;
 reset();RadarLift_Service(0,1);CHECK(RadarLift_Claim());CHECK(!RadarLift_Claim());CHECK(RadarLift_Enable(0));CHECK(!RadarLift_Begin(1,0,&seq));
 g_radar_lift_request=1;RadarLift_Service(0,1);CHECK(!g_radar_lift_sequence&&!g_radar_lift_done);
 CHECK(RadarLift_BeginBounded(1,0,4,&seq));RadarLift_Service(0,1);RadarLift_Service(4,1);CHECK(RadarLift_Release(seq));
 CHECK(RadarLift_Enable(4));CHECK(RadarLift_BeginBounded(1,1,4,&seq));RadarLift_Service(4,1);RadarLift_Service(8,1);CHECK(RadarLift_Release(seq));RadarLift_Unclaim();cases++;
 reset();g_radar_lift_enable=1;CHECK(RadarLift_Begin(2,0,&seq));RadarLift_Service(0,1);RadarLift_Service(4,1);bits=g_radar_lift_phase_bits;
 g_radar_lift_request=1;RadarLift_Service(5,1);CHECK(g_radar_lift_sequence==seq&&g_radar_lift_state==1&&g_radar_lift_result==1);
 RadarLift_Cancel();CHECK(!RadarLift_Release(seq));RadarLift_Service(6,1);CHECK(!RadarLift_Release(g_radar_lift_sequence)&&g_radar_lift_phase_bits==bits);CHECK(RadarLift_Claim()&&RadarLift_Enable(6));CHECK(g_radar_lift_state==3&&g_radar_lift_result==5);CHECK(RadarLift_BeginBounded(2,1,8,&seq));RadarLift_Service(6,1);RadarLift_Service(10,1);RadarLift_Service(14,1);CHECK(g_radar_lift_state==2&&g_radar_lift_result==1&&g_radar_lift_done==2);CHECK(RadarLift_Release(seq));cases++;
}
unsigned test_entry(void){speed_release();if(failed)return failed;limits();if(failed)return failed;queue_and_lock();if(failed)return failed;long_delay_and_wrap();if(failed)return failed;stops();return failed;}
"""
(T/'unit.c').write_text(source,'utf8');(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }')
commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I.','-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
from radar_zone2_arm_runner_20261111 import run
result=run(R,T,B,source,commands)
(E/'lift_arm_result.json').write_text(json.dumps({**result,'actual_lift_source':True,'gpio_and_irq_stubs':True,'hardware':False},indent=2),'utf8')
