from pathlib import Path
import json
R=Path.cwd();T=R/'工创赛/tmp/radar_us733_20261112/lift_tests';T.mkdir(parents=True,exist_ok=True)
E=R/'工创赛/evidence/navigation-planning/2026-11-12/radar-zone2/us733';B=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
P=R/'3.stm32f407工程/CHASSIS/HARDWARE/RADAR_SERVO'
(T/'radar_lift.h').write_bytes((P/'radar_lift.h').read_bytes())
(T/'zdt_emm_config.h').write_text('#define APP_MOTOR_OFFLINE_ONLY 0\n')
(T/'app_entry_config.h').write_text('#define APP_IS_DEBUG 1\n')
(T/'stm32f4xx.h').write_text('\n#include <stdint.h>\n#define __get_PRIMASK() primask\nstatic uint32_t primask;\n#define __disable_irq() (primask=1U)\n#define __enable_irq() (primask=0U)\ntypedef struct { uint16_t BSRRL,BSRRH; } GPIO_TypeDef;\nstatic GPIO_TypeDef bank_b,bank_d;\n#define GPIOB (&bank_b)\n#define GPIOD (&bank_d)\ntypedef struct { uint32_t GPIO_Pin,GPIO_Mode,GPIO_OType,GPIO_PuPd,GPIO_Speed; } GPIO_InitTypeDef;\n#define RCC_AHB1Periph_GPIOB 1U\n#define RCC_AHB1Periph_GPIOD 2U\n#define ENABLE 1U\n#define GPIO_Pin_8 256U\n#define GPIO_Pin_10 1024U\n#define GPIO_Pin_11 2048U\n#define GPIO_Pin_12 4096U\n#define GPIO_Mode_OUT 1U\n#define GPIO_OType_PP 0U\n#define GPIO_PuPd_DOWN 0U\n#define GPIO_Speed_2MHz 0U\nstatic void RCC_AHB1PeriphClockCmd(uint32_t a,uint32_t b){(void)a;(void)b;}\nstatic void GPIO_ResetBits(GPIO_TypeDef *a,uint16_t b){(void)a;(void)b;}\nstatic void GPIO_StructInit(GPIO_InitTypeDef *a){a->GPIO_Pin=0U;}\nstatic void GPIO_Init(GPIO_TypeDef *a,GPIO_InitTypeDef *b){(void)a;(void)b;}\n\ntypedef struct { volatile uint32_t CR1,DIER,SR,PSC,ARR,CNT,EGR; } TIM_TypeDef;\ntypedef struct { volatile uint32_t CTRL,CYCCNT; } DWT_TypeDef;\ntypedef struct { volatile uint32_t DEMCR; } CoreDebug_TypeDef;\ntypedef struct { volatile uint32_t CFGR; } RCC_TypeDef;\ntypedef struct { volatile uint32_t APB1FZ; } DBGMCU_TypeDef;\nstatic TIM_TypeDef tim7;static DWT_TypeDef dwt;static CoreDebug_TypeDef debug;\nstatic RCC_TypeDef rcc;static DBGMCU_TypeDef dbg;static uint32_t pending,clock_hz=42000000U,counter_stuck,priority;\n#define TIM7 (&tim7)\n#define DWT (&dwt)\n#define CoreDebug (&debug)\n#define RCC (&rcc)\n#define DBGMCU (&dbg)\nstatic uint32_t SystemCoreClock=168000000U;\n#define __NOP() do{if(!counter_stuck && (dwt.CTRL&1U))dwt.CYCCNT++;}while(0)\n#define DWT_CTRL_NOCYCCNT_Msk 0x2000000U\n#define DWT_CTRL_CYCCNTENA_Msk 1U\n#define CoreDebug_DEMCR_TRCENA_Msk 0x1000000U\n#define RCC_CFGR_PPRE1 0x1c00U\n#define TIM7_IRQn 55U\n#define RCC_APB1Periph_TIM7 32U\n#define TIM_EGR_UG 1U\n#define TIM_DIER_UIE 1U\n#define TIM_SR_UIF 1U\n#define TIM_CR1_CEN 1U\n#define DBGMCU_APB1_FZ_DBG_TIM7_STOP 32U\ntypedef struct {uint32_t PCLK1_Frequency;} RCC_ClocksTypeDef;\ntypedef struct {uint32_t NVIC_IRQChannel,NVIC_IRQChannelPreemptionPriority,NVIC_IRQChannelSubPriority,NVIC_IRQChannelCmd;} NVIC_InitTypeDef;\nstatic void RCC_GetClocksFreq(RCC_ClocksTypeDef *c){c->PCLK1_Frequency=clock_hz;}\nstatic void RCC_APB1PeriphClockCmd(uint32_t a,uint32_t b){(void)a;(void)b;}\nstatic void NVIC_ClearPendingIRQ(uint32_t a){(void)a;pending=0;}\nstatic void NVIC_Init(NVIC_InitTypeDef *n){priority=n->NVIC_IRQChannelPreemptionPriority;}\n')
product=(P/'radar_lift.c').read_text('utf8')
source=product+r'''

volatile unsigned failed,assertions,cases;
#define CHECK(x) do{assertions++;if(!(x)){failed=__LINE__;return;}}while(0)
static void reset(void){
 initialized=0;phase=7;start_tick=total=direction=active_budget=active_interval=0;
 timer_active=terminal_reason=safe_cycle=0;pending_long=pending_steps=pending_direction=pending_budget=pending_interval=claimed=0;
 g_radar_lift_interval_us=733;g_radar_lift_enable=1;g_radar_lift_request=g_radar_lift_state=g_radar_lift_result=g_radar_lift_sequence=g_radar_lift_done=g_radar_lift_phase_bits=0;
 g_radar_lift_steps=32;g_radar_lift_direction=0;last_service=safe_service=0;tim7.CR1=tim7.DIER=tim7.SR=tim7.CNT=0;dwt.CTRL=dwt.CYCCNT=debug.DEMCR=0;
 rcc.CFGR=0x1000;clock_hz=42000000;counter_stuck=0;pending=0;primask=0;SystemCoreClock=168000000;
}
static void irq(uint32_t cycles){dwt.CYCCNT+=cycles;tim7.SR|=1;pending=1;TIM7_IRQHandler();}
static void start(uint32_t n,uint32_t dir){uint32_t seq;CHECK(RadarLift_BeginBounded(n,dir,100000,&seq));RadarLift_Service(0,1);CHECK(g_radar_lift_state==1&&timer_active&&seq==1);}
static void complete_periods(void){static const uint32_t periods[]={733,1000,2000,3000,4000};uint32_t j,i,seq,bits;
 for(j=0;j<5;j++){
  reset();g_radar_lift_interval_us=periods[j];CHECK(!RadarLift_BeginBounded(8559,1,nominal_ms(8559,periods[j])-1,&seq));
  CHECK(RadarLift_BeginBounded(8559,1,nominal_ms(8559,periods[j]),&seq));pending=1;tim7.SR=1;tim7.CNT=999;
  g_radar_lift_interval_us=4001;g_radar_lift_steps=1;g_radar_lift_direction=0;RadarLift_Service(0,1);
  CHECK(!pending&&!tim7.SR&&!tim7.CNT&&tim7.PSC==83&&tim7.ARR==periods[j]-1&&priority==4&&active_interval==periods[j]&&total==8559&&direction==1);
  TIM7_IRQHandler();RadarLift_Service(0,1);CHECK(!g_radar_lift_done&&!g_radar_lift_phase_bits);CHECK(!RadarLift_Release(seq));
  for(i=1;i<=8559;i++){if(i%10==0)RadarLift_Service(i*periods[j]/1000,1);irq(periods[j]*168);CHECK(g_radar_lift_done==i);}
  CHECK(!timer_active&&!tim7.CR1&&!tim7.DIER&&!tim7.SR&&!pending&&terminal_reason==1&&g_radar_lift_state==1);
  bits=g_radar_lift_phase_bits;irq(periods[j]*168);CHECK(g_radar_lift_done==8559&&g_radar_lift_phase_bits==bits);
  RadarLift_Service(nominal_ms(8559,periods[j]),1);CHECK(g_radar_lift_state==2&&g_radar_lift_result==1);CHECK(!RadarLift_Release(seq-1));CHECK(RadarLift_Release(seq)&&!g_radar_lift_phase_bits&&!g_radar_lift_enable);CHECK(RadarLift_Release(seq));cases++;
 }
}
static void freshness(void){uint32_t mode,bits,seq;
 for(mode=0;mode<6;mode++){
  reset();start(10,0);if(failed)return;irq(733*168);bits=g_radar_lift_phase_bits;
  if(mode==0)g_radar_lift_enable=0;
  if(mode==1)g_radar_lift_request=2;
  if(mode==2)safe_service=0;
  if(mode==3)dwt.CTRL=0;
  if(mode==4)debug.DEMCR=0;
  if(mode==5)dwt.CTRL|=DWT_CTRL_NOCYCCNT_Msk;
  irq(733*168);CHECK(!timer_active&&g_radar_lift_done==1&&g_radar_lift_phase_bits==bits);
  CHECK(terminal_reason==(mode==1?5:2));RadarLift_Service(2,1);CHECK(g_radar_lift_state==3&&g_radar_lift_result==(mode==1?5:2));CHECK(!RadarLift_Release(g_radar_lift_sequence));cases++;
 }
 reset();start(10,0);if(failed)return;dwt.CYCCNT=safe_cycle+SystemCoreClock/10;irq(0);CHECK(g_radar_lift_done==1&&timer_active);irq(1);CHECK(g_radar_lift_done==1&&!timer_active&&terminal_reason==2);bits=g_radar_lift_phase_bits;
 dwt.CYCCNT=safe_cycle;irq(0);CHECK(g_radar_lift_done==1&&bits==g_radar_lift_phase_bits);RadarLift_Cancel();RadarLift_Service(102,1);CHECK(g_radar_lift_state==3&&g_radar_lift_result==2&&terminal_reason==2);cases++;
 reset();start(10,0);if(failed)return;safe_service=0;irq(733*168);RadarLift_Service(1,1);CHECK(g_radar_lift_state==3&&g_radar_lift_result==2);RadarLift_Cancel();RadarLift_Service(2,1);CHECK(g_radar_lift_result==2&&terminal_reason==2);cases++;
 reset();dwt.CYCCNT=0xfffffff0U;start(10,0);if(failed)return;irq(733*168);CHECK(g_radar_lift_done==1&&timer_active);cases++;
 reset();start(10,0);if(failed)return;dwt.CYCCNT+=SystemCoreClock/10+1;RadarLift_Service(101,1);CHECK(!timer_active&&g_radar_lift_result==2&&!g_radar_lift_done);cases++;
 reset();start(1,0);if(failed)return;irq(733*168);g_radar_lift_enable=0;RadarLift_Service(1,0);CHECK(g_radar_lift_state==2&&g_radar_lift_result==1&&terminal_reason==1);cases++;
 reset();counter_stuck=1;CHECK(RadarLift_Begin(1,0,&seq));RadarLift_Service(0,1);CHECK(g_radar_lift_result==2&&!timer_active&&!g_radar_lift_done);cases++;
}
static void transaction(void){uint32_t seq,i,bits;
 reset();start(8,0);if(failed)return;RadarLift_Service(20,1);CHECK(!g_radar_lift_done);irq(20*168000);CHECK(g_radar_lift_done==1&&g_radar_lift_phase_bits==1);irq(0);CHECK(g_radar_lift_done==2&&g_radar_lift_phase_bits==3);cases++;
 reset();start(8,1);if(failed)return;irq(733*168);CHECK(g_radar_lift_done==1&&g_radar_lift_phase_bits==8);cases++;

 reset();CHECK(RadarLift_Begin(2,0,&seq));g_radar_lift_steps=64;g_radar_lift_direction=1;g_radar_lift_interval_us=4000;RadarLift_Service(0,1);CHECK(total==2&&direction==0&&active_interval==733&&active_budget==252);cases++;
 for(i=0;i<3;i++){
  reset();start(2,0);if(failed)return;
  if(i){irq(733*168);CHECK(g_radar_lift_done==1);}
  bits=g_radar_lift_phase_bits;RadarLift_Cancel();irq(733*168);CHECK(g_radar_lift_done==(i?1:0)&&bits==g_radar_lift_phase_bits);
  RadarLift_Service(2,1);CHECK(g_radar_lift_sequence==2&&g_radar_lift_state==3&&g_radar_lift_result==5&&!pending_long);CHECK(!RadarLift_Release(2));
  CHECK(RadarLift_Begin(1,1,&seq));RadarLift_Service(3,1);CHECK(timer_active&&seq==3);irq(733*168);RadarLift_Service(4,1);CHECK(RadarLift_Release(seq));cases++;
 }
 reset();start(1,0);if(failed)return;irq(733*168);RadarLift_Service(1,1);CHECK(g_radar_lift_state==2);CHECK(RadarLift_Begin(1,0,&seq));RadarLift_Cancel();RadarLift_Service(2,1);CHECK(g_radar_lift_result==5&&!RadarLift_Release(2));cases++;
 reset();start(3,0);if(failed)return;g_radar_lift_request=1;RadarLift_Service(1,1);CHECK(g_radar_lift_sequence==1&&total==3&&timer_active);irq(733*168);RadarLift_Service(2,1);CHECK(g_radar_lift_done==1);cases++;
 reset();CHECK(RadarLift_Begin(1,0,&seq));RadarLift_Cancel();RadarLift_Service(0,1);CHECK(g_radar_lift_sequence==1&&g_radar_lift_result==5&&!g_radar_lift_done&&!timer_active);cases++;
 reset();RadarLift_Service(0,1);CHECK(RadarLift_Claim()&&RadarLift_Enable(0));CHECK(!RadarLift_Begin(1,0,&seq));CHECK(RadarLift_BeginBounded(1,0,1,&seq));RadarLift_Service(0,1);irq(733*168);RadarLift_Service(1,1);CHECK(RadarLift_Release(seq));RadarLift_Unclaim();CHECK(!claimed);cases++;
 reset();primask=1;RadarLift_Service(0,1);CHECK(primask==1);CHECK(RadarLift_Available()&&primask==1);RadarLift_GetSnapshot(0);CHECK(primask==1);primask=0;cases++;
}
static void limits(void){uint32_t seq,i;static const uint32_t bad[]={0,1,732,4001,0xffffffffU};
 for(i=0;i<5;i++){reset();g_radar_lift_interval_us=bad[i];CHECK(!RadarLift_Begin(1,0,&seq));CHECK(!RadarLift_BeginBounded(1,0,100,&seq));g_radar_lift_request=1;RadarLift_Service(0,1);CHECK(g_radar_lift_result==3&&!timer_active);cases++;}
 reset();CHECK(!RadarLift_Begin(65,0,&seq)&&!RadarLift_Begin(0,0,&seq)&&!RadarLift_Begin(1,2,&seq));CHECK(!RadarLift_BeginBounded(65536,0,100000,&seq));CHECK(!RadarLift_BeginBounded(0xffffffffU,0,100000,&seq));CHECK(!RadarLift_BeginBounded(1,0,0x80000000U,&seq));CHECK(RadarLift_BeginBounded(65535,1,48038,&seq));RadarLift_Service(0,1);CHECK(total==65535);cases++;
 reset();g_radar_lift_request=1;g_radar_lift_steps=64;g_radar_lift_interval_us=733;RadarLift_Service(0,1);CHECK(active_budget==297);cases++;
 reset();clock_hz=42000001;CHECK(RadarLift_Begin(1,0,&seq));RadarLift_Service(0,1);CHECK(g_radar_lift_state==3&&g_radar_lift_result==2&&!timer_active);cases++;
 reset();rcc.CFGR=0;clock_hz=84000000;CHECK(RadarLift_Begin(1,0,&seq));RadarLift_Service(0,1);CHECK(tim7.PSC==83&&timer_active);cases++;
 reset();start(10,0);if(failed)return;active_budget=1;RadarLift_Service(2,1);CHECK(!timer_active&&g_radar_lift_result==2&&!g_radar_lift_done);cases++;
}
unsigned test_entry(void){complete_periods();if(failed)return failed;freshness();if(failed)return failed;transaction();if(failed)return failed;limits();return failed;}
'''
(T/'unit.c').write_text(source,'utf8');(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }')
commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I.','-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
from radar_zone2_arm_runner_20261111 import run
result=run(R,T,B,source,commands)
(E/'lift_arm_result.json').write_text(json.dumps({**result,'actual_lift_source':True,'gpio_and_irq_stubs':True,'hardware':False},indent=2),'utf8')
