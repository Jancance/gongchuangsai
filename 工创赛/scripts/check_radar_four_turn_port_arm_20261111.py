from pathlib import Path
import re,subprocess,sys,struct,json
R=Path.cwd();T=R/'工创赛/tmp/radar_four_turn_20261111/port_tests';T.mkdir(exist_ok=True);E=R/'工创赛/evidence/navigation-planning/2026-11-11/radar-zone2/four-turn-software';B=Path('D:/Keil_v5/ARM/arm5_compiler/bin');P=R/'3.stm32f407工程/CHASSIS/HARDWARE/RADAR_V6'
for n in ['radar_client.h','radar_uart_port.h']:(T/n).write_bytes((P/n).read_bytes())
(T/'FreeRTOS.h').write_text('#include <stdint.h>\ntypedef uint32_t TickType_t;\n#define configTICK_RATE_HZ 100U\n')
(T/'task.h').write_text('#include <stdint.h>\nuint32_t xTaskGetTickCount(void);\n')
(T/'stm32f4xx.h').write_text('''#include <stdint.h>
#include <stddef.h>
typedef struct {uint32_t SR,DR,CR1;} USART_TypeDef;
extern USART_TypeDef uart_fixture;
#define USART3 (&uart_fixture)
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define USART_SR_RXNE 32U
#define USART_SR_TC 64U
#define USART_SR_TXE 128U
#define USART_IT_RXNE 32U
#define USART_IT_TC 64U
#define USART_IT_TXE 128U
#define RESET 0U
#define ENABLE 1U
#define DISABLE 0U
#define __get_PRIMASK fixture_primask
#define __disable_irq fixture_disable
#define __enable_irq fixture_enable
uint32_t __get_PRIMASK(void);void __disable_irq(void);void __enable_irq(void);
void USART_ITConfig(USART_TypeDef *,uint32_t,uint32_t);
uint32_t USART_GetITStatus(USART_TypeDef *,uint32_t);
''')
source=r'''
#include "stm32f4xx.h"
volatile unsigned failed,assertions,cases;
static uint32_t mask,tick,errors,completed;
USART_TypeDef uart_fixture;
uint32_t __get_PRIMASK(void){return mask;}void __disable_irq(void){mask=1;}void __enable_irq(void){mask=0;}
uint32_t xTaskGetTickCount(void){return tick;}
void USART_ITConfig(USART_TypeDef *u,uint32_t bit,uint32_t en){if(en)u->CR1|=bit;else u->CR1&=~bit;}
uint32_t USART_GetITStatus(USART_TypeDef *u,uint32_t bit){return (u->SR&u->CR1&bit)!=0;}
void RadarClient_OnUartErrorInterrupt(uint32_t code){(void)code;errors++;}
void RadarClient_OnUartTxCompleteInterrupt(void){completed++;}
'''+(P/'radar_uart_port.c').read_text('utf8').replace('#include "stm32f4xx.h"','')+r'''
#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)
unsigned test_entry(void){uint8_t rx[4],b,tx[2]={17,18};uint32_t key;
 CHECK(RadarUartPort_StartReceive(rx,4));uart_fixture.SR=USART_SR_RXNE;uart_fixture.DR=1;USART3_IRQHandler();uart_fixture.DR=2;USART3_IRQHandler();uart_fixture.DR=3;USART3_IRQHandler();CHECK(errors==0);uart_fixture.DR=4;USART3_IRQHandler();CHECK(errors==1&&!RadarUartPort_ReceiveIsActive());CHECK(RadarUartPort_ReadByte(&b)&&b==1);CHECK(RadarUartPort_ReadByte(&b)&&b==2);CHECK(RadarUartPort_ReadByte(&b)&&b==3);CHECK(!RadarUartPort_ReadByte(&b));cases++;
 CHECK(RadarUartPort_StartReceive(rx,4));uart_fixture.DR=9;USART3_IRQHandler();CHECK(RadarUartPort_ReadByte(&b)&&b==9);cases++;
 CHECK(RadarUartPort_StartTransmit(tx,2));CHECK(!RadarUartPort_StartTransmit(tx,2));uart_fixture.SR=USART_SR_TXE;USART3_IRQHandler();CHECK(uart_fixture.DR==17&&completed==0);USART3_IRQHandler();CHECK(uart_fixture.DR==18&&completed==0);CHECK(!(uart_fixture.CR1&USART_IT_TXE)&&(uart_fixture.CR1&USART_IT_TC));USART3_IRQHandler();CHECK(completed==0);uart_fixture.SR=USART_SR_TC;USART3_IRQHandler();CHECK(completed==1);USART3_IRQHandler();CHECK(completed==1);cases++;
 uart_fixture.SR=USART_SR_ORE|USART_SR_RXNE;USART3_IRQHandler();CHECK(errors==2);cases++;
 mask=1;key=RadarUartPort_EnterCritical();CHECK(key==1&&mask==1);RadarUartPort_ExitCritical(key);CHECK(mask==1);mask=0;key=RadarUartPort_EnterCritical();CHECK(key==0&&mask==1);RadarUartPort_ExitCritical(key);CHECK(mask==0);cases++;
 s_last_tick=0xfffffff0U;s_ms=100;s_tick_remainder=0;tick=0xfffffff5U;CHECK(RadarUartPort_GetTick()==150);tick=5;CHECK(RadarUartPort_GetTick()==310);cases++;
 return 0;}
'''
(T/'unit.c').write_text(source,'utf8');(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }')
commands=[[str(B/'armcc.exe'),'--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','-I.','-c','unit.c','-o','unit.o'],[str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
from radar_zone2_arm_runner_20261111 import run
result=run(R,T,B,source,commands)
(E/'port_arm_result.json').write_text(json.dumps({**result,'actual_port_c':True,'register_stub':True,'tick_rate_fixture':100,'hardware':False},indent=2),'utf8')
