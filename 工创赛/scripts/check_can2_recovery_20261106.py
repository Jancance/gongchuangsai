"""Execute actual DM query module and actual CAN1 init/filter backup with fake registers/bus."""
from pathlib import Path
import json,subprocess,hashlib
R=Path(__file__).resolve().parents[1];S=R/'tmp/integration10_can2_recovery_20261106/project'
O=R/'evidence/integration/2026-11-06/can2-recovery'
def main():
    assert json.loads((O/'matlab-result.json').read_text())['passed']
    can=(S/'CHASSIS/SYSTEM/CAN/can.c').read_bytes().decode('gbk')
    start=can.index('typedef struct {\n') if 'typedef struct {\n' in can else -1
    start=can.index('typedef struct {\r\n') if start<0 else start
    can=can.replace('\r\n','\n');start=can.index('typedef struct {\n',can.index('void CAN1_ResetSession'))
    end=can.index('uint32_t CAN1_LoopbackTest',start)
    can=can[start:end];can=can[:can.rfind('/*')]
    dm=(S/'HARDWARE/DM_CAN2/dm_can2.c').read_text(encoding='utf-8')
    dm='\n'.join(line for line in dm.splitlines() if not line.startswith('#include'))
    prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(v) (v)
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define GPIO_Pin_0 1
#define GPIO_Pin_1 2
#define GPIO_Pin_12 4096
#define GPIO_Pin_13 8192
#define GPIO_PinSource0 0
#define GPIO_PinSource1 1
#define GPIO_PinSource12 12
#define GPIO_PinSource13 13
#define GPIO_AF_CAN1 9
#define GPIO_AF_CAN2 9
#define GPIO_Mode_AF 2
#define GPIO_OType_PP 0
#define GPIO_Speed_25MHz 1
#define GPIO_Speed_50MHz 2
#define GPIO_PuPd_NOPULL 0
#define RCC_AHB1Periph_GPIOD 8
#define RCC_AHB1Periph_GPIOB 2
#define RCC_APB1Periph_CAN1 (1U<<25)
#define RCC_APB1Periph_CAN2 (1U<<26)
#define CAN_Mode_Normal 0
#define CAN_SJW_1tq 0
#define CAN_BS1_11tq 0x000a0000
#define CAN_BS2_2tq 0x00100000
#define CAN_InitStatus_Success 1
#define CAN_FilterMode_IdMask 0
#define CAN_FilterScale_32bit 1
#define CAN_FIFO0 0
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 0
#define CAN_FLAG_FOV0 1
#define CAN_ESR_BOFF 4
#define CAN_MCR_INRQ 1
#define CAN_MSR_INAK 1
#define CAN_MCR_NART 16
#define CAN_MCR_ABOM 64
#define DM_CAN2_NODE_ID 1
#define DM_CAN2_MASTER_ID 0
#define DM_CAN2_MODE 2
typedef struct {uint32_t PCLK1_Frequency;} RCC_ClocksTypeDef;
typedef struct {uint32_t GPIO_Pin,GPIO_Mode,GPIO_OType,GPIO_Speed,GPIO_PuPd;} GPIO_InitTypeDef;
typedef struct {uint32_t CAN_Mode,CAN_SJW,CAN_BS1,CAN_BS2,CAN_Prescaler,CAN_NART,CAN_ABOM;} CAN_InitTypeDef;
typedef struct {uint32_t CAN_FilterNumber,CAN_FilterMode,CAN_FilterScale,CAN_FilterIdHigh,CAN_FilterIdLow,CAN_FilterMaskIdHigh,CAN_FilterMaskIdLow,CAN_FilterFIFOAssignment,CAN_FilterActivation;} CAN_FilterInitTypeDef;
typedef struct {uint32_t FR1,FR2;} FilterReg;
typedef struct {uint32_t MCR,MSR,ESR,BTR,FMR,FM1R,FS1R,FFA1R,FA1R;FilterReg sFilterRegister[28];} CAN_TypeDef;
static CAN_TypeDef hw1,hw2;
#define CAN1 (&hw1)
#define CAN2 (&hw2)
static struct {uint32_t APB1ENR;} clocks;
#define RCC (&clocks)
#define GPIOB ((void*)1)
#define GPIOD ((void*)2)
typedef struct {uint8_t Data[8];uint32_t rid;} CanRxMsg;
typedef struct {uint8_t Data[8];uint32_t rid;} CanTxMsg;
typedef struct {uint32_t position_raw,velocity_raw,torque_raw,motor_state;float position_rad,velocity_rad_s,torque_nm;} DM_CAN2_Feedback;
static uint32_t tick,can2_resets,can1_resets,ack_delay=5,tx_begin,tx_pending,rx_pending,cancels,txs,last_rid,checks,failures;
#define C(v) do{checks++;if(!(v)){failures++;printf("FAIL %u %s\n",__LINE__,#v);}}while(0)
uint32_t xTaskGetTickCount(void){return tick;}
void vTaskDelay(uint32_t n){tick+=n;}
void RCC_GetClocksFreq(RCC_ClocksTypeDef*p){p->PCLK1_Frequency=42000000;}
void RCC_AHB1PeriphClockCmd(uint32_t a,uint32_t b){(void)a;(void)b;}
void RCC_APB1PeriphClockCmd(uint32_t a,uint32_t b){if(b)clocks.APB1ENR|=a;}
void GPIO_PinAFConfig(void*a,uint32_t b,uint32_t c){(void)a;(void)b;(void)c;}
void GPIO_StructInit(GPIO_InitTypeDef*p){memset(p,0,sizeof(*p));}
void GPIO_Init(void*a,GPIO_InitTypeDef*p){(void)a;(void)p;}
void CAN_DeInit(CAN_TypeDef*c){if(c==CAN1){can1_resets++;memset(c,0,sizeof(*c));c->FMR=14U<<8;}else{can2_resets++;memset(c,0,sizeof(*c));}tx_pending=rx_pending=0;}
void CAN_StructInit(CAN_InitTypeDef*c){memset(c,0,sizeof(*c));}
uint32_t CAN_Init(CAN_TypeDef*c,CAN_InitTypeDef*p){c->BTR=p->CAN_Mode|p->CAN_SJW|p->CAN_BS1|p->CAN_BS2|(p->CAN_Prescaler-1);c->MCR=(p->CAN_NART?16:0)|(p->CAN_ABOM?64:0);return 1;}
void CAN_FilterInit(CAN_FilterInitTypeDef*p){unsigned b=1U<<p->CAN_FilterNumber;hw1.FMR|=1;hw1.sFilterRegister[p->CAN_FilterNumber].FR1=p->CAN_FilterIdHigh<<16|p->CAN_FilterIdLow;hw1.sFilterRegister[p->CAN_FilterNumber].FR2=p->CAN_FilterMaskIdHigh<<16|p->CAN_FilterMaskIdLow;hw1.FM1R=(hw1.FM1R&~b)|(p->CAN_FilterMode?b:0);hw1.FS1R=(hw1.FS1R&~b)|(p->CAN_FilterScale?b:0);hw1.FFA1R=(hw1.FFA1R&~b)|(p->CAN_FilterFIFOAssignment?b:0);hw1.FA1R=(hw1.FA1R&~b)|(p->CAN_FilterActivation?b:0);hw1.FMR&=~1U;}
uint32_t CAN_Transmit(CAN_TypeDef*c,CanTxMsg*p){(void)c;tx_pending=1;rx_pending=0;tx_begin=tick;last_rid=p->rid;txs++;return 0;}
uint32_t CAN_TransmitStatus(CAN_TypeDef*c,uint8_t m){(void)c;(void)m;if(tx_pending && (uint32_t)(tick-tx_begin)>=ack_delay){tx_pending=0;rx_pending=1;return 1;}return 2;}
void CAN_CancelTransmit(CAN_TypeDef*c,uint8_t m){(void)c;(void)m;tx_pending=0;cancels++;}
uint32_t CAN_MessagePending(CAN_TypeDef*c,uint32_t f){(void)c;(void)f;return rx_pending;}
void CAN_Receive(CAN_TypeDef*c,uint32_t f,CanRxMsg*r){(void)c;(void)f;memset(r,0,sizeof(*r));r->rid=last_rid;rx_pending=0;}
void CAN_ClearFlag(CAN_TypeDef*c,uint32_t f){(void)c;(void)f;}
uint32_t CAN_GetFlagStatus(CAN_TypeDef*c,uint32_t f){(void)c;(void)f;return 0;}
uint32_t DM_CAN2_BuildRead(uint8_t rid,CanTxMsg*t){memset(t,0,sizeof(*t));t->rid=rid;return 1;}
uint32_t DM_CAN2_BuildStatus(CanTxMsg*t){memset(t,0,sizeof(*t));t->rid=0xcc;return 1;}
uint32_t DM_CAN2_ParseRead(CanRxMsg*r,uint8_t rid,uint32_t*v){(void)r;*v=rid==8?1:rid==7?0:rid==10?2:rid;return 1;}
uint32_t DM_CAN2_DecodeRange(uint32_t bits,float*v){*v=bits==0x15?12.5f:bits==0x16?30:10;return 1;}
uint32_t DM_CAN2_ParseStatus(CanRxMsg*r,float a,float b,float c,DM_CAN2_Feedback*f){(void)r;(void)a;(void)b;(void)c;memset(f,0,sizeof(*f));f->position_rad=1;return 1;}
'''
    tail=r'''
static void query(void){g_dm_can2_read_request=2;DM_CAN2_Service(1,0);}
static void finish(void){for(unsigned i=0;i<100 && g_dm_can2_state<4;i++){tick+=5;DM_CAN2_Service(1,0);}}
int main(void){
 tick=100;C(CAN1_InitMode(0)==1);C(can1_resets==1);
 query();finish();C(g_dm_can2_state==4 && g_dm_can2_result==1 && g_dm_can2_feedback_valid);
 C(can2_resets==1 && g_dm_can2_hardware_initializations==1 && !(hw2.MCR&16) && (hw2.MCR&64));
 query();finish();C(g_dm_can2_state==4 && can2_resets==1 && g_dm_can2_hardware_initializations==1);
 hw1.FA1R|=1U<<15;hw1.FS1R|=1U<<15;hw1.FM1R|=1U<<15;hw1.FFA1R|=1U<<15;
 hw1.sFilterRegister[15].FR1=0x12345678;hw1.sFilterRegister[15].FR2=0x87654321;
 C(CAN1_InitMode(0)==1);C((hw1.FA1R&((1U<<14)|(1U<<15)|1U))==((1U<<14)|(1U<<15)|1U));
 C(hw1.sFilterRegister[14].FR2==0xffe00006 && hw1.sFilterRegister[15].FR1==0x12345678 && hw1.sFilterRegister[15].FR2==0x87654321);
 C((hw1.FM1R&(1U<<15)) && (hw1.FFA1R&(1U<<15)) && !(hw1.FM1R&1));
 hw1.FA1R&=~(1U<<14);hw1.sFilterRegister[14].FR2=0;unsigned repairs=g_dm_can2_filter_repairs;
 query();finish();C(g_dm_can2_state==4 && g_dm_can2_filter_repairs==repairs+1 && can2_resets==1);
 ack_delay=15;hw2.ESR=0x80030;query();finish();C(g_dm_can2_state==4);C(g_dm_can2_first_error_valid && g_dm_can2_first_error_esr==0x80030);
 hw2.ESR=0x90050;ack_delay=100000;unsigned oldcancel=cancels;query();uint32_t begin=tick;finish();
 C(g_dm_can2_state==5 && g_dm_can2_result==4 && cancels>oldcancel && !tx_pending && (uint32_t)(tick-begin)<=35);
 C(g_dm_can2_first_error_esr==0x80030 && g_dm_can2_lec==5 && g_dm_can2_tec==9 && !g_dm_can2_bus_off);
 hw2.ESR=4;query();tick+=5;DM_CAN2_Service(1,0);C(g_dm_can2_result==7 && !tx_pending && g_dm_can2_bus_off==1);
 unsigned oldtx=txs;hw2.ESR=0;for(unsigned i=0;i<100;i++){tick+=5;DM_CAN2_Service(1,0);}C(txs==oldtx);
 ack_delay=5;query();finish();C(g_dm_can2_state==4 && can2_resets==1);
 query();DM_CAN2_CancelRead();C(g_dm_can2_state==5 && g_dm_can2_result==8 && !tx_pending);
 hw1.FMR=15U<<8;query();C(g_dm_can2_result==9 && (hw1.FMR>>8)==15);hw1.FMR=14U<<8;
 tick=0xfffffff0U;query();finish();C(g_dm_can2_state==4);
 printf("CAN2_RECOVERY actual_C checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
    unit=prefix+can+dm+tail;p=O/'actual-recovery.c';p.write_text(unit,encoding='utf-8')
    exe=O/'actual-recovery.exe';args=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',str(p),'-lm','-o',str(exe)]
    x=subprocess.run(args,capture_output=True,encoding='utf-8',errors='replace');(O/'host-build.log').write_text(x.stdout+x.stderr,encoding='utf-8');assert x.returncode==0,x.stderr[-4000:]
    x=subprocess.run([str(exe)],capture_output=True,encoding='utf-8');print(x.stdout);(O/'host-check.log').write_text(x.stdout,encoding='utf-8');assert x.returncode==0
    (O/'host-result.json').write_text(json.dumps({'passed':True,'result':x.stdout,'scope':'actual CAN1 init/filter backup and full DM query module, mocked registers/bus/time','source_sha256':{n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in ['CHASSIS/SYSTEM/CAN/can.c','HARDWARE/DM_CAN2/dm_can2.c']}},indent=2),encoding='utf-8')
if __name__=='__main__':main()
