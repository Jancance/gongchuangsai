#include <stdio.h>
#include <string.h>
typedef unsigned int uint32_t; typedef unsigned char uint8_t;
typedef struct {unsigned int ExtId; unsigned char DLC,Data[8];} CanRxMsg;
#define MOTION_STOP_IDLE 0U
struct {uint32_t ESR;} fake_can;
#define CAN1 (&fake_can)
CanRxMsg g_can_enable_zero_reply,g_can_enable_reply;
volatile uint32_t g_can_disable_request;
volatile uint32_t g_can_enable_address;
volatile uint32_t g_can_enable_esr;
volatile uint32_t g_can_enable_request;
volatile uint32_t g_can_enable_result;
volatile uint32_t g_can_enable_zero_result;
volatile uint32_t g_can_guard_ready;
volatile uint32_t g_can_poll_enable;
volatile uint32_t g_can_stop_request;
volatile uint32_t g_motion_auto_stop_cut_vm;
volatile uint32_t g_motion_auto_stop_state;
volatile uint32_t g_motion_diag_fail_stop_once;
volatile uint32_t g_motion_diag_fault_latched;
volatile uint32_t g_motion_diag_hold_report;
volatile uint32_t g_motion_diag_hold_stop_service;
volatile uint32_t g_rtos_fault;
volatile uint32_t g_safety_fault;
volatile uint32_t g_safety_ready;
volatile uint32_t g_vm_test_request;
static unsigned int zcalls,ecalls,stage,checks,zcode,ecode,last_address,bad_call;
static volatile uint32_t *during;
static unsigned int during_value;
static uint32_t CAN1_SetEmmSpeedRaw(uint8_t address,uint8_t dir,unsigned int speed,uint8_t acc,CanRxMsg *reply){
zcalls++;last_address=address; if(dir||speed||acc||stage)bad_call=1; stage=1;reply->Data[0]=0xF6;if(during)*during=during_value;return zcode;}
static uint32_t CAN1_SetMotorEnable(uint8_t address,uint8_t enabled,CanRxMsg *reply){
ecalls++;if(address!=last_address||enabled!=1||stage!=1)bad_call=1;stage=2;reply->Data[0]=0xF3;return ecode;}
static uint32_t App_CanEnableTestReady(void)
{
    return ((g_vm_test_request == 1U) &&
            (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
            (g_can_poll_enable == 1U) &&
            (g_motion_diag_fault_latched == 0U) &&
            (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
            (g_motion_auto_stop_cut_vm == 0U) &&
            (g_motion_diag_hold_report == 0U) &&
            (g_motion_diag_hold_stop_service == 0U) &&
            (g_motion_diag_fail_stop_once == 0U) &&
            (g_can_stop_request == 0U) &&
            (g_can_disable_request == 0U)) ? 1U : 0U;
}


static void step(void){
        if (g_can_enable_request == 1U)
        {
            uint32_t address = g_can_enable_address;
            CanRxMsg zero_reply = {0};
            CanRxMsg enable_reply = {0};

            /* 地址本轮冻结，请求先消费，任何失败均不自动重试 */
            g_can_enable_request = 0U;
            g_can_enable_result = 0U;
            g_can_enable_zero_result = 0U;

            if ((address < 1U) || (address > 4U))
            {
                g_can_enable_result = 7U;
            }
            else if (App_CanEnableTestReady() != 1U)
            {
                g_can_enable_result = 9U;
            }
            else
            {
                /* 加速度0使零速目标立即生效；不靠历史测试结果放行 */
                g_can_enable_zero_result = CAN1_SetEmmSpeedRaw(
                    (uint8_t)address, 0U, 0U, 0U, &zero_reply);
                if (g_can_enable_zero_result != 1U)
                {
                    /* 零速未获02应答，不发使能帧 */
                    g_can_enable_result = 10U;
                }
                else if (App_CanEnableTestReady() != 1U)
                {
                    /* 等待零速应答期间撤VM/故障/停止请求均取消使能 */
                    g_can_enable_result = 9U;
                }
                else
                {
                    g_can_enable_result = CAN1_SetMotorEnable(
                        (uint8_t)address, 1U, &enable_reply);
                }
            }

            g_can_enable_zero_reply = zero_reply;
            g_can_enable_reply = enable_reply;
            g_can_enable_esr = CAN1->ESR;
        }
}
static void base(void){
g_can_disable_request=0;
g_can_enable_address=0;
g_can_enable_esr=0;
g_can_enable_request=0;
g_can_enable_result=0;
g_can_enable_zero_result=0;
g_can_guard_ready=0;
g_can_poll_enable=0;
g_can_stop_request=0;
g_motion_auto_stop_cut_vm=0;
g_motion_auto_stop_state=0;
g_motion_diag_fail_stop_once=0;
g_motion_diag_fault_latched=0;
g_motion_diag_hold_report=0;
g_motion_diag_hold_stop_service=0;
g_rtos_fault=0;
g_safety_fault=0;
g_safety_ready=0;
g_vm_test_request=0;
g_vm_test_request=g_safety_ready=g_can_guard_ready=g_can_poll_enable=1;
g_can_enable_address=1;zcalls=ecalls=stage=bad_call=0;zcode=ecode=1;during=0;during_value=0;
memset(&g_can_enable_zero_reply,0xA5,sizeof(g_can_enable_zero_reply));
memset(&g_can_enable_reply,0xA5,sizeof(g_can_enable_reply));}
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %d\n",__LINE__);return 1;}}while(0)
int main(void){unsigned int i;
volatile uint32_t *bad[]={&g_rtos_fault,&g_safety_fault,&g_motion_diag_fault_latched,&g_motion_auto_stop_state,&g_motion_auto_stop_cut_vm,&g_motion_diag_hold_report,&g_motion_diag_hold_stop_service,&g_motion_diag_fail_stop_once,&g_can_stop_request,&g_can_disable_request};
volatile uint32_t *good[]={&g_vm_test_request,&g_safety_ready,&g_can_guard_ready,&g_can_poll_enable};
base();step();CHECK(!zcalls&&!ecalls);
for(i=1;i<=4;i++){base();g_can_enable_address=i;g_can_enable_request=1;step();CHECK(zcalls==1&&ecalls==1&&!bad_call&&last_address==i);CHECK(g_can_enable_zero_result==1&&g_can_enable_result==1&&!g_can_enable_request);CHECK(g_can_enable_zero_reply.Data[0]==0xF6&&g_can_enable_reply.Data[0]==0xF3);step();CHECK(zcalls==1&&ecalls==1);}
for(i=0;i<2;i++){base();g_can_enable_address=i?5:0;g_can_enable_request=1;step();CHECK(g_can_enable_result==7&&!zcalls&&!ecalls&&!g_can_enable_request);CHECK(!g_can_enable_reply.Data[0]&&!g_can_enable_zero_reply.Data[0]);}
for(i=0;i<10;i++){base();*bad[i]=1;g_can_enable_request=1;step();CHECK(g_can_enable_result==9&&!zcalls&&!ecalls&&!g_can_enable_request);CHECK(!g_can_enable_reply.Data[0]);*bad[i]=0;step();CHECK(!zcalls&&!ecalls);
base();during=bad[i];during_value=1;g_can_enable_request=1;step();CHECK(zcalls==1&&!ecalls&&g_can_enable_result==9&&!g_can_enable_request);CHECK(!g_can_enable_reply.Data[0]);}
for(i=0;i<4;i++){base();*good[i]=0;g_can_enable_request=1;step();CHECK(g_can_enable_result==9&&!zcalls&&!ecalls);*good[i]=1;step();CHECK(!zcalls&&!ecalls);
base();during=good[i];during_value=0;g_can_enable_request=1;step();CHECK(zcalls==1&&!ecalls&&g_can_enable_result==9);}
for(i=2;i<=8;i++){base();zcode=i;g_can_enable_request=1;step();CHECK(zcalls==1&&!ecalls&&g_can_enable_zero_result==i&&g_can_enable_result==10);CHECK(!g_can_enable_reply.Data[0]);step();CHECK(zcalls==1&&!ecalls);}
base();ecode=5;fake_can.ESR=0x123;g_can_enable_request=1;step();CHECK(zcalls==1&&ecalls==1&&g_can_enable_result==5&&g_can_enable_esr==0x123);
base();during=&g_can_enable_address;during_value=4;g_can_enable_request=1;step();CHECK(last_address==1&&!bad_call&&ecalls==1);
printf("PASS %u checks (actual ready helper and enable branch, mocked CAN)\n",checks);return 0;}
