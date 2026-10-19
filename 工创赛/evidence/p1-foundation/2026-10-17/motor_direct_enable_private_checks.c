#include <stdio.h>
#include <string.h>
typedef unsigned int uint32_t;typedef unsigned char uint8_t;
typedef struct {uint32_t ExtId;uint8_t DLC,Data[8];} CanRxMsg;
#define MOTION_STOP_IDLE 0U
struct {uint32_t ESR;} fake_can;
#define CAN1 (&fake_can)
CanRxMsg g_can_enable_reply;
volatile uint32_t g_can_disable_request;
volatile uint32_t g_can_enable_address;
volatile uint32_t g_can_enable_esr;
volatile uint32_t g_can_enable_request;
volatile uint32_t g_can_enable_result;
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
static unsigned int calls,last_address,last_enabled,rcode,checks;
static uint32_t CAN1_SetMotorEnable(uint8_t address,uint8_t enabled,CanRxMsg *reply){calls++;last_address=address;last_enabled=enabled;reply->Data[0]=0xF3;return rcode;}
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
            CanRxMsg reply = {0};

            /* 地址本轮冻结，请求先消费，拒绝或失败均不自动重试 */
            g_can_enable_request = 0U;
            g_can_enable_result = 0U;

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
                g_can_enable_result = CAN1_SetMotorEnable(
                    (uint8_t)address, 1U, &reply);
            }

            g_can_enable_reply = reply;
            g_can_enable_esr = CAN1->ESR;
        }
}
static void base(void){
g_can_disable_request=0;
g_can_enable_address=0;
g_can_enable_esr=0;
g_can_enable_request=0;
g_can_enable_result=0;
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
g_vm_test_request=g_safety_ready=g_can_guard_ready=g_can_poll_enable=1;g_can_enable_address=1;calls=0;rcode=1;
memset(&g_can_enable_reply,0xA5,sizeof(g_can_enable_reply));}
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %d\n",__LINE__);return 1;}}while(0)
int main(void){unsigned int i;
volatile uint32_t *bad[]={&g_rtos_fault,&g_safety_fault,&g_motion_diag_fault_latched,&g_motion_auto_stop_state,&g_motion_auto_stop_cut_vm,&g_motion_diag_hold_report,&g_motion_diag_hold_stop_service,&g_motion_diag_fail_stop_once,&g_can_stop_request,&g_can_disable_request};
volatile uint32_t *good[]={&g_vm_test_request,&g_safety_ready,&g_can_guard_ready,&g_can_poll_enable};
base();step();CHECK(!calls);
for(i=1;i<=4;i++){base();g_can_enable_address=i;g_can_enable_request=1;step();CHECK(calls==1&&last_address==i&&last_enabled==1);CHECK(g_can_enable_result==1&&!g_can_enable_request&&g_can_enable_reply.Data[0]==0xF3);step();CHECK(calls==1);}
for(i=0;i<2;i++){base();g_can_enable_address=i?5:0;g_can_enable_request=1;step();CHECK(g_can_enable_result==7&&!calls&&!g_can_enable_request);CHECK(!g_can_enable_reply.Data[0]);}
for(i=0;i<10;i++){base();*bad[i]=1;g_can_enable_request=1;step();CHECK(g_can_enable_result==9&&!calls&&!g_can_enable_request);CHECK(!g_can_enable_reply.Data[0]);*bad[i]=0;step();CHECK(!calls);}
for(i=0;i<4;i++){base();*good[i]=0;g_can_enable_request=1;step();CHECK(g_can_enable_result==9&&!calls&&!g_can_enable_request);*good[i]=1;step();CHECK(!calls);}
for(i=2;i<=8;i++){base();rcode=i;fake_can.ESR=0x123;g_can_enable_request=1;step();CHECK(calls==1&&g_can_enable_result==i&&g_can_enable_esr==0x123&&!g_can_enable_request);step();CHECK(calls==1);}
printf("PASS %u checks (actual direct-enable branch/helper, mocked CAN)\n",checks);return 0;}
