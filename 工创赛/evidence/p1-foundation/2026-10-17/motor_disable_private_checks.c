#include <stdio.h>
#include <string.h>
typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef struct { uint32_t ExtId; uint8_t DLC; uint8_t Data[8]; } CanRxMsg;
#define MOTION_STOP_IDLE 0U
struct { uint32_t ESR; } fake_can;
#define CAN1 (&fake_can)
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
CanRxMsg g_can_disable_reply;
static unsigned int calls,last_address,last_enabled,reply_code=1,checks;
static uint32_t CAN1_SetMotorEnable(uint8_t address,uint8_t enabled,CanRxMsg *reply){calls++;last_address=address;last_enabled=enabled;reply->Data[0]=0xF3;return reply_code;}
volatile uint32_t g_vm_test_request;
volatile uint32_t g_can_disable_address;
volatile uint32_t g_can_disable_request;
volatile uint32_t g_can_disable_result;
volatile uint32_t g_can_disable_esr;
volatile uint32_t g_motion_diag_hold_report;
volatile uint32_t g_motion_diag_hold_stop_service;
volatile uint32_t g_motion_diag_fail_stop_once;
volatile uint32_t g_motion_diag_fault_latched;
volatile uint32_t g_motion_auto_stop_state;
volatile uint32_t g_motion_auto_stop_cut_vm;
static void step(void) {
        if (g_can_disable_request == 1U)
        {
            uint32_t address = g_can_disable_address;
            CanRxMsg reply = {0};

            /* 拒绝也消费请求，不等条件恢复后自动发送 */
            g_can_disable_request = 0U;
            g_can_disable_result = 0U;

            if ((address < 1U) || (address > 4U))
            {
                g_can_disable_result = 7U;
            }
            else if ((g_vm_test_request != 1U) ||
                     (g_rtos_fault != 0U) ||
                     (g_safety_fault != 0U) ||
                     (g_safety_ready != 1U) ||
                     (g_can_guard_ready != 1U) ||
                     (g_motion_diag_fault_latched != 0U) ||
                     (g_motion_auto_stop_state != MOTION_STOP_IDLE) ||
                     (g_motion_auto_stop_cut_vm != 0U) ||
                     (g_motion_diag_hold_report != 0U) ||
                     (g_motion_diag_hold_stop_service != 0U) ||
                     (g_motion_diag_fail_stop_once != 0U))
            {
                /* 静态测试条件不满足，返回9且不发送 */
                g_can_disable_result = 9U;
            }
            else
            {
                g_can_disable_result =
                    CAN1_SetMotorEnable((uint8_t)address, 0U, &reply);
            }

            g_can_disable_reply = reply;
            g_can_disable_esr = CAN1->ESR;
        }
}
static void base(void){
g_vm_test_request=0;
g_can_disable_address=0;
g_can_disable_request=0;
g_can_disable_result=0;
g_can_disable_esr=0;
g_motion_diag_hold_report=0;
g_motion_diag_hold_stop_service=0;
g_motion_diag_fail_stop_once=0;
g_motion_diag_fault_latched=0;
g_motion_auto_stop_state=0;
g_motion_auto_stop_cut_vm=0;
g_vm_test_request=g_safety_ready=g_can_guard_ready=1;
g_can_disable_address=1; calls=0;reply_code=1;
memset(&g_can_disable_reply,0xA5,sizeof(g_can_disable_reply));
}
#define CHECK(x) do {checks++;if(!(x)){printf("FAIL line %d\n",__LINE__);return 1;}} while(0)
int main(void){unsigned int i;
volatile uint32_t *rejects[]={&g_rtos_fault,&g_safety_fault,&g_motion_diag_fault_latched,&g_motion_auto_stop_state,&g_motion_auto_stop_cut_vm,&g_motion_diag_hold_report,&g_motion_diag_hold_stop_service,&g_motion_diag_fail_stop_once};
volatile uint32_t *ready[]={&g_vm_test_request,&g_safety_ready,&g_can_guard_ready};
base();step();CHECK(calls==0);
for(i=1;i<=4;i++){base();g_can_disable_address=i;g_can_disable_request=1;step();CHECK(calls==1 && last_address==i && last_enabled==0);CHECK(g_can_disable_request==0 && g_can_disable_result==1);step();CHECK(calls==1);}
for(i=0;i<2;i++){base();g_can_disable_address=i?5:0;g_can_disable_request=1;step();CHECK(calls==0 && g_can_disable_result==7 && g_can_disable_request==0);CHECK(g_can_disable_reply.Data[0]==0);}
for(i=0;i<8;i++){base();*rejects[i]=1;g_can_disable_request=1;step();CHECK(calls==0 && g_can_disable_result==9 && g_can_disable_request==0);CHECK(g_can_disable_reply.Data[0]==0);*rejects[i]=0;step();CHECK(calls==0);}
for(i=0;i<3;i++){base();*ready[i]=0;g_can_disable_request=1;step();CHECK(calls==0 && g_can_disable_result==9 && g_can_disable_request==0);*ready[i]=1;step();CHECK(calls==0);}
base();reply_code=5;fake_can.ESR=0x123;g_can_disable_request=1;step();CHECK(g_can_disable_result==5 && g_can_disable_esr==0x123);
printf("PASS %u checks (actual main disable branch, mocked CAN)\n",checks);return 0;}
