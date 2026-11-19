"""Exercise actual new CAN broadcast function with a simulated CAN peripheral."""
from build_integration23_sync_20261115 import run,P,E

def main():
    can=(P/'CHASSIS/SYSTEM/CAN/can.c').read_bytes().decode('gb18030')
    fn=can[can.index('uint32_t CAN1_StartQueuedSourceAxes(void)'):can.index('uint32_t StartupAxis_Exchange(')]
    prefix=r'''
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#ifndef APP_MOTOR_OFFLINE_ONLY
#define APP_MOTOR_OFFLINE_ONLY 0
#endif
#define CAN_Id_Extended 1
#define CAN_RTR_Data 0
#define CAN1 1
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define pdMS_TO_TICKS(x) (x)
typedef unsigned TickType_t;
typedef struct{unsigned ExtId,IDE,RTR,DLC;uint8_t Data[8];}CanTxMsg;
static unsigned checks,tick,allow=1,sent,cancelled,result=1;
static unsigned g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_startup_axes_vm_request=1,s_can1_normal_ready=1;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t a,const uint8_t*q,uint32_t n){
 CHECK(a==0 && n==3 && q[0]==0xff && q[1]==0x66 && q[2]==0x6b);return allow;
}
static unsigned xTaskGetTickCount(void){return tick;}
static void vTaskDelay(unsigned n){tick+=n;}
static unsigned CAN_Transmit(unsigned bus,CanTxMsg*t){
 CHECK(bus==1 && t->ExtId==0 && t->IDE==1 && t->RTR==0 && t->DLC==3);
 CHECK(t->Data[0]==0xff && t->Data[1]==0x66 && t->Data[2]==0x6b);sent++;return 0;
}
static unsigned CAN_TransmitStatus(unsigned b,unsigned m){(void)b;(void)m;return result;}
static void CAN_CancelTransmit(unsigned b,unsigned m){(void)b;(void)m;cancelled++;}
'''
    suffix=r'''
int main(void){
#if APP_MOTOR_OFFLINE_ONLY
 CHECK(CAN1_StartQueuedSourceAxes()==9 && !sent);
#else
 CHECK(CAN1_StartQueuedSourceAxes()==1 && sent==1 && !cancelled);
 allow=0;CHECK(CAN1_StartQueuedSourceAxes()==9 && sent==1);allow=1;
 g_safety_fault=1;CHECK(CAN1_StartQueuedSourceAxes()==9 && sent==1);g_safety_fault=0;
 result=2;CHECK(CAN1_StartQueuedSourceAxes()==4 && sent==2 && cancelled==1);
 result=0;tick=0;CHECK(CAN1_StartQueuedSourceAxes()==4 && tick==20 && cancelled==2);
#endif
 printf("PASS actual CAN sync broadcast: %u checks, offline=%u; exact frame, existing owner/fault admission and TX failure/timeout\n",checks,APP_MOTOR_OFFLINE_ONLY);return 0;
}
'''
    run('actual-can-sync-broadcast',prefix+fn+suffix)
    # Offline stubs are intentionally unused when the production branch is absent.
    offline=(prefix+fn+suffix).replace('#define APP_MOTOR_OFFLINE_ONLY 0','#define APP_MOTOR_OFFLINE_ONLY 1')
    # Referencing fixture globals keeps -Werror useful for actual production code.
    offline=offline.replace(' CHECK(CAN1_StartQueuedSourceAxes()==9 && !sent);',
        ' (void)g_safety_fault;(void)g_rtos_fault;(void)g_can_ctrl_fault_latched;'
        '(void)g_startup_axes_vm_request;(void)s_can1_normal_ready;'
        '(void)result;(void)allow;(void)cancelled;CHECK(CAN1_StartQueuedSourceAxes()==9 && !sent);')
    run('actual-can-sync-broadcast-offline',offline)
    (E/'broadcast-checks.txt').write_text('Actual CAN broadcast online and offline passed.\n',encoding='utf-8')

if __name__=='__main__':main()
