#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t s_vehicle_code_valid,g_vehicle_scan_bypassed,g_vehicle_run_id,g_vehicle_stage,g_vehicle_cause;
static char s_vehicle_task_code[16];
uint32_t VehicleMission_ReadSourceTask(char code[16],uint32_t *run,uint32_t *batch)
{
 uint32_t valid=0U;
 if(!code || !run || !batch)return 0U;
 taskENTER_CRITICAL();
 if(s_vehicle_code_valid && !g_vehicle_scan_bypassed && g_vehicle_run_id &&
    (g_vehicle_stage==2U || g_vehicle_stage==5U) && !g_vehicle_cause){
  memcpy(code,s_vehicle_task_code,16U);*run=g_vehicle_run_id;
  *batch=g_vehicle_stage==2U?1U:2U;valid=1U;
 }
 taskEXIT_CRITICAL();return valid;
}


int main(void){char code[16];uint32_t run=0,batch=0;unsigned checks=0;
#define CK(v) do{checks++;if(!(v))return 1;}while(0)
 CK(!VehicleMission_ReadSourceTask(code,&run,&batch));
 memcpy(s_vehicle_task_code,"146+123+614+231",16);s_vehicle_code_valid=1;g_vehicle_run_id=10;g_vehicle_stage=2;
 CK(VehicleMission_ReadSourceTask(code,&run,&batch) && run==10 && batch==1 && !memcmp(code,s_vehicle_task_code,16));
 g_vehicle_stage=5;CK(VehicleMission_ReadSourceTask(code,&run,&batch) && batch==2);
 g_vehicle_stage=3;CK(!VehicleMission_ReadSourceTask(code,&run,&batch));g_vehicle_stage=2;
 g_vehicle_scan_bypassed=1;CK(!VehicleMission_ReadSourceTask(code,&run,&batch));g_vehicle_scan_bypassed=0;
 g_vehicle_cause=1;CK(!VehicleMission_ReadSourceTask(code,&run,&batch));g_vehicle_cause=0;
 g_vehicle_run_id=0;CK(!VehicleMission_ReadSourceTask(code,&run,&batch));
 CK(!VehicleMission_ReadSourceTask(0,&run,&batch));
 printf("source QR provider checks=%u failures=0\n",checks);return 0;}
