#include <stdint.h>
#include <stdio.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t g_vehicle_stage=2,g_vehicle_run_id=42,g_vehicle_state=8,g_stage2_ready,loaded,advances;
static uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3]){
 (void)colors;return loaded && run==42 && batch==1;}
static uint32_t SourceStage2_IsActive(void){return 0;}
static void VehicleMission_Advance(void){advances++;g_vehicle_state=4;}
static uint32_t VehicleMission_SourceBatchComplete(void)
{
 uint32_t colors[3],ready=0U;
 if(g_vehicle_stage!=2U || !g_vehicle_run_id)return 0U;
 taskENTER_CRITICAL();
 ready=Vision_ReadLoadedBatch(g_vehicle_run_id,1U,colors);
 taskEXIT_CRITICAL();return ready;
}


static void service(void){
    if(g_vehicle_state==8U){
        if(!SourceStage2_IsActive() && VehicleMission_SourceBatchComplete()){
            g_stage2_ready=1U;VehicleMission_Advance();
        }
        return;
    }

}

int main(void){unsigned n=0;
#define CK(v) do{n++;if(!(v))return 1;}while(0)
 service();CK(g_vehicle_state==8 && !g_stage2_ready && !advances);
 loaded=1;g_vehicle_run_id=43;service();CK(g_vehicle_state==8 && !advances);
 g_vehicle_run_id=42;g_vehicle_stage=3;service();CK(g_vehicle_state==8 && !advances);
 g_vehicle_stage=2;service();CK(g_vehicle_state==4 && g_stage2_ready && advances==1);
 service();CK(advances==1);
 printf("source loaded handoff checks=%u failures=0\n",n);return 0;}
