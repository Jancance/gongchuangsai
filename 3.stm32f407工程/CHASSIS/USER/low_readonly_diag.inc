/* 只读28驱动配置与状态；不发使能/运动/清错、不修改已有参考和故障。 */
#include "power.h"
volatile uint32_t g_recovery_diag_request,g_recovery_diag_cycle,g_recovery_diag_done;
/* request1 remains read-only; request2 is explicit user-approved Clog_Ms=3000. */
volatile uint32_t g_recovery_config_ack,g_recovery_config_result;
volatile uint8_t g_recovery_config_before[36],g_recovery_config_after[36];
volatile uint32_t g_recovery_diag_result[5];
volatile uint8_t g_recovery_diag_raw[4][8],g_recovery_diag_config[36];
static void Low_ReadonlyDiag(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
 PowerCommandStatus power;CanRxMsg reply;uint32_t i,j,request,ack,result;uint8_t before[36],after[36];
 static const uint8_t commands[4]={0x1a,0x3a,0x35,0x36};
 static const uint8_t lengths[4]={4,3,5,7};uint8_t query[2],configuration[36];
 request=g_recovery_diag_request;if(request!=1U && request!=2U)return;
 g_recovery_diag_request=0U;g_recovery_diag_done=0U;
 Power_ReadCommandStatus(&power);
 if(!power.active || Low_Busy() || VehicleSourceParallel_Owns() || g_it_state==1U ||
  g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || g_chassis_auto_active ||
  g_lift_ruler_stop_pending || g_vehicle_arm_stop_pending || g_source_parallel_stop_mask){
  for(i=0U;i<5U;i++)g_recovery_diag_result[i]=9U;
  g_recovery_diag_cycle++;return;
 }
 for(i=0U;i<4U;i++){
  query[0]=commands[i];query[1]=0x6bU;memset(&reply,0,sizeof(reply));
  g_recovery_diag_result[i]=VehicleArm_Exchange(6U,query,2U,lengths[i],&reply);
  for(j=0U;j<8U;j++)g_recovery_diag_raw[i][j]=reply.Data[j];
 }

 if(request==2U){
  /* Reuse normal status/zero-speed checks. This request never runs in motion. */
  result=9U;ack=0U;memset(before,0,sizeof(before));memset(after,0,sizeof(after));
  if(g_recovery_diag_result[0]==1U && g_recovery_diag_result[1]==1U &&
     g_recovery_diag_result[2]==1U && g_recovery_diag_result[3]==1U &&
     !(g_recovery_diag_raw[1][1]&0x0cU) &&
     g_recovery_diag_raw[2][1]<=1U && !g_recovery_diag_raw[2][2] && !g_recovery_diag_raw[2][3])
   result=CAN1_ApplyExtensionClog3000(before,after,&ack);
  g_recovery_config_result=result;g_recovery_config_ack=ack;g_recovery_diag_result[4]=result;
  for(i=0U;i<36U;i++){
   g_recovery_config_before[i]=before[i];g_recovery_config_after[i]=after[i];
   g_recovery_diag_config[i]=after[i];
  }
  g_recovery_diag_done=1U;g_recovery_diag_cycle++;return;
 }
 memset(configuration,0,sizeof(configuration));
 g_recovery_diag_result[4]=CAN1_ReadExtensionConfiguration(configuration);
 for(i=0U;i<36U;i++)g_recovery_diag_config[i]=configuration[i];
 g_recovery_diag_done=1U;g_recovery_diag_cycle++;
#endif
}
