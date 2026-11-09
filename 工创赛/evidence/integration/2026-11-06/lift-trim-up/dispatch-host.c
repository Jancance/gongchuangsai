
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "lift_ruler_adapter.h"
#include "lift_ruler_calibration.h"
static unsigned checks,failures,moves,stops,enable,forbidden;static uint32_t now,moved_at;
static uint32_t scale=100;static double initial=-123.4,delta=0,speed=0;static const char*test;
static void ck(unsigned v,unsigned line){checks++;if(!v){failures++;printf("FAIL %s line%u\n",test,line);}}
#define CK(v) ck((v),__LINE__)
static void rawpos(uint8_t*r,double value){int64_t p=(int64_t)llround(value*scale);uint32_t u=(uint32_t)(p<0?-p:p);r[1]=p<0;r[2]=u>>24;r[3]=u>>16;r[4]=u>>8;r[5]=u;}
static void update(void){
 double t=(double)(uint32_t)(now-moved_at)*.001,ta=.5,v=180,a=360,total=2.0;
 if(!moves){delta=speed=0;return;}
 if(t<ta){delta=.5*a*t*t;speed=a*t;}
 else if(t<1.5){delta=45+v*(t-ta);speed=v;}
 else if(t<total){double q=total-t;delta=270.0-.5*a*q*q;speed=a*q;}
 else{delta=270.0;speed=0;}
}
static uint32_t bus(void*context,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 uint32_t u;double angle;LiftRulerController*c=context;
 update();memset(r,0,8);r[0]=q[0];r[rn-1]=0x6b;
 if(!strcmp(test,"power_loss") && moves && (uint32_t)(now-moved_at)>500)return 0;
 if(n==2){
  switch(q[0]){
   case 0x1a:r[2]=(scale==100?0x80:0);if(!strcmp(test,"bad_options"))r[2]|=2;break;
   case 0x36:angle=initial-delta;
    if(!strcmp(test,"position_frozen"))angle=initial;
    if(moves && !strcmp(test,"wrong_direction"))angle=initial+3;
    if(moves && !strcmp(test,"overshoot"))angle=initial-273.0;
    rawpos(r,angle);if(!strcmp(test,"bad_position_sign"))r[1]=2;
    if(moves && !strcmp(test,"read_failure"))return 0;
    break;
   case 0x31:{uint32_t enc=(uint32_t)(16000LL+(int64_t)-llround(delta*65536/360))&65535U;
    if(moves && !strcmp(test,"bad_encoder"))enc=(enc+1000U)&65535U;
    r[1]=enc>>8;r[2]=enc;break;}
   case 0x3a:r[1]=(enable?1:0)|((moves && delta>=270.0)?2:0);
    if(moves && !strcmp(test,"disabled"))r[1]&=~1U;
    if(moves && !strcmp(test,"protection"))r[1]|=4;
    if(moves && !strcmp(test,"early_done_bit"))r[1]|=2;
    break;
   case 0x35:u=(uint32_t)llround(speed/6*10);
    if(!strcmp(test,"initial_fraction_speed") && !moves)u=5;
    if(!strcmp(test,"fraction_speed") && moves && delta>=270.0)u=5;
    if(!strcmp(test,"overspeed") && moves)u=500;
    r[1]=0;r[2]=u>>8;r[3]=u;break;
   case 0x33:rawpos(r,initial-270.0+(!strcmp(test,"target_mismatch")?10:0));break;
   default:forbidden++;return 0;
  }
 }else if(n==5 && q[0]==0xf3){
  CK(LiftRuler_CommandAllowed(c,q,n));enable=1;r[1]=2;
  if(!strcmp(test,"enable_ack_lost"))return 0;
 }else if(n==15 && q[0]==0xfd){
  CK(LiftRuler_CommandAllowed(c,q,n));
  u=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  CK(q[1]==1 && q[2]==0 && q[3]==60 && q[4]==0 && q[5]==60);
  CK((((unsigned)q[6]<<8)|q[7])==300 && u==2400U*360U*scale/3200U && q[12]==2 && q[13]==0 && q[14]==0x6b);
  moves++;moved_at=now;r[1]=2;if(!strcmp(test,"move_ack_lost"))return 0;
 }else if(n==4 && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b){
  stops++;if(!strcmp(test,"stop_ack_missing") || (!strcmp(test,"stop_then_ack") && stops<3))return 0;r[1]=2;
 }else{forbidden++;return 0;}
 return 1;
}

#include <stdlib.h>
typedef struct {uint8_t Data[8],DLC,IDE,RTR;uint32_t ExtId;} CanRxMsg;
typedef CanRxMsg CanTxMsg;
typedef uint32_t TickType_t;
typedef struct {uint32_t active;} PowerCommandStatus;
volatile uint32_t g_safety_ready,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
volatile uint32_t g_can_test_result,g_can_guard_ready,g_vm_test_request,g_chassis_auto_active;
volatile uint32_t g_vehicle_arm_state,g_startup_axes_state,g_dm_jog_request,g_dm_jog_state;
volatile uint32_t g_startup_axes_vm_request,g_vehicle_arm_stop_pending;
static uint32_t power_active=1,s_can1_normal_ready=1,other_axis,legacy_calls,g_vehicle_key;
static uint8_t pending_frame[15],pending_length;static CanRxMsg queued;static unsigned queued_valid;
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_Mode_Normal 0
#define CAN_Id_Extended 1
#define CAN_RTR_Data 0
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(v) (v)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define VEHICLE_LIFT_RULER_ONLY 1U
#define VEHICLE_EXTENSION_HOME_ONLY 0U
#define VEHICLE_STARTUP_GROUP_ENABLE 1U
#define VEHICLE_DM_STARTUP_ONLY 0U
#define VEHICLE_STARTUP_GROUP_ONLY 0U
#define VEHICLE_INTEGRATED_ENABLE 1U
static uint32_t xTaskGetTickCount(void){return now;}
static void vTaskDelay(uint32_t t){now+=t;}
static void Power_ReadCommandStatus(PowerCommandStatus*p){p->active=power_active;}
static uint32_t CAN1_InitMode(unsigned mode){(void)mode;return 1;}
static uint32_t CAN_MessagePending(unsigned can,unsigned fifo){(void)can;(void)fifo;return queued_valid;}
static void StartupHome_Receive(CanRxMsg*r){*r=queued;queued_valid=0;}
static uint32_t CAN_TransmitStatus(unsigned can,unsigned mb){(void)can;(void)mb;return CAN_TxStatus_Ok;}
static void CAN_CancelTransmit(unsigned can,unsigned mb){(void)can;(void)mb;}
static uint32_t CAN1_ExchangeSingle(uint8_t address,const uint8_t*q,uint8_t n,uint8_t rn,CanRxMsg*r){
 if(address!=5){other_axis++;return 0;}
 memset(r,0,sizeof(*r));return bus(&g_lift_ruler_controller,q,n,rn,r->Data);
}
static uint8_t CAN_Transmit(unsigned can,const CanTxMsg*t){
 uint8_t address=t->ExtId>>8;unsigned packet=t->ExtId&255;(void)can;
 if(address!=5){other_axis++;return 3;}
 if(!packet){pending_frame[0]=t->Data[0];pending_length=1;}
 memcpy(pending_frame+pending_length,t->Data+1,t->DLC-1);pending_length+=t->DLC-1;
 if(pending_length==15){memset(&queued,0,sizeof(queued));
  queued_valid=bus(&g_lift_ruler_controller,pending_frame,15,3,queued.Data);
  queued.DLC=3;queued.IDE=CAN_Id_Extended;queued.RTR=CAN_RTR_Data;queued.ExtId=(uint32_t)address<<8;
 }return 0;
}
uint32_t VehicleArm_Exchange(uint8_t,const uint8_t*,uint8_t,uint8_t,void*);
static void StartupAxes_Service(unsigned a,unsigned b){(void)a;(void)b;legacy_calls++;}
static unsigned StartupAxes_Holding(void){legacy_calls++;return 1;}
static void DM_CAN2_ModuleService(unsigned a,unsigned b){(void)a;(void)b;legacy_calls++;}
static unsigned VehicleArm_Busy(void){legacy_calls++;return 0;}
static void VehicleArm_Service(void){legacy_calls++;}
static void App_CombinedTestStep(void){legacy_calls++;}
LiftRulerController g_lift_ruler_controller;
volatile uint32_t g_lift_ruler_request,g_lift_ruler_rejected;
volatile uint32_t g_lift_ruler_state,g_lift_ruler_phase,g_lift_ruler_cause,g_lift_ruler_samples;
volatile uint32_t g_lift_ruler_options,g_lift_ruler_encoder_before,g_lift_ruler_encoder_after;
volatile uint32_t g_lift_ruler_stop_pending,g_lift_ruler_stop_unconfirmed,g_lift_ruler_vm_request;
volatile int32_t g_lift_ruler_before_raw,g_lift_ruler_after_raw,g_lift_ruler_target_raw;
extern volatile uint32_t g_safety_ready,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
extern volatile uint32_t g_can_test_result,g_can_guard_ready,g_vm_test_request,g_chassis_auto_active;
static uint32_t ruler_ready(void)
{
 return g_safety_ready==1U && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched &&
  !g_chassis_auto_active && g_vehicle_arm_state==0U && g_startup_axes_state==0U && !g_dm_jog_request && g_dm_jog_state==0U;
}
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t ruler_exchange(void*unused,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r)
{
 CanRxMsg rx;uint32_t result;(void)unused;memset(&rx,0,sizeof(rx));
 if(n>15 || rn>8)return 0;
 result=VehicleArm_Exchange(5U,q,(uint8_t)n,(uint8_t)rn,&rx);
 if(result!=1U)return 0;
 memcpy(r,rx.Data,8);return 1;
}
#endif
static void ruler_publish(void)
{
 g_lift_ruler_phase=g_lift_ruler_controller.phase;g_lift_ruler_cause=g_lift_ruler_controller.cause;
 g_lift_ruler_samples=g_lift_ruler_controller.samples;g_lift_ruler_options=g_lift_ruler_controller.options;
 g_lift_ruler_encoder_before=g_lift_ruler_controller.encoder_before;g_lift_ruler_encoder_after=g_lift_ruler_controller.encoder_after;
 g_lift_ruler_before_raw=(int32_t)g_lift_ruler_controller.command_before_raw;
 g_lift_ruler_after_raw=(int32_t)g_lift_ruler_controller.position_after_raw;
 g_lift_ruler_target_raw=(int32_t)g_lift_ruler_controller.driver_target_raw;
 g_lift_ruler_stop_pending=g_lift_ruler_controller.stop_pending;
 g_lift_ruler_stop_unconfirmed=g_lift_ruler_controller.stop_unconfirmed;
 g_lift_ruler_vm_request=g_lift_ruler_controller.vm_request;
 g_lift_ruler_state=g_lift_ruler_controller.state;
}
void LiftRuler_DebugKey(void)
{
 if(!VEHICLE_LIFT_RULER_ONLY)return;
 taskENTER_CRITICAL();
 g_lift_ruler_request=(g_lift_ruler_controller.state==LRC_IDLE && !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;
 taskEXIT_CRITICAL();
}
void LiftRuler_DebugService(void)
{
 uint32_t request;
#if !APP_MOTOR_OFFLINE_ONLY
 uint32_t ready,now;PowerCommandStatus power;
#endif
 if(!VEHICLE_LIFT_RULER_ONLY)return;
 taskENTER_CRITICAL();request=g_lift_ruler_request;g_lift_ruler_request=0;taskEXIT_CRITICAL();
#if APP_MOTOR_OFFLINE_ONLY
 if(request)g_lift_ruler_rejected=2U;
 g_vm_test_request=0U;ruler_publish();return;
#else
 now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;ready=ruler_ready();
 if(request==LRC_REQUEST_START && !LiftRuler_Start(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;
 if(request==LRC_REQUEST_STOP)LiftRuler_Stop(&g_lift_ruler_controller,now);
 Power_ReadCommandStatus(&power);
 LiftRuler_Service(&g_lift_ruler_controller,now,ready,power.active,g_can_guard_ready,ruler_exchange,0);
 g_vm_test_request=g_lift_ruler_controller.vm_request;ruler_publish();
#endif
}
void LiftRuler_DebugSafety(void)
{
 if(!VEHICLE_LIFT_RULER_ONLY)return;
 if(!ruler_ready() && g_lift_ruler_controller.state!=LRC_IDLE && g_lift_ruler_controller.state<LRC_DONE)
  g_lift_ruler_request=LRC_REQUEST_STOP;
 /* Keep ordinary platform Safety_Update. No new fault path cuts VM. */
 g_vm_test_request=g_lift_ruler_controller.vm_request;
}
uint32_t LiftRuler_DebugCanAllowed(uint8_t address,const uint8_t*q,uint32_t n)
{
 return VEHICLE_LIFT_RULER_ONLY && address==5U && LiftRuler_CommandAllowed(&g_lift_ruler_controller,q,n);
}
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t length,uint8_t reply_length,void *out)
{
    CanRxMsg *reply=(CanRxMsg*)out;
#if !APP_MOTOR_OFFLINE_ONLY
    CanRxMsg rx;CanTxMsg tx;uint8_t packet=0U,offset=1U,chunk,mailbox,status,flush;
    TickType_t start;uint32_t r;
#endif
    if(!q || !out || (address!=5U && address!=6U))return 7U;
    if(VEHICLE_LIFT_RULER_ONLY && address!=5U)return 9U;
    if(length==2U && q[1]==0x6b &&
       (((q[0]==0x1a || q[0]==0x31) && reply_length==4U) || ((q[0]==0x36 || q[0]==0x33) && reply_length==7U) ||
        (q[0]==0x3a && reply_length==3U) || (q[0]==0x35 && reply_length==5U)))
        return CAN1_ExchangeSingle(address,q,length,reply_length,reply);
#if (VEHICLE_EXTENSION_HOME_ONLY || VEHICLE_STARTUP_GROUP_ENABLE)
    if(address==6U && length==2U && q[0]==0x3bU && q[1]==0x6bU && reply_length==3U)
        return CAN1_ExchangeSingle(6U,q,length,3U,reply);
    if(address==6U && length==3U && q[0]==0x9cU && q[1]==0x48U && q[2]==0x6bU && reply_length==3U)
        return CAN1_ExchangeSingle(6U,q,length,3U,reply);
#if !APP_MOTOR_OFFLINE_ONLY
    if(address==6U && length==4U && q[0]==0x9aU && q[1]==2U && q[2]==0U && q[3]==0x6bU && reply_length==3U){
        if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
        return CAN1_ExchangeSingle(6U,q,length,3U,reply);
    }
#endif
#endif
    if(length==4U && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
#if APP_MOTOR_OFFLINE_ONLY
    return 9U;
#else
    if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
       (VEHICLE_LIFT_RULER_ONLY ? !LiftRuler_DebugCanAllowed(address,q,length) :
        (!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U)))return 9U;
    if(length==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
    if(length!=15U || q[0]!=0xfd || q[1]>1 || q[12]!=2 || q[13]!=0 || q[14]!=0x6b || reply_length!=3U)return 7U;
    {
        uint32_t speed=((uint32_t)q[6]<<8)|q[7];
        if(!speed || speed>(address==5U?18464U:1060U) || (!q[2] && !q[3]) || (!q[4] && !q[5]))return 7U;
    }
    if(!s_can1_normal_ready){r=CAN1_InitMode(CAN_Mode_Normal);if(r!=1U)return r;s_can1_normal_ready=1U;}
    for(flush=0;flush<3 && CAN_MessagePending(CAN1,CAN_FIFO0);flush++)StartupHome_Receive(&rx);
    while(offset<length) {
        memset(&tx,0,sizeof(tx));chunk=length-offset;if(chunk>7)chunk=7;
        tx.ExtId=((uint32_t)address<<8)|packet;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;tx.DLC=chunk+1;tx.Data[0]=q[0];
        memcpy(&tx.Data[1],&q[offset],chunk);mailbox=CAN_Transmit(CAN1,&tx);if(mailbox>2)return 4U;
        start=xTaskGetTickCount();
        for(;;){status=CAN_TransmitStatus(CAN1,mailbox);if(status==CAN_TxStatus_Ok)break;
            if(status==CAN_TxStatus_Failed || xTaskGetTickCount()-start>=pdMS_TO_TICKS(20U)){CAN_CancelTransmit(CAN1,mailbox);return 4U;}
            vTaskDelay(pdMS_TO_TICKS(1U));}
        offset+=chunk;packet++;
    }
    start=xTaskGetTickCount();
    while(xTaskGetTickCount()-start<pdMS_TO_TICKS(30U)) {
        if(CAN_MessagePending(CAN1,CAN_FIFO0)) {
            StartupHome_Receive(&rx);
            if(rx.IDE==CAN_Id_Extended && rx.RTR==CAN_RTR_Data && rx.ExtId==((uint32_t)address<<8) && rx.Data[0]==0xfd)
            { *reply=rx;return rx.DLC==3 && rx.Data[2]==0x6b ? 1U:6U; }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }return 5U;
#endif
}
static void dispatch(void){
                if(VEHICLE_LIFT_RULER_ONLY)LiftRuler_DebugService();
                else {
                StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);
                if(!VEHICLE_LIFT_RULER_ONLY && !VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(!VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_DM_STARTUP_ONLY && !VEHICLE_STARTUP_GROUP_ONLY && (VehicleArm_Busy() || g_vehicle_arm_stop_pending))VehicleArm_Service();
                if(!VEHICLE_LIFT_RULER_ONLY && !VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(!VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_DM_STARTUP_ONLY && !VEHICLE_STARTUP_GROUP_ONLY && !VehicleArm_Busy() && (!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding()) && g_startup_axes_state!=1U && g_startup_axes_state!=2U && g_startup_axes_state!=3U) App_CombinedTestStep();
                }

}
static void mission_key(void){uint32_t key;
    taskENTER_CRITICAL();key=g_vehicle_key;g_vehicle_key=0U;taskEXIT_CRITICAL();
    if(VEHICLE_LIFT_RULER_ONLY){if(key)LiftRuler_DebugKey();return;}
}

static void reset_case(const char*name){
 test=name;now=0;moves=stops=enable=forbidden=other_axis=legacy_calls=0;delta=speed=0;scale=100;
 queued_valid=pending_length=0;g_lift_ruler_request=g_lift_ruler_rejected=g_vm_test_request=0;
 g_safety_ready=g_can_test_result=g_can_guard_ready=1;g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 g_chassis_auto_active=g_vehicle_arm_state=g_startup_axes_state=g_dm_jog_request=g_dm_jog_state=0;
 g_startup_axes_vm_request=g_vehicle_arm_stop_pending=g_vehicle_key=0;power_active=1;
 LiftRuler_Init(&g_lift_ruler_controller);
}
int main(void){
 const char*names[]={"normal_scaled","cancel","stop_ack_missing","stop_then_ack","guard_lost"};
 unsigned i,j,cancelled;uint8_t en[]={0xf3,0xab,1,0,0x6b};CanRxMsg rx;
 for(i=0;i<sizeof(names)/sizeof(names[0]);i++){
  reset_case(names[i]);
  for(j=0;j<4;j++){dispatch();LiftRuler_DebugSafety();now+=50;}
  CK(!moves && !enable && !g_vm_test_request && !legacy_calls);
  g_vehicle_key=1;mission_key();CK(!g_vehicle_key && g_lift_ruler_request==LRC_REQUEST_START);
  cancelled=0;
  for(j=0;j<180;j++){
   if(moves && !cancelled && i>0){
    if(i==4)g_safety_ready=0;
    else{g_vehicle_key=1;mission_key();}
    cancelled=1;
   }
   LiftRuler_DebugSafety();dispatch();now+=50;
   if(g_lift_ruler_state>=LRC_DONE && !g_lift_ruler_stop_pending)break;
  }
#if APP_MOTOR_OFFLINE_ONLY
  CK(!moves && !enable && !stops && !g_vm_test_request && g_lift_ruler_rejected==2);
#else
  CK(moves==1 && !legacy_calls && !other_axis && !forbidden && g_vm_test_request==1);
  if(!i)CK(g_lift_ruler_state==LRC_DONE && g_lift_ruler_samples==1);
  else{CK(g_lift_ruler_state>=LRC_CANCELLED && !g_lift_ruler_samples);
   if(i==2)CK(g_lift_ruler_stop_pending && g_lift_ruler_stop_unconfirmed && stops>3);
   else CK(!g_lift_ruler_stop_pending && stops>=1);
  }
  j=moves;g_vehicle_key=1;mission_key();dispatch();CK(moves==j);
#endif
 }
 reset_case("admission");g_startup_axes_state=4;g_startup_axes_vm_request=1;
 CK(VehicleArm_Exchange(6,en,5,3,&rx)!=1);CK(VehicleArm_Exchange(5,en,5,3,&rx)!=1);
 CK(!moves && !enable && !other_axis);
 printf("DISPATCH offline=%u scenarios=6 checks=%u failures=%u\n",APP_MOTOR_OFFLINE_ONLY,checks,failures);
 return failures?1:0;
}
