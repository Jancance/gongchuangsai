
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "lift_ruler_calibration.h"
static const char*test;
static uint32_t now,move_at,home_at,home_duration;
static unsigned checks,failures,moves,homes,stops,aborts,forbidden,enabled,engine;
static double motor,origin,goal,move_speed,move_accel,home_origin;
static void ck(unsigned ok,unsigned line){checks++;if(!ok){failures++;printf("FAIL %s line%u\n",test,line);}}
#define CK(x) ck((x),__LINE__)
static uint32_t scale=100;
static void update(void){
 if(engine==1){
  double distance=fabs(goal-origin),v=move_speed*6,a=move_accel*6,ta=v/a,tc,t=(uint32_t)(now-move_at)*.001,x;
  if(distance<v*ta){ta=sqrt(distance/a);v=a*ta;tc=0;}else tc=distance/v-ta;
  if(t<ta)x=.5*a*t*t;
  else if(t<ta+tc)x=.5*a*ta*ta+v*(t-ta);
  else if(t<2*ta+tc){double q=2*ta+tc-t;x=distance-.5*a*q*q;}
  else{x=distance;engine=0;}
  motor=origin+(goal>=origin?x:-x);
 }else if(engine==2){
  double u=(double)(uint32_t)(now-home_at)/home_duration;
  if(u>=1){motor=-12465;engine=0;}else motor=home_origin+(-12465-home_origin)*u;
 }
}
static double rpm_now(void){
 if(engine==2)return 60;
 if(engine==1){
  double distance=fabs(goal-origin),v=move_speed*6,a=move_accel*6,ta=v/a,tc,t=(uint32_t)(now-move_at)*.001;
  if(distance<v*ta){ta=sqrt(distance/a);v=a*ta;tc=0;}else tc=distance/v-ta;
  if(t<ta)return a*t/6;
  if(t<ta+tc)return v/6;
  if(t<2*ta+tc)return a*(2*ta+tc-t)/6;
 }return 0;
}
static void rawpos(uint8_t*r,double p){
 int64_t signed_raw=(int64_t)llround(p*scale);uint32_t v=(uint32_t)llabs(signed_raw);
 r[1]=signed_raw<0;r[2]=v>>24;r[3]=v>>16;r[4]=v>>8;r[5]=v;
}
static void home_params(uint8_t*r){
 const uint8_t value[17]={0x22,0,1,0,60,0,0,0x9c,0x40,1,0x2c,3,0x20,0,60,0,0x6b};
 memcpy(r,value,17);if(!strcmp(test,"wrong_menu"))r[4]=30;
}
static uint32_t bus(void*context,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 LiftRulerController*c=context;uint32_t u;double p;update();memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 if(!strcmp(test,"read_loss") && moves && q[0]!=0xfe && q[0]!=0x9c)return 0;
 if(n==2){
  switch(q[0]){
   case 0x1a:r[2]=scale==100?0x80:0;if(!strcmp(test,"bad_options"))r[2]|=2;break;
   case 0x22:CK(rn==17);home_params(r);break;
   case 0x36:
    p=motor;
    if(!strcmp(test,"position_frozen") && moves && !homes)p=origin;
    if(!strcmp(test,"overshoot") && moves && !homes)p=goal-100;
    rawpos(r,p);
    if(!strcmp(test,"bad_position") || (!strcmp(test,"bad_home_position") && c->phase==26))r[1]=2;
    break;
   case 0x31:
    u=(uint32_t)((12345LL+(int64_t)llround(motor*65536/360))&65535U);
    if(!strcmp(test,"bad_encoder") && moves)u=(u+1000)&65535U;
    r[1]=u>>8;r[2]=u;break;
   case 0x3a:r[1]=(enabled?1:0)|(moves && !engine?2:0);
    if(!strcmp(test,"disabled") && moves)r[1]&=~1U;
    if(!strcmp(test,"protection") && moves)r[1]|=4U;break;
   case 0x35:u=(uint32_t)llround(rpm_now()*10);
    if(!strcmp(test,"initial_moving") && !moves)u=5;
    if(!strcmp(test,"home_speed_nonzero") && homes && !engine)u=5;
    if(!strcmp(test,"overspeed") && moves && !homes)u=5000;
    r[1]=0;r[2]=u>>8;r[3]=u;break;
   case 0x33:rawpos(r,goal+(!strcmp(test,"wrong_target")?10:0));break;
   case 0x3b:r[1]=engine==2?7:3;
    if(!strcmp(test,"home_stale_idle"))r[1]=3;
    if(!strcmp(test,"home_failure") && homes)r[1]|=8;break;
   default:forbidden++;return 0;
  }return 1;
 }
 if(n==5 && q[0]==0xf3){CK(LiftRuler_CommandAllowed(c,q,n));enabled=1;r[1]=2;return 1;}
 if(n==15 && q[0]==0xfd){
  CK(LiftRuler_CommandAllowed(c,q,n));CK(q[12]==2 && !q[13] && q[14]==0x6b);
  u=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  move_speed=(((unsigned)q[6]<<8)|q[7])*.1;move_accel=((unsigned)q[2]<<8)|q[3];
  CK(move_accel==(((unsigned)q[4]<<8)|q[5]));
  CK((c->operation==0 && move_speed==300 && move_accel==60) ||
     (c->operation>0 && fabs(move_speed-1846.4)<1e-6 && move_accel==45055));
  CK(q[1]==(c->operation==1?0:1));
  origin=motor;goal=motor+(q[1]?-1:1)*(double)u/scale;
  if(c->operation==1)CK(fabs(goal-((double)c->top_position_raw/scale+c->down_target_pulses*360.0/3200))<.001);move_at=now;engine=1;moves++;r[1]=2;
  if(!strcmp(test,"move_ack_lost"))return 0;return 1;
 }
 if(n==4 && q[0]==0x9a){
  CK(LiftRuler_CommandAllowed(c,q,n));CK(q[1]==2 && !q[2] && c->home_params_valid);
  homes++;home_origin=motor;home_duration=c->operation==0?5000:1000;home_at=now;engine=2;r[1]=2;
  if(!strcmp(test,"home_ack_lost"))return 0;return 1;
 }
 if(n==3 && q[0]==0x9c){aborts++;if(!strcmp(test,"abort_ack_missing"))return 0;engine=0;r[1]=2;return 1;}
 if(n==4 && q[0]==0xfe){stops++;if(!strcmp(test,"stop_ack_missing"))return 0;engine=0;r[1]=2;return 1;}
 forbidden++;return 0;
}

#include "lift_ruler_adapter.h"
typedef struct {uint8_t Data[8],DLC,IDE,RTR;uint32_t ExtId;} CanRxMsg;
typedef CanRxMsg CanTxMsg;
typedef uint32_t TickType_t;
typedef struct {uint32_t active;} PowerCommandStatus;
volatile uint32_t g_safety_ready,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
volatile uint32_t g_can_test_result,g_can_guard_ready,g_vm_test_request,g_chassis_auto_active;
volatile uint32_t g_vehicle_arm_state,g_startup_axes_state,g_dm_jog_request,g_dm_jog_state;
volatile uint32_t g_startup_axes_vm_request,g_vehicle_arm_stop_pending;
static uint32_t power_active=1,s_can1_normal_ready=1,other_axis,legacy_calls,g_vehicle_key;
static uint8_t pending_frame[15],pending_length;static CanRxMsg queue[4];static unsigned qi,qn;
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_Mode_Normal 0
#define CAN_Id_Extended 1
#define CAN_RTR_Data 0
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define CAN_TxStatus_NoMailBox 4
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 30U
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
static uint32_t CAN_MessagePending(unsigned can,unsigned fifo){(void)can;(void)fifo;return qn-qi;}
static void StartupHome_Receive(CanRxMsg*r){*r=queue[qi++];}
static uint32_t CAN_TransmitStatus(unsigned can,unsigned mb){(void)can;(void)mb;return CAN_TxStatus_Ok;}
static void CAN_CancelTransmit(unsigned can,unsigned mb){(void)can;(void)mb;}
static uint32_t CAN1_ExchangeSingle(uint8_t address,const uint8_t*q,uint8_t n,uint8_t rn,CanRxMsg*r){
 if(address!=5){other_axis++;return 0;}
#if APP_MOTOR_OFFLINE_ONLY
 (void)q;(void)n;(void)rn;(void)r;return 9U;
#else
 memset(r,0,sizeof(*r));return bus(&g_lift_ruler_controller,q,n,rn,r->Data);
#endif
}
static uint8_t CAN_Transmit(unsigned can,const CanTxMsg*t){
 uint8_t address=t->ExtId>>8;unsigned packet=t->ExtId&255;(void)can;
 if(address!=5){other_axis++;return 3;}
 if(t->Data[0]==0x22 && t->DLC==2 && t->Data[1]==0x6b){
  uint8_t payload[17];unsigned offset=1,j,n;home_params(payload);qi=0;qn=3;
  for(j=0;j<3;j++){
   memset(&queue[j],0,sizeof(queue[j]));n=17-offset;if(n>7)n=7;
   queue[j].Data[0]=0x22;memcpy(queue[j].Data+1,payload+offset,n);offset+=n;
   queue[j].DLC=n+1;queue[j].ExtId=0x500U+j;queue[j].IDE=1;queue[j].RTR=0;
  }
  if(!strcmp(test,"params_bad_sequence"))queue[1].ExtId=0x503;
  return 0;
 }
 if(!packet){pending_frame[0]=t->Data[0];pending_length=1;}
 memcpy(pending_frame+pending_length,t->Data+1,t->DLC-1);pending_length+=t->DLC-1;
 if(pending_length==15){qi=0;qn=0;memset(&queue[0],0,sizeof(queue[0]));
  qn=bus(&g_lift_ruler_controller,pending_frame,15,3,queue[0].Data);
  queue[0].DLC=3;queue[0].IDE=1;queue[0].RTR=0;queue[0].ExtId=(uint32_t)address<<8;
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
volatile uint32_t g_lift_compare_operation,g_lift_compare_top_valid,g_lift_compare_home_flags;
volatile uint32_t g_lift_compare_home_seen,g_lift_compare_home_reported,g_lift_compare_home_exit;
volatile uint32_t g_lift_compare_speed_raw,g_lift_compare_move_elapsed_ms;
volatile int32_t g_lift_compare_top_raw;
volatile uint32_t g_lift_compare_target_pulses=108400U; /* configuration, not a motion request */
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
 if(n==2U && q[0]==0x22U && q[1]==0x6bU && rn==17U)
  return LiftRuler_ReadHomeParameters(r)==1U;
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
 g_lift_compare_operation=g_lift_ruler_controller.operation;
 g_lift_compare_top_valid=g_lift_ruler_controller.top_valid;
 g_lift_compare_top_raw=(int32_t)g_lift_ruler_controller.top_position_raw;
 g_lift_compare_home_flags=g_lift_ruler_controller.home_flags;
 g_lift_compare_home_seen=g_lift_ruler_controller.home_seen_running;
 g_lift_compare_home_reported=g_lift_ruler_controller.home_reported;
 g_lift_compare_home_exit=g_lift_ruler_controller.home_exit_confirmed;
 g_lift_compare_speed_raw=g_lift_ruler_controller.speed_raw;
 g_lift_compare_move_elapsed_ms=g_lift_ruler_controller.move_elapsed_ms;
 g_lift_ruler_state=g_lift_ruler_controller.state;
}
void LiftRuler_DebugKey(void)
{
 if(!VEHICLE_LIFT_RULER_ONLY)return;
 taskENTER_CRITICAL();
 g_lift_ruler_request=((g_lift_ruler_controller.state==LRC_IDLE || g_lift_ruler_controller.state==LRC_DONE) &&
  !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;
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
 if(request==LRC_REQUEST_START){
  if(!LiftRuler_SetDownTarget(&g_lift_ruler_controller,g_lift_compare_target_pulses))g_lift_ruler_rejected=4U;
  else if(!LiftRuler_Start(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;
 }
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
    if(VEHICLE_LIFT_RULER_ONLY && address==5U && reply_length==3U &&
       ((length==2U && q[0]==0x3bU && q[1]==0x6bU) ||
        (length==3U && q[0]==0x9cU && q[1]==0x48U && q[2]==0x6bU)))
        return CAN1_ExchangeSingle(5U,q,length,3U,reply);
    if(length==4U && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b)
        return CAN1_ExchangeSingle(address,q,length,3U,reply);
#if APP_MOTOR_OFFLINE_ONLY
    return 9U;
#else
    if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
       (VEHICLE_LIFT_RULER_ONLY ? !LiftRuler_DebugCanAllowed(address,q,length) :
        (!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U)))return 9U;
    if(VEHICLE_LIFT_RULER_ONLY && address==5U && length==4U && reply_length==3U &&
       q[0]==0x9aU && q[1]==2U && q[2]==0U && q[3]==0x6bU)
        return CAN1_ExchangeSingle(5U,q,length,3U,reply);
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
#if !APP_MOTOR_OFFLINE_ONLY
uint32_t LiftRuler_ReadHomeParameters(uint8_t out[17])
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result;
    uint8_t mailbox, status, flush, packet = 0U, offset = 1U, chunk;
    if(!VEHICLE_LIFT_RULER_ONLY || !g_lift_ruler_controller.vm_request || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
    if (out == NULL) return 7U;
    memset(out, 0, 17U);

    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U) return result;
        s_can1_normal_ready = 1U;
    }
    for (flush = 0U; flush < 3U && CAN_MessagePending(CAN1, CAN_FIFO0); flush++)
        StartupHome_Receive(&rx);
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = 0x0500U;
    tx.IDE = CAN_Id_Extended; tx.RTR = CAN_RTR_Data;
    tx.DLC = 2U; tx.Data[0] = 0x22U; tx.Data[1] = 0x6BU;
    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox) return 4U;
    started = xTaskGetTickCount();
    for (;;)
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
        if (status == CAN_TxStatus_Ok) break;
        if (status == CAN_TxStatus_Failed ||
            (TickType_t)(xTaskGetTickCount() - started) >= pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS))
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return 4U;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    started = xTaskGetTickCount();
    out[0] = 0x22U;
    while ((TickType_t)(xTaskGetTickCount() - started) < pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            StartupHome_Receive(&rx);
            if (rx.IDE == CAN_Id_Extended && rx.RTR == CAN_RTR_Data &&
                (rx.ExtId >> 8) == 5U && rx.DLC != 0U && rx.Data[0] == 0x22U)
            {

                chunk = (uint8_t)(17U - offset);
                if (chunk > 7U) chunk = 7U;
                if (rx.ExtId != (0x0500U | packet) || rx.DLC != (uint8_t)(chunk + 1U))
                    return 6U;
                memcpy(&out[offset], &rx.Data[1], chunk);
                offset = (uint8_t)(offset + chunk);
                if (offset == 17U) return out[16] == 0x6BU ? 1U : 6U;
                packet++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return 5U;
}
#else
uint32_t LiftRuler_ReadHomeParameters(uint8_t p[17]){(void)p;return 9U;}
#endif
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
 test=name;now=0;moves=homes=stops=aborts=forbidden=enabled=engine=0;motor=0;scale=100;
 other_axis=legacy_calls=qi=qn=pending_length=0;g_lift_ruler_request=g_lift_ruler_rejected=g_vm_test_request=0;g_lift_compare_target_pulses=108400U;
 g_safety_ready=g_can_test_result=g_can_guard_ready=1;g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 g_chassis_auto_active=g_vehicle_arm_state=g_startup_axes_state=g_dm_jog_request=g_dm_jog_state=0;
 g_startup_axes_vm_request=g_vehicle_arm_stop_pending=g_vehicle_key=0;power_active=1;
 LiftRuler_Init(&g_lift_ruler_controller);
}
static void run_case(const char*name){
 unsigned k,i,cancelled,before_m,before_h;uint32_t base;reset_case(name);
 for(i=0;i<10;i++){dispatch();LiftRuler_DebugSafety();now+=20;}
 CK(!moves && !homes && !g_vm_test_request && !legacy_calls);
 for(k=0;k<5;k++){
  g_vehicle_key=1;mission_key();CK(!g_vehicle_key && g_lift_ruler_request==LRC_REQUEST_START);
  base=now;cancelled=0;
  for(i=0;i<1700;i++){
   now=base+i*20;
   if(!cancelled && ((!strcmp(name,"cancel_home") && homes) ||
      ((!strcmp(name,"cancel_pre") || !strcmp(name,"stop_ack_missing")) && moves))){
    g_vehicle_key=1;mission_key();cancelled=1;
   }
   LiftRuler_DebugSafety();dispatch();
#if APP_MOTOR_OFFLINE_ONLY
   if(i>4)break;
#else
   if(g_lift_ruler_state>=LRC_DONE && !g_lift_ruler_stop_pending)break;
#endif
  }
  CK(!other_axis && !legacy_calls && !forbidden);
#if APP_MOTOR_OFFLINE_ONLY
  CK(!moves && !homes && !stops && !g_vm_test_request && g_lift_ruler_rejected==2);break;
#else
  CK(g_vm_test_request==1);
  if(!strcmp(name,"normal")){
   CK(g_lift_ruler_state==LRC_DONE && g_lift_compare_operation==(k==0?1U:(k%2?2U:3U)) && g_lift_ruler_samples==k+1 && g_lift_compare_top_valid);
   CK(moves==k+1 && homes==1+k/2);
   if(k%2==0)CK(g_lift_compare_home_seen && g_lift_compare_home_reported);
   before_m=moves;before_h=homes;
   for(i=0;i<10;i++){now+=20;dispatch();LiftRuler_DebugSafety();}
   CK(moves==before_m && homes==before_h && g_vm_test_request==1);
   continue;
  }
  CK(g_lift_ruler_state!=LRC_DONE && !g_lift_compare_top_valid && !g_lift_ruler_samples);
  if(!strcmp(name,"stop_ack_missing"))CK(g_lift_ruler_stop_pending && g_lift_ruler_stop_unconfirmed);
  else CK(!g_lift_ruler_stop_pending && stops>0);
  if(homes)CK(aborts>0);
  break;
#endif
 }
#if !APP_MOTOR_OFFLINE_ONLY
 if(!strcmp(name,"normal")){
  g_lift_compare_target_pulses=110801U;
  LiftRuler_DebugKey();dispatch();CK(g_lift_ruler_rejected==4 && g_lift_compare_operation==3 && g_vm_test_request==1);
  g_lift_compare_target_pulses=108000U;LiftRuler_DebugKey();dispatch();
  CK(g_lift_ruler_controller.down_target_pulses==108000U && g_vm_test_request==1 && g_lift_compare_operation==1);
 }
#endif
}
int main(void){
 const char*names[]={"normal","wrong_menu","params_bad_sequence","cancel_pre","cancel_home",
 "home_ack_lost","stop_ack_missing"};unsigned i;uint8_t en[]={0xf3,0xab,1,0,0x6b},home[]={0x9a,2,0,0x6b};CanRxMsg rx;
 for(i=0;i<sizeof(names)/sizeof(names[0]);i++)run_case(names[i]);
 reset_case("legacy_bypass");g_startup_axes_state=4;g_startup_axes_vm_request=1;
 CK(VehicleArm_Exchange(6,en,5,3,&rx)!=1);CK(VehicleArm_Exchange(5,en,5,3,&rx)!=1);
 CK(VehicleArm_Exchange(5,home,4,3,&rx)!=1);CK(!moves && !homes && !enabled);
 printf("COMPARE_DISPATCH offline=%u scenarios=8 checks=%u failures=%u\n",APP_MOTOR_OFFLINE_ONLY,checks,failures);
 return failures?1:0;
}
