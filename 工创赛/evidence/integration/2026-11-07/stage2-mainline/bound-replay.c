
#include "source_stage2_backend.h"
#include "source_stage2_owner.h"
#include "source_stage2_vision_bridge.h"
#include "source_vision_runtime.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint32_t now_ms;
static uint8_t uart_bytes[128];static unsigned uart_head,uart_tail;
volatile uint32_t g_uart6_rx_errors;
static uint32_t xTaskGetTickCount(void){return now_ms;}
static void USART6_VisionEnableReceive(void){}
static uint8_t USART6_VisionReadByte(uint8_t*b){if(uart_tail==uart_head)return 0;*b=uart_bytes[uart_tail++];return 1;}
static uint8_t USART6_VisionSend(const uint8_t*b,uint16_t n){(void)b;(void)n;return 1;}
static uint32_t SourceObserver_SharedByte(uint8_t b,uint32_t now,uint32_t active){(void)b;(void)now;(void)active;return 0;}
static uint32_t SourceObserver_TrySend(uint8_t(*send)(const uint8_t*,uint16_t)){(void)send;return 0;}
static uint32_t g_it_request,g_it_state,g_lift_compare_target_pulses=118800;
static uint32_t g_source_parallel_cause,g_vehicle_arm_cause,g_source_parallel_state;
static uint32_t g_source_parallel_stop_mask,g_vehicle_arm_stop_pending,g_lift_ruler_stop_pending;
static uint32_t owned,busy,arm_calls,tray_calls,grip_calls,route_calls,route_polls;
static uint32_t checks,failures,negative;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d t%u\n",__LINE__,now_ms);}}while(0)
static uint32_t it_platform(void){return 1;}
static uint32_t SingleTeach_ReferencesReady(void){return 1;}
static uint32_t VehicleArm_Busy(void){return busy;}
static uint32_t VehicleSourceParallel_Owns(void){return owned;}
static void VehicleSourceParallel_Cancel(void){owned=busy=0;g_source_parallel_stop_mask=0;}
static uint32_t VehicleArm_BeginSourceParallel(const SPF_Command*c,uint32_t verified,uint32_t driving){
 CK(verified==1);if(c->kind==SPF_CMD_ROUTE_AND_STOW)CK(driving==1);
 CK(c->axes_mask>0 && c->axes_mask<=7);arm_calls++;owned=busy=1;g_source_parallel_state=1;
 if(c->kind==SPF_CMD_PICK){
  CK(c->goal.lift_pulses==118800);
  CK(c->goal.dm_deg==(c->item==0?200:c->item==1?190:165));
  CK(c->goal.extension_mm==(c->item==0?80:c->item==1?48:60));
 }
 return 1;}
static uint32_t jiazhua(float deg){CK(deg==140 || deg==210);grip_calls++;return 1;}
static uint32_t CarrierPwm_Write(uint32_t pwm){
 CK(pwm==(tray_calls==0?500:tray_calls==1?1389:2278));tray_calls++;return 1;}
volatile uint32_t g_src_txn_revision=0U,g_src_txn_consumed=0U,g_src_txn_seq=0U;
volatile uint32_t g_src_txn_tick=0U,g_src_txn_result=0U,g_src_txn_age_ms=0U,g_src_slot=1U;
volatile uint32_t g_src_now_ms=0U,g_src_txn_takes=0U;
volatile float g_src_extension_mm=0.0f;
volatile float g_src_pick_deg=180.0f,g_src_chassis_dx_mm=0.0f,g_src_chassis_dy_mm=0.0f;
static uint32_t g_src_txn_last_seq=0U;
/* 3秒停盘窗口；取件角允许 0~270（源区分支），底盘纠偏暂限 ±20mm。 */
uint32_t SrcTxn_Take(uint32_t now)
{
 float deg,dx,dy;
 g_src_txn_takes++;
 if(g_src_txn_revision==0U || g_src_txn_revision==g_src_txn_consumed){g_src_txn_result=2U;return 0U;}
 deg=g_src_pick_deg;dx=g_src_chassis_dx_mm;dy=g_src_chassis_dy_mm;
 if(!(deg>=0.0f && deg<=270.0f) || !(dx>=-20.0f && dx<=20.0f) || !(dy>=-20.0f && dy<=20.0f) ||
    g_src_slot<1U || g_src_slot>3U || !(g_src_extension_mm>=0.0f && g_src_extension_mm<=170.0f)){g_src_txn_result=4U;return 0U;}
 if(g_src_txn_seq==0U || g_src_txn_seq==g_src_txn_last_seq){g_src_txn_result=5U;return 0U;}
 g_src_txn_age_ms=(uint32_t)(now-g_src_txn_tick);
 if(g_src_txn_age_ms>3000U){g_src_txn_result=3U;return 0U;}
 g_src_txn_last_seq=g_src_txn_seq;
 g_src_txn_consumed=g_src_txn_revision;
 g_src_txn_result=1U;
 return 1U;
}
/* ===== 视觉串口解析（USART6 PC6/PC7 115200 8N1）=====
 * 帧：CC func ... DD。长度按功能码定：0x02 -> 6字节(左/右/下颜色)，0x05 -> 7字节(转盘中心 x,y u16小端)。
 * 解析按长度驱动，不靠 0xCC/0xDD 做重同步——因为坐标的低/高字节本身可能就是 0xCC/0xDD。
 * 调试注入：写 g_vis_test_byte 后把 g_vis_test_push 置1，poll 会把它当成一个收到的字节，
 * 这样不接香橙派也能验证帧状态机（不能证明电气层，只证明解析）。 */
volatile uint32_t g_vis_rx_bytes=0U,g_vis_rx_bad=0U,g_vis_rx_armed=0U;
volatile uint32_t g_vis_test_byte=0U,g_vis_test_push=0U;
volatile uint32_t g_vis_mat_cycle=0U,g_vis_mat_left=0U,g_vis_mat_right=0U,g_vis_mat_down=0U,g_vis_mat_tick=0U;
volatile uint32_t g_vis_center_cycle=0U,g_vis_center_x=0U,g_vis_center_y=0U,g_vis_center_tick=0U;
/* 视觉区 -> 机械臂取件姿态的标定表（左/右/下三个抓取区），taught_mask 每位置示教完置位 */
volatile float g_vis_region_deg[3]={180.0f,180.0f,180.0f};
volatile float g_vis_region_ext_mm[3]={0.0f,0.0f,0.0f};
volatile uint32_t g_vis_taught_mask=0U,g_vis_pick_region=0U;
/* 像素->毫米：与底盘纠偏参考中心；mm_per_px=0 表示未标定，纠偏保持 0 而不猜数 */
volatile float g_vis_mm_per_px=0.0f,g_vis_ref_x=0.0f,g_vis_ref_y=0.0f;
volatile uint32_t g_vis_txn_result=0U;
volatile uint32_t g_vis_material_valid=0U,g_vis_center_valid=0U;
volatile uint32_t g_vis_mat_consumed=0U,g_vis_metric_valid=0U;
volatile uint32_t g_vis_command_request=0U,g_vis_command_sent=0U,g_vis_command_rejected=0U;
static uint32_t vis_last_byte_ms=0U,vis_errors_seen=0U;

/* Task order is independent of region and carrier slot. No actuator access. */
volatile uint32_t g_vis_task_valid=0U,g_vis_task_batch=0U,g_vis_task_item=0U,g_vis_task_run_id=0U;
volatile uint32_t g_vis_task_colors[3]={0U,0U,0U};
volatile uint32_t g_vis_pending_seq=0U,g_vis_pending_slot=0U,g_vis_pending_grasped=0U;
volatile uint32_t g_vis_loaded_mask=0U,g_vis_batch_complete=0U;
static char vis_task_code[16];

uint32_t Vision_SetTask(const char *code,uint32_t batch,uint32_t run)
{
 uint32_t i,j,offset;uint32_t first=0U,second=0U,rings1=0U,rings2=0U;
 if(!code || !run || (batch!=1U && batch!=2U) || g_vis_pending_seq)return 0U;
 for(i=0U;i<15U;i++){
  if(i==3U || i==7U || i==11U){if(code[i]!='+')return 0U;}
  else if(code[i]<'1' || code[i]>'6')return 0U;
 }
 if(code[15]!='\0')return 0U;
 for(i=0U;i<3U;i++){
  first|=1U<<(code[i]-'1');second|=1U<<(code[8U+i]-'1');
  if(code[4U+i]>'3' || code[12U+i]>'3')return 0U;
  rings1|=1U<<(code[4U+i]-'1');rings2|=1U<<(code[12U+i]-'1');
  for(j=0U;j<i;j++)if(code[i]==code[j] || code[8U+i]==code[8U+j])return 0U;
 }
 if(first!=second || rings1!=7U || rings2!=7U)return 0U;
 if(g_vis_task_valid && g_vis_task_run_id==run && g_vis_task_batch==batch && !memcmp(vis_task_code,code,16U))return 1U;
 memcpy(vis_task_code,code,16U);offset=batch==1U?0U:8U;
 for(i=0U;i<3U;i++)g_vis_task_colors[i]=(uint32_t)(code[offset+i]-'0');
 g_vis_task_valid=1U;g_vis_task_run_id=run;g_vis_task_batch=batch;g_vis_task_item=0U;
 g_vis_loaded_mask=g_vis_batch_complete=0U;g_vis_material_valid=0U;
 return 1U;
}

/* Only the existing mission/CAN owner calls these functions. External screen,
 * sensor or vision providers must supply actual evidence, not motor ACKs. */
uint32_t Vision_ReportGrasp(uint32_t run,uint32_t sequence,uint32_t source_clear,uint32_t held,
 uint32_t evidence_ms,uint32_t now)
{
 if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq || source_clear!=1U || held!=1U ||
    (uint32_t)(now-evidence_ms)>1000U)return 0U;
 g_vis_pending_grasped=1U;return 1U;
}

uint32_t Vision_ReportPlaced(uint32_t run,uint32_t sequence,uint32_t slot,uint32_t released,
 uint32_t present,uint32_t stable,uint32_t evidence_ms,uint32_t now)
{
 if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq || slot!=g_vis_pending_slot ||
    !g_vis_pending_grasped || released!=1U || present!=1U || stable!=1U ||
    (uint32_t)(now-evidence_ms)>1000U || g_vis_task_item>=3U)return 0U;
 g_vis_loaded_mask|=1U<<g_vis_task_item;g_vis_task_item++;
 g_vis_pending_seq=g_vis_pending_slot=g_vis_pending_grasped=0U;
 g_vis_material_valid=0U;
 if(g_vis_task_item==3U && g_vis_loaded_mask==7U)g_vis_batch_complete=1U;
 return 1U;
}

uint32_t Vision_RejectUnstarted(uint32_t run,uint32_t sequence)
{
 if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq ||
    g_vis_pending_grasped)return 0U;
 g_vis_pending_seq=g_vis_pending_slot=0U;
 /* The consumed frame remains consumed; retry requires a fresh observation. */
 return 1U;
}

uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3])
{
 uint32_t i;
 if(!colors || !g_vis_task_valid || run!=g_vis_task_run_id || batch!=g_vis_task_batch ||
    !g_vis_batch_complete || g_vis_loaded_mask!=7U || g_vis_task_item!=3U || g_vis_pending_seq)return 0U;
 for(i=0U;i<3U;i++)colors[i]=g_vis_task_colors[i];
 return 1U;
}

static uint8_t vis_buf[10];
static uint32_t vis_len=0U,vis_expect=0U;

static void Vision_Byte(uint8_t b)
{
 uint32_t tk=(uint32_t)xTaskGetTickCount();
 if(vis_len && (uint32_t)(tk-vis_last_byte_ms)>150U){vis_len=vis_expect=0U;g_vis_rx_bad++;}
 vis_last_byte_ms=tk;
 if(vis_len==0U){if(b==0xCCU){vis_buf[0]=b;vis_len=1U;}return;}
 if(vis_len>=sizeof(vis_buf)){vis_len=0U;vis_expect=0U;g_vis_rx_bad++;return;}
 vis_buf[vis_len++]=b;
 if(vis_len==2U)
 {
  if(b==0x02U)vis_expect=6U;
  else if(b==0x05U)vis_expect=7U;
  else {vis_len=0U;vis_expect=0U;g_vis_rx_bad++;return;}
 }
 if(vis_expect && vis_len==vis_expect)
 {
  if(vis_buf[vis_expect-1U]==0xDDU)
  {
   if(vis_buf[1]==0x02U)
   {
    if(vis_buf[2]>6U || vis_buf[3]>6U || vis_buf[4]>6U)g_vis_rx_bad++;
    else{g_vis_mat_left=vis_buf[2];g_vis_mat_right=vis_buf[3];g_vis_mat_down=vis_buf[4];
     g_vis_mat_tick=tk;g_vis_mat_cycle++;g_vis_material_valid=1U;}
   }
   else
   {g_vis_center_x=(uint32_t)vis_buf[2]|((uint32_t)vis_buf[3]<<8);
    g_vis_center_y=(uint32_t)vis_buf[4]|((uint32_t)vis_buf[5]<<8);
    g_vis_center_tick=tk;g_vis_center_cycle++;g_vis_center_valid=1U;}
  }else g_vis_rx_bad++;
  vis_len=0U;vis_expect=0U;
 }
}
void Vision_Poll(void)
{
 uint8_t b,command[3];uint32_t budget=128U,request;
#if VEHICLE_INTEGRATED_ENABLE
 char task_code[16];uint32_t task_run,task_batch;
 if(VehicleMission_ReadSourceTask(task_code,&task_run,&task_batch))
  (void)Vision_SetTask(task_code,task_batch,task_run);
#endif
 if(!g_vis_rx_armed){USART6_VisionEnableReceive();g_vis_rx_armed=1U;}
 if(vis_errors_seen!=g_uart6_rx_errors){
  vis_errors_seen=g_uart6_rx_errors;vis_len=vis_expect=0U;
  g_vis_material_valid=g_vis_center_valid=0U;
 }
 while(budget-- && USART6_VisionReadByte(&b)){g_vis_rx_bytes++;
  if(!SourceObserver_SharedByte(b,(uint32_t)xTaskGetTickCount(),vis_len!=0U))Vision_Byte(b);}
 (void)SourceObserver_TrySend(USART6_VisionSend);
 if(g_vis_test_push){g_vis_test_push=0U;Vision_Byte((uint8_t)g_vis_test_byte);}
 request=g_vis_command_request;
 if(request){
  if(request!=1U && request!=2U && request!=5U && request!=0x14U && request!=0x15U){
   g_vis_command_request=0U;g_vis_command_rejected++;
  }else{
   command[0]=0xaaU;command[1]=(uint8_t)request;command[2]=0xbbU;
   if(USART6_VisionSend(command,3U)){
    g_vis_command_request=0U;g_vis_command_sent++;
    if(request==2U)g_vis_material_valid=0U;
    if(request==5U)g_vis_center_valid=0U;
   }
  }
 }
}

static uint32_t Vision_TakeTarget(uint32_t now)
{
 uint32_t region=g_vis_pick_region,color;
 float dx=0.0f,dy=0.0f;
 g_vis_txn_result=0U;g_vis_metric_valid=0U;
 if(region>2U || !(g_vis_taught_mask&(1U<<region))){g_vis_txn_result=6U;return 0U;}
 if(!g_vis_material_valid || !g_vis_mat_cycle || (uint32_t)(now-g_vis_mat_tick)>3000U){g_vis_txn_result=7U;return 0U;}
 if(g_vis_mat_cycle==g_vis_mat_consumed){g_vis_txn_result=8U;return 0U;}
 color=region==0U?g_vis_mat_left:(region==1U?g_vis_mat_right:g_vis_mat_down);
 if(!color){g_vis_txn_result=9U;return 0U;}
 if(g_vis_mm_per_px>0.0f && g_vis_center_valid && g_vis_center_cycle &&
    (uint32_t)(now-g_vis_center_tick)<=3000U){
  dx=((float)g_vis_center_x-g_vis_ref_x)*g_vis_mm_per_px;
  dy=((float)g_vis_center_y-g_vis_ref_y)*g_vis_mm_per_px;
  g_vis_metric_valid=1U;
 }
 g_src_pick_deg=g_vis_region_deg[region];g_src_extension_mm=g_vis_region_ext_mm[region];
 g_src_chassis_dx_mm=dx;g_src_chassis_dy_mm=dy;
 g_src_txn_seq++;g_src_txn_tick=now;g_src_txn_revision++;
 if(!SrcTxn_Take(now))return 0U;
 g_vis_mat_consumed=g_vis_mat_cycle;return 1U;
}

static uint32_t Vision_TakeTaskTarget(uint32_t now)
{
 uint32_t colors[3],i,count=0U,region=0U,desired;
 if(!g_vis_task_valid || g_vis_task_item>=3U){g_vis_txn_result=10U;return 0U;}
 if(g_vis_pending_seq){g_vis_txn_result=11U;return 0U;}
 if(!g_vis_material_valid || (uint32_t)(now-g_vis_mat_tick)>3000U){g_vis_txn_result=7U;return 0U;}
 desired=g_vis_task_colors[g_vis_task_item];
 colors[0]=g_vis_mat_left;colors[1]=g_vis_mat_right;colors[2]=g_vis_mat_down;
 for(i=0U;i<3U;i++)if(colors[i]==desired){count++;region=i;}
 if(count!=1U){g_vis_txn_result=count?12U:9U;return 0U;}
 g_vis_pick_region=region;g_src_slot=g_vis_task_item+1U;
 if(!Vision_TakeTarget(now))return 0U;
 g_vis_pending_seq=g_src_txn_seq;g_vis_pending_slot=g_src_slot;
 g_vis_pending_grasped=0U;return 1U;
}


#include "source_stage2_backend.inc"
static uint32_t align_begin(void*p,const SPF_Command*c,uint32_t now){(void)p;(void)c;(void)now;return 1;}
static uint32_t align_poll(void*p,const SPF_Command*c,S2_Event*e,uint32_t now){
 (void)p;memset(e,0,sizeof(*e));e->run_id=c->run_id;e->sequence=c->sequence;e->evidence_flags=S2_ALIGNED;e->observed_ms=now;return 1;}
static uint32_t route_begin(void*p,const SPF_Command*c,uint32_t now){
 (void)p;(void)c;(void)now;CK(owned==1 && g_vis_loaded_mask==7 && g_vis_batch_complete);route_calls++;return 1;}
static uint32_t route_poll(void*p,const SPF_Command*c,uint32_t now){(void)p;(void)c;(void)now;return ++route_polls>=3?1:0;}
static uint32_t route_stop(void*p,uint32_t now){(void)p;(void)now;return 1;}
static uint32_t path(void*p,const SPF_Command*c){(void)p;(void)c;return 1;}
static void material(unsigned item){
 uint8_t frame[3][6]={{0xcc,2,4,6,1,0xdd},{0xcc,2,4,6,0,0xdd},{0xcc,2,0,6,0,0xdd}};
 memcpy(uart_bytes,frame[item],6);uart_head=6;uart_tail=0;Vision_Poll();
}
int main(int argc,char**argv){
 SPF_Config cfg;S2_BackendHooks hooks={0,align_begin,align_poll,route_begin,route_poll,route_stop,path};
 S2_ServoTiming timing={700,400};SPF_Command cmd;S2_Event evidence;S2_SourceObservation metadata;uint32_t colors[3];
 (void)argv;negative=argc>1;
 CK(Vision_SetTask("146+123+614+231",1,10));g_vis_taught_mask=7;
 g_vis_region_deg[0]=190;g_vis_region_deg[1]=165;g_vis_region_deg[2]=200;
 g_vis_region_ext_mm[0]=48;g_vis_region_ext_mm[1]=60;g_vis_region_ext_mm[2]=80;
 memset(&cfg,0,sizeof(cfg));cfg.run_id=10;cfg.pose_version=7;
 cfg.live_references_valid=cfg.deploy_sweep_valid=cfg.carrier_height_valid=1;
 cfg.transfer_sweep_valid=cfg.stow_while_driving_valid=1;cfg.transaction_timeout_ms=2000;
 cfg.requested_colors[0]=1;cfg.requested_colors[1]=4;cfg.requested_colors[2]=6;
 cfg.deploy=(SPF_Pose){180,48,0};cfg.transfer=(SPF_Pose){35.758f,0,7200};cfg.place=cfg.transfer;
 cfg.initial=(SPF_Pose){0,0,0};cfg.gripper_open_deg=210;cfg.gripper_close_deg=140;cfg.initial_gripper_deg=145;
 CK(SourceStage2Backend_Start(&cfg,&hooks,&timing,0));
 for(now_ms=10;now_ms<16000;now_ms+=10){
  if(owned){owned=busy=0;g_source_parallel_state=3;}
  if(g_source_stage2_state==SPF_WAIT_TARGET){
   material(g_vis_task_item);memset(&metadata,0,sizeof(metadata));
   metadata.run_id=10;metadata.pose_version=7;metadata.instance_id=100+g_vis_task_item;
   metadata.captured_ms=now_ms;metadata.usable_until_ms=now_ms+3000;
   metadata.capture_verified=metadata.stationary=metadata.sweep_valid=1;metadata.safe_exit_budget_ms=2400;
   CK(SourceStage2Backend_Select(&metadata,now_ms));
  }
  if(SourceStage2Backend_ReadActive(&cmd) && (cmd.kind==SPF_CMD_CLOSE || cmd.kind==SPF_CMD_RELEASE)){
   memset(&evidence,0,sizeof(evidence));evidence.run_id=10;evidence.sequence=cmd.sequence;
   evidence.observed_ms=now_ms;evidence.instance_id=100+g_vis_task_item;
   evidence.color_id=g_vis_task_colors[g_vis_task_item];evidence.slot=g_vis_task_item+1;
   evidence.evidence_flags=cmd.kind==SPF_CMD_CLOSE?(S2_SOURCE_CLEAR|S2_HELD):(S2_RELEASED|S2_PRESENT|S2_STABLE);
   if(negative && cmd.kind==SPF_CMD_RELEASE)evidence.evidence_flags&=~S2_STABLE;
   (void)SourceStage2Backend_Observe(&evidence,now_ms);
  }
  SourceStage2_Service(now_ms);
  if(g_source_stage2_completed || (g_source_stage2_fault && !SourceStage2_IsActive()))break;
 }
 if(!negative){
  CK(g_source_stage2_completed && g_source_stage2_state==SPF_DONE && g_vis_task_item==3);
  CK(Vision_ReadLoadedBatch(10,1,colors) && colors[0]==1 && colors[1]==4 && colors[2]==6);
  CK(g_source_stage2_loaded_mask==7 && route_calls==1 && route_polls==3);
  CK(tray_calls==3 && grip_calls==7 && arm_calls==13);
 }else{
  CK(!g_source_stage2_completed && g_source_stage2_fault && !SourceStage2_IsActive());
  CK(!g_vis_loaded_mask && !route_calls && g_vis_task_item==0);
 }
 printf("stage2 bound replay negative=%u checks=%u failures=%u arm_groups=%u tray=%u route_publications=%u\n",
  negative,checks,failures,arm_calls,tray_calls,route_calls);return failures?1:0;
}
