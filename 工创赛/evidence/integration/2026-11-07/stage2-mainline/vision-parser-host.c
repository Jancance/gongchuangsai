
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "source_stage2_vision_bridge.h"
static uint32_t now_ms;
static uint8_t incoming[2048],sent[32];
static uint32_t rx_head,rx_tail,rx_enable_count,tx_count,tx_busy;
volatile uint32_t g_uart6_rx_errors;
static uint32_t xTaskGetTickCount(void){return now_ms;}
static void USART6_VisionEnableReceive(void){rx_enable_count++;}
static uint8_t USART6_VisionReadByte(uint8_t*p){if(rx_tail==rx_head)return 0;*p=incoming[rx_tail++];return 1;}
static uint8_t USART6_VisionSend(const uint8_t*p,uint16_t n){if(tx_busy)return 0;memcpy(sent,p,n);tx_count++;return 1;}
/* MA1 receive/CRC/request path is tested separately using its actual module. */
static uint32_t SourceObserver_SharedByte(uint8_t b,uint32_t now,uint32_t active){(void)b;(void)now;(void)active;return 0;}
static uint32_t SourceObserver_TrySend(uint8_t(*send)(const uint8_t*,uint16_t)){(void)send;return 0;}
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

static const uint8_t material_wire[][6]={{204,2,0,0,0,221},{204,2,0,0,1,221},{204,2,0,0,2,221},{204,2,0,0,3,221},{204,2,0,0,4,221},{204,2,0,0,5,221},{204,2,0,0,6,221},{204,2,0,1,0,221},{204,2,0,1,1,221},{204,2,0,1,2,221},{204,2,0,1,3,221},{204,2,0,1,4,221},{204,2,0,1,5,221},{204,2,0,1,6,221},{204,2,0,2,0,221},{204,2,0,2,1,221},{204,2,0,2,2,221},{204,2,0,2,3,221},{204,2,0,2,4,221},{204,2,0,2,5,221},{204,2,0,2,6,221},{204,2,0,3,0,221},{204,2,0,3,1,221},{204,2,0,3,2,221},{204,2,0,3,3,221},{204,2,0,3,4,221},{204,2,0,3,5,221},{204,2,0,3,6,221},{204,2,0,4,0,221},{204,2,0,4,1,221},{204,2,0,4,2,221},{204,2,0,4,3,221},{204,2,0,4,4,221},{204,2,0,4,5,221},{204,2,0,4,6,221},{204,2,0,5,0,221},{204,2,0,5,1,221},{204,2,0,5,2,221},{204,2,0,5,3,221},{204,2,0,5,4,221},{204,2,0,5,5,221},{204,2,0,5,6,221},{204,2,0,6,0,221},{204,2,0,6,1,221},{204,2,0,6,2,221},{204,2,0,6,3,221},{204,2,0,6,4,221},{204,2,0,6,5,221},{204,2,0,6,6,221},{204,2,1,0,0,221},{204,2,1,0,1,221},{204,2,1,0,2,221},{204,2,1,0,3,221},{204,2,1,0,4,221},{204,2,1,0,5,221},{204,2,1,0,6,221},{204,2,1,1,0,221},{204,2,1,1,1,221},{204,2,1,1,2,221},{204,2,1,1,3,221},{204,2,1,1,4,221},{204,2,1,1,5,221},{204,2,1,1,6,221},{204,2,1,2,0,221},{204,2,1,2,1,221},{204,2,1,2,2,221},{204,2,1,2,3,221},{204,2,1,2,4,221},{204,2,1,2,5,221},{204,2,1,2,6,221},{204,2,1,3,0,221},{204,2,1,3,1,221},{204,2,1,3,2,221},{204,2,1,3,3,221},{204,2,1,3,4,221},{204,2,1,3,5,221},{204,2,1,3,6,221},{204,2,1,4,0,221},{204,2,1,4,1,221},{204,2,1,4,2,221},{204,2,1,4,3,221},{204,2,1,4,4,221},{204,2,1,4,5,221},{204,2,1,4,6,221},{204,2,1,5,0,221},{204,2,1,5,1,221},{204,2,1,5,2,221},{204,2,1,5,3,221},{204,2,1,5,4,221},{204,2,1,5,5,221},{204,2,1,5,6,221},{204,2,1,6,0,221},{204,2,1,6,1,221},{204,2,1,6,2,221},{204,2,1,6,3,221},{204,2,1,6,4,221},{204,2,1,6,5,221},{204,2,1,6,6,221},{204,2,2,0,0,221},{204,2,2,0,1,221},{204,2,2,0,2,221},{204,2,2,0,3,221},{204,2,2,0,4,221},{204,2,2,0,5,221},{204,2,2,0,6,221},{204,2,2,1,0,221},{204,2,2,1,1,221},{204,2,2,1,2,221},{204,2,2,1,3,221},{204,2,2,1,4,221},{204,2,2,1,5,221},{204,2,2,1,6,221},{204,2,2,2,0,221},{204,2,2,2,1,221},{204,2,2,2,2,221},{204,2,2,2,3,221},{204,2,2,2,4,221},{204,2,2,2,5,221},{204,2,2,2,6,221},{204,2,2,3,0,221},{204,2,2,3,1,221},{204,2,2,3,2,221},{204,2,2,3,3,221},{204,2,2,3,4,221},{204,2,2,3,5,221},{204,2,2,3,6,221},{204,2,2,4,0,221},{204,2,2,4,1,221},{204,2,2,4,2,221},{204,2,2,4,3,221},{204,2,2,4,4,221},{204,2,2,4,5,221},{204,2,2,4,6,221},{204,2,2,5,0,221},{204,2,2,5,1,221},{204,2,2,5,2,221},{204,2,2,5,3,221},{204,2,2,5,4,221},{204,2,2,5,5,221},{204,2,2,5,6,221},{204,2,2,6,0,221},{204,2,2,6,1,221},{204,2,2,6,2,221},{204,2,2,6,3,221},{204,2,2,6,4,221},{204,2,2,6,5,221},{204,2,2,6,6,221},{204,2,3,0,0,221},{204,2,3,0,1,221},{204,2,3,0,2,221},{204,2,3,0,3,221},{204,2,3,0,4,221},{204,2,3,0,5,221},{204,2,3,0,6,221},{204,2,3,1,0,221},{204,2,3,1,1,221},{204,2,3,1,2,221},{204,2,3,1,3,221},{204,2,3,1,4,221},{204,2,3,1,5,221},{204,2,3,1,6,221},{204,2,3,2,0,221},{204,2,3,2,1,221},{204,2,3,2,2,221},{204,2,3,2,3,221},{204,2,3,2,4,221},{204,2,3,2,5,221},{204,2,3,2,6,221},{204,2,3,3,0,221},{204,2,3,3,1,221},{204,2,3,3,2,221},{204,2,3,3,3,221},{204,2,3,3,4,221},{204,2,3,3,5,221},{204,2,3,3,6,221},{204,2,3,4,0,221},{204,2,3,4,1,221},{204,2,3,4,2,221},{204,2,3,4,3,221},{204,2,3,4,4,221},{204,2,3,4,5,221},{204,2,3,4,6,221},{204,2,3,5,0,221},{204,2,3,5,1,221},{204,2,3,5,2,221},{204,2,3,5,3,221},{204,2,3,5,4,221},{204,2,3,5,5,221},{204,2,3,5,6,221},{204,2,3,6,0,221},{204,2,3,6,1,221},{204,2,3,6,2,221},{204,2,3,6,3,221},{204,2,3,6,4,221},{204,2,3,6,5,221},{204,2,3,6,6,221},{204,2,4,0,0,221},{204,2,4,0,1,221},{204,2,4,0,2,221},{204,2,4,0,3,221},{204,2,4,0,4,221},{204,2,4,0,5,221},{204,2,4,0,6,221},{204,2,4,1,0,221},{204,2,4,1,1,221},{204,2,4,1,2,221},{204,2,4,1,3,221},{204,2,4,1,4,221},{204,2,4,1,5,221},{204,2,4,1,6,221},{204,2,4,2,0,221},{204,2,4,2,1,221},{204,2,4,2,2,221},{204,2,4,2,3,221},{204,2,4,2,4,221},{204,2,4,2,5,221},{204,2,4,2,6,221},{204,2,4,3,0,221},{204,2,4,3,1,221},{204,2,4,3,2,221},{204,2,4,3,3,221},{204,2,4,3,4,221},{204,2,4,3,5,221},{204,2,4,3,6,221},{204,2,4,4,0,221},{204,2,4,4,1,221},{204,2,4,4,2,221},{204,2,4,4,3,221},{204,2,4,4,4,221},{204,2,4,4,5,221},{204,2,4,4,6,221},{204,2,4,5,0,221},{204,2,4,5,1,221},{204,2,4,5,2,221},{204,2,4,5,3,221},{204,2,4,5,4,221},{204,2,4,5,5,221},{204,2,4,5,6,221},{204,2,4,6,0,221},{204,2,4,6,1,221},{204,2,4,6,2,221},{204,2,4,6,3,221},{204,2,4,6,4,221},{204,2,4,6,5,221},{204,2,4,6,6,221},{204,2,5,0,0,221},{204,2,5,0,1,221},{204,2,5,0,2,221},{204,2,5,0,3,221},{204,2,5,0,4,221},{204,2,5,0,5,221},{204,2,5,0,6,221},{204,2,5,1,0,221},{204,2,5,1,1,221},{204,2,5,1,2,221},{204,2,5,1,3,221},{204,2,5,1,4,221},{204,2,5,1,5,221},{204,2,5,1,6,221},{204,2,5,2,0,221},{204,2,5,2,1,221},{204,2,5,2,2,221},{204,2,5,2,3,221},{204,2,5,2,4,221},{204,2,5,2,5,221},{204,2,5,2,6,221},{204,2,5,3,0,221},{204,2,5,3,1,221},{204,2,5,3,2,221},{204,2,5,3,3,221},{204,2,5,3,4,221},{204,2,5,3,5,221},{204,2,5,3,6,221},{204,2,5,4,0,221},{204,2,5,4,1,221},{204,2,5,4,2,221},{204,2,5,4,3,221},{204,2,5,4,4,221},{204,2,5,4,5,221},{204,2,5,4,6,221},{204,2,5,5,0,221},{204,2,5,5,1,221},{204,2,5,5,2,221},{204,2,5,5,3,221},{204,2,5,5,4,221},{204,2,5,5,5,221},{204,2,5,5,6,221},{204,2,5,6,0,221},{204,2,5,6,1,221},{204,2,5,6,2,221},{204,2,5,6,3,221},{204,2,5,6,4,221},{204,2,5,6,5,221},{204,2,5,6,6,221},{204,2,6,0,0,221},{204,2,6,0,1,221},{204,2,6,0,2,221},{204,2,6,0,3,221},{204,2,6,0,4,221},{204,2,6,0,5,221},{204,2,6,0,6,221},{204,2,6,1,0,221},{204,2,6,1,1,221},{204,2,6,1,2,221},{204,2,6,1,3,221},{204,2,6,1,4,221},{204,2,6,1,5,221},{204,2,6,1,6,221},{204,2,6,2,0,221},{204,2,6,2,1,221},{204,2,6,2,2,221},{204,2,6,2,3,221},{204,2,6,2,4,221},{204,2,6,2,5,221},{204,2,6,2,6,221},{204,2,6,3,0,221},{204,2,6,3,1,221},{204,2,6,3,2,221},{204,2,6,3,3,221},{204,2,6,3,4,221},{204,2,6,3,5,221},{204,2,6,3,6,221},{204,2,6,4,0,221},{204,2,6,4,1,221},{204,2,6,4,2,221},{204,2,6,4,3,221},{204,2,6,4,4,221},{204,2,6,4,5,221},{204,2,6,4,6,221},{204,2,6,5,0,221},{204,2,6,5,1,221},{204,2,6,5,2,221},{204,2,6,5,3,221},{204,2,6,5,4,221},{204,2,6,5,5,221},{204,2,6,5,6,221},{204,2,6,6,0,221},{204,2,6,6,1,221},{204,2,6,6,2,221},{204,2,6,6,3,221},{204,2,6,6,4,221},{204,2,6,6,5,221},{204,2,6,6,6,221}};
static const uint8_t center_wire[][7]={{204,5,0,0,0,0,221},{204,5,0,0,1,0,221},{204,5,0,0,204,0,221},{204,5,0,0,221,0,221},{204,5,0,0,255,0,221},{204,5,0,0,0,1,221},{204,5,0,0,221,204,221},{204,5,0,0,204,221,221},{204,5,0,0,255,255,221},{204,5,1,0,0,0,221},{204,5,1,0,1,0,221},{204,5,1,0,204,0,221},{204,5,1,0,221,0,221},{204,5,1,0,255,0,221},{204,5,1,0,0,1,221},{204,5,1,0,221,204,221},{204,5,1,0,204,221,221},{204,5,1,0,255,255,221},{204,5,204,0,0,0,221},{204,5,204,0,1,0,221},{204,5,204,0,204,0,221},{204,5,204,0,221,0,221},{204,5,204,0,255,0,221},{204,5,204,0,0,1,221},{204,5,204,0,221,204,221},{204,5,204,0,204,221,221},{204,5,204,0,255,255,221},{204,5,221,0,0,0,221},{204,5,221,0,1,0,221},{204,5,221,0,204,0,221},{204,5,221,0,221,0,221},{204,5,221,0,255,0,221},{204,5,221,0,0,1,221},{204,5,221,0,221,204,221},{204,5,221,0,204,221,221},{204,5,221,0,255,255,221},{204,5,255,0,0,0,221},{204,5,255,0,1,0,221},{204,5,255,0,204,0,221},{204,5,255,0,221,0,221},{204,5,255,0,255,0,221},{204,5,255,0,0,1,221},{204,5,255,0,221,204,221},{204,5,255,0,204,221,221},{204,5,255,0,255,255,221},{204,5,0,1,0,0,221},{204,5,0,1,1,0,221},{204,5,0,1,204,0,221},{204,5,0,1,221,0,221},{204,5,0,1,255,0,221},{204,5,0,1,0,1,221},{204,5,0,1,221,204,221},{204,5,0,1,204,221,221},{204,5,0,1,255,255,221},{204,5,221,204,0,0,221},{204,5,221,204,1,0,221},{204,5,221,204,204,0,221},{204,5,221,204,221,0,221},{204,5,221,204,255,0,221},{204,5,221,204,0,1,221},{204,5,221,204,221,204,221},{204,5,221,204,204,221,221},{204,5,221,204,255,255,221},{204,5,204,221,0,0,221},{204,5,204,221,1,0,221},{204,5,204,221,204,0,221},{204,5,204,221,221,0,221},{204,5,204,221,255,0,221},{204,5,204,221,0,1,221},{204,5,204,221,221,204,221},{204,5,204,221,204,221,221},{204,5,204,221,255,255,221},{204,5,255,255,0,0,221},{204,5,255,255,1,0,221},{204,5,255,255,204,0,221},{204,5,255,255,221,0,221},{204,5,255,255,255,0,221},{204,5,255,255,0,1,221},{204,5,255,255,221,204,221},{204,5,255,255,204,221,221},{204,5,255,255,255,255,221}};

static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line %d\n",__LINE__);}}while(0)
static void feed(const uint8_t*p,unsigned n){memcpy(incoming+rx_head,p,n);rx_head+=n;Vision_Poll();if(rx_tail==rx_head)rx_head=rx_tail=0;}
static void material(unsigned a,unsigned b,unsigned c){uint8_t p[6]={0xcc,2,(uint8_t)a,(uint8_t)b,(uint8_t)c,0xdd};feed(p,6);}
int main(void){
 uint32_t old,bad,rev,loaded[3];S2_Event event;uint8_t center[7]={0xcc,5,0x40,1,0x58,2,0xdd};
 uint8_t embedded[7]={0xcc,5,0xcc,0xdd,0xcc,0xdd,0xdd};
 uint8_t broken[6]={0xcc,2,1,2,3,0};uint8_t unknown[2]={0xcc,0x55};
 uint8_t partial[3]={0xcc,2,1};uint8_t combined[13]={0xcc,2,1,2,3,0xdd,0xcc,5,10,0,20,0,0xdd};
 uint8_t burst[132];unsigned i;
 Vision_Poll();CK(rx_enable_count==1 && tx_count==0);
 now_ms=100;material(1,2,3);feed(center,7);
 CK(g_vis_mat_cycle==1 && g_vis_material_valid && g_vis_mat_left==1 && g_vis_mat_right==2 && g_vis_mat_down==3);
 CK(g_vis_center_cycle==1 && g_vis_center_x==320 && g_vis_center_y==600);
 g_vis_pick_region=0;g_vis_taught_mask=0;CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==6);
 g_vis_taught_mask=1;g_vis_region_deg[0]=193;g_vis_region_ext_mm[0]=48;
 g_vis_mm_per_px=0.4f;g_vis_ref_x=310;g_vis_ref_y=590;
 CK(Vision_TakeTarget(now_ms) && g_src_txn_result==1);
 CK(g_src_pick_deg==193 && g_src_extension_mm==48 && fabsf(g_src_chassis_dx_mm-4)<.001f && fabsf(g_src_chassis_dy_mm-4)<.001f && g_vis_metric_valid);
 rev=g_src_txn_revision;CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==8 && g_src_txn_revision==rev);
 material(0,2,3);CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==9);
 material(1,2,3);now_ms=3101;CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==7);
 now_ms=3100;CK(Vision_TakeTarget(now_ms));
 old=g_vis_mat_cycle;bad=g_vis_rx_bad;feed(broken,6);CK(g_vis_mat_cycle==old && g_vis_rx_bad==bad+1);
 bad=g_vis_rx_bad;material(255,2,3);CK(g_vis_mat_cycle==old && g_vis_rx_bad==bad+1);
 bad=g_vis_rx_bad;feed(unknown,2);CK(g_vis_rx_bad==bad+1);
 feed(embedded,7);CK(g_vis_center_x==0xddcc && g_vis_center_y==0xddcc);
 now_ms=4000;feed(partial,3);bad=g_vis_rx_bad;now_ms=4200;material(1,2,3);
 CK(g_vis_rx_bad==bad+1 && g_vis_mat_cycle==old+1 && vis_len==0);
 feed(combined,13);CK(g_vis_center_x==10 && g_vis_center_y==20 && g_vis_mat_right==2);
 g_uart6_rx_errors++;Vision_Poll();CK(!g_vis_material_valid && !g_vis_center_valid);
 CK(!Vision_TakeTarget(now_ms) && g_vis_txn_result==7);
 material(1,2,3);g_vis_mm_per_px=0;CK(Vision_TakeTarget(now_ms) && !g_vis_metric_valid && g_src_chassis_dx_mm==0 && g_src_chassis_dy_mm==0);
 material(1,2,3);g_vis_region_ext_mm[0]=171;CK(!Vision_TakeTarget(now_ms) && g_src_txn_result==4);
 g_vis_region_ext_mm[0]=48;g_vis_region_deg[0]=NAN;CK(!Vision_TakeTarget(now_ms) && g_src_txn_result==4);
 g_vis_region_deg[0]=193;CK(Vision_TakeTarget(now_ms));
 material(1,2,3);tx_busy=1;g_vis_command_request=2;old=tx_count;Vision_Poll();
 CK(g_vis_command_request==2 && tx_count==old);
 tx_busy=0;Vision_Poll();CK(g_vis_command_request==0 && tx_count==old+1 && sent[0]==0xaa && sent[1]==2 && sent[2]==0xbb && !g_vis_material_valid);
 g_vis_command_request=99;old=tx_count;Vision_Poll();CK(g_vis_command_request==0 && g_vis_command_rejected==1 && tx_count==old);
 now_ms=0xfffffff0U;material(1,2,3);now_ms=0x20U;CK(Vision_TakeTarget(now_ms));
 for(i=0;i<sizeof(burst);i++)burst[i]=0x77;
 old=g_vis_rx_bytes;feed(burst,sizeof(burst));CK(g_vis_rx_bytes-old==128);Vision_Poll();CK(g_vis_rx_bytes-old==132);

 for(i=0;i<sizeof(material_wire)/sizeof(material_wire[0]);i++){
  feed(material_wire[i],6);
  CK(g_vis_mat_left==material_wire[i][2] && g_vis_mat_right==material_wire[i][3] && g_vis_mat_down==material_wire[i][4]);
 }
 for(i=0;i<sizeof(center_wire)/sizeof(center_wire[0]);i++){
  feed(center_wire[i],7);
  CK(g_vis_center_x==((uint32_t)center_wire[i][2]|((uint32_t)center_wire[i][3]<<8)) &&
     g_vis_center_y==((uint32_t)center_wire[i][4]|((uint32_t)center_wire[i][5]<<8)));
 }
 CK(rx_enable_count==1);

 /* Same actual C path as request13: QR order -> region -> target -> evidence. */
 CK(!Vision_SetTask("111+123+111+123",1,10));
 CK(!Vision_SetTask("146+124+614+231",1,10));
 CK(!Vision_SetTask("146+123+123+231",1,10));
 CK(!Vision_SetTask("146+123+614+231",1,0));
 CK(Vision_SetTask("146+123+614+231",1,10));
 CK(g_vis_task_colors[0]==1 && g_vis_task_colors[1]==4 && g_vis_task_colors[2]==6);
 CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==7);
 g_vis_taught_mask=7;g_vis_region_deg[0]=190;g_vis_region_deg[1]=160;g_vis_region_deg[2]=200;
 g_vis_region_ext_mm[0]=48;g_vis_region_ext_mm[1]=60;g_vis_region_ext_mm[2]=80;
 material(1,1,6);CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==12);
 material(4,6,1);CK(Vision_TakeTaskTarget(now_ms));
 CK(g_vis_pick_region==2 && g_src_slot==1 && g_src_pick_deg==200 && g_src_extension_mm==80);
 CK(g_vis_task_item==0 && g_vis_loaded_mask==0 && !g_vis_batch_complete);
 CK(!Vision_ReadLoadedBatch(10,1,loaded));
 rev=g_vis_pending_seq;
 CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==11);
 CK(!Vision_SetTask("146+123+614+231",2,10));
 CK(!Vision_ReportGrasp(11,rev,1,1,now_ms,now_ms));
 CK(!Vision_ReportGrasp(10,rev,0,1,now_ms,now_ms));
 CK(!Vision_ReportGrasp(10,rev,1,1,now_ms-1001U,now_ms));
 CK(!Vision_ReportPlaced(10,rev,1,1,1,1,now_ms,now_ms));
 CK(Vision_ReportGrasp(10,rev,1,1,now_ms,now_ms));
 CK(!Vision_ReportPlaced(10,rev,2,1,1,1,now_ms,now_ms));
 CK(!Vision_ReportPlaced(10,rev,1,1,1,0,now_ms,now_ms));
 CK(Vision_ReportPlaced(10,rev,1,1,1,1,now_ms,now_ms));
 CK(g_vis_task_item==1 && g_vis_loaded_mask==1 && !g_vis_pending_seq);
 CK(!Vision_ReportPlaced(10,rev,1,1,1,1,now_ms,now_ms));
 CK(Vision_SetTask("146+123+614+231",1,10) && g_vis_task_item==1);
 material(4,6,0);CK(Vision_TakeTaskTarget(now_ms) && g_vis_pick_region==0 && g_src_slot==2);
 rev=g_vis_pending_seq;CK(Vision_ReportGrasp(10,rev,1,1,now_ms,now_ms));
 CK(Vision_ReportPlaced(10,rev,2,1,1,1,now_ms,now_ms));
 material(0,6,0);CK(Vision_TakeTaskTarget(now_ms) && g_vis_pick_region==1 && g_src_slot==3);
 rev=g_vis_pending_seq;CK(Vision_ReportGrasp(10,rev,1,1,now_ms,now_ms));
 CK(Vision_ReportPlaced(10,rev,3,1,1,1,now_ms,now_ms));
 CK(g_vis_task_item==3 && g_vis_loaded_mask==7 && g_vis_batch_complete);
 CK(Vision_ReadLoadedBatch(10,1,loaded) && loaded[0]==1 && loaded[1]==4 && loaded[2]==6);
 CK(!Vision_ReadLoadedBatch(11,1,loaded) && !Vision_ReadLoadedBatch(10,2,loaded));
 CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==10);
 CK(Vision_SetTask("146+123+614+231",2,10));
 CK(g_vis_task_colors[0]==6 && g_vis_task_item==0 && !g_vis_batch_complete);
 CK(Vision_SetTask("146+123+614+231",1,11) && g_vis_task_run_id==11 && !g_vis_material_valid);
 material(4,6,1);CK(Vision_TakeTaskTarget(now_ms));rev=g_vis_pending_seq;
 memset(&event,0,sizeof(event));event.run_id=11;event.slot=1;event.observed_ms=now_ms;
 event.evidence_flags=S2_HELD;CK(!S2_VisionGrasp(0,11,rev,&event,now_ms));
 event.evidence_flags=S2_SOURCE_CLEAR|S2_HELD;CK(S2_VisionGrasp(0,11,rev,&event,now_ms));
 event.evidence_flags=S2_RELEASED|S2_PRESENT;CK(!S2_VisionPlaced(0,11,rev,&event,now_ms));
 event.evidence_flags|=S2_STABLE;CK(S2_VisionPlaced(0,11,rev,&event,now_ms));
 CK(g_vis_task_item==1 && g_vis_loaded_mask==1 && !g_vis_pending_seq);
 material(4,6,0);CK(Vision_TakeTaskTarget(now_ms));rev=g_vis_pending_seq;
 CK(!Vision_RejectUnstarted(12,rev) && g_vis_pending_seq==rev);
 CK(Vision_RejectUnstarted(11,rev) && !g_vis_pending_seq && g_vis_mat_consumed==g_vis_mat_cycle);
 CK(!Vision_TakeTaskTarget(now_ms) && g_vis_txn_result==8);
 printf("stage2 parser/txn checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
