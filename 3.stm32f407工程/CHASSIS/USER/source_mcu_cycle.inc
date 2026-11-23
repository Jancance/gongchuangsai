/* Single MCU owner: real preparation -> visual alignment -> two beeps ->3pieces.
 * Pi supplies observations only. Existing motion, freshness and STOP semantics apply.
 * States0idle,1prepare,2observe-align,3move,4finish,5hold/beep,6observe-pick,
 * 7committed-piece,8done,9fault. No host decisions between the single start and done. */
#include "Buzzer.h"
#include "saoma.h"
#include "screen.h"
#include <stdio.h>
#include <stdlib.h>
extern volatile uint32_t g_source_align_direction,g_source_align_command_seq,g_source_align_chain_active;
extern volatile float g_source_align_distance_mm;
volatile uint32_t g_source_cycle_request,g_source_cycle_state,g_source_cycle_fault;
volatile uint32_t g_source_cycle_run,g_source_cycle_pose,g_source_cycle_item,g_source_cycle_prep;
volatile uint32_t g_source_cycle_colors[3]={0U,0U,0U};
volatile float g_source_cycle_yaw[3]={195.0f,165.0f,180.0f};
volatile float g_source_cycle_extension[3]={164.0708363f,167.9699163f,0.0f};
volatile uint32_t g_source_cycle_rx,g_source_cycle_rx_bad,g_source_cycle_tx,g_source_cycle_rejected;
volatile uint32_t g_source_cycle_body_steps,g_source_cycle_beeps,g_source_cycle_beep_started,g_source_cycle_beep_done;
volatile uint32_t g_source_cycle_started,g_source_cycle_done,g_source_cycle_piece_ms[3];
volatile uint32_t g_source_cycle_scheduled,g_source_cycle_due_tick,g_source_cycle_planned_region;
volatile uint32_t g_source_cycle_start_delay_ms[3],g_source_cycle_start_lag_ms;
volatile uint32_t g_source_cycle_grip_command_ms[3];
volatile int32_t g_source_cycle_right_milli_mm,g_source_cycle_forward_milli_mm;
volatile uint32_t g_source_cycle_error_milli_mm;
static uint32_t sc_pending,sc_step_sequence,sc_frame,sc_rx_tick,sc_frame_age;
static uint32_t sc_valid,sc_target_ready,sc_colors[3],sc_used,sc_active,sc_last_byte;
static uint32_t sc_continuous,sc_target_color;
static uint32_t SC_InitialObserveAllowed(void)
{
 return g_source_cycle_state==1U && g_source_cycle_prep==4U &&
  g_it_axes==(SPF_DM|SPF_EXT) && g_it_dm_deg==180.0f && g_it_extension_mm==65.0f &&
  it_at_top() && fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)<=2.0f;
}
static uint32_t sc_beep_started,sc_last_tx;
volatile uint32_t g_source_cycle_pick_beeps,g_source_cycle_pick_beep_pending;
volatile uint32_t g_source_cycle_pick_beep_ms[3];
static void SC_PickBeep(uint32_t now)
{
 if(!g_source_cycle_pick_beep_pending || Buzzer_IsBusy())return;
 if(g_source_cycle_pick_beeps<3U)g_source_cycle_pick_beep_ms[g_source_cycle_pick_beeps]=now;
 g_source_cycle_pick_beeps++;g_source_cycle_pick_beep_pending--;
 Buzzer_StartTimed(1U,80U,80U);
}
static char sc_input[192];
/* TaskA owns scanner queue/parser and screen. CAN owns task validation and
 * publishes locked_run last; the mailbox stays immutable until consumed. */
extern volatile uint32_t g_scan_display_result,g_scan_display_count;
volatile uint32_t g_source_scan_owner_run,g_source_scan_locked_run;
volatile uint32_t g_source_scan_pending,g_source_scan_accept,g_source_scan_result;
volatile uint32_t g_source_scan_frame_start_ms,g_source_scan_frame_end_ms;
volatile uint32_t g_source_scan_rejected,g_source_scan_displayed_run;
char g_source_scan_code[16];
static uint32_t sc_scan_seen;
static volatile uint32_t sc_bound_batch;
static uint32_t sc_retract_pending,sc_bound_slots[3];
static uint32_t sc_transport;
void SourceCycle_ScanService(void)
{
#if SOURCE_CYCLE_TEST_215
 /* Independent debug mode does not own the scanner, its mailbox or display. */
 return;
#else
 uint32_t run=g_source_cycle_run,state=g_source_cycle_state;
 if(sc_bound_batch)return;
 if(state==1U && g_source_scan_owner_run!=run){
  SAOMA_EnableReceive();
  taskENTER_CRITICAL();
  sc_scan_seen=g_saoma_accept_count;g_source_scan_pending=0U;
  g_source_scan_result=0U;g_source_scan_code[0]='\0';
  g_source_scan_owner_run=run;
  taskEXIT_CRITICAL();
 }
 if(g_source_scan_locked_run==run && run){
  if(g_source_scan_displayed_run!=run){
   g_scan_display_result=Screen_ShowTaskCode(g_source_scan_code);
   if(g_scan_display_result==SCREEN_OK)g_scan_display_count++;
   g_source_scan_displayed_run=run;
  }
  return;
 }
 if(state!=1U || !run || g_source_scan_owner_run!=run || g_source_scan_pending)return;
 taskENTER_CRITICAL();
 if(g_source_cycle_state==1U && g_source_cycle_run==run &&
    g_source_scan_owner_run==run && g_source_scan_locked_run!=run &&
    !g_source_scan_pending && SAOMA_DataReady && g_saoma_accept_count!=sc_scan_seen){
  memcpy(g_source_scan_code,SAOMA_Data,16U);
  g_source_scan_accept=g_saoma_accept_count;g_source_scan_result=g_saoma_last_result;
  g_source_scan_frame_start_ms=g_saoma_frame_start_ms;
  g_source_scan_frame_end_ms=g_saoma_frame_end_ms;
  sc_scan_seen=g_saoma_accept_count;SAOMA_DataReady=0U;
  g_source_scan_pending=1U;
 }
 taskEXIT_CRITICAL();
#endif
}
static uint32_t SC_ScanTask(void)
{
#if SOURCE_CYCLE_TEST_215
 /* User test order only: yellow, red, black; not a four-part scanned task. */
 g_source_cycle_colors[0]=2U;g_source_cycle_colors[1]=1U;g_source_cycle_colors[2]=5U;
 return 1U;
#else
 uint32_t i,accepted;char code[16];
 if(g_source_scan_locked_run==g_source_cycle_run)return 1U;
 if(g_source_scan_owner_run!=g_source_cycle_run || !g_source_scan_pending)return 0U;
 taskENTER_CRITICAL();
 memcpy(code,g_source_scan_code,16U);
 accepted=g_source_scan_result;
 taskEXIT_CRITICAL();
 /* Task mailbox has no actuator access. Full rule semantics stay centralized. */
 if(accepted==1U && Vision_SetTask(code,1U,g_source_cycle_run)){
  for(i=0U;i<3U;i++)g_source_cycle_colors[i]=g_vis_task_colors[i];
  taskENTER_CRITICAL();
  g_source_scan_locked_run=g_source_cycle_run;
  g_source_scan_pending=0U;
  taskEXIT_CRITICAL();
  return 1U;
 }
 g_source_scan_rejected++;
 taskENTER_CRITICAL();g_source_scan_pending=0U;taskEXIT_CRITICAL();
 return 0U;
#endif
}
void SourceCycle_KeyEvent(void)
{
 if(g_source_cycle_state==0U)g_source_cycle_request=1U;
 else if(g_source_cycle_state<8U)g_source_cycle_request=9U;
}
static uint16_t SC_Crc(const char *p,uint32_t n)
{
 uint16_t c=0xffffU;uint32_t i,j;
 for(i=0;i<n;i++){c^=(uint16_t)(uint8_t)p[i]<<8;
  for(j=0;j<8;j++)c=(uint16_t)((c&0x8000U)?((uint32_t)c<<1)^0x1021U:(uint32_t)c<<1);}
 return c;
}
static void SC_Pose(void)
{if(++g_source_cycle_pose==0U)g_source_cycle_pose=1U;sc_valid=sc_frame=0U;}
static void SC_Fail(uint32_t why)
{
 if(!g_source_cycle_fault)g_source_cycle_fault=why;
 g_source_cycle_scheduled=0U;
 g_source_cycle_state=9U;
 if(Low_Busy())g_low_request=9U;
 else if(g_it_state==1U)g_it_request=9U;
 if(SourceBodyStep_Busy())g_source_body_step_request=9U;
}
static uint32_t SC_Fresh(uint32_t now)
{return sc_valid && (uint32_t)(now-sc_rx_tick)<=200U && sc_frame_age<=200U &&
 (uint32_t)(now-sc_rx_tick)+sc_frame_age<=200U;}
static void SC_Line(uint32_t now)
{
 unsigned long run,pose,seq,valid,norm,ready,left,right,down,age,color=0,delay0=0,delay1=0,delay2=0;long dx,dy;
 int n=0,count;uint32_t crc,continuous;
 if(sc_used && sc_input[sc_used-1U]=='\r')sc_used--;
 sc_input[sc_used]='\0';
 continuous=!strncmp(sc_input,"OC2,",4U);
 if(continuous)count=sscanf(sc_input,"OC2,%lu,%lu,%lu,%lu,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%n",
  &run,&pose,&seq,&valid,&dx,&dy,&norm,&ready,&left,&right,&down,&age,&color,&delay0,&delay1,&delay2,&n);
 else count=sscanf(sc_input,"OC1,%lu,%lu,%lu,%lu,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%lu,%n",
  &run,&pose,&seq,&valid,&dx,&dy,&norm,&ready,&left,&right,&down,&age,&n);
 if(count!=(continuous?16:12) || n<=0 || (uint32_t)n+4U!=sc_used){g_source_cycle_rx_bad++;return;}
 {uint32_t i;crc=0U;for(i=0;i<4U;i++){
  char h=sc_input[n+(int)i];uint32_t v;
  if(h>='0' && h<='9')v=(uint32_t)(h-'0');
  else if(h>='A' && h<='F')v=(uint32_t)(h-'A'+10);
  else {g_source_cycle_rx_bad++;return;}crc=(crc<<4)|v;
 }}
 if(crc!=SC_Crc(sc_input,(uint32_t)n) || valid>1UL || ready>1UL || color>6UL ||
    delay0>0x7fffffffUL || delay1>0x7fffffffUL || delay2>0x7fffffffUL ||
    left>6UL || right>6UL || down>6UL || seq==0UL || seq>0xffffffffUL ||
    norm>0xffffffffUL || dx<-2147483647L || dx>2147483647L || dy<-2147483647L || dy>2147483647L){g_source_cycle_rx_bad++;return;}
 if(run!=g_source_cycle_run || pose!=g_source_cycle_pose ||
   (g_source_cycle_state!=2U && g_source_cycle_state!=6U) ||
   (sc_frame && (int32_t)((uint32_t)seq-sc_frame)<=0) || age>200UL)return;
 sc_frame=(uint32_t)seq;sc_rx_tick=now;sc_frame_age=(uint32_t)age;
 sc_continuous=continuous;sc_valid=(uint32_t)valid;sc_target_ready=sc_continuous?(uint32_t)ready:0U;
 sc_target_color=(uint32_t)color;
 g_source_cycle_start_delay_ms[0]=(uint32_t)delay0;g_source_cycle_start_delay_ms[1]=(uint32_t)delay1;g_source_cycle_start_delay_ms[2]=(uint32_t)delay2;
 if(g_source_cycle_state==2U && sc_valid){
  g_source_cycle_right_milli_mm=(int32_t)dx;g_source_cycle_forward_milli_mm=(int32_t)dy;
  g_source_cycle_error_milli_mm=(uint32_t)norm;
 }
 sc_colors[0]=(uint32_t)left;sc_colors[1]=(uint32_t)right;sc_colors[2]=(uint32_t)down;
 g_source_cycle_rx++;
 if(g_source_cycle_state==6U && sc_valid){
  g_vis_mat_left=sc_colors[0];g_vis_mat_right=sc_colors[1];g_vis_mat_down=sc_colors[2];
  g_vis_mat_cycle++;g_vis_mat_tick=now;g_vis_material_valid=1U;
 }
}
static uint32_t SourceCycle_SharedByte(uint8_t b,uint32_t now,uint32_t binary_active)
{
 if(sc_active && (uint32_t)(now-sc_last_byte)>=200U)sc_active=sc_used=0U;
 if(!sc_active){if(binary_active || b!='O')return 0U;sc_active=1U;sc_used=0U;}
 sc_last_byte=now;
 if(b=='\n'){SC_Line(now);sc_active=sc_used=0U;return 1U;}
 if(b<32U && b!='\r'){sc_active=sc_used=0U;g_source_cycle_rx_bad++;return 1U;}
 if(sc_used>=sizeof(sc_input)-1U){sc_active=sc_used=0U;g_source_cycle_rx_bad++;return 1U;}
 sc_input[sc_used++]=(char)b;return 1U;
}
static void SourceCycle_Telemetry(uint32_t now)
{
 char wire[192];int n;
 if((uint32_t)(now-sc_last_tx)<100U)return;
 n=snprintf(wire,sizeof(wire),"SC2,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
  (unsigned long)g_source_cycle_run,(unsigned long)g_source_cycle_state,(unsigned long)g_source_cycle_pose,
  (unsigned long)g_source_cycle_prep,(unsigned long)g_source_cycle_item,
  (unsigned long)g_source_cycle_colors[0],(unsigned long)g_source_cycle_colors[1],(unsigned long)g_source_cycle_colors[2],
  (unsigned long)g_low_accepted_sequence,(unsigned long)g_low_done_sequence,(unsigned long)g_low_phase,
  (unsigned long)g_source_cycle_scheduled,(unsigned long)((g_source_cycle_scheduled && (int32_t)(g_source_cycle_due_tick-now)>0)?g_source_cycle_due_tick-now:0U));
 if(n<=0 || (uint32_t)n>=sizeof(wire)-7U)return;
 snprintf(wire+n,sizeof(wire)-(uint32_t)n,"%04X\r\n",SC_Crc(wire,(uint32_t)n));
#if APP_ZONE2_MISSION_ENABLE
 if(sc_bound_batch){
  if(RegionPlacement_Send(wire,(uint16_t)((uint32_t)n+6U),g_source_cycle_run,
                         g_source_cycle_pose,g_source_cycle_item,now)){
   sc_last_tx=now;g_source_cycle_tx++;
  }
  return;
 }
#endif
 if(USART6_VisionSend((const uint8_t*)wire,(uint16_t)((uint32_t)n+6U))){sc_last_tx=now;g_source_cycle_tx++;}
}
static void SourceCycle_Service(uint32_t now)
{
 uint32_t request=g_source_cycle_request;g_source_cycle_request=0U;
 if(request==9U && g_source_cycle_state && g_source_cycle_state!=8U &&
    g_source_cycle_state!=9U){SC_Fail(2U);return;}
 if(request==1U){
  if(g_source_cycle_state || g_it_state || Low_Busy() || SourceBodyStep_Busy() ||
    g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || !g_safety_ready || g_can_test_result!=1U){g_source_cycle_rejected++;return;}
  sc_bound_batch=0U;
  g_source_cycle_run++;if(!g_source_cycle_run)g_source_cycle_run=1U;
  g_source_cycle_colors[0]=g_source_cycle_colors[1]=g_source_cycle_colors[2]=0U;
  g_source_cycle_started=now;g_source_cycle_state=1U;SC_Pose();
 }
 if(!g_source_cycle_state || g_source_cycle_state==8U || g_source_cycle_state==9U)return;
 if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || g_it_state==9U ||
    g_low_state==9U || g_startup_axes_cause || g_lift_ruler_cause || g_dm_jog_first_fault ||
    g_source_body_step_reason || g_source_body_step_state==9U){SC_Fail(g_it_cause?g_it_cause:12U);return;}
 if(sc_beep_started && !g_source_cycle_beep_done && !Buzzer_IsBusy())g_source_cycle_beep_done=now;
 SC_PickBeep(now);
 if(g_source_cycle_state==1U){
  static const uint32_t prep[5]={1U,2U,7U,6U,3U};
  if(!sc_bound_batch && !SC_ScanTask())return;
  if(sc_retract_pending){
   if(g_it_rejected){SC_Fail(g_it_rejected);return;}
   if(g_it_request || g_it_state!=2U || g_it_action!=3U)return;
   sc_retract_pending=0U;
  }
  if(sc_pending){
   if(g_it_rejected){SC_Fail(g_it_rejected);return;}
   if(g_it_state!=2U || g_it_request || g_it_action!=prep[g_source_cycle_prep])return;
   sc_pending=0U;g_source_cycle_prep++;
  }
  if(g_source_cycle_prep>=5U){g_source_cycle_state=2U;SC_Pose();return;}
  if(g_it_state==1U || g_it_request)return;
  if(sc_bound_batch && g_source_cycle_prep==0U && it_at_top()){g_source_cycle_prep++;return;}
  if(sc_bound_batch && g_source_cycle_prep==1U && StartupAxes_Holding() &&
     g_extension_home_reference_valid && g_extension_hold_confirmed){g_source_cycle_prep++;return;}
  g_it_carrier_auto_deg=-1.0f;
  if(sc_bound_batch && g_source_cycle_prep==4U &&
     fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>2.0f){
   g_it_axes=SPF_EXT;g_it_extension_mm=0.0f;sc_retract_pending=1U;g_it_request=3U;return;
  }
  if(g_source_cycle_prep==4U){g_it_axes=SPF_DM|SPF_EXT;g_it_dm_deg=180.0f;g_it_extension_mm=65.0f;}
  g_it_request=prep[g_source_cycle_prep];sc_pending=1U;return;
 }
 if(g_source_cycle_state==2U){
  int32_t right=g_source_cycle_right_milli_mm,forward=g_source_cycle_forward_milli_mm;
  uint32_t axis,distance;int32_t amount;
  if(!SC_Fresh(now))return;
  if(g_source_cycle_error_milli_mm<=5000U){
   SC_Pose();
   if(g_source_align_chain_active){g_source_body_step_request=7U;g_source_cycle_state=4U;}
   else g_source_cycle_state=5U;
   return;
  }
  axis=labs((long)right)>=labs((long)forward)?0U:1U;amount=axis?forward:right;
  distance=labs((long)amount)>=20000L?20U:(uint32_t)((labs((long)amount)+500L)/1000L);if(distance<1U)distance=1U;
  g_source_align_direction=axis?(amount>0?0U:3U):(amount>0?2U:1U);
  g_source_align_distance_mm=(float)distance;sc_step_sequence=g_source_align_command_seq;
  SC_Pose();g_source_body_step_request=2U;g_source_cycle_state=3U;g_source_cycle_body_steps++;return;
 }
 if(g_source_cycle_state==3U){
  if(g_source_align_command_seq==sc_step_sequence || g_source_body_step_state!=3U || g_chassis_auto_active)return;
  /* Existing SourceBodyStep marks state3 only after four-wheel STOP15. */
  g_source_cycle_state=2U;SC_Pose();return;
 }
 if(g_source_cycle_state==4U){
  if(g_source_body_step_request || g_source_align_chain_active)return;
  g_source_cycle_state=5U;return;
 }
 if(g_source_cycle_state==5U){
  if(!g_low_wheel_hold_ready){if(!g_low_wheel_hold_request && !g_low_wheel_hold_stage)g_low_wheel_hold_request=1U;
   if(g_low_wheel_hold_stage==99U)SC_Fail(12U);return;}
  if(!sc_beep_started){Buzzer_StartTimed(2U,80U,80U);sc_beep_started=1U;
   g_source_cycle_beeps=2U;g_source_cycle_beep_started=now;}
  /* First beep releases pickup; remaining beeps continue independently. */
  g_source_cycle_state=6U;SC_Pose();return;
 }
 if(g_source_cycle_state==6U){
  uint32_t i,region=0U,color;
  if(!SC_Fresh(now) || !sc_continuous || !sc_target_ready || g_it_state!=2U || g_low_request || Low_Busy())return;
  color=g_source_cycle_colors[g_source_cycle_item];
  if(sc_target_color!=color)return;
  for(i=1U;i<3U;i++)if(g_source_cycle_start_delay_ms[i]<g_source_cycle_start_delay_ms[region])region=i;
  for(i=0U;i<3U;i++)if(sc_colors[i]){
   uint32_t j,match=0U;
   for(j=g_source_cycle_item;j<3U;j++)if(g_source_cycle_colors[j]==sc_colors[i])match++;
   if(match!=1U)return;
   for(j=0U;j<i;j++)if(sc_colors[j]==sc_colors[i])return;
  }
  g_low_sequence=g_low_accepted_sequence+1U;if(!g_low_sequence)g_low_sequence=1U;
  g_low_slot=g_source_cycle_item+1U;g_low_color=color;g_low_region=region;
  if(sc_bound_batch){
   for(i=0U;i<3U && sc_bound_slots[i]!=color;i++){}
   if(i==3U){SC_Fail(12U);return;}
   g_low_slot=i+1U;
  }
  g_low_target_yaw=g_source_cycle_yaw[region];g_low_target_extension_mm=g_source_cycle_extension[region];
  g_low_pose_version=g_low_scene_id=1U;g_low_confirm_pose_sequence=g_low_sequence;
  g_low_confirm_frame_cycle=sc_frame;sc_pending=g_low_sequence;
  g_source_cycle_planned_region=region;g_source_cycle_due_tick=now+g_source_cycle_start_delay_ms[region];
  g_source_cycle_scheduled=1U;
  /* MCU owns the future start. No host timer or later image can issue it. */
  g_source_cycle_state=7U;SC_Pose();return;
 }
 if(g_source_cycle_state==7U){
  if(g_source_cycle_scheduled){
   if((int32_t)(now-g_source_cycle_due_tick)<0)return;
   g_source_cycle_scheduled=0U;
   if(!Low_TryStart(now)){g_source_cycle_rejected++;g_source_cycle_state=6U;SC_Pose();return;}
   if(g_low_state==1U){g_source_cycle_pick_beep_pending++;SC_PickBeep(now);}
   g_source_cycle_start_lag_ms=(uint32_t)xTaskGetTickCount()-g_source_cycle_due_tick;return;
  }
  if(g_low_state!=2U || g_low_done_sequence!=sc_pending)return;
#if APP_ZONE2_MISSION_ENABLE
  if(sc_bound_batch){
   uint8_t item=(uint8_t)g_source_cycle_item,color=(uint8_t)g_source_cycle_colors[item];
   if(!AppZone2_StationConfirm(g_source_cycle_run,item,APP_ZONE2_SOURCE_GRIP,color,0U) ||
      !AppZone2_StationConfirm(g_source_cycle_run,item,APP_ZONE2_SOURCE_SLOT,color,0U)){
    SC_Fail(12U);return;
   }
  }
#endif
  g_source_cycle_piece_ms[g_source_cycle_item]=g_low_elapsed_ms;
  g_source_cycle_grip_command_ms[g_source_cycle_item]=g_low_grip_command_ms;g_source_cycle_item++;
  if(g_source_cycle_item==3U){
   if(sc_bound_batch){sc_transport=0U;g_source_cycle_state=10U;SC_Pose();return;}
   g_source_cycle_done=now;g_source_cycle_state=8U;SC_Pose();return;
  }
  g_source_cycle_state=6U;SC_Pose();return;
 }
 if(g_source_cycle_state==10U){
  if(g_it_request)return;
  if(g_it_rejected){SC_Fail(g_it_rejected);return;}
  if(sc_transport && g_it_state!=2U)return;
  switch(sc_transport){
  case 0U:
   if(!it_at_top())g_it_request=4U;
   break;
  case 1U:
   if(!it_at_top()){SC_Fail(12U);return;}
   g_it_axes=SPF_EXT;g_it_extension_mm=0.0f;g_it_request=3U;
   break;
  case 2U:
   g_it_axes=SPF_DM;g_it_dm_deg=0.0f;g_it_request=3U;
   break;
  default:
   g_source_cycle_done=now;g_source_cycle_state=8U;SC_Pose();return;
  }
  sc_transport++;
 }
}
