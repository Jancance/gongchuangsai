/* Third-region execution: single initial alignment, then wheel hold and three
 * serial pick/place paths. Unknown calibration remains pending, never guessed.
 * state10/11 are manual bench loading/wait-for-transfer, not competition mode.
 * All failures use existing IT/Body STOP; no new fault thresholds are added. */
volatile uint32_t g_stage3_request,g_stage3_state,g_stage3_fault;
volatile uint32_t g_stage3_run,g_stage3_pose,g_stage3_item,g_stage3_phase;
volatile uint32_t g_stage3_align_done,g_stage3_config_ready,g_stage3_rejected;
volatile uint32_t g_stage3_colors[3]={2U,1U,5U},g_stage3_positions[3]={1U,2U,3U};
/* Explicit bench inventory; normal mission binds committed real inventory. */
volatile uint32_t g_stage3_slot_colors[3]={2U,1U,5U},g_stage3_task_run,g_stage3_task_batch;
volatile uint32_t g_stage3_teach_sequence,g_stage3_teach_saved_sequence;
volatile float g_stage3_carrier_yaw=36.5f,g_stage3_pick_depth=-1.0f;
/* Indexed by physical ring number, never by image left/right. */
volatile float g_stage3_target_yaw[3]={-1.0f,180.0f,-1.0f};
volatile float g_stage3_target_x[3]={-1.0f,0.0f,-1.0f};
volatile float g_stage3_target_depth[3]={-1.0f,-1.0f,-1.0f};
volatile uint32_t g_stage3_test_first_item,g_stage3_test_count=3U;
volatile uint32_t g_stage3_teach_request,g_stage3_teach_result;
/* result:0 idle,1 saved,2 pending real DM feedback,3 not-ready/cancelled. */
volatile uint32_t g_stage3_teach_missing,g_stage3_teach_dm_source,g_stage3_teach_z_source;
volatile float g_stage3_teach_measured_x=-1.0f;
volatile float g_stage3_actual_yaw,g_stage3_actual_x,g_stage3_actual_depth,g_stage3_actual_z_mm;
volatile uint32_t g_stage3_rx,g_stage3_rx_bad,g_stage3_tx,g_stage3_body_steps;
volatile int32_t g_stage3_right_um,g_stage3_forward_um,g_stage3_yaw_mdeg;
volatile uint32_t g_stage3_error_norm_um,g_stage3_motion_unusable;
volatile int32_t g_stage3_error_yaw_mdeg;
extern volatile float g_source_align_turn_deg;
static uint32_t s3_pending,s3_wait_tick,s3_loader_slot,s3_loader_dwell,s3_step_seq;
static uint32_t s3_valid,s3_seq,s3_rx_tick,s3_age,s3_active,s3_used,s3_last_byte,s3_tx_tick;
static uint32_t s3_first,s3_count,s3_colors[3],s3_positions[3],s3_slots[3];
static float s3_pick,s3_carrier,s3_yaw[3],s3_x[3],s3_depth[3];
static char s3_input[192];
extern volatile uint32_t g_lift_diag_request,g_lift_diag_cycle,g_lift_diag_done,g_lift_diag_result[6];
extern volatile uint32_t g_lift_diag_raw[6][8];
static uint32_t s3_teach_pending,s3_teach_dm_cycle,s3_teach_dm_tick,s3_teach_holding;
static float s3_teach_z,s3_teach_x;

uint32_t Stage3_Owns(void)
{return g_region_dispatch_area==3U && g_stage3_state!=0U;}
uint32_t Stage3_FineTurnAllowed(void)
{return g_region_dispatch_area==3U && g_stage3_state==3U && !g_stage3_align_done;}
static void S3_Pose(void)
{if(++g_stage3_pose==0U)g_stage3_pose=1U;s3_valid=s3_seq=0U;}
static void S3_Fail(uint32_t reason)
{
 if(!g_stage3_fault)g_stage3_fault=reason;
 g_stage3_state=9U;s3_valid=0U;
 /* Replace even unconsumed owner requests before the backend service runs. */
 if(SourceBodyStep_Busy() || g_source_body_step_request)g_source_body_step_request=9U;
 if(g_it_state==1U || g_it_request)g_it_request=9U;
}
void Stage3_KeyEvent(void)
{
 if(g_stage3_state==0U)g_stage3_request=10U;
 else if(g_stage3_state==11U)g_stage3_request=1U;
 else if(g_stage3_state==7U)g_stage3_request=2U;
 else if(g_stage3_state!=6U && g_stage3_state!=9U)g_stage3_request=9U;
}
void Stage3_Arrive(void){g_stage3_request=1U;}
uint32_t Stage3_BindMission(const Stage3_Task *task,const uint32_t slots[3])
{
 uint32_t i,j,found;
 if(!task || !slots || !task->run_id || task->batch_id<1U || task->batch_id>2U ||
    (g_stage3_state!=0U && g_stage3_state!=6U && g_stage3_state!=7U && g_stage3_state!=11U) ||
    g_it_state==1U || g_it_request || g_stage3_request || s3_pending || s3_teach_pending)return 0U;
 for(i=0U;i<3U;i++){
  if(task->colors[i]<1U || task->colors[i]>6U || task->positions[i]<1U || task->positions[i]>3U || slots[i]<1U || slots[i]>6U)return 0U;
  for(j=0U;j<i;j++)if(task->colors[i]==task->colors[j] || task->positions[i]==task->positions[j] || slots[i]==slots[j])return 0U;
  found=0U;for(j=0U;j<3U;j++)if(slots[j]==task->colors[i])found++;
  if(found!=1U)return 0U;
 }
 for(i=0U;i<3U;i++){g_stage3_colors[i]=task->colors[i];g_stage3_positions[i]=task->positions[i];g_stage3_slot_colors[i]=slots[i];}
 g_stage3_task_run=task->run_id;g_stage3_task_batch=task->batch_id;return 1U;
}
static uint32_t S3_Platform(void)
{return g_safety_ready && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault &&
 !g_can_ctrl_fault_latched && !Low_Busy() && !SourceBodyStep_Busy() && g_it_state!=1U && g_it_state!=9U;}
static void S3_ReadActual(void)
{
 g_stage3_actual_yaw=g_dm_can2_logical_position_deg;
 g_stage3_actual_x=(g_extension_home_reference_deg-g_startup_extension_current_deg)*
  (3200.0f/360.0f)/VEHICLE_EXTENSION_PULSES_PER_MM;
 if(g_lift_ruler_controller.top_scale){
  g_stage3_actual_depth=(g_vehicle_lift_current_deg-
   (float)g_lift_ruler_controller.top_position_raw/(float)g_lift_ruler_controller.top_scale)*(3200.0f/360.0f);
  g_stage3_actual_z_mm=g_stage3_actual_depth/SOURCE_LIFT_PULSES_PER_MM;
 }
}
static uint32_t S3_Config(void)
{
 uint32_t i,j,found,first=g_stage3_test_first_item,count=g_stage3_test_count;
 if(first>=3U || count<1U || first+count>3U ||
    !(g_stage3_pick_depth>=3200.0f && g_stage3_pick_depth<=(float)SPF_STAGE3_MAX_LIFT_PULSES) ||
    !(g_stage3_carrier_yaw>=0.0f && g_stage3_carrier_yaw<=270.0f) ||
    g_stage3_target_yaw[1]!=180.0f || g_stage3_target_x[1]!=0.0f)return 0U;
 for(i=0U;i<3U;i++){
  if(g_stage3_colors[i]<1U || g_stage3_colors[i]>6U || g_stage3_positions[i]<1U || g_stage3_positions[i]>3U)return 0U;
  for(j=0U;j<i;j++)if(g_stage3_colors[j]==g_stage3_colors[i] || g_stage3_positions[j]==g_stage3_positions[i])return 0U;
  found=0U;for(j=0U;j<3U;j++)if(g_stage3_slot_colors[j]==g_stage3_colors[i])found++;
  if(found!=1U)return 0U;
 }
 for(i=first;i<first+count;i++){
  j=g_stage3_positions[i]-1U;
  if(!(g_stage3_target_yaw[j]>=0.0f && g_stage3_target_yaw[j]<=270.0f) ||
     !(g_stage3_target_x[j]>=0.0f && g_stage3_target_x[j]<=170.0f) ||
     !(g_stage3_target_depth[j]>=3200.0f && g_stage3_target_depth[j]<=(float)SPF_STAGE3_MAX_LIFT_PULSES))return 0U;
 }
 return 1U;
}
static uint32_t S3_TeachReady(void)
{
 return (g_stage3_state==0U || g_stage3_state==7U || g_stage3_state==11U) &&
  SingleTeach_ReferencesReady() && g_it_state==2U && !g_it_request &&
  !g_it_cause && !g_it_rejected && !g_stage3_request && !SourceBodyStep_Busy() && !g_source_body_step_request;
}
static void S3_Teach(void)
{
 uint32_t n=g_stage3_teach_request,needs_dm,index;g_stage3_teach_request=0U;
 if(s3_teach_pending && !S3_TeachReady()){
  s3_teach_pending=0U;g_stage3_teach_result=3U;g_stage3_teach_missing=1U;
 }
 if(n && !s3_teach_pending){
  g_stage3_teach_result=3U;g_stage3_teach_missing=0U;
  g_stage3_teach_z_source=g_stage3_teach_dm_source=0U;
  if(n>7U || !S3_TeachReady()){g_stage3_teach_missing=1U;return;}
  if(n!=5U && n<6U && (!g_vehicle_lift_feedback_valid || !g_lift_ruler_controller.top_scale)){
   g_stage3_teach_missing=2U;return;
  }
  if((n==2U || n==4U || n==6U || n==7U) && !(g_stage3_teach_measured_x>=0.0f && g_stage3_teach_measured_x<=170.0f)){
   g_stage3_teach_missing=4U;return;
  }
  /* Existing IT completion forwards actual VSP/LRC samples. Never use target Z. */
  s3_teach_z=g_stage3_actual_depth;s3_teach_x=g_stage3_teach_measured_x;
  s3_teach_pending=n;g_stage3_teach_result=2U;
  if(++g_stage3_teach_sequence==0U)g_stage3_teach_sequence=1U;
  if(n==2U || n==4U || n==6U || n==7U)g_stage3_teach_measured_x=-1.0f;
  if(n==2U || n==4U || n==5U || n==6U || n==7U){
   s3_teach_dm_cycle=g_dm_can2_read_cycle;s3_teach_dm_tick=g_dm_can2_feedback_tick;
   s3_teach_holding=g_dm_posvel_holding?1U:0U;
   /* Holding already owns and polls CAN2. Never run generic read beside it. */
   if(!s3_teach_holding)g_dm_can2_read_request=2U;
  }
 }
 if(!s3_teach_pending)return;
 n=s3_teach_pending;needs_dm=(n==2U || n==4U || n==5U || n==6U || n==7U);
 if(needs_dm){
  g_stage3_teach_missing=8U;
  if(s3_teach_holding){
   if(!g_dm_posvel_holding){s3_teach_pending=0U;g_stage3_teach_result=3U;return;}
   if(g_dm_can2_feedback_tick==s3_teach_dm_tick || !it_dm_fresh())return;
   g_stage3_teach_dm_source=1U;
  }else{
   if(g_dm_can2_read_cycle==s3_teach_dm_cycle)return;
   if(g_dm_can2_state==5U){s3_teach_pending=0U;g_stage3_teach_result=3U;return;}
   if(g_dm_can2_state!=4U || !it_dm_fresh())return;
   g_stage3_teach_dm_source=2U;
  }
 }
 if(n==1U)g_stage3_pick_depth=s3_teach_z;
 else if(n>=2U && n<=4U){
  if(n!=3U){g_stage3_target_yaw[n-2U]=g_stage3_actual_yaw;g_stage3_target_x[n-2U]=s3_teach_x;}
  g_stage3_target_depth[n-2U]=s3_teach_z;
 }else if(n==6U || n==7U){
  index=n==6U?0U:2U;g_stage3_target_yaw[index]=g_stage3_actual_yaw;g_stage3_target_x[index]=s3_teach_x;
 }else g_stage3_carrier_yaw=g_stage3_actual_yaw;
 if(n!=5U && n<6U)g_stage3_teach_z_source=1U;
 s3_teach_pending=0U;g_stage3_teach_missing=0U;g_stage3_teach_saved_sequence=g_stage3_teach_sequence;g_stage3_teach_result=1U;
}
static void S3_LockConfig(void)
{
 uint32_t i;
 s3_first=g_stage3_test_first_item;s3_count=g_stage3_test_count;
 s3_pick=g_stage3_pick_depth;s3_carrier=g_stage3_carrier_yaw;
 for(i=0U;i<3U;i++){
  s3_colors[i]=g_stage3_colors[i];s3_positions[i]=g_stage3_positions[i];s3_slots[i]=g_stage3_slot_colors[i];
  s3_yaw[i]=g_stage3_target_yaw[i];s3_x[i]=g_stage3_target_x[i];s3_depth[i]=g_stage3_target_depth[i];
 }
 g_stage3_item=s3_first;g_stage3_phase=0U;g_stage3_state=5U;
}
static void S3_Command(uint32_t action)
{g_it_carrier_auto_deg=-1.0f;g_it_request=action;s3_pending=action;}
static uint32_t S3_Complete(void)
{
 if(!s3_pending)return 1U;
 if(g_it_rejected){S3_Fail(g_it_rejected);return 0U;}
 if(g_it_request || g_it_state!=2U || g_it_action!=s3_pending)return 0U;
 s3_pending=0U;return 1U;
}
static void S3_Axis(uint32_t axis,float value)
{
 g_it_axes=axis;
 if(axis==SPF_DM)g_it_dm_deg=value;
 else if(axis==SPF_EXT)g_it_extension_mm=value;
 else g_it_lift_pulses=value;
 S3_Command(3U);
}
static uint32_t S3_Fresh(uint32_t now)
{return s3_valid && (uint32_t)(now-s3_rx_tick)<=200U && s3_age<=200U &&
 (uint32_t)(now-s3_rx_tick)+s3_age<=200U;}
static void Stage3_Line(uint32_t now)
{
 unsigned long run,pose,seq,valid,norm,age;long right,forward,yaw,error_yaw;int n=0,count;
 uint32_t crc=0U,i;
 if(s3_used && s3_input[s3_used-1U]=='\r')s3_used--;
 s3_input[s3_used]='\0';
 count=sscanf(s3_input,"O31,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%ld,%lu,%n",&run,&pose,&seq,&valid,&right,&forward,&yaw,&norm,&error_yaw,&age,&n);
 if(count!=10 || n<=0 || (uint32_t)n+4U!=s3_used){g_stage3_rx_bad++;return;}
 for(i=0U;i<4U;i++){
  char h=s3_input[(uint32_t)n+i];uint32_t v;
  if(h>='0' && h<='9')v=(uint32_t)(h-'0');
  else if(h>='A' && h<='F')v=(uint32_t)(h-'A'+10);
  else {g_stage3_rx_bad++;return;}crc=(crc<<4)|v;
 }
 if(crc!=SC_Crc(s3_input,(uint32_t)n) || valid>1UL || !seq || seq>0xffffffffUL ||
    right<-2147483647L || right>2147483647L || forward<-2147483647L || forward>2147483647L ||
    yaw<-2147483647L || yaw>2147483647L || norm>0xffffffffUL ||
    error_yaw<-2147483647L || error_yaw>2147483647L){g_stage3_rx_bad++;return;}
 if(g_stage3_state!=2U || run!=g_stage3_run || pose!=g_stage3_pose || age>200UL ||
    (s3_seq && (int32_t)((uint32_t)seq-s3_seq)<=0))return;
 s3_seq=(uint32_t)seq;s3_rx_tick=now;s3_age=(uint32_t)age;s3_valid=(uint32_t)valid;
 g_stage3_right_um=(int32_t)right;g_stage3_forward_um=(int32_t)forward;g_stage3_yaw_mdeg=(int32_t)yaw;g_stage3_rx++;
 g_stage3_error_norm_um=(uint32_t)norm;g_stage3_error_yaw_mdeg=(int32_t)error_yaw;
}
static uint32_t Stage3_SharedByte(uint8_t b,uint32_t now,uint32_t binary_active)
{
 if(s3_active && (uint32_t)(now-s3_last_byte)>=200U)s3_active=s3_used=0U;
 if(!s3_active){if(binary_active || b!='O')return 0U;s3_active=1U;s3_used=0U;}
 s3_last_byte=now;
 if(b=='\n'){Stage3_Line(now);s3_active=s3_used=0U;return 1U;}
 if((b<32U && b!='\r') || s3_used>=sizeof(s3_input)-1U){s3_active=s3_used=0U;g_stage3_rx_bad++;return 1U;}
 s3_input[s3_used++]=(char)b;return 1U;
}
static void Stage3_Telemetry(uint32_t now)
{
 char wire[192];int n;
 if((uint32_t)(now-s3_tx_tick)<100U)return;
 n=snprintf(wire,sizeof(wire),"S31,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
  (unsigned long)g_stage3_run,(unsigned long)g_stage3_state,(unsigned long)g_stage3_pose,(unsigned long)g_stage3_item,
  (unsigned long)g_stage3_colors[0],(unsigned long)g_stage3_colors[1],(unsigned long)g_stage3_colors[2],
  (unsigned long)g_stage3_align_done,(unsigned long)g_stage3_config_ready);
 if(n<=0 || (uint32_t)n>=sizeof(wire)-7U)return;
 snprintf(wire+n,sizeof(wire)-(uint32_t)n,"%04X\r\n",SC_Crc(wire,(uint32_t)n));
 if(USART6_VisionSend((const uint8_t*)wire,(uint16_t)((uint32_t)n+6U))){s3_tx_tick=now;g_stage3_tx++;}
}
static void Stage3_Service(uint32_t now)
{
 uint32_t request=g_stage3_request;g_stage3_request=0U;
 S3_ReadActual();
 if(request){
  if(s3_teach_pending || g_stage3_teach_request){g_stage3_teach_result=3U;g_stage3_teach_missing=1U;}
  s3_teach_pending=0U;g_stage3_teach_request=0U;
 }else S3_Teach();
 g_stage3_config_ready=S3_Config();
 if(request==9U && g_stage3_state && g_stage3_state!=6U && g_stage3_state!=9U){S3_Fail(2U);return;}
 if(request==10U || request==1U){
  if((request==10U && g_stage3_state!=0U) || (request==1U && g_stage3_state!=0U && g_stage3_state!=11U && g_stage3_state!=6U) ||
     !S3_Platform() || SourceBodyStep_WheelWindow() || g_it_request){g_stage3_rejected++;return;}
  if(++g_stage3_run==0U)g_stage3_run=1U;
  g_stage3_state=request==10U?10U:1U;g_stage3_phase=0U;g_stage3_item=0U;g_stage3_align_done=0U;
  s3_pending=s3_loader_slot=s3_loader_dwell=0U;
  g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;S3_Pose();
 }
 if(!g_stage3_state || g_stage3_state==6U || g_stage3_state==9U || g_stage3_state==11U)return;
 if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || g_it_state==9U || g_low_state==9U ||
    g_startup_axes_cause || g_lift_ruler_cause || g_dm_jog_first_fault || g_source_body_step_reason || g_source_body_step_state==9U){
  S3_Fail(g_it_cause?g_it_cause:12U);return;
 }
 /* Explicit one-step calibration, using the same SBS executor and epoch fence.
  * Host sets its direction/distance or signed turn first; never writes state. */
 if(request==30U){
  if(g_stage3_state!=2U || !S3_Platform() || s3_pending || g_it_request ||
     SourceBodyStep_Busy() || g_source_body_step_request){g_stage3_rejected++;return;}
  /* Explicit manual calibration is not a successful visual alignment. */
  g_stage3_align_done=0U;S3_Pose();
  g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;
  g_stage3_state=4U;
  if(g_source_align_chain_active)g_source_body_step_request=7U;
  return;
 }
 if((request==20U || request==21U) && g_stage3_state==2U){
  if(SourceBodyStep_Busy() || g_source_body_step_request || g_it_request || g_it_state!=2U){g_stage3_rejected++;return;}
  s3_step_seq=g_source_align_command_seq;S3_Pose();g_stage3_state=3U;g_stage3_body_steps++;
  g_source_body_step_request=request==20U?2U:3U;return;
 }
 if(g_stage3_state==10U){
  if(s3_pending){
   if(!S3_Complete())return;
   if(s3_loader_slot==2U){g_stage3_state=11U;S3_Pose();return;}
   s3_wait_tick=now;s3_loader_dwell=1U;return;
  }
  if(s3_loader_dwell){if((uint32_t)(now-s3_wait_tick)<2000U)return;s3_loader_dwell=0U;s3_loader_slot++;}
  g_it_carrier_deg=(float)(s3_loader_slot*120U);S3_Command(10U);return;
 }
 if(g_stage3_state==1U){
  static const uint32_t prep[]={1U,2U,7U,6U,3U,3U};
  if(s3_pending){if(!S3_Complete())return;g_stage3_phase++;}
  if(g_stage3_phase==1U && StartupAxes_Holding() && g_extension_home_reference_valid &&
     g_extension_hold_confirmed)g_stage3_phase++;
  if(g_stage3_phase==0U && it_at_top()){g_stage3_phase++;return;}
  if(g_stage3_phase==6U){g_stage3_state=2U;S3_Pose();return;}
  if(g_stage3_phase==4U){g_it_axes=SPF_EXT;g_it_extension_mm=0.0f;}
  if(g_stage3_phase==5U){g_it_axes=SPF_DM;g_it_dm_deg=180.0f;}
  S3_Command(g_stage3_phase==0U && g_lift_ruler_controller.top_valid?4U:prep[g_stage3_phase]);return;
 }
 if(g_stage3_state==2U){
  int32_t right=g_stage3_right_um,forward=g_stage3_forward_um,yaw=g_stage3_yaw_mdeg,amount;
  float angle=(float)yaw/1000.0f;
  uint32_t axis;
  if(!S3_Fresh(now))return;
  if(g_stage3_error_norm_um<=2000U && labs((long)g_stage3_error_yaw_mdeg)<=1000L){
   g_stage3_align_done=1U;S3_Pose();g_stage3_state=4U;
   if(g_source_align_chain_active)g_source_body_step_request=7U;return;
  }
  /* Unsupported sub-step motion is observed again; it is never rounded into
   * an invented movement or a new permanent fault. */
  if(labs((long)g_stage3_error_yaw_mdeg)>1000L){
   if(!(fabsf(angle)>1.0f)){g_stage3_motion_unusable++;s3_valid=0U;return;}
   s3_step_seq=g_source_align_command_seq;S3_Pose();g_stage3_state=3U;g_stage3_body_steps++;
   g_source_align_turn_deg=angle>10.0f?10.0f:(angle< -10.0f?-10.0f:angle);g_source_body_step_request=3U;
  }else{
   axis=labs((long)right)>=labs((long)forward)?0U:1U;amount=axis?forward:right;
   if(labs((long)amount)<1000L){g_stage3_motion_unusable++;s3_valid=0U;return;}
   s3_step_seq=g_source_align_command_seq;S3_Pose();g_stage3_state=3U;g_stage3_body_steps++;
   g_source_align_direction=axis?(amount>0?0U:3U):(amount>0?2U:1U);
   g_source_align_distance_mm=fabsf((float)amount)/1000.0f;
   if(g_source_align_distance_mm>30.0f)g_source_align_distance_mm=30.0f;
   g_source_body_step_request=2U;
  }
  return;
 }
 if(g_stage3_state==3U){
  if(g_source_align_command_seq==s3_step_seq || g_source_body_step_state!=3U || g_chassis_auto_active)return;
  g_stage3_state=2U;S3_Pose();return;
 }
 if(g_stage3_state==4U){
  if(g_source_body_step_request || g_source_align_chain_active)return;
  if(!g_low_wheel_hold_ready){
   if(!g_low_wheel_hold_request && !g_low_wheel_hold_stage)g_low_wheel_hold_request=1U;
   if(g_low_wheel_hold_stage==99U)S3_Fail(12U);return;
  }
  if(g_stage3_align_done && g_stage3_config_ready)S3_LockConfig();else g_stage3_state=7U;return;
 }
 if(g_stage3_state==7U){
  if(request==2U && !g_stage3_align_done){
   if(!S3_Platform() || s3_pending || g_it_request || SourceBodyStep_Busy() ||
      g_source_body_step_request){g_stage3_rejected++;return;}
   g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;
   g_stage3_phase=0U;g_stage3_state=1U;S3_Pose();return;
  }
  if(request==2U && g_stage3_align_done && g_stage3_config_ready && g_low_wheel_hold_ready && it_at_top())S3_LockConfig();
  return;
 }
 if(g_stage3_state==5U){
  uint32_t p=g_stage3_phase,target=g_stage3_item<3U?s3_positions[g_stage3_item]-1U:0U;
  if(s3_pending){if(!S3_Complete())return;g_stage3_phase++;p=g_stage3_phase;}
  /* Real referenced feedback satisfies ensure-top; never synthesize LRC DONE. */
  if((p==1U || p==7U || p==12U) && it_at_top()){g_stage3_phase++;return;}
  switch(p){
  case 0U:{
   uint32_t slot=0U;
   while(slot<3U && s3_slots[slot]!=s3_colors[g_stage3_item])slot++;
   /* Config validation plus locked inventory guarantee a unique color match. */
   g_it_carrier_deg=(float)(slot*120U);S3_Command(10U);break;
  }
  case 1U:S3_Command(4U);break;
  case 2U:S3_Axis(SPF_EXT,0.0f);break;
  case 3U:S3_Axis(SPF_DM,s3_carrier);break;
  case 4U:S3_Command(17U);break; /* Stage3 preclose before pickup descent. */
  case 5U:S3_Axis(SPF_LIFT,s3_pick);break;
  case 6U:S3_Command(5U);break;
  case 7U:S3_Command(4U);break;
  case 8U:S3_Axis(SPF_DM,s3_yaw[target]);break;
  case 9U:S3_Axis(SPF_EXT,s3_x[target]);break;
  case 10U:S3_Axis(SPF_LIFT,s3_depth[target]);break;
  case 11U:S3_Command(6U);break;
  case 12U:S3_Command(4U);break;
  case 13U:S3_Axis(SPF_EXT,0.0f);break;
  case 14U:
   if(++g_stage3_item<s3_first+s3_count){g_stage3_phase=0U;return;}
   S3_Axis(SPF_DM,0.0f);break;
  case 15U:g_it_carrier_deg=240.0f;S3_Command(10U);break;
  case 16U:S3_Command(6U);break;
  default:g_stage3_state=6U;S3_Pose();break;
  }
 }
}
