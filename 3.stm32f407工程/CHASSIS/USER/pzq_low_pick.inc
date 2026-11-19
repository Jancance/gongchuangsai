/* 低位逐件动作：预判只可就位，实际新鲜停稳观测才允许下降。
 * 不写抓牢/放稳库存；每个新序号只完成一件命令链。 */
volatile uint32_t g_low_request,g_low_state,g_low_phase,g_low_fault,g_low_rejected;
volatile uint32_t g_low_sequence,g_low_accepted_sequence,g_low_done_sequence;
volatile uint32_t g_low_slot=1U,g_low_color,g_low_region,g_low_pose_version,g_low_scene_id;
volatile float g_low_target_yaw=180.0f,g_low_target_extension_mm=0.0f;
volatile uint32_t g_low_confirm_pose_sequence,g_low_confirm_frame_cycle;
volatile uint32_t g_low_elapsed_ms,g_low_clear_ms,g_low_clear_valid,g_low_release_ms;
volatile float g_low_actual_pick_depth_pulses,g_low_actual_rise_mm;
static uint32_t low_begin,low_phase_tick,low_it_sequence,low_slot,low_color;
static uint32_t low_pose_version,low_scene_id;
static float low_yaw,low_extension,low_measured_depth;
static uint32_t low_depth_tick,low_ext_tick;

static uint32_t Low_Busy(void){return g_low_state==1U;}
static uint32_t Low_Loaded(void){return Low_Busy() && g_low_phase==5U;}
#define LOW_OBSERVE_SETTLE_EXTENSION 1
/* 仅顶部180度、从收回位返回65mm观察位时借用既有有界收敛窗口。 */
static uint32_t Low_ObserveReturnSettling(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 0U;
#else
 return Low_Busy() && g_low_phase==8U && g_it_state==1U &&
  g_it_action==3U && g_it_axes==SPF_EXT && it_platform() && it_at_top() &&
  SingleTeach_ReferencesReady() && fabsf(g_dm_can2_logical_position_deg-180.0f)<=1.0f &&
  vsp_group.command.kind==SPF_CMD_TRANSFER && vsp_group.command.axes_mask==SPF_EXT &&
  vsp_group.command.goal.extension_mm==65.0f &&
  fabsf(vsp_axis[0].before-g_extension_home_reference_deg)<=2.0f;
#endif
}

static uint32_t Low_ReadDepth(float *depth)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)depth;return 0U;
#else
 VSP_Axis sample;memset(&sample,0,sizeof(sample));
 if(!g_lift_ruler_controller.top_valid || !g_lift_ruler_controller.top_scale ||
    VehicleSourceParallel_Owns())return 0U;
 sample.options=g_lift_ruler_controller.top_scale==100U?0x80U:0U;
 if(!VSP_Position(5U,&sample))return 0U;
 *depth=(sample.current-SingleTeach_TopDeg())*(3200.0f/360.0f);
 low_measured_depth=*depth;low_depth_tick=(uint32_t)xTaskGetTickCount();
 return *depth>=0.0f && *depth<=(float)SPF_MAX_LIFT_PULSES+400.0f;
#endif
}
static uint32_t Low_ExtensionAt(float mm)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)mm;return 0U;
#else
 VSP_Axis sample;memset(&sample,0,sizeof(sample));
 if(VehicleSourceParallel_Owns() || !VSP_Position(6U,&sample))return 0U;
 low_ext_tick=(uint32_t)xTaskGetTickCount();
 return fabsf(sample.current-g_extension_home_reference_deg+
  mm*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f)<=(mm==0.0f?2.0f:SOURCE_EXTENSION_DONE_DEG);
#endif
}
static uint32_t Low_AtHover(void)
{float depth;return Low_ReadDepth(&depth) && it_at_top();}
static uint32_t Low_PoseAllowed(void)
{
 return Low_Busy() && (g_low_phase==4U || g_low_phase==5U) &&
  g_it_dm_deg==low_yaw && g_it_extension_mm==low_extension &&
  low_pose_version && low_scene_id;
}
static uint32_t Low_PickAllowed(void)
{
 uint32_t now=(uint32_t)xTaskGetTickCount();
 return Low_PoseAllowed() && g_low_phase==4U &&
  (uint32_t)(now-low_depth_tick)<=200U && (uint32_t)(now-low_ext_tick)<=200U &&
  it_at_top() &&
  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg+
   low_extension*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f)<=(low_extension==0.0f?2.0f:SOURCE_EXTENSION_DONE_DEG);
}
static uint32_t Low_XYAllowed(void)
{
 if(!Low_Busy() || !Low_AtHover())return 0U;
 if(g_low_phase==1U)return g_it_axes==SPF_DM && g_it_dm_deg==low_yaw;
 if(g_low_phase==2U)return g_it_axes==SPF_EXT && g_it_dm_deg==low_yaw &&
  g_it_extension_mm==low_extension && fabsf(g_dm_can2_logical_position_deg-low_yaw)<=1.0f;
 return 0U;
}
static uint32_t Low_TransferGate(uint32_t now,float depth,uint32_t fresh)
{
 if(!Low_Loaded() || !fresh || depth<0.0f)return 0U;
 /* 换算后向下取整到虚拟脉冲，防止浮点尾差把恰好65mm算成超过。 */
 g_low_actual_rise_mm=floorf(g_low_actual_pick_depth_pulses-depth)/SOURCE_LIFT_PULSES_PER_MM;
 if(g_low_actual_rise_mm<=65.0f)return 0U;
 if(!g_low_clear_valid){g_low_clear_valid=1U;g_low_clear_ms=now-low_begin;}
 return 1U;
}
static void Low_Command(uint32_t request,uint32_t phase,uint32_t now)
{low_it_sequence=g_it_sequence;g_it_request=request;g_low_phase=phase;low_phase_tick=now;}
static uint32_t Low_TryStart(uint32_t now)
{
 if(!g_low_sequence || g_low_sequence==g_low_accepted_sequence ||
  Low_Busy() || g_low_state==9U || g_region_state==1U ||
  g_it_auto_state==1U || g_it_auto_state==9U || g_pzq_near_state==1U ||
  g_it_state!=2U || g_it_request || g_it_cause || g_it_rejected ||
  !it_platform() || !it_dm_fresh() || !SingleTeach_ReferencesReady() ||
  SourceBodyStep_Busy() || SourceBodyStep_WheelWindow() || VehicleArm_Busy() ||
  Stage2Fast_IsActive() || SourceStage2_IsActive() || g_lift_ruler_request ||
  g_lift_ruler_stop_pending || !g_low_pose_version || !g_low_scene_id ||
  g_low_slot<1U || g_low_slot>3U || g_low_region>2U ||
  g_low_color<1U || g_low_color>6U ||
  !(g_low_target_yaw>=0.0f && g_low_target_yaw<=270.0f) ||
  !(g_low_target_extension_mm>=0.0f && g_low_target_extension_mm<=170.0f) ||
  (g_low_region==2U && (g_low_target_yaw!=180.0f || g_low_target_extension_mm!=0.0f)) ||
  fabsf(g_dm_can2_logical_position_deg-180.0f)>1.0f ||
  !Low_AtHover() || !Low_ExtensionAt(65.0f))return 0U;
 low_slot=g_low_slot;low_color=g_low_color;low_yaw=g_low_target_yaw;
 low_extension=g_low_target_extension_mm;low_pose_version=g_low_pose_version;low_scene_id=g_low_scene_id;
 g_low_accepted_sequence=g_low_sequence;g_low_state=1U;
 g_low_fault=g_low_elapsed_ms=g_low_clear_ms=g_low_clear_valid=g_low_release_ms=0U;
 g_low_actual_pick_depth_pulses=g_low_actual_rise_mm=0.0f;low_begin=now;
 g_it_axes=SPF_DM;g_it_dm_deg=low_yaw;g_it_extension_mm=65.0f;
 g_it_carrier_auto_deg=(float)(low_slot-1U)*120.0f;
 Low_Command(3U,1U,now);return 1U;
}
static uint32_t Low_TryConfirm(uint32_t now)
{
 uint32_t colors[3],i,count=0U;
 if(!Low_Busy() || g_low_phase!=3U || g_it_state!=2U || g_it_request ||
  g_low_confirm_pose_sequence!=g_low_accepted_sequence ||
  !g_low_confirm_frame_cycle || g_low_confirm_frame_cycle!=g_vis_mat_cycle ||
  !g_vis_material_valid || (uint32_t)(now-g_vis_mat_tick)>200U ||
  (int32_t)(g_vis_mat_tick-low_phase_tick)<0 || !it_dm_fresh() ||
  fabsf(g_dm_can2_logical_position_deg-low_yaw)>1.0f ||
  !Low_AtHover() || !Low_ExtensionAt(low_extension))return 0U;
 if((uint32_t)((uint32_t)xTaskGetTickCount()-g_vis_mat_tick)>200U)return 0U;
 colors[0]=g_vis_mat_left;colors[1]=g_vis_mat_right;colors[2]=g_vis_mat_down;
 for(i=0U;i<3U;i++)if(colors[i]==low_color)count++;
 if(count!=1U)return 0U;
 g_it_dm_deg=low_yaw;g_it_extension_mm=low_extension;g_it_lift_pulses=29700.0f;
 Low_Command(14U,4U,now);return 1U;
}
#include "low_readonly_diag.inc"
static void Low_Service(uint32_t now)
{
 uint32_t request=g_low_request,expected;g_low_request=0U;
 Low_ReadonlyDiag();
 if(request==9U && Low_Busy()){g_low_state=9U;g_low_fault=2U;g_it_request=9U;return;}
 if(request==1U && !Low_TryStart(now))g_low_rejected++;
 if(request==2U && !Low_TryConfirm(now))g_low_rejected++;
 if(!Low_Busy())return;
 g_low_elapsed_ms=now-low_begin;
 if(!it_platform() || g_it_cause || g_it_rejected || g_it_state==9U ||
  (g_low_phase!=3U && (uint32_t)(now-low_phase_tick)>=16000U)){
  g_low_fault=g_it_cause?g_it_cause:12U;g_low_state=9U;
  g_it_request=0U;it_fail(g_low_fault);return;
 }
 /* 错过停顿或普通识别歧义只等待新观测，不故障锁死、也不盲降。 */
 if(g_low_phase==3U)return;
 if(g_low_phase==6U){
  if((uint32_t)(now-low_phase_tick)<50U)return;
  g_it_axes=SPF_DM;g_it_dm_deg=180.0f;g_it_extension_mm=0.0f;
  Low_Command(3U,7U,now);return;
 }
 if(g_it_state!=2U || g_it_request || g_it_sequence==low_it_sequence)return;
 expected=g_low_phase==4U?14U:(g_low_phase==5U?15U:3U);
 if(g_it_action!=expected)return;
 switch(g_low_phase){
 case 1U:
  g_it_axes=SPF_EXT;g_it_dm_deg=low_yaw;g_it_extension_mm=low_extension;
  Low_Command(3U,2U,now);break;
 case 2U:g_low_phase=3U;low_phase_tick=now;break;
 case 4U:
#if !APP_MOTOR_OFFLINE_ONLY
  if(!vsp_axis[1].position_cycle || (uint32_t)((uint32_t)xTaskGetTickCount()-vsp_axis[1].position_tick)>150U){
   g_low_fault=14U;g_low_state=9U;it_fail(14U);return;
  }
  g_low_actual_pick_depth_pulses=(vsp_axis[1].current-SingleTeach_TopDeg())*(3200.0f/360.0f);
  if(!g_it_lead5_grip_done && jiazhua(140.0f)!=1U){
   g_low_fault=7U;g_low_state=9U;it_fail(7U);return;
  }
  g_it_lift_pulses=1800.0f;Low_Command(15U,5U,now);break;
#else
  g_low_fault=14U;g_low_state=9U;it_fail(14U);return;
#endif
 case 5U:
  if(!g_low_clear_valid){g_low_fault=14U;g_low_state=9U;it_fail(14U);return;}
  if(jiazhua(210.0f)!=1U){g_low_fault=7U;g_low_state=9U;it_fail(7U);return;}
  low_phase_tick=(uint32_t)xTaskGetTickCount();g_low_release_ms=low_phase_tick-low_begin;
  g_low_phase=6U;break;
 case 7U:
  g_it_axes=SPF_EXT;g_it_dm_deg=180.0f;g_it_extension_mm=65.0f;
  Low_Command(3U,8U,now);break;
 case 8U:g_low_state=2U;g_low_phase=10U;g_low_done_sequence=g_low_accepted_sequence;break;
 default:g_low_fault=13U;g_low_state=9U;it_fail(13U);break;
 }
}
