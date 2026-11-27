/* 停盘后的看护三件试验：实际观测选择取料区，任务序号选择车载槽。
 * 仅报告命令链完成，不写入抓牢、放稳或正式库存事实。 */
volatile uint32_t g_region_request,g_region_state,g_region_phase,g_region_item;
volatile uint32_t g_region_sequence,g_region_accepted_sequence,g_region_rejected,g_region_fault;
volatile uint32_t g_region_observation_cycle,g_region_colors[3],g_region_task_colors[3];
volatile float g_region_yaw_deg[3],g_region_extension_mm[3];
volatile uint32_t g_region_elapsed_ms,g_region_release_ms[3];
static float region_yaw[3],region_extension[3];
static uint32_t region_map[3],region_begin,region_phase_tick,region_it_sequence;
static uint32_t Region_Busy(void){return g_region_state==1U;}
static uint32_t Region_Source(void){return region_map[g_region_item];}
static uint32_t Region_PoseAllowed(void)
{
 uint32_t k;
 if(!Region_Busy() || g_region_item>=3U ||
    (g_region_phase!=2U && g_region_phase!=3U))return 0U;
 k=Region_Source();
 return g_it_dm_deg==region_yaw[k] && g_it_extension_mm==region_extension[k];
}
static uint32_t Region_AtObserveExtension(void)
{
 return fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg-
  STARTUP_EXTENSION_EXTEND_SIGN*65.0f*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f)<=45.0f;
}
static uint32_t Region_ObserveYawAllowed(void)
{
 return Region_Busy() && g_region_item==0U && g_region_phase==1U &&
  Region_Source()!=2U && Region_AtObserveExtension() &&
  fabsf(g_dm_can2_logical_position_deg-180.0f)<=1.0f &&
  g_it_axes==SPF_DM && g_it_dm_deg==region_yaw[Region_Source()];
}
static uint32_t Region_ObservePickAllowed(void)
{
 return Region_PoseAllowed() && g_region_phase==2U && g_region_item==0U &&
  Region_Source()!=2U && Region_AtObserveExtension();
}
static void Region_Aim(uint32_t now)
{
 uint32_t k=Region_Source();
 g_it_dm_deg=region_yaw[k];g_it_extension_mm=0.0f;
 /* 首件若在下部，当前已是180度，仅收回伸缩；左右目标在已确认的
  * 顶部65mm观察姿态转臂。后续件从上件回送结束的收回姿态出发。 */
 g_it_axes=(g_region_item==0U && k==2U)?SPF_EXT:SPF_DM;
 g_it_carrier_auto_deg=(float)g_region_item*120.0f;
 region_it_sequence=g_it_sequence;g_it_request=3U;
 g_region_phase=1U;region_phase_tick=now;
}
static uint32_t Region_TryStart(uint32_t now)
{
 uint32_t i,j,k,observed[3],seen=0U;
 if(Low_Busy() || !g_region_sequence || g_region_sequence==g_region_accepted_sequence ||
  g_region_state==1U || g_region_state==9U || g_it_auto_state==1U || g_it_auto_state==9U ||
  g_pzq_near_state==1U || g_it_state!=2U || g_it_request || g_it_cause || g_it_rejected ||
  !it_platform() || !it_at_top() || !it_dm_fresh() || !SingleTeach_ReferencesReady() ||
  SourceBodyStep_Busy() || SourceBodyStep_WheelWindow() || VehicleArm_Busy() ||
  Stage2Fast_IsActive() || SourceStage2_IsActive() || g_lift_ruler_request ||
  g_lift_ruler_stop_pending || !Region_AtObserveExtension() ||
  fabsf(g_dm_can2_logical_position_deg-180.0f)>1.0f || !g_vis_material_valid ||
  !g_region_observation_cycle || g_region_observation_cycle!=g_vis_mat_cycle ||
  (uint32_t)(now-g_vis_mat_tick)>1000U)return 0U;
 observed[0]=g_vis_mat_left;observed[1]=g_vis_mat_right;observed[2]=g_vis_mat_down;
 for(i=0U;i<3U;i++){
  if(g_region_colors[i]!=observed[i] || observed[i]<1U || observed[i]>6U ||
   !(g_region_yaw_deg[i]>=0.0f && g_region_yaw_deg[i]<=270.0f) ||
   !(g_region_extension_mm[i]>=0.0f && g_region_extension_mm[i]<=170.0f))return 0U;
  for(j=0U;j<i;j++)if(observed[i]==observed[j])return 0U;
 }
 if(g_region_yaw_deg[2]!=180.0f || g_region_extension_mm[2]!=0.0f)return 0U;
 for(i=0U;i<3U;i++){
  k=3U;for(j=0U;j<3U;j++)if(observed[j]==g_region_task_colors[i])k=j;
  if(k==3U || (seen&(1U<<k)))return 0U;
  seen|=1U<<k;region_map[i]=k;
 }
 for(i=0U;i<3U;i++){
  region_yaw[i]=g_region_yaw_deg[i];region_extension[i]=g_region_extension_mm[i];
  g_region_release_ms[i]=0U;
 }
 g_region_accepted_sequence=g_region_sequence;g_region_state=1U;
 g_region_item=g_region_fault=g_region_elapsed_ms=0U;region_begin=now;
 Region_Aim(now);return 1U;
}
static void Region_Service(uint32_t now)
{
 uint32_t request=g_region_request,expected,k;g_region_request=0U;
 if(request==9U && Region_Busy()){
  g_region_fault=2U;g_region_state=9U;g_it_request=9U;return;
 }
 if(request==1U && !Region_TryStart(now))g_region_rejected++;
 if(!Region_Busy())return;
 g_region_elapsed_ms=now-region_begin;
 if(!it_platform() || g_it_cause || g_it_rejected || g_it_state==9U){
  g_region_fault=g_it_cause?g_it_cause:12U;g_region_state=9U;
  g_it_request=0U;it_fail(g_region_fault);return;
 }
 if(g_region_phase==4U){
  if((uint32_t)(now-region_phase_tick)<50U)return;
  if(g_region_item<2U){g_region_item++;Region_Aim(now);return;}
  g_it_axes=SPF_DM;g_it_dm_deg=180.0f;g_it_extension_mm=0.0f;
  region_it_sequence=g_it_sequence;g_it_request=3U;
  g_region_phase=5U;region_phase_tick=now;return;
 }
 if(g_it_state!=2U || g_it_request || g_it_sequence==region_it_sequence)return;
 expected=g_region_phase==2U?14U:(g_region_phase==3U?15U:3U);
 if(g_it_action!=expected)return;
 if(g_region_phase==1U){
  k=Region_Source();g_it_dm_deg=region_yaw[k];g_it_extension_mm=region_extension[k];
  g_it_lift_pulses=29700.0f;region_it_sequence=g_it_sequence;
  g_it_request=14U;g_region_phase=2U;
 }else if(g_region_phase==2U){
  if(!g_it_lead5_grip_done && jiazhua(140.0f)!=1U){
   g_region_fault=7U;g_region_state=9U;it_fail(7U);return;
  }
  g_it_lift_pulses=1800.0f;region_it_sequence=g_it_sequence;
  g_it_request=15U;g_region_phase=3U;
 }else if(g_region_phase==3U){
  if(jiazhua(210.0f)!=1U){g_region_fault=7U;g_region_state=9U;it_fail(7U);return;}
  region_phase_tick=(uint32_t)xTaskGetTickCount();
  g_region_release_ms[g_region_item]=region_phase_tick-region_begin;
  g_region_phase=4U;return;
 }else if(g_region_phase==5U){
  g_it_axes=SPF_EXT;g_it_dm_deg=180.0f;g_it_extension_mm=65.0f;
  region_it_sequence=g_it_sequence;g_it_request=3U;g_region_phase=6U;
 }else if(g_region_phase==6U){g_region_state=2U;g_region_phase=7U;}
 else {g_region_fault=13U;g_region_state=9U;it_fail(13U);return;}
 region_phase_tick=now;
}
