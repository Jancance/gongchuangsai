/* 单件人工看护示教入口；所有服务由原唯一CAN任务调用。
 * 1真实顶部回零，2伸缩真实回零，3锁存位姿，4带料/空爪回顶，
 * 5闭爪，6松爪，7只读DM，8载盘0度，9停止。无自动循环、无库存伪确认。 */
volatile uint32_t g_it_request,g_it_state,g_it_cause,g_it_rejected,g_it_action,g_it_sequence;
volatile uint32_t g_it_axes=2U,g_it_elapsed_ms,g_it_lift_permit;
volatile uint32_t g_it_place_depth_limit_pulses=0U; /* 放料新示教前为0，不用历史高度。 */
volatile float g_it_dm_deg=180.0f,g_it_extension_mm=48.0f,g_it_lift_pulses=108400.0f;
static uint32_t it_begin,it_phase,it_lift_samples,it_dm_cycle,it_seen_samples;
static SPF_Command it_command;

static uint32_t it_platform(void)
{return g_safety_ready && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault &&
 !g_can_ctrl_fault_latched && !g_chassis_auto_active;}
static uint32_t it_at_top(void)
{return g_lift_ruler_controller.top_valid && g_vehicle_lift_feedback_valid &&
 fabsf(g_vehicle_lift_current_deg-(float)g_lift_ruler_controller.top_position_raw/
       (float)g_lift_ruler_controller.top_scale)<=1.0f;}
static void it_fail(uint32_t why)
{
 if(!g_it_cause)g_it_cause=why;g_it_state=9U;g_it_lift_permit=0U;
 if(VehicleSourceParallel_Owns())VehicleSourceParallel_Cancel();
 if(g_lift_ruler_state>0U && g_lift_ruler_state<4U)g_lift_ruler_request=2U;
 if(g_startup_axes_state>=1U && g_startup_axes_state<=3U)StartupAxes_Cancel();
 if(g_dm_posvel_holding || DM_CAN2_JogIsBusy())g_dm_jog_request=3U;
}
static void it_done(void){g_it_state=2U;g_it_lift_permit=0U;}
static uint32_t it_dm_fresh(void)
{return g_dm_can2_feedback_valid && (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100U;}

void Integration10_Service(void)
{
 uint32_t request=g_it_request,now=(uint32_t)xTaskGetTickCount();
 g_it_request=0U;
 /* 仅转发真实回零/运动反馈，不把旧完成值覆盖到当前位姿。 */
 if(g_lift_ruler_samples!=it_seen_samples){
  it_seen_samples=g_lift_ruler_samples;
  if(g_lift_ruler_controller.top_valid && g_lift_ruler_controller.scale){
   g_vehicle_lift_reference_valid=1U;g_vehicle_lift_feedback_valid=1U;
   g_vehicle_lift_current_deg=(float)g_lift_ruler_controller.position_after_raw/
                              (float)g_lift_ruler_controller.scale;
  }
 }
 if(request==9U){it_fail(2U);request=0U;}
 if(request){
  if(g_it_state==1U || g_it_state==9U || request>8U){g_it_rejected=1U;}
  else {
   g_it_rejected=0U;g_it_action=request;it_phase=0U;it_begin=now;g_it_elapsed_ms=0U;
   if(!it_platform())g_it_rejected=2U;
   else if(request==1U){
    if(g_lift_ruler_state!=0U || VehicleArm_Busy())g_it_rejected=3U;
    else {g_it_lift_permit=1U;it_lift_samples=g_lift_ruler_samples;
     g_lift_ruler_request=1U;g_it_state=1U;}
   }else if(!g_lift_ruler_controller.top_valid || g_lift_ruler_cause ||
             g_lift_ruler_state!=4U || VehicleArm_Busy())g_it_rejected=4U;
   else if(request==2U){
    if(!it_at_top() || g_startup_axes_state!=0U || !StartupAxes_Begin())g_it_rejected=5U;
    else g_it_state=1U;
   }else if(request==3U){
    uint32_t axes=g_it_axes;
    if(!SingleTeach_ReferencesReady() || !axes || (axes&~7U) ||
       !(g_it_dm_deg>=0 && g_it_dm_deg<=270) ||
       !(g_it_extension_mm>=0 && g_it_extension_mm<=170) ||
       !(g_it_lift_pulses>=0 && g_it_lift_pulses<=108400) ||
       ((axes&3U) && (!it_at_top() || (axes&4U))) ||
       ((axes&4U) && g_it_lift_pulses<3200) ||
       ((axes&4U) && !(fabsf(g_dm_can2_logical_position_deg-180.0f)<=3.0f &&
          fabsf(g_startup_extension_current_deg-(g_extension_home_reference_deg-48.0f*3.6f))<=3.6f) &&
          (!g_it_place_depth_limit_pulses || g_it_place_depth_limit_pulses>108400U ||
           g_it_lift_pulses>(float)g_it_place_depth_limit_pulses)) ||
       ((axes&1U) && (fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>1.0f ||
                     ((axes&2U) && g_it_extension_mm>0.01f))))g_it_rejected=6U;
    else {
     memset(&it_command,0,sizeof(it_command));
     it_command.kind=SPF_CMD_TRANSFER;it_command.sequence=++g_it_sequence;
     it_command.axes_mask=axes;it_command.goal.dm_deg=g_it_dm_deg;
     it_command.goal.extension_mm=g_it_extension_mm;it_command.goal.lift_pulses=g_it_lift_pulses;
     it_command.dm_deg_s=300;it_command.extension_rpm=106;it_command.lift_rpm=1846.4f;
     it_command.stepper_accel_rpm_s=45055;it_command.keep_vm_on=1U;
     if(!g_dm_posvel_holding){it_dm_cycle=g_dm_can2_read_cycle;g_dm_can2_read_request=2U;it_phase=1U;}
     else it_phase=2U;
     g_it_state=1U;
    }
   }else if(request==4U){
    g_it_lift_permit=1U;it_lift_samples=g_lift_ruler_samples;
    g_lift_ruler_request=3U;g_it_state=1U;
   }else if(request==5U || request==6U){
    if(jiazhua(request==5U?140.0f:210.0f)!=1U)g_it_rejected=7U;
    else g_it_state=1U;
   }else if(request==7U){
    it_dm_cycle=g_dm_can2_read_cycle;
    if(!g_dm_posvel_holding){g_dm_can2_read_request=2U;it_phase=1U;}
    else it_phase=2U;
    g_it_state=1U;
   }else if(request==8U){
    if(!it_at_top() || CarrierPwm_Write(500U)!=1U)g_it_rejected=8U;
    else g_it_state=1U;
   }
  }
 }
 /* 即使已失败也保留原后端STOP重试和真实反馈服务。 */
 StartupAxes_Service(it_platform(),g_can_guard_ready);
 if(VehicleSourceParallel_Owns())VehicleArm_Service();
 if(g_it_state!=1U)return;
 g_it_elapsed_ms=now-it_begin;
 if(!it_platform() || g_lift_ruler_cause || g_startup_axes_cause || g_dm_jog_first_fault){it_fail(3U);return;}
 if(g_it_elapsed_ms>=(g_it_action==1U || g_it_action==4U?60000U:15000U)){it_fail(4U);return;}
 switch(g_it_action){
 case 1U:case 4U:
  if(g_lift_ruler_rejected){it_fail(5U);return;}
  if(g_lift_ruler_samples!=it_lift_samples && g_lift_ruler_state==4U)it_done();
  break;
 case 2U:
  if(g_startup_axes_state==4U && g_extension_home_reference_valid && g_extension_hold_confirmed)it_done();
  break;
 case 3U:
  if(it_phase==1U){
   if(g_dm_can2_state==5U){it_fail(6U);return;}
   if(g_dm_can2_read_cycle==it_dm_cycle || !it_dm_fresh())break;
   it_phase=2U;
  }
  if(it_phase==2U){
   if(!it_dm_fresh())break;
   if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U)){it_fail(7U);return;}
   it_phase=3U;break;
  }
  if(g_source_parallel_state==4U){it_fail(8U);return;}
  if(g_source_parallel_state==3U){it_done();}
  break;
 case 5U:case 6U:
  if(g_it_elapsed_ms>=600U)it_done();break;
 case 7U:
  if(g_dm_can2_state==5U){it_fail(6U);return;}
  if((it_phase==2U || g_dm_can2_read_cycle!=it_dm_cycle) && it_dm_fresh())it_done();
  break;
 case 8U:
  if(g_it_elapsed_ms>=1500U)it_done();break;
 default:it_fail(9U);break;
 }
}
