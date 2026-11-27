/* Exactly one region owns mechanical commands and the text UART observation. */
#if SOURCE_CYCLE_TEST_215
volatile uint32_t g_region_dispatch_area=3U,g_region_dispatch_rejected;
#else
volatile uint32_t g_region_dispatch_area,g_region_dispatch_rejected;
#endif
volatile uint32_t g_region_dispatch_test_enable=1U;
static RegionModuleRequest rm_task;
static volatile RegionModuleStatus rm_status;
static uint32_t rm_bound,rm_wait_dm_stop,rm_fault_after_stop;
#if APP_ZONE2_MISSION_ENABLE
static uint32_t map_ticket,rm_departing,rm_preturn,rm_observe_ticket,rm_observe_stamp;
static uint32_t rm_startup_reference_action;
volatile uint32_t g_region_startup_reference_state,g_region_startup_reference_requests;
volatile uint32_t g_region_x_prepare_ms,g_region_x_ready_ms;
volatile uint32_t g_region_approach_turn_ms,g_region_departure_ms,g_region_return_done_ms;
volatile uint32_t g_region_prepare_started_ms;
volatile float g_region_prepare_started_remaining_mm;
#endif

static uint32_t RM_Faulted(void)
{
 return g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
  g_source_cycle_fault || g_stage3_fault || g_it_cause || g_low_fault ||
  g_low_state==9U || g_startup_axes_cause || g_lift_ruler_cause ||
  g_dm_jog_first_fault || g_source_body_step_reason;
}

#if APP_ZONE2_MISSION_ENABLE
uint32_t RegionDispatch_StartupReferencesReady(void)
{
 if(RM_Faulted() || g_it_rejected)return 2U;
 if(!StartupAxes_Holding())return 0U;
 if(g_region_startup_reference_state==2U)return SingleTeach_ReferencesReady()?1U:2U;
 if(!g_region_startup_reference_state){
  if(SingleTeach_ReferencesReady() && it_at_top()){
   g_region_startup_reference_state=2U;return 1U;
  }
  if(g_it_request || g_it_state==1U || VehicleArm_Busy() || Low_Busy())return 0U;
  rm_startup_reference_action=g_lift_ruler_controller.top_valid?4U:1U;
  g_it_carrier_auto_deg=-1.0f;g_it_request=rm_startup_reference_action;
  g_region_startup_reference_requests++;g_region_startup_reference_state=1U;
  return 0U;
 }
 if(g_it_request || g_it_state==1U)return 0U;
 if(g_it_state!=2U || g_it_action!=rm_startup_reference_action ||
    !SingleTeach_ReferencesReady() || !it_at_top())return 2U;
 g_region_startup_reference_state=2U;return 1U;
}
#endif

static uint32_t RM_Idle(void)
{
 return S3_Platform() && !RM_Faulted() &&
  (!g_source_cycle_state || g_source_cycle_state==8U) &&
  (!g_stage3_state || g_stage3_state==6U || g_stage3_state==11U) &&
  !g_it_request && !g_low_request && !g_source_cycle_request && !g_stage3_request &&
  !g_stage3_teach_request && !s3_teach_pending && !g_stage3_manual_state && !g_stage3_replay_enable &&
  !g_source_body_step_request && !SourceBodyStep_WheelWindow() &&
  (!g_chassis_auto_active
#if APP_ZONE2_MISSION_ENABLE
   || (AppZone2_StationApproaching() && it_at_top() &&
       fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)<=SOURCE_EXTENSION_ZERO_DONE_DEG)
#endif
  ) &&
  !g_lift_ruler_request && !g_lift_ruler_stop_pending && !VehicleArm_Busy() && !g_vehicle_arm_stop_pending &&
  !VehicleSourceParallel_Owns() && !Stage2Fast_IsActive() && !SourceStage2_IsActive() &&
  !g_dm_jog_request && !DM_CAN2_JogIsBusy() && g_it_auto_state!=1U && g_it_auto_state!=9U &&
  g_region_state!=1U && g_pzq_near_state!=1U;
}
static uint32_t RM_Valid(const RegionModuleRequest *r)
{
 uint32_t i,j;
 if(!r->run_id || r->batch_id<1U || r->batch_id>2U ||
    r->module<REGION_MODULE_SOURCE || r->module>REGION_MODULE_STORAGE_BASE)return 0U;
 for(i=0U;i<3U;i++){
  uint32_t slot=0U;
  if(r->colors[i]<1U || r->colors[i]>6U || r->slots[i]<1U || r->slots[i]>6U ||
     r->rings[i]<1U || r->rings[i]>3U)return 0U;
  for(j=0U;j<i;j++)if(r->colors[j]==r->colors[i] || r->rings[j]==r->rings[i])return 0U;
  for(j=0U;j<3U;j++)slot+=r->slots[j]==r->colors[i];
  if(slot!=1U)return 0U;
 }
 if(r->module==REGION_MODULE_SOURCE)return 1U;
 if(!(g_stage3_pick_depth>=3200.0f && g_stage3_pick_depth<=SPF_STAGE3_MAX_LIFT_PULSES &&
      g_stage3_carrier_yaw>=0.0f && g_stage3_carrier_yaw<=270.0f))return 0U;
 for(i=0U;i<3U;i++){
  j=r->rings[i]-1U;
  if(!(g_stage3_target_yaw[j]>=0.0f && g_stage3_target_yaw[j]<=270.0f &&
       g_stage3_target_x[j]>=0.0f && g_stage3_target_x[j]<=170.0f &&
       g_stage3_target_depth[j]>=3200.0f && g_stage3_target_depth[j]<=SPF_STAGE3_MAX_LIFT_PULSES))return 0U;
  if(r->module==REGION_MODULE_ROUGH &&
     !(g_stage3_recovery_depth[j]>=3200.0f && g_stage3_recovery_depth[j]<=SPF_STAGE3_MAX_LIFT_PULSES))return 0U;
  if(r->module==REGION_MODULE_STACK && g_stage3_target_depth[j]-60.0f*SOURCE_LIFT_PULSES_PER_MM<3200.0f)return 0U;
 }
 return 1U;
}
uint32_t RegionDispatch_Begin(const RegionModuleRequest *r)
{
 uint32_t ok=0U;
 if(!r)return 0U;
 taskENTER_CRITICAL();
 if((rm_status==REGION_MODULE_UNKNOWN || rm_status==REGION_MODULE_DONE ||
     (rm_status==REGION_MODULE_STOPPED && !rm_bound)) &&
    (!rm_task.run_id || (int32_t)(r->run_id-rm_task.run_id)>0) && RM_Idle() && RM_Valid(r)){
  rm_task=*r;rm_bound=rm_wait_dm_stop=rm_fault_after_stop=0U;rm_status=REGION_MODULE_BUSY;ok=1U;
 }
 taskEXIT_CRITICAL();
 return ok;
}
RegionModuleStatus RegionDispatch_Poll(uint32_t run_id)
{
 RegionModuleStatus state;
 taskENTER_CRITICAL();
 state=run_id && run_id==rm_task.run_id?rm_status:REGION_MODULE_UNKNOWN;
 taskEXIT_CRITICAL();
 return state;
}
static uint32_t RM_Stop(uint32_t run_id,uint32_t active)
{
 uint32_t ok=0U;
 taskENTER_CRITICAL();
 if((active || (run_id && run_id==rm_task.run_id)) &&
    (rm_status==REGION_MODULE_BUSY || rm_status==REGION_MODULE_STOPPING)){
  if(rm_bound==1U){
   if(rm_task.module==REGION_MODULE_SOURCE)g_source_cycle_request=9U;
   else g_stage3_request=9U;
  }
  rm_status=REGION_MODULE_STOPPING;ok=1U;
 }
 taskEXIT_CRITICAL();
 return ok;
}
uint32_t RegionDispatch_Stop(uint32_t run_id){return RM_Stop(run_id,0U);}
uint32_t RegionDispatch_StopActive(void){return RM_Stop(0U,1U);}

static void RM_Start(uint32_t now)
{
 uint32_t i;
 if(rm_task.module==REGION_MODULE_SOURCE){
  g_region_dispatch_area=1U;sc_bound_batch=rm_task.batch_id;
  for(i=0U;i<3U;i++){g_source_cycle_colors[i]=rm_task.colors[i];sc_bound_slots[i]=rm_task.slots[i];}
  g_source_cycle_run=rm_task.run_id;g_source_cycle_prep=g_source_cycle_item=sc_pending=0U;
  sc_beep_started=g_source_cycle_beeps=g_source_cycle_beep_started=g_source_cycle_beep_done=0U;
  g_source_cycle_pick_beeps=g_source_cycle_pick_beep_pending=g_source_cycle_scheduled=0U;
  g_source_cycle_done=g_source_cycle_body_steps=0U;
  g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;
  g_source_cycle_started=now;SC_Pose();g_source_cycle_state=1U;
 }else{
  g_region_dispatch_area=3U;s3_module_run=rm_task.run_id;
  for(i=0U;i<3U;i++){
   g_stage3_colors[i]=rm_task.colors[i];g_stage3_positions[i]=rm_task.rings[i];g_stage3_slot_colors[i]=rm_task.slots[i];
  }
  g_stage3_cycle_mode=rm_task.module==REGION_MODULE_STACK?2U:
                      (rm_task.module==REGION_MODULE_STORAGE_BASE?0U:1U);
  g_stage3_test_first_item=0U;g_stage3_test_count=3U;
  g_stage3_task_run=rm_task.run_id;g_stage3_task_batch=rm_task.batch_id;
  g_stage3_run=rm_task.run_id-1U;g_stage3_request=1U;
 }
 rm_bound=1U;
#if APP_ZONE2_MISSION_ENABLE
 if(rm_task.module==REGION_MODULE_SOURCE){
  /* 复用首次按键建立的基准，同时转臂和伸缩，不再执行回零链。 */
  g_source_cycle_prep=4U;g_it_axes=SPF_DM|SPF_EXT;
  g_it_dm_deg=180.0f;g_it_extension_mm=65.0f;
  if(jiazhua(210.0f)!=1U){rm_fault_after_stop=1U;rm_status=REGION_MODULE_STOPPING;return;}
  g_it_request=3U;rm_preturn=1U;g_region_x_prepare_ms=g_region_approach_turn_ms=now;
 }else{
  g_it_axes=SPF_DM;g_it_dm_deg=g_stage3_observe_yaw;g_it_extension_mm=0.0f;
  if(jiazhua(210.0f)!=1U){rm_fault_after_stop=1U;rm_status=REGION_MODULE_STOPPING;return;}
  g_it_request=3U;rm_preturn=3U;g_region_approach_turn_ms=now;
 }
#endif
}
static uint32_t RM_StopSettled(void)
{
 return !g_it_request && g_it_state!=1U && !g_low_request && !Low_Busy() &&
  !g_source_body_step_request && !SourceBodyStep_Busy() && !g_chassis_auto_active &&
  !g_lift_ruler_request && !g_lift_ruler_stop_pending && !VehicleArm_Busy() && !g_vehicle_arm_stop_pending &&
  !VehicleSourceParallel_Owns() && !g_dm_jog_request && !DM_CAN2_JogIsBusy() &&
  (!rm_wait_dm_stop || (!g_dm_posvel_holding && g_dm_jog_stop_result==1U));
}
uint32_t RegionDispatch_Arrive(uint32_t area)
{
 if(rm_status==REGION_MODULE_BUSY || rm_status==REGION_MODULE_STOPPING || (area!=1U && area!=3U) ||
    (g_source_cycle_state && g_source_cycle_state!=8U) ||
    (g_stage3_state && g_stage3_state!=6U && g_stage3_state!=11U) ||
    g_it_request || g_it_state==1U || g_it_state==9U || Low_Busy() ||
    SourceBodyStep_Busy() || SourceBodyStep_WheelWindow() || g_chassis_auto_active){
  g_region_dispatch_rejected++;return 0U;
 }
 g_region_dispatch_area=area;
 s3_module_run=0U;
 if(area==1U)g_source_cycle_request=1U;else Stage3_Arrive();
 return 1U;
}
void RegionDispatch_KeyEvent(void)
{
 if(RegionDispatch_StopActive())return;
 if(g_region_dispatch_area==1U){SourceCycle_KeyEvent();return;}
 if(g_region_dispatch_test_enable){g_region_dispatch_area=3U;Stage3_KeyEvent();}
}
void RegionDispatch_ScanService(void)
{if(g_region_dispatch_area==1U)SourceCycle_ScanService();}
static void RegionDispatch_Service(uint32_t now)
{
 uint32_t source,done,fault;
 taskENTER_CRITICAL();
 if(rm_status==REGION_MODULE_STOPPING && !rm_bound)rm_status=REGION_MODULE_STOPPED;
 if(rm_status==REGION_MODULE_BUSY && !rm_bound){
  if(RM_Faulted() || !RM_Valid(&rm_task))rm_status=REGION_MODULE_FAULT;
  else if(RM_Idle())RM_Start(now);
 }
 if(rm_status==REGION_MODULE_STOPPING && rm_bound==1U){
  rm_wait_dm_stop=g_dm_posvel_holding || DM_CAN2_JogIsBusy();
  if(rm_task.module==REGION_MODULE_SOURCE){SC_Fail(2U);if(g_it_request)g_it_request=9U;}
  else S3_Fail(2U);
  if(g_source_body_step_request || SourceBodyStep_WheelWindow())g_source_body_step_request=9U;
  if(rm_wait_dm_stop && g_it_request!=9U)g_it_request=9U;
  rm_bound=2U;
 }
#if APP_ZONE2_MISSION_ENABLE
 if(rm_status==REGION_MODULE_BUSY && rm_bound==1U && rm_task.module!=REGION_MODULE_SOURCE &&
    g_stage3_state==7U && g_stage3_align_done && g_stage3_config_ready && g_low_wheel_hold_ready &&
    it_at_top() && !g_stage3_request && !g_it_request && !SourceBodyStep_WheelWindow())
   g_stage3_request=2U;
#endif
 taskEXIT_CRITICAL();
#if APP_ZONE2_MISSION_ENABLE
 if(rm_preturn && rm_status==REGION_MODULE_BUSY){
  if(g_it_rejected || g_it_state==9U){rm_fault_after_stop=1U;rm_status=REGION_MODULE_STOPPING;return;}
  if(g_it_request || g_it_state==1U)return;
  if(g_it_state!=2U || g_it_action!=3U)return;
  if(rm_task.module==REGION_MODULE_SOURCE){
   /* IT3仅在整个并行动作组真实完成后返回2；不用固定延时。 */
   g_region_x_ready_ms=now;
   g_source_cycle_prep=5U;g_source_cycle_state=2U;sc_stream_started=0U;SC_Pose();
  }
  rm_preturn=0U;
 }
#endif
 if(g_region_dispatch_area==1U)SourceCycle_Service(now);else if(g_region_dispatch_area==3U)Stage3_Service(now);
 if(rm_status==REGION_MODULE_STOPPING){
  if(rm_bound==2U && !g_source_cycle_request && !g_stage3_request && RM_StopSettled())
   rm_status=rm_fault_after_stop?REGION_MODULE_FAULT:REGION_MODULE_STOPPED;
  return;
 }
 if(rm_status!=REGION_MODULE_BUSY || rm_bound!=1U)return;
 source=rm_task.module==REGION_MODULE_SOURCE;
 fault=source?(g_source_cycle_fault || g_source_cycle_state==9U):
       (g_stage3_fault || g_stage3_state==9U || g_stage3_run!=rm_task.run_id);
 done=source?(g_source_cycle_run==rm_task.run_id && g_source_cycle_state==8U && g_source_cycle_item==3U):
      (g_stage3_state==6U && g_stage3_locked_run==rm_task.run_id && g_stage3_placed_count==3U &&
       (rm_task.module!=REGION_MODULE_ROUGH ? !g_stage3_recovered_count && g_stage3_phase==36U :
        g_stage3_recovered_count==3U && g_stage3_phase==36U));
 taskENTER_CRITICAL();
 if(rm_status==REGION_MODULE_BUSY){
  if(fault){rm_fault_after_stop=1U;rm_status=REGION_MODULE_STOPPING;}
  else if(done){rm_status=REGION_MODULE_DONE;
#if APP_ZONE2_MISSION_ENABLE
   if(rm_departing)g_region_return_done_ms=now;
#endif
  }
 }
 taskEXIT_CRITICAL();
}
static uint32_t RegionDispatch_SharedByte(uint8_t b,uint32_t now,uint32_t binary_active)
{return g_region_dispatch_area==3U?Stage3_SharedByte(b,now,binary_active):SourceCycle_SharedByte(b,now,binary_active);}
static void RegionDispatch_Telemetry(uint32_t now)
{if(g_region_dispatch_area==3U)Stage3_Telemetry(now);else SourceCycle_Telemetry(now);}

#if APP_ZONE2_MISSION_ENABLE
/* 真实导航停车后推进原有姿态票据，使停车前图像失效；
 * 即使未发出非零纠偏也执行，香橙派已有帧围栏跟随该票据。 */
void RegionDispatch_StationStopped(uint32_t ticket)
{
 if(!ticket || ticket!=map_ticket || rm_bound!=1U || rm_status!=REGION_MODULE_BUSY)return;
 if(rm_task.module==REGION_MODULE_SOURCE && g_source_cycle_state==2U){
  sc_stream_started=0U;SC_Pose();
 }else if(rm_task.module!=REGION_MODULE_SOURCE && g_stage3_state==2U){
  s3_stream_started=0U;S3_Pose();
 }
}
uint32_t RegionDispatch_ParallelFaulted(void)
{return (AppZone2_StationApproaching() || rm_departing) && RM_Faulted();}
uint32_t RegionDispatch_StopServiceNeeded(void){return rm_status==REGION_MODULE_STOPPING;}
uint32_t RegionDispatch_SourceDepartureCommitted(void)
{return rm_departing && rm_task.module==REGION_MODULE_SOURCE && map_ticket==g_source_cycle_run;}
uint32_t RegionDispatch_MotionAllowed(void)
{
 if(!map_ticket || rm_status!=REGION_MODULE_BUSY || RM_Faulted())return 0U;
 if(!rm_bound)return !SourceBodyStep_Busy() && AppZone2_StationApproaching() && it_at_top() &&
  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)<=SOURCE_EXTENSION_ZERO_DONE_DEG;
 if(rm_bound!=1U)return 0U;
 if(rm_departing){
  if(rm_task.module==REGION_MODULE_SOURCE)return
   g_source_cycle_state==7U && g_source_cycle_item==2U && low_last_piece &&
   ((Low_Busy() && (g_low_phase==7U || g_low_phase==8U) && Low_DepartureClear()) ||
    (g_low_state==2U && g_low_phase==10U));
  return g_stage3_state==5U && g_stage3_phase>=33U && g_stage3_phase<=36U;
 }
 return !SourceBodyStep_Busy() && AppZone2_StationApproaching() && (rm_preturn ||
  (rm_task.module==REGION_MODULE_SOURCE?g_source_cycle_state==1U:
   (g_stage3_state==0U || g_stage3_state==6U || g_stage3_state==1U)));
}
uint32_t RegionDispatch_MotionRequestAllowed(uint32_t request)
{
 if(!RegionDispatch_MotionAllowed())return 0U;
 if(!request || request==9U)return 1U;
 if(rm_departing)return request==3U;
 return request==3U || (!rm_preturn && (request==6U || request==7U ||
        (request==10U && rm_task.module!=REGION_MODULE_SOURCE)));
}
uint32_t RegionDispatch_ParallelAxesAllowed(uint32_t axes,float yaw,float extension)
{
 float observe=rm_task.module==REGION_MODULE_SOURCE?180.0f:g_stage3_observe_yaw;
 if(!RegionDispatch_MotionAllowed() || !it_at_top() || !axes || (axes&~(SPF_DM|SPF_EXT)))return 0U;
 if(rm_departing)return (!(axes&SPF_DM) || yaw==0.0f) &&
                            (!(axes&SPF_EXT) || extension==0.0f);
 return (!(axes&SPF_DM) || yaw==observe) && (!(axes&SPF_EXT) || extension==0.0f ||
         (rm_task.module==REGION_MODULE_SOURCE && extension==65.0f));
}
uint32_t RegionDispatch_ApproachTurnAllowed(void)
{return rm_preturn && RegionDispatch_ParallelAxesAllowed(g_it_axes,g_it_dm_deg,g_it_extension_mm);}
uint32_t RegionDispatch_ObservationReady(uint32_t ticket,uint32_t *stamp)
{
 uint32_t ready;
 if(!stamp || !ticket || ticket!=map_ticket || rm_preturn || rm_departing || RM_Faulted() ||
    g_it_request || g_it_state!=2U || VehicleArm_Busy() || VehicleSourceParallel_Owns())return 0U;
 ready=rm_task.module==REGION_MODULE_SOURCE?
  (g_source_cycle_state==2U && g_source_cycle_prep>=5U && g_source_cycle_run==ticket):
  (g_stage3_state==2U && s3_module_run==ticket);
 if(!ready || !it_at_top() || !it_dm_fresh() || !g_dm_posvel_holding ||
    fabsf(g_dm_can2_logical_position_deg-(rm_task.module==REGION_MODULE_SOURCE?180.0f:g_stage3_observe_yaw))>3.0f ||
    fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg-
      STARTUP_EXTENSION_EXTEND_SIGN*(rm_task.module==REGION_MODULE_SOURCE?65.0f:0.0f)*
      VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f)>
      (rm_task.module==REGION_MODULE_SOURCE?SOURCE_EXTENSION_DONE_DEG:SOURCE_EXTENSION_ZERO_DONE_DEG))return 0U;
 if(rm_observe_ticket!=ticket){rm_observe_ticket=ticket;rm_observe_stamp=(uint32_t)xTaskGetTickCount();}
 *stamp=rm_observe_stamp;return 1U;
}
uint32_t RegionDispatch_WantsService(void)
{
 AppZone2Station station;
 return g_region_startup_reference_state==1U || map_ticket || AppZone2_StationRead(&station);
}

/* Active chassis may consume visual observations, but never enters the
 * mechanical dispatcher or schedules the next grasp from this path. */
void RegionDispatch_AlignmentService(void)
{
 AppZone2Station station;
 uint32_t source,now=(uint32_t)xTaskGetTickCount();
 if(!g_chassis_auto_active || !map_ticket || rm_bound!=1U || rm_status!=REGION_MODULE_BUSY ||
    !AppZone2_StationRead(&station) || station.ticket!=map_ticket || RM_Faulted())return;
 source=station.kind==APP_ZONE2_SOURCE;
 if(source ? (!sc_bound_batch || g_source_cycle_run!=map_ticket ||
              g_source_cycle_state!=2U || g_source_cycle_request) :
             (s3_module_run!=map_ticket || g_stage3_state!=2U ||
              g_stage3_request || g_stage3_replay_state))return;
 g_src_now_ms=now;
 Vision_Poll();
 /* Stream alignment in state 2 only. Normal mechanical service remains idle-only. */
 if(!g_it_request && (source?!g_source_cycle_request:!g_stage3_request)){
  if(source)SourceCycle_Service(now);else Stage3_Service(now);
 }
}

void RegionDispatch_MapService(uint32_t now)
{
 AppZone2Station station;
 RegionModuleRequest request;
 RegionModuleStatus state;
 uint32_t i;
 if(rm_departing){
  state=RegionDispatch_Poll(map_ticket);
  if(state==REGION_MODULE_FAULT || state==REGION_MODULE_STOPPED){
   (void)RegionDispatch_StopActive();return;
  }
  if(state!=REGION_MODULE_DONE)return;
  rm_departing=0U;map_ticket=0U;
 }
 if(!AppZone2_StationRead(&station)){
  if(map_ticket){
   (void)RegionDispatch_Stop(map_ticket);
   state=RegionDispatch_Poll(map_ticket);
   if(state!=REGION_MODULE_BUSY && state!=REGION_MODULE_STOPPING)map_ticket=0U;
  }
  return;
 }
 if(!map_ticket){
  memset(&request,0,sizeof(request));
  request.run_id=station.ticket;request.batch_id=station.batch;
  switch(station.kind){
  case APP_ZONE2_SOURCE: request.module=REGION_MODULE_SOURCE;break;
  case APP_ZONE2_ROUGH: request.module=REGION_MODULE_ROUGH;break;
  case APP_ZONE2_STORAGE_BASE: request.module=REGION_MODULE_STORAGE_BASE;break;
  case APP_ZONE2_STORAGE_STACK: request.module=REGION_MODULE_STACK;break;
  default: (void)AppZone2_StationComplete(station.ticket,0U);return;
  }
  for(i=0U;i<3U;i++){
   request.colors[i]=request.slots[i]=station.colors[i];
   request.rings[i]=request.module==REGION_MODULE_SOURCE?i+1U:station.rings[i];
  }
  if(!RM_Valid(&request) || RM_Faulted()){
   (void)AppZone2_StationComplete(station.ticket,0U);return;
  }
  if(!RegionDispatch_Begin(&request))return;
  map_ticket=station.ticket;
 }
 if(map_ticket!=station.ticket)return;
 /* 第三件释放并完成实际退让后登记原有回执；
  * 底盘可离站，转臂回位仍由当前机构任务继续服务。 */
 if(!rm_departing && !RM_Faulted() && !SourceBodyStep_Busy()){
  uint32_t source_clear=rm_task.module==REGION_MODULE_SOURCE && sc_bound_batch &&
   g_source_cycle_state==7U && g_source_cycle_item==2U && Low_Busy() && low_last_piece &&
   (g_low_phase==7U || g_low_phase==8U) && Low_DepartureClear() &&
   g_low_release_wait_ms>=SOURCE_RELEASE_DWELL_MS;
  uint32_t stage_clear=rm_task.module!=REGION_MODULE_SOURCE && g_stage3_state==5U &&
   g_stage3_phase==33U && g_stage3_placed_count==3U && g_stage3_item==3U &&
   (rm_task.module!=REGION_MODULE_ROUGH || g_stage3_recovered_count==3U) &&
   !s3_pending && !g_it_request && g_it_state==2U && it_at_top();
  if(source_clear){
   uint8_t color=(uint8_t)g_source_cycle_colors[2];
   if(!AppZone2_StationConfirm(map_ticket,2U,APP_ZONE2_SOURCE_GRIP,color,0U) ||
      !AppZone2_StationConfirm(map_ticket,2U,APP_ZONE2_SOURCE_SLOT,color,0U)){
    SC_Fail(12U);return;
   }
  }
  if(source_clear || stage_clear){
   uint32_t accepted;
#if APP_ZONE2_UNVERIFIED_TRIAL_ENABLE
   if(rm_task.module==REGION_MODULE_STORAGE_BASE || rm_task.module==REGION_MODULE_STACK)
    accepted=AppZone2_StationCompleteUnverified(map_ticket);
   else
#endif
    accepted=AppZone2_StationComplete(map_ticket,1U);
   if(!accepted){if(source_clear)SC_Fail(12U);return;}
   rm_departing=1U;g_region_departure_ms=now;return;
  }
 }
 state=RegionDispatch_Poll(map_ticket);
 if(state==REGION_MODULE_DONE || state==REGION_MODULE_FAULT || state==REGION_MODULE_STOPPED){
#if APP_ZONE2_UNVERIFIED_TRIAL_ENABLE
  if(state==REGION_MODULE_DONE &&
     (station.kind==APP_ZONE2_STORAGE_BASE || station.kind==APP_ZONE2_STORAGE_STACK))
   (void)AppZone2_StationCompleteUnverified(map_ticket);
  else
#endif
   (void)AppZone2_StationComplete(map_ticket,state==REGION_MODULE_DONE);
  map_ticket=0U;
 }
}
#else
uint32_t RegionDispatch_WantsService(void){return 0U;}
#endif
