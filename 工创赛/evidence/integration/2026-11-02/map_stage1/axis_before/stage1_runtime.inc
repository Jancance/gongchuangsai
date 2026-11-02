/* 第一阶段对现有底盘执行器的接线层；只由main.c包含一次。
 * 状态0待机1移动2扫码3完成4失败5取消。所有业务转换由任务A拥有。
 * PC13负责提交启动/取消，USART1只观察，不靠电脑心跳许可运行。 */
volatile uint32_t g_stage1_state=0U,g_stage1_cause=0U,g_stage1_run_id=0U;
volatile uint32_t g_stage1_key_request=0U,g_stage1_denied=0U,g_stage1_scan_valid=0U;
static uint32_t s_stage1_begin_ms,s_stage1_end_ms,s_stage1_scan_ms;
static uint32_t s_stage1_scan_baseline,s_stage1_display_baseline;
static FieldStage1Plan s_stage1_plan;
char g_stage1_task_code[16]={0};

/* 仅正常完成且停稳关VM后重开；不清系统故障/取消锁存。 */
static uint32_t App_Stage1Rearm(void)
{
    ChassisAutoStatus status;ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(g_stage1_state!=3U || g_combined_test.state!=3U || g_combined_test.cause ||
       g_combined_test.completed!=MOTION_ACTION_COUNT || s_motion_session_open ||
       g_forward_hold.trip || g_heading_exec_trip || g_serial_motion_link_lost) return 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if(!ChassisAuto_RearmCompleted(status.session)) return 0U;
    g_combined_test=combined;g_forward_hold=forward;
    g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
    HeadingSession_Init(&s_heading_exec);
    s_motion_has_action=s_combined_turn=s_combined_powered_segment=s_combined_keep_vm=0U;
    s_serial_start_permit=0U;g_serial_motion_state=g_serial_motion_result=0U;
    s_route_heading_reference=(RouteHeadingReference){0};s_route_heading_turn_active=0U;
    s_perimeter_odom_started=s_perimeter_completed_seen=0U;
    memset((void*)g_perimeter_records,0,sizeof(g_perimeter_records));
    g_stage1_state=0U;g_stage1_scan_valid=0U; /* Normal preparation may precede a temporarily-not-ready heading source. */
    return 1U;
}
/* 先校验停车后的真实接收时间，再交给串口屏；旧码/坏码不改屏幕。 */
static uint32_t App_Stage1FreshCode(uint32_t now)
{
    return g_stage1_state==2U && g_saoma_accept_count!=s_stage1_scan_baseline &&
        g_saoma_last_result==1U &&
        (uint32_t)(g_saoma_frame_start_ms-s_stage1_scan_ms)>0U &&
        (uint32_t)(g_saoma_frame_start_ms-s_stage1_scan_ms)<0x80000000U &&
        (uint32_t)(now-g_saoma_frame_end_ms)<VEHICLE_STAGE1_SCAN_FRESH_MS &&
        FieldMap_TaskCodeValid(SAOMA_Data);
}
static void App_Stage1Service(void)
{
#if VEHICLE_STAGE1_ENABLE
    uint32_t key,now=(uint32_t)xTaskGetTickCount();ChassisAutoStatus status;PowerCommandStatus power;
    taskENTER_CRITICAL();key=g_stage1_key_request;g_stage1_key_request=0U;taskEXIT_CRITICAL();
    if(key==2U && (g_stage1_state==1U || g_stage1_state==2U)) {
        if(g_stage1_state==1U) MotionStop();
        g_stage1_cause=4U;s_stage1_end_ms=now;g_stage1_state=5U;return;
    }
    if(key==1U) {
        if(g_stage1_state!=0U && g_stage1_state!=3U) {g_stage1_denied++;return;}
        if(!FieldMap_Stage1(&s_stage1_plan)) {g_stage1_cause=10U;g_stage1_state=4U;return;}
        taskENTER_CRITICAL();
        if(g_radar_zone_ready!=1U || g_radar_zone_selected!=VEHICLE_STAGE1_ZONE ||
           g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
           (g_stage1_state==3U && !App_Stage1Rearm()) || !App_SerialIdleReady(now))
            {g_stage1_denied++;taskEXIT_CRITICAL();return;}
        /* 第一阶段里程观察使用起步车头局部坐标，不继承旧绕圈绝对目标。 */
        memset(s_perimeter_targets,0,sizeof(s_perimeter_targets));
        s_perimeter_targets[1][1]=s_stage1_plan.left_cm*10.0f;
        s_serial_start_permit=1U;g_combined_test.request=1U;
        g_serial_motion_state=2U;g_serial_motion_result=2U;g_serial_motion_accepted++;
        g_stage1_run_id++;g_stage1_cause=0U;g_stage1_scan_valid=0U;
        memset(g_stage1_task_code,0,sizeof(g_stage1_task_code));
        s_stage1_begin_ms=now;s_stage1_end_ms=0U;g_stage1_state=1U;
        taskEXIT_CRITICAL();
    }
    if(g_stage1_state==1U) {
        if(g_combined_test.state>=4U) {
            g_stage1_cause=2U;s_stage1_end_ms=now;
            g_stage1_state=(g_combined_test.state==5U)?5U:4U;return;
        }
        if((uint32_t)(now-s_stage1_begin_ms)>=VEHICLE_STAGE1_MOVE_TIMEOUT_MS) {
            MotionStop();g_stage1_cause=6U;s_stage1_end_ms=now;g_stage1_state=4U;return;
        }
        if(g_combined_test.state==3U) {
            (void)ChassisAuto_ReadStatus(&status);Power_ReadCommandStatus(&power);
            if(status.active || status.result!=1U || status.cause || status.rotation_stop_mask!=15U ||
               power.active || g_vm_test_request) return;
            s_stage1_scan_baseline=g_saoma_accept_count;s_stage1_display_baseline=g_scan_display_count;
            s_stage1_scan_ms=now;
            if(VEHICLE_STAGE1_SCAN_AFTER_STOP) g_stage1_state=2U;
            else {g_stage1_state=3U;s_stage1_end_ms=now;}
        }
    }
    if(g_stage1_state==2U) {
        if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched) {
            g_stage1_cause=5U;s_stage1_end_ms=now;g_stage1_state=4U;return;
        }
        /* 拒绝路上或上次旧码：停车后新合法码，并有新显示发送成功记录。 */
        if(App_Stage1FreshCode(now) && g_scan_display_count!=s_stage1_display_baseline &&
           g_scan_display_result==SCREEN_OK) {
            memcpy(g_stage1_task_code,SAOMA_Data,sizeof(g_stage1_task_code));
            g_stage1_scan_valid=1U;s_stage1_end_ms=now;g_stage1_state=3U;
        } else if((uint32_t)(now-s_stage1_scan_ms)>=VEHICLE_STAGE1_SCAN_TIMEOUT_MS) {
            g_stage1_cause=3U;s_stage1_end_ms=now;g_stage1_state=4U;
        }
    }
#endif
}
static void App_Stage1Telemetry(void)
{
#if VEHICLE_STAGE1_ENABLE
    static char frame[320];static uint32_t last,sequence,dropped;
    uint32_t now=(uint32_t)xTaskGetTickCount(),elapsed;int count;uint16_t crc;
    if((uint32_t)(now-last)<200U || !App_SerialDiagnosticWindow(now,20U)) return;
    last=now;elapsed=g_stage1_state==0U?0U:((g_stage1_state>=3U?s_stage1_end_ms:now)-s_stage1_begin_ms);
    count=snprintf(frame,sizeof(frame),"ST1,%lu,%lu,1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)sequence++,(unsigned long)now,(unsigned long)g_stage1_run_id,
        (unsigned long)g_stage1_state,(unsigned long)VEHICLE_STAGE1_ZONE,(unsigned long)g_combined_test.index,
        (unsigned long)g_combined_test.completed,(unsigned long)MOTION_ACTION_COUNT,
        (unsigned long)s_stage1_plan.start.x_mm,(unsigned long)s_stage1_plan.start.y_mm,
        (unsigned long)s_stage1_plan.stop.x_mm,(unsigned long)s_stage1_plan.stop.y_mm,
        (unsigned long)s_stage1_plan.config_id,(unsigned long)g_stage1_scan_valid,
        (unsigned long)g_stage1_cause,(unsigned long)elapsed,(unsigned long)g_chassis_auto_session,
        (unsigned long)g_stage1_denied,(unsigned long)dropped,(unsigned long)s_stage1_plan.nominal_scan_gap_mm);
    if(count<0 || (uint32_t)count>=sizeof(frame)) {dropped++;return;}
    crc=App_SerialCrc(frame,(uint32_t)count);
    count=snprintf(frame+count,sizeof(frame)-(uint32_t)count,"%04X\r\n",crc);
    if(count<0 || !App_DebugSendString(frame)) dropped++;
#endif
}

/* 任务码单独带CRC与run/config标识回传；不是无校验SCAN文字的推测值。 */
static void App_Stage1CodeTelemetry(void)
{
#if VEHICLE_STAGE1_ENABLE
    static char frame[96];static uint32_t sent_run;
    uint32_t now=(uint32_t)xTaskGetTickCount();int count;uint16_t crc;
    if(!g_stage1_scan_valid || !g_stage1_run_id || sent_run==g_stage1_run_id ||
       !App_SerialDiagnosticWindow(now,40U)) return;
    count=snprintf(frame,sizeof(frame),"SC1,%lu,%lu,1,%lu,%lu,%s,",
        (unsigned long)g_stage1_run_id,(unsigned long)now,(unsigned long)g_stage1_run_id,
        (unsigned long)s_stage1_plan.config_id,g_stage1_task_code);
    if(count<0 || (uint32_t)count>=sizeof(frame)) return;
    crc=App_SerialCrc(frame,(uint32_t)count);
    (void)snprintf(frame+count,sizeof(frame)-(uint32_t)count,"%04X\r\n",crc);
    if(App_DebugSendString(frame)) sent_run=g_stage1_run_id;
#endif
}
