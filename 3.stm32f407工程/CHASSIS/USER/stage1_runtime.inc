/* 第一阶段：本轮有效码可在途中锁存；0待机1移动2等码3完成4失败5取消6扫码停车。
 * 请求停车不等于停稳，显示串口发送成功也不等于屏幕可视验收。
 * 第二阶段当前只有带run_id的交接状态，不发未实现的路线/机械臂动作。 */
volatile uint32_t g_stage1_state=0U,g_stage1_cause=0U,g_stage1_run_id=0U;
volatile uint32_t g_stage1_key_request=0U,g_stage1_denied=0U,g_stage1_scan_valid=0U;
volatile uint32_t g_stage2_ready=0U,g_stage2_run_id=0U;
volatile uint32_t g_stage1_scan_bypassed=0U;
volatile uint32_t g_stage1_display_attempts=0U;
static uint32_t s_stage1_begin_ms,s_stage1_end_ms,s_stage1_scan_ms;
static uint32_t s_stage1_scan_baseline,s_stage1_stop_ms,s_stage1_display_ms;
static uint32_t s_stage1_display_done,s_stage1_code_ms;
static FieldStage1Plan s_stage1_plan;
char g_stage1_task_code[16]={0};

static uint32_t App_Stage1Rearm(void)
{
    ChassisAutoStatus status;ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(g_stage1_stop_requested || g_stage2_ready || g_stage1_state!=3U ||
       g_combined_test.state!=3U || g_combined_test.cause ||
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
    g_stage1_state=0U;g_stage1_scan_valid=0U;
    return 1U;
}
static uint32_t App_Stage1FreshCode(uint32_t now)
{
    return (g_stage1_state==1U || g_stage1_state==2U) && !g_stage1_scan_bypassed && !g_stage1_scan_valid &&
        g_saoma_accept_count!=s_stage1_scan_baseline && g_saoma_last_result==1U &&
        (uint32_t)(g_saoma_frame_start_ms-s_stage1_begin_ms)>0U &&
        (uint32_t)(g_saoma_frame_start_ms-s_stage1_begin_ms)<0x80000000U &&
        (uint32_t)(now-g_saoma_frame_end_ms)<VEHICLE_STAGE1_SCAN_FRESH_MS &&
        FieldMap_TaskCodeValid(SAOMA_Data);
}
static uint32_t App_Stage1Stopped(void)
{
    ChassisAutoStatus status;PowerCommandStatus power;
    (void)ChassisAuto_ReadStatus(&status);Power_ReadCommandStatus(&power);
    if(g_combined_test.state!=3U || g_combined_test.cause || s_motion_session_open ||
       g_forward_hold.trip || g_heading_exec_trip || status.active ||
       (power.active && !StartupAxes_Holding()) || g_vm_test_request || g_safety_fault || g_rtos_fault ||
       g_can_ctrl_fault_latched) return 0U;
    if(g_stage1_motion_skipped) return status.state==CHASSIS_AUTO_IDLE && !status.result && !status.cause;
    return status.state==CHASSIS_AUTO_DONE && status.result==1U && !status.cause &&
        status.rotation_stop_mask==15U && g_combined_test.index<MOTION_ACTION_COUNT &&
        status.session==g_combined_test.session[g_combined_test.index];
}
static void App_Stage1Fail(uint32_t cause,uint32_t now)
{
    g_stage2_ready=0U;g_stage1_cause=cause;s_stage1_end_ms=now;g_stage1_state=4U;
}
static void App_Stage1Service(void)
{
#if VEHICLE_STAGE1_ENABLE
    uint32_t key,now=(uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();key=g_stage1_key_request;g_stage1_key_request=0U;taskEXIT_CRITICAL();
    if(key==2U && (g_stage1_state==1U || g_stage1_state==2U || g_stage1_state==6U || g_stage1_state==7U)) {
        StartupAxes_Cancel();
        if(g_stage1_state==1U || g_stage1_state==6U) MotionStop();
        g_stage2_ready=0U;g_stage1_cause=4U;s_stage1_end_ms=now;g_stage1_state=5U;return;
    }
    if(key==1U) {
        if(g_stage1_state!=0U && g_stage1_state!=3U) {g_stage1_denied++;return;}
        if(!FieldMap_Stage1(&s_stage1_plan)) {App_Stage1Fail(10U,now);return;}
        taskENTER_CRITICAL();
        if(g_radar_zone_ready!=1U || g_radar_zone_selected!=VEHICLE_STAGE1_ZONE ||
           g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
           (g_stage1_state==3U && !App_Stage1Rearm()) || !App_SerialIdleReady(now))
            {g_stage1_denied++;taskEXIT_CRITICAL();return;}
        memset(s_perimeter_targets,0,sizeof(s_perimeter_targets));
        s_perimeter_targets[1][0]=s_stage1_plan.forward_cm*10.0f;
        s_perimeter_targets[2][0]=s_stage1_plan.forward_cm*10.0f;
        s_perimeter_targets[2][1]=s_stage1_plan.left_cm*10.0f;
        g_stage1_stop_requested=g_stage1_motion_skipped=0U;
        s_serial_start_permit=0U;g_combined_test.request=0U;
        g_serial_motion_state=2U;g_serial_motion_result=2U;g_serial_motion_accepted++;
        g_stage1_run_id++;if(!g_stage1_run_id)g_stage1_run_id=1U;
        g_stage1_cause=g_stage1_scan_valid=g_stage2_ready=g_stage2_run_id=0U;
        g_stage1_display_attempts=s_stage1_display_done=0U;
        memset(g_stage1_task_code,0,sizeof(g_stage1_task_code));
        s_stage1_scan_baseline=g_saoma_accept_count;
        s_stage1_begin_ms=now;s_stage1_end_ms=0U;g_stage1_state=7U;
        g_stage1_scan_bypassed=VEHICLE_SCAN_BYPASS;
        g_serial_motion_state=0U;g_serial_motion_result=0U;
        if(!StartupAxes_Begin()){g_stage1_state=4U;g_stage1_cause=9U;}
        taskEXIT_CRITICAL();
    }
    if(g_stage1_state==7U) {
        if(g_startup_axes_state==5U || g_startup_axes_state==6U || g_startup_axes_state==7U || g_startup_axes_state==8U)
        {App_Stage1Fail(9U,now);return;}
        if(StartupAxes_Holding()) {
            if(!App_SerialIdleReady(now)){App_Stage1Fail(9U,now);return;}
            s_serial_start_permit=1U;g_combined_test.request=1U;
            g_serial_motion_state=2U;g_serial_motion_result=2U;
            s_stage1_begin_ms=now;s_stage1_scan_baseline=g_saoma_accept_count;g_stage1_state=1U;
        } else return;
    }
    if(g_stage1_state==1U || g_stage1_state==2U || g_stage1_state==6U || g_stage1_state==7U) {
        if(g_startup_axes_state==5U || g_startup_axes_state==6U){MotionStop();App_Stage1Fail(9U,now);return;}
        if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || g_serial_motion_link_lost) {
            if(g_stage1_state!=2U)MotionStop();App_Stage1Fail(5U,now);return;
        }
        if(g_combined_test.state>=4U) {App_Stage1Fail(2U,now);return;}
        if(!g_stage1_scan_bypassed && App_Stage1FreshCode(now)) {
            memcpy(g_stage1_task_code,SAOMA_Data,sizeof(g_stage1_task_code));
            SAOMA_DataReady=0U;g_stage1_scan_valid=1U;s_stage1_code_ms=now;
            if(g_stage1_state==1U && !App_Stage1Stopped()) {
                taskENTER_CRITICAL();g_stage1_stop_requested=1U;taskEXIT_CRITICAL();
                s_stage1_stop_ms=now;g_stage1_state=6U;
            }
        }
    }
    if(g_stage1_state==1U) {
        if((uint32_t)(now-s_stage1_begin_ms)>=VEHICLE_STAGE1_MOVE_TIMEOUT_MS) {
            MotionStop();App_Stage1Fail(6U,now);return;
        }
        if(App_Stage1Stopped()) {s_stage1_scan_ms=now;g_stage1_state=2U;}
    }
    if(g_stage1_state==6U) {
        if(App_Stage1Stopped()) {s_stage1_scan_ms=now;g_stage1_state=2U;}
        else if((uint32_t)(now-s_stage1_stop_ms)>=VEHICLE_STAGE1_QR_STOP_TIMEOUT_MS) {
            MotionStop();App_Stage1Fail(8U,now);return;
        }
    }
    if(g_stage1_state==2U) {
        if((g_stage1_scan_bypassed || (g_stage1_scan_valid && s_stage1_display_done)) && App_Stage1Stopped()) {
            g_stage1_cause=0U;s_stage1_end_ms=now;g_stage1_state=3U;
            g_stage2_run_id=g_stage1_run_id;g_stage2_ready=1U;
        } else if(!g_stage1_scan_bypassed && !g_stage1_scan_valid &&
                  (uint32_t)(now-s_stage1_scan_ms)>=VEHICLE_STAGE1_SCAN_TIMEOUT_MS) {
            App_Stage1Fail(3U,now);
        }
    }
    /* 故障或停车证明失效后撤销交接，保留原任务码供取证。 */
    if(g_stage2_ready && !App_Stage1Stopped()) {App_Stage1Fail(5U,now);}
#endif
}
static void App_Stage1Display(uint32_t screen_ready)
{
#if VEHICLE_STAGE1_ENABLE
    uint32_t now=(uint32_t)xTaskGetTickCount();
    if(!screen_ready || !g_stage1_scan_valid || s_stage1_display_done ||
       (g_stage1_state!=2U && g_stage1_state!=6U)) return;
    /* 先完成有序停车，屏幕发送不抢占第一阶段运动决策。 */
    if(!App_Stage1Stopped())return;
    if(g_stage1_display_attempts && (uint32_t)(now-s_stage1_display_ms)<250U)return;
    if(g_stage1_display_attempts>=3U || (uint32_t)(now-s_stage1_code_ms)>=VEHICLE_STAGE1_SCAN_TIMEOUT_MS)
    {App_Stage1Fail(7U,now);return;}
    s_stage1_display_ms=now;g_stage1_display_attempts++;
    g_scan_display_result=Screen_ShowTaskCode(g_stage1_task_code);
    if(g_scan_display_result==SCREEN_OK){g_scan_display_count++;s_stage1_display_done=1U;}
#else
    (void)screen_ready;
#endif
}
static void App_Stage1Telemetry(void)
{
#if VEHICLE_STAGE1_ENABLE
    static char frame[320];static uint32_t last,sequence,dropped;
    uint32_t now=(uint32_t)xTaskGetTickCount(),elapsed;int count;uint16_t crc;
    if((uint32_t)(now-last)<200U || !App_SerialDiagnosticWindow(now,20U)) return;
    last=now;elapsed=g_stage1_state==0U?0U:(((g_stage1_state>=3U && g_stage1_state<=5U)?s_stage1_end_ms:now)-s_stage1_begin_ms);
    count=snprintf(frame,sizeof(frame),"ST1,%lu,%lu,1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)sequence++,(unsigned long)now,(unsigned long)g_stage1_run_id,
        (unsigned long)g_stage1_state,(unsigned long)VEHICLE_STAGE1_ZONE,(unsigned long)g_combined_test.index,
        (unsigned long)g_combined_test.completed,(unsigned long)MOTION_ACTION_COUNT,
        (unsigned long)s_stage1_plan.start.x_mm,(unsigned long)s_stage1_plan.start.y_mm,
        (unsigned long)s_stage1_plan.stop.x_mm,(unsigned long)s_stage1_plan.stop.y_mm,
        (unsigned long)s_stage1_plan.config_id,(unsigned long)g_stage1_scan_valid,
        (unsigned long)g_stage1_cause,(unsigned long)elapsed,(unsigned long)g_chassis_auto_session,
        (unsigned long)g_stage1_denied,(unsigned long)dropped,(unsigned long)(s_stage1_plan.nominal_scan_gap_mm*10.0f));
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
