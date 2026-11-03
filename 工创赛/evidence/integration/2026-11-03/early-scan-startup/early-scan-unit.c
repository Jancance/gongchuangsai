
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "field_map.h"
#include "app_entry.h"
#include "route_plan.h"
extern volatile RouteCheck g_route_check;
volatile uint32_t checks,failures,failure_checks[16];
float exported_centers[25][2],exported_parks[25][2],exported_plan[7],exported_actions[2][4];
uint32_t exported_config,exported_run_ms,exported_rpm;
char accepted_frames[1024];uint32_t accepted_used;
static void ck(uint32_t good){checks++;if(!good){if(failures<16)failure_checks[failures]=checks;failures++;}}
typedef struct {uint32_t trip,phase,request,run_started,backend_session;float error_deg;} ForwardHoldDebug;
typedef struct {uint32_t request,state,index,cause,completed,phase[8],session[8],elapsed_ms[8];float final_yaw[8],final_error[8];} CombinedTestDebug;
typedef struct {uint32_t active,result,cause,rotation_stop_mask,session,state,distance_reached,cut_vm;} ChassisAutoStatus;
typedef struct {uint32_t active;} PowerCommandStatus;
typedef struct {uint32_t valid;} RouteHeadingReference;
static uint32_t tick,ready,rearm_ok,stop_calls,tx_ok;
volatile CombinedTestDebug g_combined_test;
volatile ForwardHoldDebug g_forward_hold;
static ChassisAutoStatus fixture_backend;static PowerCommandStatus fixture_power;
uint32_t g_heading_exec_trip,g_heading_exec_phase,g_heading_exec_state,g_heading_exec_request,s_heading_exec;
uint32_t s_motion_session_open,s_motion_has_action,s_combined_turn,s_combined_powered_segment,s_combined_keep_vm;
uint32_t s_serial_start_permit,g_serial_motion_state,g_serial_motion_result,g_serial_motion_accepted,g_serial_motion_link_lost;
uint32_t s_route_heading_turn_active,s_perimeter_odom_started,s_perimeter_completed_seen,g_perimeter_records[9];
float s_perimeter_targets[9][3];RouteHeadingReference s_route_heading_reference;
uint32_t g_radar_zone_ready,g_radar_zone_selected,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
uint32_t g_vm_test_request,g_saoma_accept_count,g_scan_display_count,g_saoma_last_result,g_scan_display_result;
uint32_t g_saoma_frame_start_ms,g_saoma_frame_end_ms,g_chassis_auto_session;
char SAOMA_Data[16];
#define MOTION_ACTION_COUNT 2U
#define HR_IDLE 0U
#define HS_IDLE 0U
#define SCREEN_OK 1U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t xTaskGetTickCount(void){return tick;}
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *p){*p=fixture_backend;return 1;}
static void Power_ReadCommandStatus(PowerCommandStatus *p){*p=fixture_power;}
static uint32_t ChassisAuto_RearmCompleted(uint32_t session){if(!rearm_ok)return 0;fixture_backend.result=0;return 1;}
static void HeadingSession_Init(void *p){memset(p,0,4);}
static uint32_t App_SerialIdleReady(uint32_t now){return !APP_MOTOR_OFFLINE_ONLY && ready && !g_combined_test.state;}
static void MotionStop(void){stop_calls++;g_combined_test.state=5;g_vm_test_request=0;}
static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t ms){return 1;}
static uint16_t App_SerialCrc(const char *p,uint32_t n){uint16_t c=0xffff;uint32_t i,j;for(i=0;i<n;i++){c^=(uint16_t)(uint8_t)p[i]<<8;for(j=0;j<8;j++)c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1);}return c;}
static uint32_t App_DebugSendString(const char *p){uint32_t n=strlen(p);if(tx_ok && accepted_used+n<sizeof(accepted_frames)){memcpy(accepted_frames+accepted_used,p,n);accepted_used+=n;accepted_frames[accepted_used]=0;}return tx_ok;}

#define CHASSIS_AUTO_IDLE 0U
#define CHASSIS_AUTO_DONE 9U
uint32_t g_stage1_stop_requested,g_stage1_motion_skipped,SAOMA_DataReady;
uint32_t g_startup_axes_state;
static uint32_t StartupAxes_Holding(void){return g_startup_axes_state==4;}
static uint32_t StartupAxes_Begin(void){g_startup_axes_state=4;return 1;}
static void StartupAxes_Cancel(void){g_startup_axes_state=6;}

static uint32_t display_result=SCREEN_OK,display_calls;
static char displayed[16];
static uint32_t Screen_ShowTaskCode(const char *code){display_calls++;strcpy(displayed,code);return display_result;}
#include "stage1_runtime.inc"


#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
#define MOTION_BUSY 1U
#define MOTION_FAULT 4U
#define MOTION_CANCELLED 5U
#define APP_PERIMETER_GROUP_MS 300000U
static uint32_t s_combined_end_ms,s_combined_begin_ms,s_combined_stage_ms,s_combined_gap_ms;
static uint32_t s_motion_list_submit,g_translation_hold_selftest,g_motion_plan_valid;
static uint32_t g_heading_exec_phase_dummy,g_radar_servo_departure_done=1;
static float g_heading_exec_error_deg;
static struct {float yaw_deg;} s_heading_source;
static uint32_t load_calls,finish_calls,fixture_phase=HR_DONE;
uint32_t g_chassis_auto_request,g_can_jog_request;
static uint32_t App_HeadingRuntimeSource(uint32_t now){return 1;}
static uint32_t App_HeadingRuntimeExclusive(void){return 1;}
static uint32_t RouteHeading_Init(RouteHeadingReference *r,float f){r->valid=1;return 1;}
static uint32_t App_CombinedLoadAction(uint32_t index){load_calls++;return 1;}
static uint32_t MotionGetState(void){return 0;}
static float App_StageDistanceMm(void){return 150;}
static uint32_t App_StageRunMs(void){return 10000;}
static void App_CombinedTestStop(uint32_t cause){g_combined_test.state=cause==1?5:4;g_combined_test.cause=cause;}
static void App_HeadingRuntimeStep(void){g_heading_exec_phase=fixture_phase;}
static void App_ForwardHoldStep(void){g_forward_hold.phase=fixture_phase;}
static uint32_t RadarServo_OnDeparture(uint32_t zone){return 1;}
static uint32_t RadarZoneSwitch_Lock(void){return 1;}
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){return 1;}
static uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p){return 1;}
static uint32_t MotionFinish(void){finish_calls++;g_vm_test_request=0;s_motion_session_open=0;s_combined_keep_vm=0;return 1;}
static void App_TimingRecordCompleted(uint32_t a,uint32_t b,uint32_t c){}
static void App_CombinedTestStep(void)
{
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t now = (uint32_t)xTaskGetTickCount(), request, phase, index;
    taskENTER_CRITICAL();
    request = g_combined_test.request;
    g_combined_test.request = 0U;
    if (APP_IS_COMPETITION && !AppCompetitionEntry_MotionAllowed())
    {
        s_serial_start_permit=0U;
        g_forward_hold.request=g_heading_exec_request=0U;
        g_chassis_auto_request=g_can_jog_request=0U;
        g_vm_test_request=0U;
        taskEXIT_CRITICAL(); return;
    }
    if (request == 2U && (g_combined_test.state < 3U))
    { App_CombinedTestStop(1U); taskEXIT_CRITICAL(); return; }
    /* 启动请求尚未发给执行器时已取得码：撤销排队，不制造成功路程。 */
    if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested &&
        g_combined_test.state==0U && request==1U && !s_motion_session_open)
    {
        s_serial_start_permit=0U;g_stage1_motion_skipped=1U;
        g_combined_test.state=3U;s_combined_end_ms=now;
        taskEXIT_CRITICAL();return;
    }
    if (g_combined_test.state == 0U && s_motion_session_open)
    {
        /* 独立API不启动动作表，仍由唯一CAN任务服务。 */
        if (MotionGetState() == MOTION_BUSY)
        {
            if ((uint32_t)(now - s_combined_stage_ms) >=
                ((App_StageDistanceMm()>0.0f) ? (App_StageRunMs()+5000U) : 35000U))
            { App_CombinedTestStop(2U); taskEXIT_CRITICAL(); return; }
            taskEXIT_CRITICAL();
            if (s_combined_turn) App_HeadingRuntimeStep();
            else App_ForwardHoldStep();
            return;
        }
        if (MotionGetState() == MOTION_FAULT || MotionGetState() == MOTION_CANCELLED)
            App_CombinedTestStop(5U);
        taskEXIT_CRITICAL(); return;
    }
    if (g_combined_test.state == 0U)
    {
        /* 其他单段Watch入口在本模式不参与调度。整组每次上电只运行一次。 */
        g_forward_hold.request = 0U; g_heading_exec_request = 0U;
        if (request != 1U) { taskEXIT_CRITICAL(); return; }
        if (s_serial_start_permit != 1U)
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        s_serial_start_permit = 0U;
        (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
        if (status.active || status.result || status.cut_vm || (power.active && !StartupAxes_Holding()) ||
            g_vm_test_request || !App_HeadingRuntimeSource(now) ||
            !App_HeadingRuntimeExclusive() || g_can_ctrl_fault_latched ||
            (g_translation_hold_selftest != 1U) || (g_motion_plan_valid != 1U) ||
            (g_forward_hold.phase != HR_IDLE) || (g_heading_exec_phase != HR_IDLE))
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        if(!RouteHeading_Init(&s_route_heading_reference,s_heading_source.yaw_deg))
        { App_CombinedTestStop(3U);taskEXIT_CRITICAL();return; }
        s_combined_powered_segment = 0U;
        s_combined_begin_ms = s_combined_stage_ms = now;
        g_combined_test.index = 0U;
        if (!App_CombinedLoadAction(0U))
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        g_combined_test.state = 1U;
    }
    if ((g_combined_test.state < 3U) && (g_combined_test.index >= MOTION_ACTION_COUNT))
    { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state >= 3U)
    { g_forward_hold.request = g_heading_exec_request = 0U;
      g_vm_test_request = 0U; taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state == 2U)
    {
        /* 段间取得码后直接释放已停稳会话，不再装载下一段。 */
        if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested)
        {
            s_motion_list_submit=1U;(void)MotionFinish();s_motion_list_submit=0U;
            Power_ReadCommandStatus(&power);
            if(!g_vm_test_request && (!power.active || StartupAxes_Holding()))
            {g_combined_test.state=3U;s_combined_end_ms=now;}
            taskEXIT_CRITICAL();return;
        }
        /* 保留上一段诊断，仅本组成功且仍上电的停稳会话允许释放。 */
        if ((VEHICLE_BETWEEN_LEGS_MS && (uint32_t)(now - s_combined_gap_ms) < VEHICLE_BETWEEN_LEGS_MS))
        { taskEXIT_CRITICAL(); return; }
        Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
        if ((g_vm_test_request != 1U) || (power.active != 1U) ||
            status.session != g_combined_test.session[g_combined_test.index])
        { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
        g_combined_test.index++;
        index = g_combined_test.index;
        s_combined_powered_segment = 1U;
        s_combined_stage_ms = now;
        if (!App_CombinedLoadAction(index))
        { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
        g_combined_test.state = 1U;
    }
    if (g_combined_test.state == 1U)
    {
        if ((uint32_t)(now - s_combined_stage_ms) >=
                ((App_StageDistanceMm()>0.0f) ? (App_StageRunMs()+5000U) : 35000U))
        { App_CombinedTestStop(2U); taskEXIT_CRITICAL(); return; }
        /* 单段执行可能等待CAN，必须先退出临界区；Safety仍可抢占取消。 */
        taskEXIT_CRITICAL();
        if (s_combined_turn) App_HeadingRuntimeStep();
        else App_ForwardHoldStep();
        taskENTER_CRITICAL();
        if (g_combined_test.state >= 3U)
        { taskEXIT_CRITICAL(); return; }
        phase = s_combined_turn ? g_heading_exec_phase : g_forward_hold.phase;
        if (phase == HR_FAULT || phase == HR_CANCELLED)
        { App_CombinedTestStop(5U); taskEXIT_CRITICAL(); return; }
        /* 首段实际开始运行后，只展开PA1锁定的一套，不在上电或ARM时动作。 */
        if (g_combined_test.index==0U && phase==HR_RUN && !g_radar_servo_departure_done &&
            (s_combined_turn || g_forward_hold.run_started))
        {
            if (!RadarServo_OnDeparture(RadarZoneSwitch_Lock()))
            { App_CombinedTestStop(3U);taskEXIT_CRITICAL();return; }
        }
        if (phase == HR_DONE)
        {
            (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
            /* 正常DONE也会置cut_vm请求断电，此处等待关电而非判故障。 */
            if (status.active || status.state != CHASSIS_AUTO_DONE ||
                status.result != 1U || status.cause || status.rotation_stop_mask != 15U ||
                !(s_combined_turn ? App_HeadingRuntimeOwns(&status) : App_ForwardHoldOwns(&status)))
            { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
            if (g_stage1_stop_requested || (g_combined_test.index + 1U) == MOTION_ACTION_COUNT)
            {
                s_motion_list_submit = 1U;
                (void)MotionFinish();
                s_motion_list_submit = 0U;
            }
            /* 等安全任务确实发出关闭命令，不把VM请求0当作PE2已关。 */
            if ((!g_stage1_stop_requested && (g_combined_test.index + 1U) < MOTION_ACTION_COUNT) ?
                ((g_vm_test_request == 1U) && (power.active == 1U) && !status.cut_vm) :
                (!g_vm_test_request && (!power.active || StartupAxes_Holding())))
            {
                index = g_combined_test.index;
                g_combined_test.phase[index] = phase;
                g_combined_test.session[index] = status.session;
                g_combined_test.elapsed_ms[index] = now - s_combined_stage_ms;
                g_combined_test.final_yaw[index] = s_heading_source.yaw_deg;
                g_combined_test.final_error[index] = s_combined_turn ?
                    g_heading_exec_error_deg : g_forward_hold.error_deg;
#if APP_SERIAL_MOTION_CONTROL
                if (!s_combined_turn) App_TimingRecordCompleted(index, status.session, now);
#endif
                if(!g_stage1_stop_requested || status.distance_reached) g_combined_test.completed++;
                s_combined_gap_ms = now;
                g_combined_test.state = (g_stage1_stop_requested || (index + 1U) == MOTION_ACTION_COUNT) ? 3U : 2U;
                if (g_stage1_stop_requested || (index + 1U) == MOTION_ACTION_COUNT) s_combined_end_ms = now;
            }
        }
    }
    taskEXIT_CRITICAL();
}
static void reset(void){
 memset((void*)&g_combined_test,0,sizeof(g_combined_test));memset((void*)&g_forward_hold,0,sizeof(g_forward_hold));
 memset(&fixture_backend,0,sizeof(fixture_backend));memset(&fixture_power,0,sizeof(fixture_power));
 g_stage1_state=g_stage1_cause=g_stage1_key_request=g_stage1_denied=g_stage1_scan_valid=g_stage1_run_id=0;
 g_stage2_ready=g_stage2_run_id=g_stage1_stop_requested=g_stage1_motion_skipped=g_startup_axes_state=0;
 g_heading_exec_trip=g_serial_motion_link_lost=g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 s_motion_session_open=g_vm_test_request=stop_calls=0;g_radar_zone_ready=1;g_radar_zone_selected=1;
 g_saoma_accept_count=g_scan_display_count=10;g_saoma_last_result=1;g_scan_display_result=SCREEN_OK;
 g_saoma_frame_start_ms=g_saoma_frame_end_ms=0;ready=rearm_ok=tx_ok=1;tick=100;
 display_result=SCREEN_OK;display_calls=load_calls=finish_calls=0;fixture_phase=HR_DONE;
 g_forward_hold.phase=HR_IDLE;s_combined_stage_ms=0;
 strcpy(SAOMA_Data,"156+123+516+231");
}
static void start(void){g_stage1_key_request=1;App_Stage1Service();if(g_stage1_state==7)App_Stage1Service();g_startup_axes_state=0;}
static void frame(const char *code,uint32_t begin,uint32_t end){strcpy(SAOMA_Data,code);g_saoma_accept_count++;g_saoma_frame_start_ms=begin;g_saoma_frame_end_ms=end;SAOMA_DataReady=1;}
static void stopped(void){g_combined_test.state=3;fixture_backend.state=CHASSIS_AUTO_DONE;fixture_backend.result=1;fixture_backend.rotation_stop_mask=15;fixture_backend.active=0;fixture_power.active=g_vm_test_request=s_motion_session_open=0;}
static void run_tests(void){
 FieldStage1Plan p;ck(FieldMap_Stage1(&p));ck(p.forward_cm==15 && p.left_cm==105);
 reset();ready=0;start();ck(!g_stage1_run_id && g_stage1_denied==1);
#if APP_MOTOR_OFFLINE_ONLY
 ready=1;start();ck(!g_stage1_run_id && !g_combined_test.request);return;
#else
 reset();start();ck(g_stage1_state==1 && g_stage1_run_id==1);
 frame("156+123+516+231",90,180);tick=200;App_Stage1Service();ck(!g_stage1_scan_valid);
 frame("156+123+516+236",110,190);App_Stage1Service();ck(!g_stage1_scan_valid);
 frame("156+123+516+231",110,120);tick=800;App_Stage1Service();ck(!g_stage1_scan_valid);
 frame("156+123+516+231",810,820);tick=830;App_Stage1Service();
 ck(g_stage1_state==6 && g_stage1_scan_valid && g_stage1_stop_requested && !stop_calls && !g_stage2_ready);
 App_Stage1Display(1);ck(!display_calls);
 frame("123+123+321+321",840,850);tick=860;App_Stage1Service();ck(!strcmp(g_stage1_task_code,"156+123+516+231"));
 stopped();fixture_backend.rotation_stop_mask=14;App_Stage1Service();ck(!g_stage2_ready && g_stage1_state==6);
 fixture_backend.rotation_stop_mask=15;fixture_power.active=1;App_Stage1Service();ck(g_stage1_state==6);
 fixture_power.active=0;App_Stage1Service();ck(g_stage1_state==2 && !g_stage2_ready);
 App_Stage1Display(0);ck(!display_calls);App_Stage1Display(1);ck(display_calls==1 && !strcmp(displayed,"156+123+516+231"));
 App_Stage1Service();ck(g_stage1_state==3 && g_stage2_ready && g_stage2_run_id==1);
 App_Stage1Display(1);ck(display_calls==1);g_safety_fault=1;App_Stage1Service();ck(!g_stage2_ready && g_stage1_state==4);
 reset();start();tick=200;stopped();g_combined_test.completed=2;App_Stage1Service();ck(g_stage1_state==2);
 frame("123+123+321+321",210,220);tick=230;App_Stage1Service();App_Stage1Display(1);App_Stage1Service();
 ck(g_stage2_ready && !g_stage1_stop_requested && !strcmp(displayed,"123+123+321+321"));
 reset();start();tick=200;stopped();App_Stage1Service();tick=8200;App_Stage1Service();ck(g_stage1_state==4 && g_stage1_cause==3);
 reset();start();frame("156+123+516+231",110,120);tick=130;App_Stage1Service();tick=5130;App_Stage1Service();ck(stop_calls==1 && g_stage1_cause==8 && !g_stage2_ready);
 reset();start();frame("156+123+516+231",110,120);tick=130;App_Stage1Service();g_stage1_key_request=2;App_Stage1Service();ck(g_stage1_state==5 && stop_calls==1 && !g_stage2_ready);
 reset();start();tick=200;stopped();App_Stage1Service();frame("156+123+516+231",210,220);tick=230;App_Stage1Service();display_result=0;
 App_Stage1Display(1);tick+=250;App_Stage1Display(1);tick+=250;App_Stage1Display(1);tick+=250;App_Stage1Display(1);
 ck(display_calls==3 && g_stage1_state==4 && g_stage1_cause==7 && !g_stage2_ready);
 reset();start();frame("156+123+516+231",110,120);tick=130;App_Stage1Service();App_CombinedTestStep();
 ck(g_stage1_motion_skipped && g_combined_test.state==3 && !load_calls && !g_combined_test.completed);
 App_Stage1Service();App_Stage1Display(1);App_Stage1Service();ck(g_stage2_ready);
 /* Actual route function: no next segment after en-route QR, no fabricated completed distance. */
 reset();g_combined_test.state=2;g_combined_test.completed=1;g_combined_test.index=0;g_stage1_stop_requested=1;
 s_motion_session_open=1;fixture_power.active=1;g_vm_test_request=1;tick=2000;App_CombinedTestStep();
 ck(!load_calls && g_combined_test.index==0 && finish_calls==1 && g_combined_test.state==2);
 fixture_power.active=0;App_CombinedTestStep();ck(g_combined_test.state==3 && g_combined_test.completed==1 && !load_calls);
 reset();g_combined_test.state=1;g_combined_test.index=0;g_stage1_stop_requested=1;s_motion_session_open=1;
 fixture_backend.state=CHASSIS_AUTO_DONE;fixture_backend.result=1;fixture_backend.rotation_stop_mask=15;fixture_backend.distance_reached=0;
 tick=1000;App_CombinedTestStep();ck(g_combined_test.state==3 && !g_combined_test.completed && !load_calls && finish_calls==1);
 reset();g_combined_test.state=1;g_stage1_stop_requested=1;g_combined_test.request=2;App_CombinedTestStep();ck(g_combined_test.state==5);
 reset();g_combined_test.state=1;g_stage1_stop_requested=1;fixture_phase=HR_FAULT;App_CombinedTestStep();ck(g_combined_test.state==4);
 /* Tick wrap: a new frame across rollover is accepted, pre-run bytes are not. */
 reset();tick=0xfffffff0U;start();tick=20;frame("156+123+516+231",2,10);App_Stage1Service();ck(g_stage1_scan_valid);
#endif
}
int main(void){run_tests();printf("checks=%u failures=%u\n",checks,failures);for(uint32_t i=0;i<failures && i<16;i++)printf("failed check %u\n",failure_checks[i]);return failures?1:0;}
