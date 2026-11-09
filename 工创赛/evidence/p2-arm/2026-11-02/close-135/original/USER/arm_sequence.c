#include "arm_sequence.h"
#include "gripper_task.h"
#include "lift.h"
#include "lift_config.h"
#include "lift_home.h"
#include "dm_can2.h"

volatile uint32_t g_arm_sequence_enable=0U,g_arm_grip_height_ready=0U;
volatile uint32_t g_arm_sequence_request=0U,g_arm_grip_confirm=0U;
volatile uint32_t g_arm_test_lift_pulses=5000U;
volatile uint32_t g_arm_sequence_mode=0U,g_arm_return_down_pulses=0U;
volatile uint32_t g_arm_sequence_state=0U,g_arm_sequence_result=0U;
volatile uint32_t g_arm_sequence_elapsed_ms=0U;
volatile uint32_t g_arm_sequence_phase_elapsed_ms=0U,g_arm_sequence_fault_state=0U;
volatile uint32_t g_arm_top_start_ready=0U,g_arm_pick_down_pulses=0U;
volatile uint32_t g_arm_top_approach_pulses=0U,g_arm_place_confirm=0U;
static uint32_t s_pick,s_approach,s_return;
static uint32_t s_mode,s_home_requested,s_home_expected;
static uint32_t s_busy,s_started,s_phase,s_last,s_ticket,s_seen,s_expected;
static uint32_t s_pulses,s_old_up,s_old_down,s_stop_count,s_settling;

uint32_t ArmSequence_IsBusy(void){return s_busy;}
uint32_t ArmSequence_AllowLiftRequest(uint32_t request)
{return (!s_busy || request==3U || (s_expected!=0U && request==s_expected))?1U:0U;}
uint32_t ArmSequence_OwnsHome(void)
{return s_busy && s_home_requested && ((s_mode==1U && g_arm_sequence_state==3U) || (s_mode==2U && g_arm_sequence_state==11U) || (s_mode==3U && (g_arm_sequence_state==29U || g_arm_sequence_state==34U)) || (s_mode==5U && g_arm_sequence_state==33U));}
uint32_t ArmSequence_AllowHomeRequest(uint32_t request)
{return (!s_busy || request==2U || (s_home_expected==1U && request==1U))?1U:0U;}
static uint32_t HomeStopUnknown(void)
{return g_lift_home_state==9U &&
    (g_lift_home_abort_result!=1U || g_lift_home_stop_result!=1U);}
static uint32_t LiftBusy(void)
{return (g_lift_motion_state>=1U && g_lift_motion_state<=6U)?1U:0U;}
static void Restore(void)
{g_lift_up_pulses=s_old_up;g_lift_down_pulses=s_old_down;s_expected=0U;s_home_expected=0U;s_home_requested=0U;}
static void Fail(uint32_t result,uint32_t state)
{
    if(g_arm_sequence_state!=8U && g_arm_sequence_state!=9U)
        g_arm_sequence_fault_state=g_arm_sequence_state;
    Gripper_CancelControlled();
    if(s_mode==3U){g_dm_jog_request=3U;g_dm_point_enable=0U;}
    if(LiftHome_IsBusy() || g_lift_home_request==1U || HomeStopUnknown())
        g_lift_home_request=2U;
    if(LiftBusy() || g_lift_motion_request==1U || g_lift_motion_request==2U ||
       (g_lift_motion_state==9U && g_lift_stop_result!=1U))
        g_lift_motion_request=3U;
    s_expected=0U;s_home_expected=0U;g_arm_grip_confirm=0U;g_arm_sequence_enable=0U;g_arm_grip_height_ready=0U;
    g_arm_top_start_ready=0U;g_arm_place_confirm=0U;
    g_arm_sequence_state=state;g_arm_sequence_result=result;
    /* 失败后保留占用和最后夹爪输出，不自动松手；观察停止后显式清锁存。 */
}
static void Phase(uint32_t state,uint32_t now)
{g_arm_sequence_state=state;s_phase=now;s_settling=0U;g_arm_sequence_phase_elapsed_ms=0U;}
static void LiftStart(uint32_t request,uint32_t now)
{
    if(request==2U)g_lift_up_pulses=s_pulses;else g_lift_down_pulses=s_pulses;
    s_seen=0U;s_expected=request;g_lift_motion_request=request;
    Phase(request==2U?3U:5U,now);
}
/* 模式3：固定点位单循环；源下降量须本次示教，默认0拒绝。 */
volatile float g_arm_replay_pick_deg=180.0f,g_arm_replay_place_deg=35.1079407f;
static float s_replay_pick_deg,s_replay_place_deg;
static uint32_t s_dm_expected,s_dm_seen;
static const uint32_t s_carrier_down_pulses=28000U;
static const uint32_t s_carrier_return_up_pulses=27000U;
uint32_t ArmSequence_OwnsDM(void)
{return s_busy && s_mode==3U && s_dm_expected!=0U;}
uint32_t ArmSequence_AllowDMRequest(uint32_t request)
{return (!s_busy || request==3U || (s_mode==3U && s_dm_expected!=0U && request==s_dm_expected))?1U:0U;}
static void ReplayLift(uint32_t request,uint32_t pulses,uint32_t state,uint32_t now)
{
    if(request==2U)g_lift_up_pulses=pulses;else g_lift_down_pulses=pulses;
    s_seen=0U;s_expected=request;g_lift_motion_request=request;Phase(state,now);
}
static void ReplayRotate(uint32_t request,uint32_t state,uint32_t now)
{
    g_dm_point_enable=1U;
    g_dm_pick_target_deg=s_replay_pick_deg;g_dm_place_target_deg=s_replay_place_deg;
    s_dm_expected=request;s_dm_seen=0U;g_dm_jog_request=request;Phase(state,now);
}
static uint32_t ReplayLiftDone(void)
{
    if(LiftBusy())s_seen=1U;
    if(g_lift_motion_state==9U || g_lift_motion_state==8U){Fail(6U,9U);return 0U;}
    if(s_seen && g_lift_motion_state==7U && g_lift_motion_result==1U)
    {s_expected=0U;return 1U;}
    return 0U;
}
static uint32_t ReplayGripperDone(float angle,uint32_t now)
{
    uint32_t result;
    if(!s_ticket)s_ticket=Gripper_ControlledCommand(angle);
    if(!s_ticket)return 0U;
    result=Gripper_ControlledResult(s_ticket);
    if(result==2U){Fail(5U,9U);return 0U;}
    if(result==1U && !s_settling){s_settling=1U;s_phase=now;}
    return s_settling && now-s_phase>=500U;
}
static void SourceTopStart(uint32_t state,uint32_t now)
{s_seen=0U;s_home_requested=1U;s_home_expected=1U;g_lift_home_request=1U;Phase(state,now);}
static uint32_t SourceTopDone(void)
{
    if(LiftHome_IsBusy()){s_seen=1U;s_home_expected=0U;}
    if(g_lift_home_state==9U || g_lift_home_state==10U){Fail(6U,9U);return 0U;}
    if(s_seen && g_lift_home_state==8U && g_lift_home_result==1U && g_lift_home_reported==1U)
    {s_home_requested=0U;s_home_expected=0U;return 1U;}
    return 0U;
}

static void ReplayService(uint32_t ready,uint32_t other_busy,uint32_t now)
{
    uint32_t request=g_arm_sequence_request;
    float target;
    g_arm_sequence_request=0U;
    if(request==2U)
    {if(s_busy)Fail(2U,8U);else {g_arm_sequence_state=8U;g_arm_sequence_result=2U;}return;}
    if(request==3U)
    {
        if(s_busy && g_arm_sequence_state!=8U && g_arm_sequence_state!=9U)return;
        if(LiftBusy() || LiftHome_IsBusy() || DM_CAN2_JogIsBusy() || g_dm_jog_request!=0U ||
           g_dm_posvel_holding || HomeStopUnknown() || (g_lift_motion_state==9U && g_lift_stop_result!=1U) ||
           (g_dm_jog_state==8U && g_dm_jog_stop_result!=1U))return;
        Restore();s_busy=0U;s_dm_expected=0U;g_arm_sequence_enable=0U;g_arm_top_start_ready=0U;
        g_arm_sequence_state=0U;g_arm_sequence_result=0U;g_arm_sequence_fault_state=0U;return;
    }
    if(request>3U){if(s_busy)Fail(4U,9U);else g_arm_sequence_result=4U;return;}
    if(!s_busy)
    {
        if(request!=1U)return;
        if(!ready || other_busy || LiftBusy() || LiftHome_IsBusy() || DM_CAN2_JogIsBusy() ||
           g_lift_motion_state==9U || g_lift_home_state==9U || g_lift_motion_request!=0U ||
           g_lift_home_request!=0U || g_arm_sequence_enable!=1U || g_arm_top_start_ready!=1U)
        {g_arm_sequence_result=3U;return;}
        s_pick=g_arm_pick_down_pulses;
        s_replay_pick_deg=g_arm_replay_pick_deg;s_replay_place_deg=g_arm_replay_place_deg;
        if(s_pick<=ARM_SOURCE_TOP_MARGIN_PULSES || s_pick>LIFT_DOWN_MAX_PULSES || s_pick>LIFT_UP_MAX_PULSES ||
           !(s_replay_pick_deg>=0.0f && s_replay_pick_deg<=180.0f) ||
           !(s_replay_place_deg>=0.0f && s_replay_place_deg<=180.0f))
        {g_arm_sequence_result=4U;return;}
        s_mode=3U;s_busy=1U;s_started=now;s_last=now;
        s_old_up=g_lift_up_pulses;s_old_down=g_lift_down_pulses;
        s_expected=0U;s_dm_expected=0U;s_dm_seen=0U;s_home_expected=0U;s_home_requested=0U;
        s_stop_count=g_gripper_signal_stop_count;g_arm_top_start_ready=0U;
        g_arm_grip_confirm=0U;g_arm_place_confirm=0U;g_arm_sequence_result=0U;
        g_arm_sequence_elapsed_ms=0U;g_arm_sequence_fault_state=0U;
        s_ticket=0U;Phase(20U,now);return;
    }
    if(g_arm_sequence_state==8U || g_arm_sequence_state==9U)return;
    g_arm_sequence_elapsed_ms=now-s_started;g_arm_sequence_phase_elapsed_ms=now-s_phase;
    if(!ready || other_busy || g_arm_sequence_enable!=1U || g_gripper_signal_stop_count!=s_stop_count)
    {Fail(3U,9U);return;}
    if(now-s_last>500U){Fail(8U,9U);return;}
    s_last=now;
    if(g_arm_sequence_state>=22U && g_arm_sequence_state!=25U && !g_dm_posvel_holding)
    {Fail(9U,9U);return;}
    switch(g_arm_sequence_state)
    {
    case 20U:
        if(ReplayGripperDone(180.0f,now))ReplayRotate(4U,21U,now);
        break;
    case 21U:case 25U:
        if(DM_CAN2_JogIsBusy())s_dm_seen=1U;
        /* 从保持切新点位会先失能，短暂8/2是模块内部排队切换，不当成功。 */
        if(g_dm_jog_state==8U && g_dm_jog_result!=2U){Fail(9U,9U);break;}
        target=g_arm_sequence_state==21U?s_replay_pick_deg:s_replay_place_deg;
        if(s_dm_seen && g_dm_jog_state==7U && g_dm_jog_result==1U && g_dm_posvel_holding &&
           g_dm_point_latched_deg==target)
        {
            s_dm_expected=0U;
            if(g_arm_sequence_state==21U)ReplayLift(1U,s_pick,22U,now);
            else ReplayLift(1U,s_carrier_down_pulses,26U,now);
        }
        break;
    case 22U:
        if(ReplayLiftDone()){s_ticket=0U;Phase(23U,now);}
        break;
    case 23U:
        if(ReplayGripperDone(145.0f,now))ReplayLift(2U,s_pick-ARM_SOURCE_TOP_MARGIN_PULSES,24U,now);
        break;
    case 24U:
        if(ReplayLiftDone())SourceTopStart(29U,now);
        break;
    case 29U:
        if(SourceTopDone())ReplayRotate(5U,25U,now);
        break;
    case 26U:
        if(ReplayLiftDone()){s_ticket=0U;Phase(27U,now);}
        break;
    case 27U:
        if(ReplayGripperDone(180.0f,now))
        ReplayLift(2U,s_carrier_return_up_pulses,28U,now);
        break;
    case 28U:
        if(ReplayLiftDone())SourceTopStart(34U,now);
        break;
    case 34U:
        if(SourceTopDone())
        {Restore();s_busy=0U;s_dm_expected=0U;g_arm_sequence_enable=0U;
         g_arm_sequence_state=7U;g_arm_sequence_result=1U;}
        break;
    default:Fail(4U,9U);break;
    }
}

/* 模式4：仅执行本轮已示教的源下降及夹紧，不操作DM、回零或抬升。 */
static const uint32_t s_source_down_pulses=105000U;
volatile uint32_t g_arm_source_down_done_pulses=0U;
static void SourcePickService(uint32_t ready,uint32_t other_busy,uint32_t now)
{
    uint32_t request=g_arm_sequence_request;
    g_arm_sequence_request=0U;
    if(request==2U)
    {if(s_busy)Fail(2U,8U);else {g_arm_sequence_state=8U;g_arm_sequence_result=2U;}return;}
    if(request==3U)
    {
        if(s_busy && g_arm_sequence_state!=8U && g_arm_sequence_state!=9U)return;
        if(LiftBusy() || LiftHome_IsBusy() || g_lift_home_request!=0U || HomeStopUnknown() ||
           (g_lift_motion_state==9U && g_lift_stop_result!=1U))return;
        if(s_busy)Restore();
        Gripper_CancelControlled();s_busy=0U;g_arm_sequence_enable=0U;g_arm_top_start_ready=0U;
        g_arm_grip_confirm=0U;g_arm_place_confirm=0U;g_arm_grip_height_ready=0U;
        g_arm_sequence_state=0U;g_arm_sequence_result=0U;g_arm_sequence_fault_state=0U;
        g_arm_sequence_phase_elapsed_ms=0U;return;
    }
    if(request>3U){if(s_busy)Fail(4U,9U);else g_arm_sequence_result=4U;return;}
    if(!s_busy)
    {
        if(request!=1U)return;
        if(!ready || other_busy || LiftBusy() || LiftHome_IsBusy() ||
           g_lift_motion_request!=0U || g_lift_home_request!=0U ||
           g_lift_motion_state==9U || g_lift_home_state==9U ||
           g_arm_sequence_enable!=1U || g_arm_top_start_ready!=1U)
        {g_arm_sequence_result=3U;return;}
        s_mode=4U;s_busy=1U;s_started=now;s_last=now;
        s_old_up=g_lift_up_pulses;s_old_down=g_lift_down_pulses;
        s_expected=0U;s_home_expected=0U;s_home_requested=0U;
        s_stop_count=g_gripper_signal_stop_count;
        g_arm_source_down_done_pulses=0U;g_arm_top_start_ready=0U;
        g_arm_grip_confirm=0U;g_arm_place_confirm=0U;
        g_arm_sequence_result=0U;g_arm_sequence_fault_state=0U;g_arm_sequence_elapsed_ms=0U;
        ReplayLift(1U,s_source_down_pulses,30U,now);return;
    }
    if(g_arm_sequence_state==8U || g_arm_sequence_state==9U)return;
    g_arm_sequence_elapsed_ms=now-s_started;g_arm_sequence_phase_elapsed_ms=now-s_phase;
    if(!ready || other_busy || g_arm_sequence_enable!=1U || g_gripper_signal_stop_count!=s_stop_count)
    {Fail(3U,9U);return;}
    if(now-s_last>500U){Fail(8U,9U);return;}
    s_last=now;
    switch(g_arm_sequence_state)
    {
    case 30U:
        if(ReplayLiftDone())
        {
            g_arm_source_down_done_pulses=s_source_down_pulses;
            s_ticket=0U;Phase(31U,now);
        }
        break;
    case 31U:
        if(ReplayGripperDone(145.0f,now))
        {Restore();s_busy=0U;g_arm_sequence_enable=0U;g_arm_sequence_state=7U;g_arm_sequence_result=1U;}
        break;
    default:Fail(4U,9U);break;
    }
}

/* 模式5：仅从本轮源夹取点返回，104000有限上升后走既有低速回零。 */
static void SourceReturnService(uint32_t ready,uint32_t other_busy,uint32_t now)
{
    uint32_t request=g_arm_sequence_request;
    g_arm_sequence_request=0U;
    if(request==2U)
    {if(s_busy)Fail(2U,8U);else {g_arm_sequence_state=8U;g_arm_sequence_result=2U;}return;}
    if(request==3U)
    {
        if(s_busy && g_arm_sequence_state!=8U && g_arm_sequence_state!=9U)return;
        if(LiftBusy() || LiftHome_IsBusy() || g_lift_home_request!=0U || HomeStopUnknown() ||
           (g_lift_motion_state==9U && g_lift_stop_result!=1U))return;
        if(s_busy)Restore();
        Gripper_CancelControlled();s_busy=0U;g_arm_sequence_enable=0U;g_arm_grip_height_ready=0U;
        g_arm_sequence_state=0U;g_arm_sequence_result=0U;g_arm_sequence_fault_state=0U;
        g_arm_sequence_phase_elapsed_ms=0U;return;
    }
    if(request>3U){if(s_busy)Fail(4U,9U);else g_arm_sequence_result=4U;return;}
    if(!s_busy)
    {
        if(request!=1U)return;
        if(!ready || other_busy || LiftBusy() || LiftHome_IsBusy() ||
           g_lift_motion_request!=0U || g_lift_home_request!=0U ||
           g_lift_motion_state==9U || g_lift_home_state==9U ||
           g_arm_sequence_enable!=1U || g_arm_grip_height_ready!=1U)
        {g_arm_sequence_result=3U;return;}
        s_mode=5U;s_busy=1U;s_started=now;s_last=now;
        s_old_up=g_lift_up_pulses;s_old_down=g_lift_down_pulses;
        s_expected=0U;s_home_expected=0U;s_home_requested=0U;
        s_stop_count=g_gripper_signal_stop_count;g_arm_grip_height_ready=0U;
        g_arm_top_start_ready=0U;g_arm_grip_confirm=0U;g_arm_place_confirm=0U;
        g_arm_sequence_result=0U;g_arm_sequence_fault_state=0U;g_arm_sequence_elapsed_ms=0U;
        ReplayLift(2U,ARM_SOURCE_RETURN_UP_PULSES,32U,now);return;
    }
    if(g_arm_sequence_state==8U || g_arm_sequence_state==9U)return;
    g_arm_sequence_elapsed_ms=now-s_started;g_arm_sequence_phase_elapsed_ms=now-s_phase;
    if(!ready || other_busy || g_arm_sequence_enable!=1U || g_gripper_signal_stop_count!=s_stop_count)
    {Fail(3U,9U);return;}
    if(now-s_last>500U){Fail(8U,9U);return;}
    s_last=now;
    switch(g_arm_sequence_state)
    {
    case 32U:
        if(ReplayLiftDone())SourceTopStart(33U,now);
        break;
    case 33U:
        if(SourceTopDone())
        {Restore();s_busy=0U;g_arm_sequence_enable=0U;g_arm_sequence_state=7U;g_arm_sequence_result=1U;}
        break;
    default:Fail(4U,9U);break;
    }
}

void ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now)
{
    uint32_t request=g_arm_sequence_request,result;
    if((s_busy && s_mode==5U) || (!s_busy && g_arm_sequence_mode==5U))
    {SourceReturnService(ready,other_busy,now);return;}
    if((s_busy && s_mode==4U) || (!s_busy && g_arm_sequence_mode==4U))
    {SourcePickService(ready,other_busy,now);return;}
    if((s_busy && s_mode==3U) || (!s_busy && g_arm_sequence_mode==3U))
    {ReplayService(ready,other_busy,now);return;}
    g_arm_sequence_request=0U;
    if(request==2U)
    {if(s_busy){if(g_arm_sequence_state==9U)Fail(g_arm_sequence_result,9U);else Fail(2U,8U);}else {g_arm_sequence_state=8U;g_arm_sequence_result=2U;}return;}
    if(request==3U)
    {
        if(s_busy && g_arm_sequence_state!=8U && g_arm_sequence_state!=9U)return;
        if(LiftBusy() || LiftHome_IsBusy() || g_lift_home_request!=0U || HomeStopUnknown() ||
           (g_lift_motion_state==9U && g_lift_stop_result!=1U))return;
        if(s_busy)Restore();
        Gripper_CancelControlled();s_busy=0U;g_arm_sequence_enable=0U;
        g_arm_grip_confirm=0U;g_arm_grip_height_ready=0U;g_arm_top_start_ready=0U;g_arm_place_confirm=0U;g_arm_sequence_state=0U;g_arm_sequence_result=0U;g_arm_sequence_fault_state=0U;g_arm_sequence_phase_elapsed_ms=0U;return;
    }
    if(request>3U){if(s_busy)Fail(4U,9U);else g_arm_sequence_result=4U;return;}
    if(!s_busy)
    {
        if(request!=1U)return;
        if(!ready || other_busy || LiftBusy() || g_lift_motion_state==9U ||
           g_lift_motion_request!=0U || g_lift_home_request!=0U || LiftHome_IsBusy() ||
           g_lift_home_state==9U || g_arm_sequence_enable!=1U ||
           (g_arm_sequence_mode==2U?g_arm_top_start_ready!=1U:g_arm_grip_height_ready!=1U))
        {g_arm_sequence_result=3U;return;}
        s_mode=g_arm_sequence_mode;
        s_pick=g_arm_pick_down_pulses;s_approach=g_arm_top_approach_pulses;s_return=g_arm_return_down_pulses;
        s_pulses=s_mode==0U?g_arm_test_lift_pulses:s_return;
        if(s_mode>2U || s_pulses==0U || (s_mode==0U && s_pulses>10000U) || s_pulses>LIFT_DOWN_MAX_PULSES ||
           (s_mode==2U && (s_pick==0U || s_pick>LIFT_DOWN_MAX_PULSES || s_approach==0U ||
                           s_approach>=s_pick || s_approach>LIFT_UP_MAX_PULSES)))
        {g_arm_sequence_result=4U;return;}
        s_old_up=g_lift_up_pulses;s_old_down=g_lift_down_pulses;
        s_busy=1U;s_started=now;s_last=now;s_expected=0U;s_home_requested=0U;s_home_expected=0U;
        s_stop_count=g_gripper_signal_stop_count;g_arm_grip_confirm=0U;
        g_arm_sequence_result=0U;g_arm_sequence_elapsed_ms=0U;g_arm_sequence_fault_state=0U;
        g_arm_place_confirm=0U;
        if(s_mode==2U)
        {g_arm_top_start_ready=0U;g_arm_grip_height_ready=0U;Phase(13U,now);s_ticket=Gripper_ControlledCommand(180.0f);}
        else {Phase(1U,now);s_ticket=Gripper_ControlledCommand(145.0f);}
        return;
    }
    if(g_arm_sequence_state==8U || g_arm_sequence_state==9U)return;
    g_arm_sequence_elapsed_ms=now-s_started;g_arm_sequence_phase_elapsed_ms=now-s_phase;
    if(!ready || other_busy || g_arm_sequence_enable!=1U || (s_mode!=2U && g_arm_grip_height_ready!=1U) || g_gripper_signal_stop_count!=s_stop_count)
    {Fail(3U,9U);return;}
    if(now-s_last>500U){Fail(8U,9U);return;}
    s_last=now;
    switch(g_arm_sequence_state)
    {
    case 1U:
        if(!s_ticket)
        {s_ticket=Gripper_ControlledCommand(145.0f);if(!s_ticket){break;}}
        result=Gripper_ControlledResult(s_ticket);
        if(result==2U){Fail(5U,9U);break;}
        if(result==1U){g_arm_grip_confirm=0U;Phase(2U,now);}
        /* 用户指定：等待确认/子动作不设动作层超时。 */
        break;
    case 2U:
        if(g_arm_grip_confirm==1U)
        {
            g_arm_grip_confirm=0U;
            if(s_mode==1U)
            {s_seen=0U;s_home_requested=1U;s_home_expected=1U;g_lift_home_request=1U;Phase(3U,now);}
            else {if(s_mode==2U)s_pulses=s_approach;LiftStart(2U,now);}
        }
        /* 用户指定：等待确认/子动作不设动作层超时。 */
        break;
    case 3U: case 5U: case 10U: case 11U:
        if((s_mode==1U && g_arm_sequence_state==3U) || (s_mode==2U && g_arm_sequence_state==11U))
        {
            if(LiftHome_IsBusy()){s_seen=1U;s_home_expected=0U;}
            if(g_lift_home_state==9U || g_lift_home_state==10U){Fail(6U,9U);break;}
            if(s_seen && g_lift_home_state==8U && g_lift_home_result==1U && g_lift_home_reported==1U)
            {s_home_expected=0U;s_home_requested=0U;if(s_mode==2U)s_pulses=s_return;Phase(4U,now);}
            /* 用户指定：等待确认/子动作不设动作层超时。 */
            break;
        }
        if(LiftBusy())s_seen=1U;
        if(g_lift_motion_state==9U || g_lift_motion_state==8U){Fail(6U,9U);break;}
        if(s_seen && g_lift_motion_state==7U && g_lift_motion_result==1U)
        {
            s_expected=0U;
            if(s_mode==2U && g_arm_sequence_state==10U)
            {Phase(1U,now);s_ticket=Gripper_ControlledCommand(145.0f);}
            else if(s_mode==2U && g_arm_sequence_state==3U)
            {s_seen=0U;s_home_requested=1U;s_home_expected=1U;g_lift_home_request=1U;Phase(11U,now);}
            else if(g_arm_sequence_state==3U)Phase(4U,now);
            else if(s_mode==2U){g_arm_place_confirm=0U;Phase(12U,now);}
            else {Phase(6U,now);s_ticket=0U;}
        }
        /* 用户指定：等待确认/子动作不设动作层超时。 */
        break;
    case 4U:
        if(now-s_phase>=500U)LiftStart(1U,now);
        break;
    case 12U:
        if(g_arm_place_confirm==1U)
        {g_arm_place_confirm=0U;Phase(6U,now);s_ticket=0U;}
        /* 用户指定：等待确认/子动作不设动作层超时。 */
        break;
    case 13U:
        if(!s_ticket)
        {s_ticket=Gripper_ControlledCommand(180.0f);if(!s_ticket){break;}}
        result=Gripper_ControlledResult(s_ticket);
        if(result==2U){Fail(5U,9U);break;}
        if(result==1U && !s_settling){s_settling=1U;s_phase=now;}
        if(s_settling && now-s_phase>=500U)
        {s_pulses=s_pick;LiftStart(1U,now);Phase(10U,now);}
        /* 等待夹爪结果，不设动作层超时。 */
        break;
    case 6U:
        if(!s_ticket)
        {s_ticket=Gripper_ControlledCommand(180.0f);if(!s_ticket){break;}}
        result=Gripper_ControlledResult(s_ticket);
        if(result==2U){Fail(5U,9U);break;}
        if(result==1U && !s_settling){s_settling=1U;s_phase=now;}
        if(s_settling && now-s_phase>=500U)
        {Restore();s_busy=0U;g_arm_sequence_enable=0U;if(s_mode!=0U)g_arm_grip_height_ready=0U;g_arm_top_start_ready=0U;g_arm_place_confirm=0U;g_arm_sequence_state=7U;g_arm_sequence_result=1U;}
        /* 等待夹爪结果，不设动作层超时。 */
        break;
    default:Fail(4U,9U);break;
    }
}
