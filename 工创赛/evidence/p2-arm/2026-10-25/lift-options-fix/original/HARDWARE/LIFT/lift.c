#include "lift.h"
#include "lift_config.h"
#include "lift_protocol.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

const uint32_t g_lift_address = LIFT_CAN_ADDRESS;
volatile uint32_t g_lift_state = 0U;
volatile uint32_t g_lift_read_request = 0U;
volatile uint32_t g_lift_read_attempts = 0U;
volatile uint32_t g_lift_version_result = 0U;
volatile uint32_t g_lift_status_result = 0U;
CanRxMsg g_lift_version_reply;
CanRxMsg g_lift_status_reply;
static TickType_t s_wait_started = 0U;
static uint32_t s_wait_ms = 0U;


/* 所有下发仍由现有唯一CAN任务串行执行。 */
volatile uint32_t g_lift_motion_request = 0U;
volatile uint32_t g_lift_motion_state = 0U;
volatile uint32_t g_lift_motion_result = 0U;
volatile uint32_t g_lift_enable_result = 0U;
volatile uint32_t g_lift_move_result = 0U;
volatile uint32_t g_lift_stop_result = 0U;
volatile uint32_t g_lift_flags = 0U;
volatile int32_t g_lift_speed_rpm = 0;
volatile int64_t g_lift_delta_raw = 0;
volatile uint32_t g_lift_can_esr = 0U;
volatile uint32_t g_lift_options_result = 0U;
CanRxMsg g_lift_control_reply;
static CanRxMsg s_motion_reply;
static uint8_t s_direction = 0U, s_position_sign = 0U;
static uint32_t s_position_raw = 0U;
static int64_t s_before_position = 0;
static TickType_t s_motion_started = 0U, s_poll_tick = 0U;
static uint8_t s_poll_phase = 0U;

static uint32_t Lift_StopCommand(void)
{
    const uint8_t stop[4] = {0xFEU, 0x98U, 0U, 0x6BU};
    g_lift_stop_result = CAN1_LiftExchange(stop, 4U, &g_lift_control_reply);
    g_lift_can_esr = CAN1->ESR;
    return g_lift_stop_result;
}

static void Lift_MotionFault(uint32_t reason)
{
    (void)Lift_StopCommand();
    g_lift_motion_result = reason;
    g_lift_motion_state = 9U;
}

static int64_t Lift_SignedPosition(uint8_t sign, uint32_t raw)
{
    return sign ? -(int64_t)raw : (int64_t)raw;
}

static void Lift_MotionService(TickType_t now)
{
    uint32_t request, result;
    uint8_t command[12];
    int64_t delta;
    request = g_lift_motion_request;
    g_lift_motion_request = 0U;
    if (request == 3U)
    {
        result = Lift_StopCommand();
        g_lift_motion_result = (result == 1U) ? 2U : 4U;
        g_lift_motion_state = (result == 1U) ? 8U : 9U;
        return;
    }
    /* 失败锁存后忽略再次试走；先明确停止成功或重新上电会话。 */
    if (g_lift_motion_state == 9U) return;
    if ((request == 1U || request == 2U) &&
        !(g_lift_motion_state >= 1U && g_lift_motion_state <= 6U))
    {
        s_direction = (request == 1U) ? 0U : 1U;
        g_lift_motion_result = 0U;
        g_lift_enable_result = 0U; g_lift_move_result = 0U;
        g_lift_stop_result = 0U; g_lift_delta_raw = 0;
        g_lift_motion_state = 1U;
        s_motion_started = now;
    }
    /* 忙时请求不排队、不重启；继续监测原动作的反馈与超时。 */
    if (g_lift_motion_state < 1U || g_lift_motion_state > 6U) return;
    if ((TickType_t)(now - s_motion_started) >= pdMS_TO_TICKS(LIFT_MOVE_TIMEOUT_MS))
    {
        Lift_MotionFault(3U);
        return;
    }
    switch (g_lift_motion_state)
    {
    case 1U:
        g_lift_options_result = CAN1_ReadMotorOptions(LIFT_CAN_ADDRESS, &s_motion_reply);
        if (g_lift_options_result != 1U || (s_motion_reply.Data[1] & 0x02U) == 0U)
        {
            /* 当前选项必须确为Emm，不依赖固件版本号猜格式。 */
            g_lift_motion_result = 4U; g_lift_motion_state = 9U;
        }
        else g_lift_motion_state = 2U;
        break;
    case 2U:
        result = CAN1_ReadStatus(LIFT_CAN_ADDRESS, &s_motion_reply);
        if (result != 1U) { Lift_MotionFault(4U); break; }
        g_lift_flags = s_motion_reply.Data[1];
        if ((g_lift_flags & 0x0CU) != 0U) { Lift_MotionFault(5U); break; }
        g_lift_motion_state = (g_lift_flags & 1U) ? 4U : 3U;
        break;
    case 3U:
        command[0]=0xF3U; command[1]=0xABU; command[2]=1U;
        command[3]=0U; command[4]=0x6BU;
        g_lift_enable_result=CAN1_LiftExchange(command,5U,&g_lift_control_reply);
        g_lift_can_esr=CAN1->ESR;
        if (g_lift_enable_result != 1U) { Lift_MotionFault(4U); break; }
        g_lift_motion_state=4U;
        break;
    case 4U:
        result=CAN1_ReadPosition(LIFT_CAN_ADDRESS,&s_motion_reply,
                                &s_position_sign,&s_position_raw);
        if (result != 1U) { Lift_MotionFault(4U); break; }
        s_before_position=Lift_SignedPosition(s_position_sign,s_position_raw);
        g_lift_motion_state=5U;
        break;
    case 5U:
        if (!LiftProtocol_Move(s_direction,LIFT_JOG_PULSES,command))
        { Lift_MotionFault(4U); break; }
        g_lift_move_result=CAN1_LiftExchange(command,12U,&g_lift_control_reply);
        g_lift_can_esr=CAN1->ESR;
        if (g_lift_move_result != 1U) { Lift_MotionFault(4U); break; }
        g_lift_motion_state=6U; s_poll_phase=0U;
        s_poll_tick=xTaskGetTickCount();
        break;
    case 6U:
        if ((TickType_t)(now-s_poll_tick) < pdMS_TO_TICKS(LIFT_POLL_MS)) break;
        if (s_poll_phase == 0U)
        {
            result=CAN1_ReadStatus(LIFT_CAN_ADDRESS,&s_motion_reply);
            if (result != 1U) { Lift_MotionFault(4U); break; }
            g_lift_flags=s_motion_reply.Data[1];
            if ((g_lift_flags & 0x0CU) || !(g_lift_flags & 1U))
            { Lift_MotionFault(5U); break; }
            s_poll_phase=1U;
        }
        else if (s_poll_phase == 1U)
        {
            result=CAN1_ReadPosition(LIFT_CAN_ADDRESS,&s_motion_reply,
                                    &s_position_sign,&s_position_raw);
            if (result != 1U) { Lift_MotionFault(4U); break; }
            g_lift_delta_raw=Lift_SignedPosition(s_position_sign,s_position_raw)-s_before_position;
            s_poll_phase=2U;
        }
        else
        {
            int32_t speed=0;
            result=CAN1_ReadSpeed(LIFT_CAN_ADDRESS,&s_motion_reply,&speed);
            if (result != 1U) { Lift_MotionFault(4U); break; }
            g_lift_speed_rpm=speed;
            delta=g_lift_delta_raw;
            if ((g_lift_flags & 2U) && speed == 0 && (delta >= 2 || delta <= -2))
            {
                /* 新位置变化+到位+静止反馈；不以02确认包当完成。 */
                g_lift_motion_result=1U; g_lift_motion_state=7U;
            }
            s_poll_phase=0U; s_poll_tick=now;
        }
        break;
    default: break;
    }
}

void Lift_Service(uint32_t power_ready)
{
    TickType_t now = xTaskGetTickCount();
    if (power_ready != 1U)
    {
        /* 保留本次原始回复方便观察，供电恢复再新建读取会话。 */
        if (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U)
        { g_lift_motion_state=9U; g_lift_motion_result=6U; }
        g_lift_motion_request=0U;
        g_lift_state = 0U;
        return;
    }
    if (g_lift_state == 4U)
    {
        if (g_lift_read_request != 0U &&
            !(g_lift_motion_state >= 1U && g_lift_motion_state <= 6U))
        {
            g_lift_read_request = 0U;
            g_lift_state = 0U;
        }
        else
        {
            /* 动作期间不插入重新初始化会话。 */
            g_lift_read_request = 0U;
            Lift_MotionService(now);
            return;
        }
    }
    if (g_lift_motion_request != 0U)
    {
        g_lift_motion_request=0U; g_lift_motion_result=6U;
    }
    if (g_lift_read_request != 0U)
    {
        g_lift_read_request = 0U;
        g_lift_state = 0U;
    }
    if (g_lift_state == 0U)
    {
        /* 明确重新供电/重建身份会话后，才重置上轮动作状态。 */
        g_lift_motion_state = 0U;
        g_lift_motion_result = 0U;
        memset(&g_lift_version_reply, 0, sizeof(g_lift_version_reply));
        memset(&g_lift_status_reply, 0, sizeof(g_lift_status_reply));
        g_lift_version_result = 0U;
        g_lift_status_result = 0U;
        g_lift_read_attempts = 0U;
        s_wait_started = now;
        s_wait_ms = LIFT_POWER_SETTLE_MS;
        g_lift_state = 1U;
        return;
    }
    if (g_lift_state == 1U)
    {
        if ((TickType_t)(now - s_wait_started) < pdMS_TO_TICKS(s_wait_ms))
        {
            return;
        }
        g_lift_state = 2U;
    }
    if (g_lift_state == 2U)
    {
        /* 通用单帧查询支持非零地址，现有底盘运动地址保护不变。 */
        g_lift_read_attempts++;
        g_lift_version_result = CAN1_ReadVersion(LIFT_CAN_ADDRESS,
                                                 &g_lift_version_reply);
        if (g_lift_version_result == 1U)
        {
            g_lift_state = 3U;
        }
        else if (g_lift_read_attempts < LIFT_READ_MAX_ATTEMPTS)
        {
            s_wait_started = xTaskGetTickCount();
            s_wait_ms = LIFT_READ_RETRY_MS;
            g_lift_state = 1U;
        }
        else
        {
            g_lift_state = 5U;
        }
        return;
    }
    if (g_lift_state == 3U)
    {
        g_lift_status_result = CAN1_ReadStatus(LIFT_CAN_ADDRESS,
                                               &g_lift_status_reply);
        g_lift_state = (g_lift_status_result == 1U) ? 4U : 5U;
    }
}
