#include "chassis_auto_test.c"

uint32_t test_tick, test_depth;
volatile uint32_t g_rtos_fault, g_safety_fault, g_safety_ready, g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched, g_motion_auto_stop_state, g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active, g_wheel_test_cut_vm, g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4], g_can_poll_flags[4], g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
volatile uint32_t checks, failures, first_failure;

static uint32_t stop_calls, reply_tick, stop_return, cut_power;

void Power_ReadCommandStatus(PowerCommandStatus *status)
{
    *status = (PowerCommandStatus){0};
}

static void check(uint32_t valid, uint32_t line)
{
    checks++;
    if (!valid)
    {
        failures++;
        if (!first_failure) first_failure = line;
    }
}
#define CHECK(test) check((test), __LINE__)

uint32_t StartupAxes_Holding(void)
{
    CHECK(0U);
    return 0U;
}

uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    (void)reply;
    CHECK(address == 4U && test_depth == 0U);
    stop_calls++;
    test_tick = reply_tick;
    if (cut_power) ChassisAuto_SafetyUpdate(0U);
    return stop_return;
}

uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply)
{
    (void)address; (void)reply;
    CHECK(0U);
    return 0U;
}
uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t address, CanRxMsg *reply, float *rpm)
{
    (void)address; (void)reply; (void)rpm;
    CHECK(0U);
    return 0U;
}
uint32_t CAN1_ReadEmmSystemFeedback(uint8_t address, CanRxMsg *reply, CAN1_EmmSystemFeedback *feedback)
{
    (void)address; (void)reply; (void)feedback;
    CHECK(0U);
    return 0U;
}
uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply, uint8_t *sign, uint32_t *raw)
{
    (void)address; (void)reply; (void)sign; (void)raw;
    CHECK(0U);
    return 0U;
}
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enable, CanRxMsg *reply)
{
    (void)address; (void)enable; (void)reply;
    CHECK(0U);
    return 0U;
}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction, uint16_t raw,
                            uint16_t accel, CanRxMsg *reply)
{
    (void)address; (void)direction; (void)raw; (void)accel; (void)reply;
    CHECK(0U);
    return 0U;
}

static void begin_final_stop(uint32_t request)
{
    uint32_t wheel;
    stop_calls = cut_power = 0U;
    stop_return = 1U;
    test_tick = request;
    test_depth = 0U;
    ChassisAuto_Init();
    g_safety_ready = g_can_guard_ready = 1U;
    g_chassis_auto_active = s_vm_requested = 1U;
    g_chassis_auto_session = 7U;
    g_chassis_auto_state = CHASSIS_AUTO_STOP;
    g_chassis_auto_segment = 2U;
    s_motor_index = 3U;
    g_chassis_auto_stop_mask[2] = 7U;
    s_total_tick = request - 500U;
    s_status_valid = s_speed_valid = 15U;
    for (wheel = 0U; wheel < 4U; wheel++)
    {
        s_status[wheel] = 1U;
        s_status_tick[wheel] = s_speed_tick[wheel] = request;
    }
    CHECK(ChassisAuto_RequestStop() == 1U && s_finish_tick == request);
    test_tick += 250U;
    CHECK(ChassisAuto_RequestStop() == 1U && s_finish_tick == request);
}

static void when_reply(uint32_t begin, uint32_t elapsed)
{
    begin_final_stop(begin);
    test_tick = begin + 490U;
    reply_tick = begin + elapsed;
    ChassisAuto_CanStep();
    CHECK(stop_calls == 1U && test_depth == 0U && g_chassis_auto_last_address == 4U);
    if (elapsed < CHASSIS_AUTO_STOP_MS)
    {
        CHECK(g_chassis_auto_stop_mask[2] == 15U && g_chassis_auto_state == CHASSIS_AUTO_SETTLE);
        CHECK(g_chassis_auto_active == 1U && !g_chassis_auto_cut_vm);
    }
    else
    {
        CHECK(g_chassis_auto_stop_mask[2] == 7U && g_chassis_auto_state == CHASSIS_AUTO_FAULT);
        CHECK(!g_chassis_auto_active && g_chassis_auto_cut_vm && g_chassis_auto_cause == 6U);
    }
}

void test_entry(void)
{
    when_reply(1000U, 499U);
    when_reply(1000U, 500U);
    when_reply(1000U, 501U);
    when_reply(0xFFFFFF00U, 499U);
    when_reply(0xFFFFFF00U, 500U);
    when_reply(0xFFFFFF00U, 501U);

    begin_final_stop(1000U);
    test_tick = 1500U;
    ChassisAuto_CanStep();
    CHECK(!stop_calls && g_chassis_auto_stop_mask[2] == 7U);
    CHECK(g_chassis_auto_state == CHASSIS_AUTO_FAULT && g_chassis_auto_cause == 6U);

    begin_final_stop(1000U);
    test_tick = 1490U; reply_tick = 1499U; stop_return = 5U;
    ChassisAuto_CanStep();
    CHECK(stop_calls == 1U && g_chassis_auto_stop_mask[2] == 7U);
    CHECK(g_chassis_auto_state == CHASSIS_AUTO_FAULT && g_chassis_auto_cause == 5U);

    begin_final_stop(1000U);
    test_tick = 1490U; reply_tick = 1499U; cut_power = 1U;
    ChassisAuto_CanStep();
    CHECK(stop_calls == 1U && g_chassis_auto_stop_mask[2] == 7U);
    CHECK(g_chassis_auto_state == CHASSIS_AUTO_CANCELLED && !g_chassis_auto_active);
    CHECK(test_depth == 0U);
}
