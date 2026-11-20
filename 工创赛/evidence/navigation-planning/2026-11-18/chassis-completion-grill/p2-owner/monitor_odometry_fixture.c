#include "chassis_auto_test.c"

uint32_t test_tick, test_depth;
volatile uint32_t g_rtos_fault, g_safety_fault, g_safety_ready, g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched, g_motion_auto_stop_state, g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active, g_wheel_test_cut_vm, g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4], g_can_poll_flags[4], g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
volatile uint32_t checks, failures, first_failure;

static uint32_t condition, power_epoch, power_active, virtual_calls, status_calls, position_calls;

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

void Power_ReadCommandStatus(PowerCommandStatus *status)
{
    *status = (PowerCommandStatus){power_active, 1U, 0U, power_epoch};
}

uint32_t StartupAxes_Holding(void)
{
    CHECK(0U);
    return 0U;
}

uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    (void)address; (void)reply;
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

uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply)
{
    CHECK(!test_depth && address==2U);
    status_calls++;
    reply->Data[1] = 1U;
    return 1U;
}

uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply, uint8_t *sign, uint32_t *raw)
{
    (void)reply;
    CHECK(!test_depth && address==2U);
    position_calls++;
    *sign = 0U; *raw = 900U;
    test_tick++;
    return 1U;
}

uint32_t CAN1_ReadEmmSystemFeedback(uint8_t address, CanRxMsg *reply, CAN1_EmmSystemFeedback *feedback)
{
    (void)reply;
    CHECK(!test_depth && address==2U);
    virtual_calls++;
    *feedback = (CAN1_EmmSystemFeedback){test_tick, test_tick+2U, 16842754U, -70, 1U, 0U, 0U, address, 1U};
    test_tick += 3U;
    switch (condition)
    {
    case 1U: g_chassis_auto_session++; break;
    case 2U: power_epoch++; break;
    case 3U: power_active = 0U; break;
    case 4U: feedback->valid = 0U; break;
    case 5U: feedback->address = 3U; break;
    case 6U: feedback->reply_ms = feedback->request_ms-1U; break;
    case 7U: feedback->reply_ms = test_tick+1U; break;
    case 8U: return 5U;
    case 9U: g_safety_fault = 1U; break;
    case 10U: g_chassis_auto_active = 0U; break;
    case 11U: g_chassis_auto_cut_vm = 1U; break;
    }
    return 1U;
}

static void begin(uint32_t tick)
{
    ChassisAuto_Init();
    test_tick = tick;
    condition = virtual_calls = status_calls = position_calls = 0U;
    g_safety_fault = 0U;
    g_safety_ready = g_can_guard_ready = 1U;
    g_chassis_auto_active = s_vm_requested = 1U;
    g_chassis_auto_session = 7U;
    g_chassis_auto_state = CHASSIS_AUTO_VERIFY_ON;
    s_monitor_cursor = 3U;
    power_epoch = 19U;
    power_active = 1U;
}

void test_entry(void)
{
    uint32_t index;
    ChassisOdometrySamples samples;
    for (index=0U; index<=11U; index++)
    {
        begin(100U);
        condition = index;
        Auto_Monitor(7U);
        ChassisAuto_ReadOdometrySamples(&samples);
        CHECK(virtual_calls==1U && !status_calls && !position_calls && !test_depth);
        if (index == 0U)
        {
            CHECK(samples.mask==2U && samples.raw[1]==16842754U && samples.sign[1]==1U && samples.ms[1]==102U);
            CHECK(g_chassis_auto_driver_rpm[1]==-7.0f && s_speed_tick[1]==103U && s_speed_valid==2U &&
                  s_position_due==2U && s_monitor_cursor==4U);
        }
        else CHECK(samples.mask==0U);
        if (index==8U) CHECK(g_chassis_auto_cut_vm && !g_chassis_auto_active && g_chassis_auto_state==CHASSIS_AUTO_FAULT);
    }
    begin(0xFFFFFFFEU);
    Auto_OdometrySample(1U,1U,0U,10U,0xFFFFFFFFU);
    Auto_Monitor(7U);
    ChassisAuto_ReadOdometrySamples(&samples);
    CHECK(samples.mask==2U && samples.ms[1]==0U && samples.raw[1]==16842754U);
    for (index=102U; index<=103U; index++)
    {
        begin(100U);
        Auto_OdometrySample(1U,1U,0U,10U,index);
        Auto_Monitor(7U);
        ChassisAuto_ReadOdometrySamples(&samples);
        CHECK(samples.mask==2U && samples.ms[1]==index && samples.raw[1]==10U && samples.sign[1]==0U);
    }
    begin(100U);
    s_mode = 1U;
    Auto_Monitor(7U);
    CHECK(!s_odometry_samples.mask && g_chassis_auto_driver_rpm[1]==-7.0f && virtual_calls==1U);

    begin(100U);
    s_monitor_cursor = 2U;
    Auto_Monitor(7U);
    CHECK(status_calls==1U && !virtual_calls && !position_calls && s_status_valid==2U && s_monitor_cursor==3U);
    s_status_valid = s_speed_valid = s_position_valid = 15U;
    s_status_tick[0] = s_status_tick[1] = s_status_tick[2] = s_status_tick[3] = test_tick;
    s_speed_tick[0] = s_speed_tick[1] = s_speed_tick[2] = s_speed_tick[3] = test_tick;
    s_position_tick[0] = s_position_tick[1] = s_position_tick[2] = s_position_tick[3] = test_tick;
    s_position_due = 2U;
    Auto_Monitor(7U);
    CHECK(position_calls==1U && !virtual_calls && s_position.raw[1]==900U && s_monitor_cursor==3U);
    CHECK(!test_depth);
}
