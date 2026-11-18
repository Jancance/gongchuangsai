#include "chassis_auto_test.c"

uint32_t test_tick, test_depth;
volatile uint32_t g_vm_test_request, g_rtos_fault, g_safety_fault, g_safety_ready, g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched, g_motion_auto_stop_state, g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active, g_wheel_test_cut_vm, g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4], g_can_poll_flags[4], g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
volatile uint32_t checks, failures, first_failure;

static uint32_t enable_mask, normal, warm_reads, sends, stops, stop_delay, drift, drift_samples, epoch;
static uint32_t fault_mode, fault_address, status_reads, feedback_reads;
static uint32_t fail_status, fail_status_address, status_protect, attempted_status;
static uint32_t slot, phase;

static void check(uint32_t condition,uint32_t line)
{
    checks++;
    if (!condition) { failures++; if (!first_failure) first_failure=line; }
}
#define CHECK(condition) check((condition),__LINE__)

void Power_ReadCommandStatus(PowerCommandStatus *s)
{ *s=(PowerCommandStatus){g_vm_test_request,1U,0U,epoch}; }
uint32_t StartupAxes_Holding(void) { return 0U; }
uint32_t CAN1_EmmSettingsConfirmed(void) { return 1U; }

uint32_t CAN1_ReadStatus(uint8_t address,CanRxMsg *reply)
{
    CHECK(!test_depth && address==1U);
    normal=1U; warm_reads++;
    reply->Data[1]=0U;
    test_tick+=2U;
    return 1U;
}
uint32_t CAN1_ReadStatusBounded(uint8_t address,uint32_t expected,CanRxMsg *reply,uint32_t *ms)
{
    CHECK(normal && !test_depth && phase==7U && expected==slot && address>=1U && address<=4U);
    status_reads++;
    attempted_status=address-1U;
    if (fail_status && (!fail_status_address || address==fail_status_address)) {
        fail_status=0U; test_tick+=7U; return CAN1_TIMING_WINDOW;
    }
    reply->Data[1]=(enable_mask>>(address-1U))&1U;
    if (status_protect) reply->Data[1]|=4U;
    *ms=(test_tick+=2U);
    return 1U;
}
uint32_t CAN1_ReadEmmSystemFeedbackBounded(uint8_t address,uint32_t expected,CanRxMsg *reply,
    CAN1_EmmSystemFeedback *f)
{
    (void)reply;
    CHECK(normal && !test_depth && phase<=5U && (phase&1U) && expected==slot);
    feedback_reads++;
    *f=(CAN1_EmmSystemFeedback){test_tick,test_tick+2U,600U+address,0,0U,0U,0U,address,1U};
    test_tick+=2U;
    if (drift && address==3U && g_chassis_auto_state==CHASSIS_AUTO_SETTLE) {
        uint32_t j=drift_samples++;
        if (drift==1U) f->position_counts+=j;
        else if (drift==2U) {
            f->position_counts=(j%3U)!=0U;
            f->position_sign=(j%3U)==2U;
        } else f->speed_decirpm=10;
    }
    return 1U;
}
uint32_t CAN1_ReadEmmSystemFeedback(uint8_t address,CanRxMsg *reply,CAN1_EmmSystemFeedback *f)
{ (void)address; (void)reply; (void)f; CHECK(0U); return 0U; }
uint32_t CAN1_ReadPosition(uint8_t address,CanRxMsg *reply,uint8_t *sign,uint32_t *position)
{ (void)address; (void)reply; (void)sign; (void)position; CHECK(0U); return 0U; }
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address,uint8_t dir,uint16_t raw,uint16_t accel,CanRxMsg *reply)
{ (void)address; (void)dir; (void)raw; (void)accel; (void)reply; CHECK(0U); return 0U; }
uint32_t CAN1_SetMotorEnable(uint8_t address,uint8_t on,CanRxMsg *reply)
{
    (void)reply;
    CHECK(normal && !test_depth && !(phase&1U));
    if (on) enable_mask|=1UL<<(address-1U);
    else enable_mask&=~(1UL<<(address-1U));
    test_tick+=2U;
    return 1U;
}
uint32_t CAN1_SetEmmSpeedRawBounded(uint8_t address,uint8_t dir,uint16_t raw,uint16_t accel,
    uint32_t expected,CanRxMsg *reply,CAN1_EmmSpeedReceipt *r)
{
    (void)dir; (void)raw; (void)accel; (void)reply;
    CHECK(normal && !test_depth && address==phase/2U+1U && !(phase&1U) && expected==slot);
    sends++;
    *r=(CAN1_EmmSpeedReceipt){test_tick,test_tick+2U,1U,1U,1U};
    test_tick+=2U;
    if (address==fault_address) {
        if (fault_mode==1U) { test_tick+=5U; return CAN1_TIMING_WINDOW; }
        if (fault_mode==2U) CHECK(ChassisAuto_RequestStop());
        if (fault_mode==3U) epoch++;
        if (fault_mode==4U) { g_can_ctrl_fault_latched=1U; return 8U; }
    }
    return 1U;
}
uint32_t CAN1_StopMotor(uint8_t address,CanRxMsg *reply)
{
    (void)reply;
    CHECK(normal && !test_depth && address==stops+1U);
    stops++;
    test_tick+=stops==4U && stop_delay ? stop_delay : 2U;
    return 1U;
}
void Safety_CanReport(uint32_t a,uint32_t b,uint32_t c,const volatile TickType_t ticks[4])
{ (void)a; (void)b; (void)c; (void)ticks; }
void Safety_MotionFeedbackReport(uint32_t a,uint32_t b,uint32_t c,
    const volatile TickType_t speed[4],const volatile TickType_t position[4])
{ (void)a; (void)b; (void)c; (void)speed; (void)position; }

static void step(void)
{
    test_tick=slot;
    ChassisAuto_SafetyUpdate(g_vm_test_request);
    ChassisAuto_Zone2CanStep(slot,phase);
    ChassisAuto_Zone2SafetyReport();
    CHECK(!test_depth);
    slot+=10U; phase=(phase+1U)%8U;
}
static void setup(uint32_t initial)
{
    ChassisZone2CanStatus s;
    uint32_t n;
    ChassisAuto_Init();
    enable_mask=normal=warm_reads=sends=stops=stop_delay=drift=drift_samples=0U;
    fault_mode=fault_address=status_reads=feedback_reads=0U;
    fail_status=fail_status_address=status_protect=attempted_status=g_can_ctrl_fault_latched=0U;
    epoch=1U; phase=0U; slot=test_tick=initial;
    g_vm_test_request=0U;
    g_safety_ready=g_can_guard_ready=1U;
    CHECK(ChassisAuto_Zone2Prime(3U));
    for (n=0U;n<45U;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(warm_reads==1U && status_reads>=4U && feedback_reads>=4U);
    CHECK(s.feedback_ready && !s.ready && s.state==CHASSIS_AUTO_IDLE);
    CHECK(ChassisAuto_Zone2Begin(3U,s.session));
    for (n=0U;n<155U && g_chassis_auto_state!=CHASSIS_AUTO_START;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.feedback_ready && s.ready && s.session==1U && s.power_epoch==1U && !s.ack_generation);
    CHECK(enable_mask==15U && !sends && !stops);
}
static void command(uint32_t generation,uint16_t raw,uint32_t passes)
{
    ChassisZone2CanStatus s;
    ChassisMotorCommand c[4]={{1U,0U,raw},{2U,1U,raw},{3U,0U,raw},{4U,1U,raw}};
    while (phase) step();
    CHECK(ChassisAuto_Zone2Submit(3U,1U,generation,c)==passes);
    if (passes) { uint32_t i; for (i=0U;i<7U;i++) step(); }
    ChassisAuto_Zone2Read(&s);
    CHECK(!passes || fault_mode || (s.result==0U && s.ack_generation==generation &&
        s.possible_mask==15U && s.ack_mask==15U && s.tx_complete_mask==15U));
}

void test_entry(void)
{
    uint32_t n, missed;
    ChassisZone2CanStatus s, previous;
    ChassisMotorCommand zero[4]={{1U,0U,0U},{2U,1U,0U},{3U,0U,0U},{4U,1U,0U}};
    setup(1100U);
    command(1U,200U,0U);
    command(1U,0U,1U);
    CHECK(ChassisAuto_RequestStop());
    drift=2U;
    for (n=0U;n<190U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(stops==4U && s.wheel_quiet && s.result==1U && s.stop_ack_ms);
    CHECK(ChassisAuto_ReleaseCompleted(1U) && ChassisAuto_Zone2Begin(3U,1U));

    setup(2000U);
    command(1U,0U,1U);
    CHECK(ChassisAuto_RequestStop());
    drift=1U;
    for (n=0U;n<225U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(stops==4U && !s.wheel_quiet && s.result==2U && s.cause==7U);

    setup(3000U);
    command(1U,0U,1U);
    fault_mode=1U; fault_address=3U;
    command(2U,200U,1U);
    for (n=0U;n<20U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==1U && s.ack_mask==3U && s.possible_mask==7U && s.result==2U);
    CHECK(s.cause==4U && sends==7U && stops==4U && !s.wheel_quiet);

    setup(4000U);
    command(1U,0U,1U);
    fault_mode=2U; fault_address=4U;
    command(2U,200U,1U);
    for (n=0U;n<20U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==1U && s.ack_mask==7U && s.possible_mask==15U);
    CHECK(s.result==2U && sends==8U && stops==4U && !s.wheel_quiet);

    setup(5000U);
    command(1U,0U,1U);
    fault_mode=3U; fault_address=4U;
    command(2U,200U,1U);
    step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==1U && s.ack_mask==7U && s.possible_mask==15U && s.result==2U);
    CHECK(!s.wheel_quiet && !s.feedback_ready && s.cause==2U);

    setup(6000U);
    command(1U,0U,1U);
    fault_mode=4U; fault_address=2U;
    command(2U,200U,1U);
    for (n=0U;n<20U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==1U && s.ack_mask==1U && s.possible_mask==3U && s.result==2U);
    CHECK(s.cause==4U && stops==4U && g_can_ctrl_fault_latched);

    setup(7000U);
    command(1U,0U,1U);
    ChassisAuto_Zone2Read(&previous);
    fail_status=1U;
    step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.state==CHASSIS_AUTO_RUN && !s.cause);
    CHECK(s.status_misses[attempted_status]==1U &&
        s.status_ms[attempted_status]==previous.status_ms[attempted_status]);
    missed=attempted_status;
    for (n=2U;n<=6U;n++) command(n,0U,1U);
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==6U && !s.result && !s.cause && s.feedback_ready);
    CHECK(s.status_misses[missed]==1U && !s.status_miss_streak[missed] &&
        Auto_NavNewer(s.status_ms[missed],previous.status_ms[missed]));

    setup(8000U);
    command(1U,0U,1U);
    fail_status=1U;
    step();
    missed=attempted_status;
    fail_status=1U; fail_status_address=missed+1U;
    for (n=2U;n<=5U;n++) command(n,0U,1U);
    fault_mode=5U;
    command(6U,0U,1U);
    for (n=0U;n<16U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==5U && s.result==2U && s.cause==3U);
    CHECK(s.status_misses[missed]==2U && s.status_miss_streak[missed]==2U);

    setup(9000U);
    command(1U,0U,1U);
    status_protect=1U;
    step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.result==2U && s.cause==2U && !s.feedback_ready);

    setup(10000U);
    while (phase) step();
    CHECK(ChassisAuto_Zone2Submit(3U,1U,1U,zero));
    slot+=80U; /* Whole missed cycle, even though the RTOS phase still looks correct. */
    step();
    for (n=0U;n<16U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==0U && s.possible_mask==0U && !sends);
    CHECK(s.result==2U && s.cause==6U && stops==4U);

    setup(11000U);
    command(1U,0U,1U);
    CHECK(ChassisAuto_RequestStop());
    drift=3U;
    for (n=0U;n<225U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(stops==4U && !s.wheel_quiet && s.result==2U && s.cause==7U);
}
