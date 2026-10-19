#include <stdio.h>
typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef uint32_t TickType_t;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
static TickType_t test_tick;
static TickType_t xTaskGetTickCount(void) { return test_tick; }
volatile uint32_t g_rtos_fault, g_safety_fault, g_safety_ready;
volatile uint32_t g_motion_diag_fault_latched, g_motion_diag_monitor_armed;
volatile uint32_t g_motion_diag_ack_request, g_motion_diag_ack_result;
#define MOTION_BLOCK_VM_OFF         0x01U
#define MOTION_BLOCK_PLATFORM       0x02U
#define MOTION_BLOCK_CAN            0x04U
#define MOTION_BLOCK_POLICY_PENDING 0x08U
#define MOTION_BLOCK_SPEED_FEEDBACK 0x10U
#define MOTION_BLOCK_POSITION_FEEDBACK 0x20U
#define MOTION_BLOCK_DIAG_LATCHED   0x40U
#define MOTION_STOP_IDLE 0U
#define MOTION_STOP_PENDING 1U
#define MOTION_STOP_ACKED 2U
#define MOTION_STOP_VM_CUT 3U
#define MOTION_STOP_CANCELLED 4U
#define MOTION_STOP_ACK_WAIT_MS 500U
volatile uint32_t g_motion_auto_stop_state = MOTION_STOP_IDLE;
volatile uint32_t g_motion_auto_stop_ack_mask = 0U;
volatile uint32_t g_motion_auto_stop_fail_mask = 0U;
volatile uint32_t g_motion_auto_stop_result[4] = {0U};
volatile uint32_t g_motion_auto_stop_cut_vm = 0U;
volatile uint32_t g_motion_auto_stop_session = 0U;
volatile uint32_t g_motion_auto_stop_cause = 0U;
static uint32_t s_auto_stop_pending = 0U;
static uint32_t s_auto_stop_inflight = 0U;
static uint32_t s_auto_stop_seen_latch = 0U;
static uint32_t s_auto_stop_vm_requested = 0U;
static TickType_t s_auto_stop_start_tick = 0U;
/* 已在短临界区内调用；只发布关VM标志 */
static void Safety_MotionStopCut(uint32_t cause)
{
    g_motion_auto_stop_state = MOTION_STOP_VM_CUT;
    g_motion_auto_stop_cut_vm = 1U;
    g_motion_auto_stop_cause = cause;
    s_auto_stop_pending = 0U;
}

/* Safety独立推进总等待时间，不等待CAN任务 */
static void Safety_MotionStopUpdate(uint32_t vm_request)
{
    TickType_t now;
    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    s_auto_stop_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (g_motion_diag_fault_latched == 0U)
    {
        if (vm_request == 0U)
        {
            /* 仅初始或撤VM后明确确认才重置，不靠反馈恢复重置 */
            g_motion_auto_stop_state = MOTION_STOP_IDLE;
            g_motion_auto_stop_cut_vm = 0U;
            g_motion_auto_stop_cause = 0U;
            s_auto_stop_pending = 0U;
            s_auto_stop_inflight = 0U;
            s_auto_stop_seen_latch = 0U;
        }
        taskEXIT_CRITICAL();
        return;
    }
    if (vm_request != 1U)
    {
        if (g_motion_auto_stop_state == MOTION_STOP_PENDING)
        {
            g_motion_auto_stop_state = MOTION_STOP_CANCELLED;
            s_auto_stop_pending = 0U;
        }
        taskEXIT_CRITICAL();
        return;
    }
    if (s_auto_stop_seen_latch == 0U)
    {
        /* 每个锁存会话只请求一次，禁止无限重试 */
        s_auto_stop_seen_latch = 1U;
        g_motion_auto_stop_session++;
        g_motion_auto_stop_state = MOTION_STOP_PENDING;
        g_motion_auto_stop_ack_mask = 0U;
        g_motion_auto_stop_fail_mask = 0U;
        g_motion_auto_stop_result[0] = 0U;
        g_motion_auto_stop_result[1] = 0U;
        g_motion_auto_stop_result[2] = 0U;
        g_motion_auto_stop_result[3] = 0U;
        g_motion_auto_stop_cause = 0U;
        s_auto_stop_pending = 0x0FU;
        s_auto_stop_inflight = 0U;
        s_auto_stop_start_tick = now;
    }
    if (g_motion_auto_stop_state == MOTION_STOP_PENDING)
    {
        if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
            (g_safety_ready != 1U)) Safety_MotionStopCut(3U);
        else if ((TickType_t)(now - s_auto_stop_start_tick) >=
                 pdMS_TO_TICKS(MOTION_STOP_ACK_WAIT_MS))
            Safety_MotionStopCut(2U);
    }
    taskEXIT_CRITICAL();
}

uint32_t Safety_MotionStopTake(uint8_t *address, uint32_t *session)
{
    uint32_t i;
    uint32_t bit;
    uint32_t taken = 0U;
    if ((address == NULL) || (session == NULL)) return 0U;
    taskENTER_CRITICAL();
    if ((g_motion_auto_stop_state == MOTION_STOP_PENDING) &&
        (s_auto_stop_vm_requested == 1U) && (s_auto_stop_inflight == 0U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_safety_ready == 1U))
    {
        if ((TickType_t)(xTaskGetTickCount() - s_auto_stop_start_tick) >=
            pdMS_TO_TICKS(MOTION_STOP_ACK_WAIT_MS)) Safety_MotionStopCut(2U);
        else
        {
            for (i = 0U; i < 4U; i++)
            {
                bit = 1UL << i;
                if ((s_auto_stop_pending & bit) != 0U)
                {
                    s_auto_stop_pending &= ~bit;
                    s_auto_stop_inflight = bit;
                    *address = (uint8_t)(i + 1U);
                    *session = g_motion_auto_stop_session;
                    taken = 1U;
                    break;
                }
            }
        }
    }
    taskEXIT_CRITICAL();
    return taken;
}

void Safety_MotionStopReport(uint8_t address, uint32_t session,
                             uint32_t result)
{
    uint32_t bit;
    if ((address < 1U) || (address > 4U)) return;
    bit = 1UL << (address - 1U);
    taskENTER_CRITICAL();
    if ((session == g_motion_auto_stop_session) &&
        (s_auto_stop_inflight == bit))
    {
        s_auto_stop_inflight = 0U;
        g_motion_auto_stop_result[address - 1U] = result;
        if (result == 1U) g_motion_auto_stop_ack_mask |= bit;
        else g_motion_auto_stop_fail_mask |= bit;
        /* 超时或撤VM后的晚回报只记录，不得恢复完成态 */
        if (g_motion_auto_stop_state == MOTION_STOP_PENDING)
        {
            if (result != 1U) Safety_MotionStopCut(1U);
            else if ((TickType_t)(xTaskGetTickCount() - s_auto_stop_start_tick) >=
                     pdMS_TO_TICKS(MOTION_STOP_ACK_WAIT_MS))
                Safety_MotionStopCut(2U);
            else if (g_motion_auto_stop_ack_mask == 0x0FU)
                g_motion_auto_stop_state = MOTION_STOP_ACKED;
        }
    }
    taskEXIT_CRITICAL();
}

/* 反馈诊断锁存只记账，不改原故障、供电、喂狗或发送CAN */
static void Safety_MotionDiagLatchUpdate(uint32_t vm_request,
                                        uint32_t speed_fresh,
                                        uint32_t position_fresh)
{
    if (vm_request != 1U)
    {
        /* 正常撤VM不作为新反馈故障；已有锁存仍保留 */
        g_motion_diag_monitor_armed = 0U;
    }
    else if ((speed_fresh == 0x0FU) && (position_fresh == 0x0FU))
    {
        /* 首次采集未完成时不误锁存；恢复新鲜也不清旧记录 */
        g_motion_diag_monitor_armed = 1U;
    }
    else if (g_motion_diag_monitor_armed == 1U)
    {
        if (speed_fresh != 0x0FU)
        {
            g_motion_diag_fault_latched |= MOTION_BLOCK_SPEED_FEEDBACK;
        }
        if (position_fresh != 0x0FU)
        {
            g_motion_diag_fault_latched |= MOTION_BLOCK_POSITION_FEEDBACK;
        }
    }

    if (g_motion_diag_ack_request == 1U)
    {
        /* 本次立即处理，失败不排队到条件恢复时自动确认 */
        g_motion_diag_ack_request = 0U;
        if ((vm_request == 0U) && (g_rtos_fault == 0U) &&
            (g_safety_fault == 0U) && (g_safety_ready == 1U))
        {
            /* 只确认这层诊断，不代表修复硬件或重新允许运动 */
            g_motion_diag_fault_latched = 0U;
            g_motion_diag_monitor_armed = 0U;
            g_motion_diag_ack_result = 1U;
        }
        else
        {
            g_motion_diag_ack_result = 9U;
        }
    }
}


static int checks;
#define CHECK(x) do { checks++; if (!(x)) { printf("FAIL line %d: %s\n",__LINE__,#x); return 1; } } while(0)
static void reset_test(void) {
    uint32_t i;
    test_tick=0; g_rtos_fault=0; g_safety_fault=0; g_safety_ready=1;
    g_motion_diag_fault_latched=0; g_motion_diag_monitor_armed=0;
    g_motion_diag_ack_request=0; g_motion_diag_ack_result=0;
    g_motion_auto_stop_state=0; g_motion_auto_stop_ack_mask=0;
    g_motion_auto_stop_fail_mask=0; g_motion_auto_stop_cut_vm=0;
    g_motion_auto_stop_session=0; g_motion_auto_stop_cause=0;
    s_auto_stop_pending=0; s_auto_stop_inflight=0; s_auto_stop_seen_latch=0;
    s_auto_stop_vm_requested=0; s_auto_stop_start_tick=0;
    for(i=0;i<4;i++) g_motion_auto_stop_result[i]=0;
}
int main(void) {
    uint8_t addr, oldaddr; uint32_t token, oldtoken, i;
    reset_test(); Safety_MotionStopUpdate(1);
    CHECK(g_motion_auto_stop_state==MOTION_STOP_IDLE);
    CHECK(Safety_MotionStopTake(&addr,&token)==0);
    g_motion_diag_fault_latched=0x30; Safety_MotionStopUpdate(1);
    CHECK(g_motion_auto_stop_state==MOTION_STOP_PENDING);
    CHECK(Safety_MotionStopTake(NULL,&token)==0);
    for(i=1;i<=4;i++) {
        CHECK(Safety_MotionStopTake(&addr,&token)==1); CHECK(addr==i);
        CHECK(Safety_MotionStopTake(&oldaddr,&oldtoken)==0);
        test_tick+=50; Safety_MotionStopReport(addr,token,1);
        CHECK(g_motion_auto_stop_result[i-1]==1);
    }
    CHECK(g_motion_auto_stop_state==MOTION_STOP_ACKED);
    CHECK(g_motion_auto_stop_ack_mask==15); CHECK(g_motion_auto_stop_cut_vm==0);
    oldtoken=g_motion_auto_stop_session;
    test_tick=10000; Safety_MotionStopUpdate(1);
    CHECK(g_motion_auto_stop_session==oldtoken); CHECK(Safety_MotionStopTake(&addr,&token)==0);

    reset_test(); g_motion_diag_fault_latched=0x30; Safety_MotionStopUpdate(1);
    CHECK(Safety_MotionStopTake(&addr,&token)==1);
    Safety_MotionStopReport(4,token,1); CHECK(g_motion_auto_stop_ack_mask==0);
    Safety_MotionStopReport(addr,token+1,1); CHECK(g_motion_auto_stop_ack_mask==0);
    Safety_MotionStopReport(addr,token,5);
    CHECK(g_motion_auto_stop_state==MOTION_STOP_VM_CUT);
    CHECK(g_motion_auto_stop_cause==1); CHECK(g_motion_auto_stop_cut_vm==1);
    CHECK(g_motion_auto_stop_fail_mask==1); CHECK(Safety_MotionStopTake(&addr,&token)==0);
    Safety_MotionStopUpdate(0); CHECK(g_motion_auto_stop_cut_vm==1);
    g_motion_diag_fault_latched=0; Safety_MotionStopUpdate(0);
    CHECK(g_motion_auto_stop_state==MOTION_STOP_IDLE); CHECK(g_motion_auto_stop_cut_vm==0);

    reset_test(); g_motion_diag_fault_latched=0x30; Safety_MotionStopUpdate(1);
    test_tick=499; Safety_MotionStopUpdate(1); CHECK(g_motion_auto_stop_state==MOTION_STOP_PENDING);
    test_tick=500; Safety_MotionStopUpdate(1);
    CHECK(g_motion_auto_stop_cause==2); CHECK(g_motion_auto_stop_cut_vm==1);

    reset_test(); g_motion_diag_fault_latched=0x30; Safety_MotionStopUpdate(1);
    CHECK(Safety_MotionStopTake(&addr,&token)==1);
    test_tick=500; Safety_MotionStopReport(addr,token,1);
    CHECK(g_motion_auto_stop_state==MOTION_STOP_VM_CUT); CHECK(g_motion_auto_stop_cause==2);

    reset_test(); test_tick=0xFFFFFFF0U; g_motion_diag_fault_latched=0x30;
    Safety_MotionStopUpdate(1); test_tick=0x1E4U; Safety_MotionStopUpdate(1);
    CHECK(g_motion_auto_stop_cause==2); CHECK(g_motion_auto_stop_cut_vm==1);

    reset_test(); g_motion_diag_fault_latched=0x30; Safety_MotionStopUpdate(1);
    CHECK(Safety_MotionStopTake(&oldaddr,&oldtoken)==1);
    Safety_MotionStopUpdate(0); CHECK(g_motion_auto_stop_state==MOTION_STOP_CANCELLED);
    Safety_MotionStopReport(oldaddr,oldtoken,1);
    CHECK(g_motion_auto_stop_state==MOTION_STOP_CANCELLED);
    g_motion_diag_fault_latched=0; Safety_MotionStopUpdate(0);
    g_motion_diag_fault_latched=0x30; Safety_MotionStopUpdate(1);
    CHECK(Safety_MotionStopTake(&addr,&token)==1); CHECK(token!=oldtoken);
    Safety_MotionStopReport(oldaddr,oldtoken,1); CHECK(g_motion_auto_stop_ack_mask==0);

    reset_test(); g_motion_diag_fault_latched=0x30; g_safety_fault=0x20;
    Safety_MotionStopUpdate(1); CHECK(g_motion_auto_stop_cause==3);
    CHECK(g_motion_auto_stop_cut_vm==1);

    reset_test(); Safety_MotionDiagLatchUpdate(1,0,0);
    CHECK(g_motion_diag_fault_latched==0); CHECK(g_motion_diag_monitor_armed==0);
    Safety_MotionDiagLatchUpdate(1,15,15); CHECK(g_motion_diag_monitor_armed==1);
    Safety_MotionDiagLatchUpdate(1,0,0); CHECK(g_motion_diag_fault_latched==0x30);
    Safety_MotionDiagLatchUpdate(1,15,15); CHECK(g_motion_diag_fault_latched==0x30);
    g_motion_diag_ack_request=1; Safety_MotionDiagLatchUpdate(1,15,15);
    CHECK(g_motion_diag_ack_result==9); CHECK(g_motion_diag_ack_request==0);
    Safety_MotionDiagLatchUpdate(0,0,0); CHECK(g_motion_diag_fault_latched==0x30);
    g_motion_diag_ack_request=1; Safety_MotionDiagLatchUpdate(0,0,0);
    CHECK(g_motion_diag_ack_result==1); CHECK(g_motion_diag_fault_latched==0);
    CHECK(g_safety_fault==0);
    printf("PASS: %d assertions on extracted actual C stop/latch functions\n",checks);
    return 0;
}