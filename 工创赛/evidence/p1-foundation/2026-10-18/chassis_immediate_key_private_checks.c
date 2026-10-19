#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <float.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef struct {uint32_t ExtId;uint8_t DLC,Data[8];} CanRxMsg;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
static TickType_t test_tick;
static TickType_t xTaskGetTickCount(void){return test_tick;}
static unsigned int power_enabled,feeds;
static int plant_enabled[4];static int32_t plant_rpm[4];
static void Power_Enable(void){unsigned int i;if(!power_enabled){for(i=0;i<4;i++){plant_enabled[i]=1;plant_rpm[i]=0;}}power_enabled=1;}
static void Power_Disable(void){unsigned int i;power_enabled=0;for(i=0;i<4;i++)plant_rpm[i]=0;}
static void Watchdog_Feed(void){feeds++;}
volatile uint32_t g_rtos_fault;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t CAN1_SetMotorEnable(uint8_t,uint8_t,CanRxMsg*);
static uint32_t CAN1_ReadStatus(uint8_t,CanRxMsg*);
static uint32_t CAN1_ReadSpeed(uint8_t,CanRxMsg*,int32_t*);
static uint32_t CAN1_SetEmmSpeedRaw(uint8_t,uint8_t,uint16_t,uint8_t,CanRxMsg*);
static uint32_t CAN1_StopMotor(uint8_t,CanRxMsg*);
#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H


/* X形45度麦轮；数组按CAN地址1左后、2右后、3左前、4右前 */
#define CHASSIS_WHEEL_COUNT 4U
#define CHASSIS_LEFT_REAR 0U
#define CHASSIS_RIGHT_REAR 1U
#define CHASSIS_LEFT_FRONT 2U
#define CHASSIS_RIGHT_FRONT 3U

typedef struct
{
    float wheel_diameter_mm;
    float track_width_mm;
    float wheelbase_mm;
} ChassisGeometry;

typedef struct
{
    float vx_mm_s;       /* 向前为正 */
    float vy_mm_s;       /* 向左为正 */
    float wz_rad_s;      /* 俯视逆时针为正 */
} ChassisBodyVelocity;

typedef struct
{
    float rpm[CHASSIS_WHEEL_COUNT]; /* 逻辑正向，不是驱动器原始符号 */
} ChassisWheelSpeeds;

/* 用户报告轮径77，轮距/轴距约210；后两项尚非精确标定 */
extern const ChassisGeometry g_chassis_geometry;

/* 纯计算，无CAN/供电/允许输出；返回1成功、0无效且输出清零 */
/* 限值由调用方明确给出，不在本模块推断整车安全转速 */
/* 超限时四轮同比缩放；scale可为NULL，成功范围(0,1] */
uint32_t Chassis_Inverse(const ChassisGeometry *geometry,
    const ChassisBodyVelocity *body, float max_wheel_rpm,
    ChassisWheelSpeeds *wheels, float *scale);

/* 逻辑轮速估计车体速度；不是世界坐标定位或打滑补偿 */
uint32_t Chassis_Forward(const ChassisGeometry *geometry,
    const ChassisWheelSpeeds *wheels, ChassisBodyVelocity *body);

/* 仅FW_Emm且S_Vel_IS Disable使用：1原始单位=1RPM */
typedef struct
{
    uint8_t address;
    uint8_t direction;
    uint16_t speed_raw;
} ChassisMotorCommand;

/* 已实测各轮逻辑正向对应的驱动方向0/1/0/1 */
extern const uint8_t g_chassis_forward_direction[CHASSIS_WHEEL_COUNT];

/* 纯适配不发帧；返回1成功、0不可表示/无效且全部目标速度清零 */
/* 本阶段只接受整数RPM（允许浮点计算微小误差），不静默截断混合速度 */
uint32_t Chassis_EncodeEmm(const ChassisWheelSpeeds *logical,
    uint16_t max_raw, ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);

/* 输入为CAN1_ReadSpeed已解析的带符号RPM；valid_mask须是四轮新鲜有效位 */
/* 调用者负责真实时间戳新鲜度，本函数不把旧反馈自动当成有效 */
uint32_t Chassis_EmmFeedbackToLogical(const int32_t driver_rpm[CHASSIS_WHEEL_COUNT],
    uint32_t valid_mask, ChassisWheelSpeeds *logical);

/* 自动三段测试的纯指令准备：0前进、1左移、2逆时针，每轮绝对值1RPM */
/* 不含供电/使能/时间推进；实际发送仍须控制状态机授权 */
uint32_t Chassis_BasicTestCommands(const ChassisGeometry *geometry, uint32_t step,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);

#endif
#ifndef SAFETY_H
#define SAFETY_H


/* 当前关键任务编号：不把状态灯任务当成安全证明 */
#define SAFETY_TASK_APP 0U
#define SAFETY_TASK_RX  1U
#define SAFETY_TASK_KEY 2U

/* 应用监督故障，与g_rtos_fault分开记录 */
extern volatile uint32_t g_safety_fault;
extern volatile uint32_t g_safety_ready;
extern volatile uint32_t g_safety_count;
/* CAN健康和首次CAN故障上下文；节点位0～3对应地址1～4 */
extern volatile uint32_t g_can_guard_ready;
extern volatile uint32_t g_can_guard_bad_nodes;
/* 原因：1必要采集关闭，2有效数据缺失/过期，3堵转/保护 */
extern volatile uint32_t g_can_guard_cause;

/* 运动许可第一阶段：仅诊断禁止原因，尚不产生允许输出 */
#define MOTION_BLOCK_VM_OFF         0x01U
#define MOTION_BLOCK_PLATFORM       0x02U
#define MOTION_BLOCK_CAN            0x04U
/* 运动反馈时效、限值、启动授权、停止与重新允许尚未闭环 */
#define MOTION_BLOCK_POLICY_PENDING 0x08U
#define MOTION_BLOCK_SPEED_FEEDBACK 0x10U
#define MOTION_BLOCK_POSITION_FEEDBACK 0x20U
#define MOTION_BLOCK_DIAG_LATCHED   0x40U
/* 本阶段仍是静态诊断阈值，不能据此放行运动 */
#define SAFETY_MOTION_DIAG_FRESH_MS 1000U

/* 仅由Safety任务更新；当前阶段始终为0，不用Watch手动置1 */
extern volatile uint32_t g_motion_allowed;
/* 位掩码，可同时包含多个禁止原因；不等同于g_safety_fault */
extern volatile uint32_t g_motion_block_reason;
/* Safety独立计算的新鲜节点掩码，位0到3对应地址1到4 */
extern volatile uint32_t g_motion_speed_fresh_mask;
extern volatile uint32_t g_motion_position_fresh_mask;
/* 反馈诊断锁存：10速度异常、20位置异常；不是原Safety故障 */
extern volatile uint32_t g_motion_diag_fault_latched;
/* 两类四轮反馈曾完整新鲜后置1，正常撤VM后回0 */
extern volatile uint32_t g_motion_diag_monitor_armed;
/* 单次确认请求默认0；仅在VM请求0且平台健康时接受 */
extern volatile uint32_t g_motion_diag_ack_request;
/* 1确认成功，9条件不满足；拒绝的请求也消费，不延后执行 */
extern volatile uint32_t g_motion_diag_ack_result;

/* 独立CAN任务发布真实采样时间；报告时间不能代替采样时间 */
void Safety_MotionFeedbackReport(uint32_t enabled,
    uint32_t speed_valid_mask, uint32_t position_valid_mask,
    const volatile TickType_t *speed_ticks,
    const volatile TickType_t *position_ticks);

/* 静态自动停止：02只表示收到命令，不等于实际停车 */
#define MOTION_STOP_IDLE 0U
#define MOTION_STOP_PENDING 1U
#define MOTION_STOP_ACKED 2U
#define MOTION_STOP_VM_CUT 3U
#define MOTION_STOP_CANCELLED 4U
/* 从请求发布计时的静态候选，不是实际制动时间保证 */
#define MOTION_STOP_ACK_WAIT_MS 500U
extern volatile uint32_t g_motion_auto_stop_state;
extern volatile uint32_t g_motion_auto_stop_ack_mask;
extern volatile uint32_t g_motion_auto_stop_fail_mask;
extern volatile uint32_t g_motion_auto_stop_result[4];
extern volatile uint32_t g_motion_auto_stop_cut_vm;
extern volatile uint32_t g_motion_auto_stop_session;
/* cause：0无兜底、1应答失败、2总等待超时、3原平台故障 */
extern volatile uint32_t g_motion_auto_stop_cause;
/* 仅CAN任务调用，带会话号拒绝旧回报 */
uint32_t Safety_MotionStopTake(uint8_t *address, uint32_t *session);
void Safety_MotionStopReport(uint8_t address, uint32_t session,
                             uint32_t result);

/* 人工四轮短时测试：固定四台、raw1；不代表正式运动许可 */
#define WHEEL_TEST_RUN_MS 10000U
#define WHEEL_TEST_CUT_MS 10500U
#define WHEEL_TEST_PREP_MS 5000U
#define WHEEL_TEST_IDLE 0U
#define WHEEL_TEST_STARTING 1U
#define WHEEL_TEST_RUNNING 2U
#define WHEEL_TEST_STOPPING 3U
#define WHEEL_TEST_DONE 4U
#define WHEEL_TEST_CUT 5U
#define WHEEL_TEST_CANCELLED 6U
#define WHEEL_TEST_PREPARING 7U
extern volatile uint32_t g_wheel_test_state;
extern volatile uint32_t g_wheel_test_active;
extern volatile uint32_t g_wheel_test_cut_vm;
extern volatile uint32_t g_wheel_test_session;
/* 0正常定时关电，1启动失败，2停止失败，3截止超时，4健康丢失，5人工中止，6准备失败/超时 */
extern volatile uint32_t g_wheel_test_cause;
uint32_t Safety_WheelTestPrepare(void);
void Safety_WheelTestPrepareFailed(void);
uint32_t Safety_WheelTestArm(uint32_t *session);
uint32_t Safety_WheelTestStartAllowed(uint32_t session);
void Safety_WheelTestStartReport(uint32_t session, uint32_t result);
uint32_t Safety_WheelTestTakeStop(uint32_t *session);
void Safety_WheelTestStopReport(uint32_t session, uint32_t result);
void Safety_WheelTestAbort(void);

void Safety_Init(void);
void Safety_Report(uint32_t task_id);
void Safety_Update(uint32_t vm_request, uint32_t can_test_result);

void Safety_CanReport(uint32_t enabled, uint32_t valid_mask,
                      uint32_t protect_mask,
                      const volatile TickType_t *ticks);

#endif
#ifndef CHASSIS_AUTO_TEST_H
#define CHASSIS_AUTO_TEST_H

/* 仅用于1RPM三段调试，时间/反馈阈值未推广为正式运动限值 */
#define CHASSIS_AUTO_TARGET_RPM 5U
#define CHASSIS_AUTO_SPEED_LIMIT_RPM (CHASSIS_AUTO_TARGET_RPM + 2U)
#define CHASSIS_AUTO_PREP_MS 10000U
#define CHASSIS_AUTO_RUN_MS 10000U
#define CHASSIS_AUTO_STOP_MS 500U
#define CHASSIS_AUTO_GAP_MS 1000U
#define CHASSIS_AUTO_SETTLE_MS 2000U
#define CHASSIS_AUTO_FRESH_MS 1000U
#define CHASSIS_AUTO_TOTAL_MS 45000U
#define CHASSIS_AUTO_IDLE 0U
#define CHASSIS_AUTO_DISABLE 1U
#define CHASSIS_AUTO_VERIFY_OFF 2U
#define CHASSIS_AUTO_ENABLE 3U
#define CHASSIS_AUTO_VERIFY_ON 4U
#define CHASSIS_AUTO_START 5U
#define CHASSIS_AUTO_RUN 6U
#define CHASSIS_AUTO_STOP 7U
#define CHASSIS_AUTO_SETTLE 8U
#define CHASSIS_AUTO_DONE 9U
#define CHASSIS_AUTO_FAULT 10U
#define CHASSIS_AUTO_CANCELLED 11U

extern volatile uint32_t g_chassis_auto_request;
extern volatile uint32_t g_chassis_auto_dry_run;
extern volatile uint32_t g_chassis_auto_mode;
extern volatile uint32_t g_chassis_auto_state;
extern volatile uint32_t g_chassis_auto_segment; /* 0前进、1左移、2逆时针 */
extern volatile uint32_t g_chassis_auto_active;
extern volatile uint32_t g_chassis_auto_cut_vm;
extern volatile uint32_t g_chassis_auto_session;
/* result：0进行中/初始，1三段完成，2异常，3撤VM取消，9请求拒绝 */
extern volatile uint32_t g_chassis_auto_result;
/* cause：1准备，2健康，3反馈失效，4启动，5停止，6时限，7未停稳，8速度/符号异常，9人工中止 */
extern volatile uint32_t g_chassis_auto_cause;
extern volatile uint32_t g_chassis_auto_io_result;
extern volatile uint32_t g_chassis_auto_last_address;
extern volatile uint32_t g_chassis_auto_start_mask[3];
extern volatile uint32_t g_chassis_auto_stop_mask[3];
extern volatile int32_t g_chassis_auto_driver_rpm[4];

void ChassisAuto_Init(void);
uint32_t ChassisAuto_Begin(uint32_t dry_run);
void ChassisAuto_CanStep(void);
void ChassisAuto_Abort(void);
/* Safety任务独立调用，不发送/等待CAN */
void ChassisAuto_SafetyUpdate(uint32_t vm_request);
#endif

#define CHASSIS_PI 3.14159265358979323846f

const ChassisGeometry g_chassis_geometry = {77.0f, 210.0f, 210.0f};
const uint8_t g_chassis_forward_direction[CHASSIS_WHEEL_COUNT] = {0U, 1U, 0U, 1U};

static uint32_t Chassis_Finite(float value)
{
    /* 比较会拒绝NaN和正负无穷，不依赖不同工具链的isfinite实现 */
    return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1U : 0U;
}

static float Chassis_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t Chassis_GeometryFactors(const ChassisGeometry *g,
                                       float *k, float *mm_s_per_rpm)
{
    if ((g == NULL) || !Chassis_Finite(g->wheel_diameter_mm) ||
        !Chassis_Finite(g->track_width_mm) || !Chassis_Finite(g->wheelbase_mm) ||
        (g->wheel_diameter_mm <= 0.0f) || (g->track_width_mm <= 0.0f) ||
        (g->wheelbase_mm <= 0.0f)) return 0U;
    *k = 0.5f * g->track_width_mm + 0.5f * g->wheelbase_mm;
    *mm_s_per_rpm = (g->wheel_diameter_mm / 60.0f) * CHASSIS_PI;
    return (Chassis_Finite(*k) && (*k > 0.0f) &&
            Chassis_Finite(*mm_s_per_rpm) && (*mm_s_per_rpm > 0.0f)) ? 1U : 0U;
}

uint32_t Chassis_Inverse(const ChassisGeometry *geometry,
    const ChassisBodyVelocity *body, float max_wheel_rpm,
    ChassisWheelSpeeds *wheels, float *scale)
{
    float k, mm_s_per_rpm, turn, peak = 0.0f, factor = 1.0f;
    float rpm[CHASSIS_WHEEL_COUNT];
    uint32_t i;
    if (scale != NULL) *scale = 0.0f;
    if (wheels == NULL) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) wheels->rpm[i] = 0.0f;
    if ((body == NULL) || !Chassis_GeometryFactors(geometry, &k, &mm_s_per_rpm) ||
        !Chassis_Finite(body->vx_mm_s) || !Chassis_Finite(body->vy_mm_s) ||
        !Chassis_Finite(body->wz_rad_s) || !Chassis_Finite(max_wheel_rpm) ||
        (max_wheel_rpm <= 0.0f)) return 0U;
    turn = k * body->wz_rad_s;
    if (!Chassis_Finite(turn)) return 0U;
    /* 轮缘线速按当前X形和车体前/左/逆时针坐标合同计算 */
    rpm[CHASSIS_LEFT_REAR] = (body->vx_mm_s + body->vy_mm_s - turn) / mm_s_per_rpm;
    rpm[CHASSIS_RIGHT_REAR] = (body->vx_mm_s - body->vy_mm_s + turn) / mm_s_per_rpm;
    rpm[CHASSIS_LEFT_FRONT] = (body->vx_mm_s - body->vy_mm_s - turn) / mm_s_per_rpm;
    rpm[CHASSIS_RIGHT_FRONT] = (body->vx_mm_s + body->vy_mm_s + turn) / mm_s_per_rpm;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (!Chassis_Finite(rpm[i])) return 0U;
        if (Chassis_Abs(rpm[i]) > peak) peak = Chassis_Abs(rpm[i]);
    }
    if (peak > max_wheel_rpm) factor = max_wheel_rpm / peak;
    if (!Chassis_Finite(factor) || (factor <= 0.0f)) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) wheels->rpm[i] = rpm[i] * factor;
    if (scale != NULL) *scale = factor;
    return 1U;
}

uint32_t Chassis_Forward(const ChassisGeometry *geometry,
    const ChassisWheelSpeeds *wheels, ChassisBodyVelocity *body)
{
    float k, mm_s_per_rpm, lr, rr, lf, rf;
    ChassisBodyVelocity result;
    uint32_t i;
    if (body == NULL) return 0U;
    body->vx_mm_s = body->vy_mm_s = body->wz_rad_s = 0.0f;
    if ((wheels == NULL) || !Chassis_GeometryFactors(geometry, &k, &mm_s_per_rpm))
        return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        if (!Chassis_Finite(wheels->rpm[i])) return 0U;
    /* 先取四分之一，减少求和中间量溢出的机会 */
    lr = 0.25f * wheels->rpm[CHASSIS_LEFT_REAR] * mm_s_per_rpm;
    rr = 0.25f * wheels->rpm[CHASSIS_RIGHT_REAR] * mm_s_per_rpm;
    lf = 0.25f * wheels->rpm[CHASSIS_LEFT_FRONT] * mm_s_per_rpm;
    rf = 0.25f * wheels->rpm[CHASSIS_RIGHT_FRONT] * mm_s_per_rpm;
    result.vx_mm_s = lr + rr + lf + rf;
    result.vy_mm_s = lr - rr - lf + rf;
    result.wz_rad_s = (-lr + rr - lf + rf) / k;
    if (!Chassis_Finite(result.vx_mm_s) || !Chassis_Finite(result.vy_mm_s) ||
        !Chassis_Finite(result.wz_rad_s)) return 0U;
    *body = result;
    return 1U;
}

/* 计算逆解时的舍入误差容差，不作为低速死区或运动参数 */
#define CHASSIS_INTEGER_RPM_TOLERANCE 0.0001f

static void Chassis_ClearMotorCommands(ChassisMotorCommand *commands)
{
    uint32_t i;
    if (commands == NULL) return;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        commands[i].address = (uint8_t)(i + 1U);
        commands[i].direction = g_chassis_forward_direction[i];
        commands[i].speed_raw = 0U;
    }
}

uint32_t Chassis_EncodeEmm(const ChassisWheelSpeeds *logical,
    uint16_t max_raw, ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisMotorCommand result[CHASSIS_WHEEL_COUNT];
    uint32_t i;
    float magnitude;
    uint16_t raw;
    Chassis_ClearMotorCommands(commands);
    if ((commands == NULL) || (logical == NULL) || (max_raw == 0U) ||
        (max_raw > 3000U)) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (!Chassis_Finite(logical->rpm[i])) return 0U;
        magnitude = Chassis_Abs(logical->rpm[i]);
        if (magnitude > (float)max_raw + CHASSIS_INTEGER_RPM_TOLERANCE) return 0U;
        /* 已先限到协议范围，转换不会溢出；拒绝不能表示的小数速度 */
        raw = (uint16_t)(magnitude + 0.5f);
        if ((raw > max_raw) ||
            (Chassis_Abs(magnitude - (float)raw) > CHASSIS_INTEGER_RPM_TOLERANCE))
            return 0U;
        result[i].address = (uint8_t)(i + 1U);
        result[i].speed_raw = raw;
        result[i].direction = g_chassis_forward_direction[i];
        if ((raw != 0U) && (logical->rpm[i] < 0.0f)) result[i].direction ^= 1U;
    }
    /* 四台全部有效后一次提交；后面节点失败不残留前面节点的新目标 */
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) commands[i] = result[i];
    return 1U;
}

uint32_t Chassis_EmmFeedbackToLogical(const int32_t driver_rpm[CHASSIS_WHEEL_COUNT],
    uint32_t valid_mask, ChassisWheelSpeeds *logical)
{
    uint32_t i;
    float rpm[CHASSIS_WHEEL_COUNT];
    if (logical == NULL) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) logical->rpm[i] = 0.0f;
    if ((driver_rpm == NULL) || (valid_mask != 0x0FU)) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        /* ReadSpeed来自16位幅值，不接受其他来源溢出值 */
        if ((driver_rpm[i] < -65535) || (driver_rpm[i] > 65535)) return 0U;
        rpm[i] = (float)driver_rpm[i];
        if (g_chassis_forward_direction[i] == 1U) rpm[i] = -rpm[i];
    }
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) logical->rpm[i] = rpm[i];
    return 1U;
}

uint32_t Chassis_BasicTestCommands(const ChassisGeometry *geometry, uint32_t step,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisBodyVelocity body = {0.0f, 0.0f, 0.0f};
    ChassisWheelSpeeds logical;
    float k, mm_s_per_rpm;
    Chassis_ClearMotorCommands(commands);
    if ((commands == NULL) || (step > 2U) ||
        !Chassis_GeometryFactors(geometry, &k, &mm_s_per_rpm)) return 0U;
    if (step == 0U) body.vx_mm_s = mm_s_per_rpm;
    else if (step == 1U) body.vy_mm_s = mm_s_per_rpm;
    else body.wz_rad_s = mm_s_per_rpm / k;
    if (!Chassis_Inverse(geometry, &body, 1.0f, &logical, NULL)) return 0U;
    return Chassis_EncodeEmm(&logical, 1U, commands);
}

/* 仅用于当前静态平台，后续运动控制需另行冻结更快的限值 */
#define SAFETY_STARTUP_MS 1000U
#define SAFETY_APP_MS     1000U
#define SAFETY_RX_MS       300U
#define SAFETY_KEY_MS      100U
/* 静态接入候选：先等待驱动器上电，已有健康后不重新宽限 */
#define SAFETY_CAN_BOOT_MS  3000U
#define SAFETY_CAN_FRESH_MS 1000U
#define SAFETY_FAULT_CAN_DATA    0x20U
#define SAFETY_FAULT_CAN_PROTECT 0x40U

volatile uint32_t g_safety_fault = 0U;
volatile uint32_t g_safety_ready = 0U;
volatile uint32_t g_safety_count = 0U;
/* 运动许可诊断不参与现有VM供电决策；本阶段固定禁止运动 */
volatile uint32_t g_motion_allowed = 0U;
volatile uint32_t g_motion_block_reason = MOTION_BLOCK_VM_OFF |
    MOTION_BLOCK_PLATFORM | MOTION_BLOCK_CAN | MOTION_BLOCK_POLICY_PENDING |
    MOTION_BLOCK_SPEED_FEEDBACK | MOTION_BLOCK_POSITION_FEEDBACK;

volatile uint32_t g_motion_speed_fresh_mask = 0U;
volatile uint32_t g_motion_position_fresh_mask = 0U;
/* 仅诊断：恢复新鲜不会清除此记录；复位会清除RAM记录 */
volatile uint32_t g_motion_diag_fault_latched = 0U;
volatile uint32_t g_motion_diag_monitor_armed = 0U;
volatile uint32_t g_motion_diag_ack_request = 0U;
volatile uint32_t g_motion_diag_ack_result = 0U;
volatile uint32_t g_motion_auto_stop_state = MOTION_STOP_IDLE;
volatile uint32_t g_motion_auto_stop_ack_mask = 0U;
volatile uint32_t g_motion_auto_stop_fail_mask = 0U;
volatile uint32_t g_motion_auto_stop_result[4] = {0U};
volatile uint32_t g_motion_auto_stop_cut_vm = 0U;
volatile uint32_t g_motion_auto_stop_session = 0U;
volatile uint32_t g_motion_auto_stop_cause = 0U;
volatile uint32_t g_wheel_test_state = WHEEL_TEST_IDLE;
volatile uint32_t g_wheel_test_active = 0U;
volatile uint32_t g_wheel_test_cut_vm = 0U;
volatile uint32_t g_wheel_test_session = 0U;
volatile uint32_t g_wheel_test_cause = 0U;
static uint32_t s_wheel_test_used = 0U;
static uint32_t s_wheel_test_vm_requested = 0U;
static TickType_t s_wheel_test_start_tick = 0U;

static uint32_t s_auto_stop_pending = 0U;
static uint32_t s_auto_stop_inflight = 0U;
static uint32_t s_auto_stop_seen_latch = 0U;
static uint32_t s_auto_stop_vm_requested = 0U;
static TickType_t s_auto_stop_start_tick = 0U;
static volatile uint32_t s_motion_feedback_enabled = 0U;
static volatile uint32_t s_motion_speed_valid = 0U;
static volatile uint32_t s_motion_position_valid = 0U;
static volatile TickType_t s_motion_speed_ticks[4];
static volatile TickType_t s_motion_position_ticks[4];

static TickType_t s_last_tick[3];
static uint32_t s_seen_mask = 0U;
static TickType_t s_start_tick = 0U;
static uint32_t s_started = 0U;
static uint32_t s_was_ready = 0U;

volatile uint32_t g_can_guard_ready = 0U;
volatile uint32_t g_can_guard_bad_nodes = 0U;
volatile uint32_t g_can_guard_cause = 0U;

/* 两个任务共享的快照字段，配合短临界区发布/读取 */
static volatile uint32_t s_can_enabled = 0U;
static volatile uint32_t s_can_valid = 0U;
static volatile uint32_t s_can_protect = 0U;
static volatile TickType_t s_can_ticks[4];
static uint32_t s_can_requested = 0U;
static uint32_t s_can_was_ready = 0U;
static TickType_t s_can_power_tick = 0U;

/* 调度前调用，默认不允许VM */
void Safety_Init(void)
{
    uint32_t i;

    ChassisAuto_Init();
    g_safety_fault = 0U;
    g_safety_ready = 0U;
    g_safety_count = 0U;
    g_motion_allowed = 0U;
    g_motion_block_reason = MOTION_BLOCK_VM_OFF | MOTION_BLOCK_PLATFORM |
                            MOTION_BLOCK_CAN | MOTION_BLOCK_POLICY_PENDING |
                            MOTION_BLOCK_SPEED_FEEDBACK |
                            MOTION_BLOCK_POSITION_FEEDBACK;
    g_motion_speed_fresh_mask = 0U;
    g_motion_position_fresh_mask = 0U;
    g_motion_diag_fault_latched = 0U;
    g_motion_diag_monitor_armed = 0U;
    g_motion_diag_ack_request = 0U;
    g_motion_diag_ack_result = 0U;
    g_motion_auto_stop_state = MOTION_STOP_IDLE;
    g_motion_auto_stop_ack_mask = 0U;
    g_motion_auto_stop_fail_mask = 0U;
    g_motion_auto_stop_cut_vm = 0U;
    g_motion_auto_stop_session = 0U;
    g_motion_auto_stop_cause = 0U;
    g_wheel_test_state = WHEEL_TEST_IDLE;
    g_wheel_test_active = 0U;
    g_wheel_test_cut_vm = 0U;
    g_wheel_test_session = 0U;
    g_wheel_test_cause = 0U;
    s_wheel_test_used = 0U;
    s_wheel_test_vm_requested = 0U;
    s_wheel_test_start_tick = 0U;
    s_auto_stop_pending = 0U;
    s_auto_stop_inflight = 0U;
    s_auto_stop_seen_latch = 0U;
    s_auto_stop_vm_requested = 0U;
    s_auto_stop_start_tick = 0U;
    for (i = 0U; i < 4U; i++) g_motion_auto_stop_result[i] = 0U;
    s_motion_feedback_enabled = 0U;
    s_motion_speed_valid = 0U;
    s_motion_position_valid = 0U;
    s_seen_mask = 0U;
    s_start_tick = 0U;
    s_started = 0U;
    s_was_ready = 0U;
    for (i = 0U; i < 3U; i++)
    {
        s_last_tick[i] = 0U;
    }
    g_can_guard_ready = 0U;
    g_can_guard_bad_nodes = 0U;
    g_can_guard_cause = 0U;
    s_can_enabled = 0U;
    s_can_valid = 0U;
    s_can_protect = 0U;
    s_can_requested = 0U;
    s_can_was_ready = 0U;
    s_can_power_tick = 0U;
    for (i = 0U; i < 4U; i++)
    {
        s_can_ticks[i] = 0U;
        s_motion_speed_ticks[i] = 0U;
        s_motion_position_ticks[i] = 0U;
    }
    Power_Disable();
}

/* 由相应任务在一轮实际工作结束后报告，不由其他任务代报 */
void Safety_Report(uint32_t task_id)
{
    if (task_id < 3U)
    {
        taskENTER_CRITICAL();
        s_last_tick[task_id] = xTaskGetTickCount();
        s_seen_mask |= 1UL << task_id;
        taskEXIT_CRITICAL();
    }
}

/* 任务A在刷新快照后发布，短临界区保证监督读到同一组数据 */
void Safety_CanReport(uint32_t enabled, uint32_t valid_mask,
                      uint32_t protect_mask,
                      const volatile TickType_t *ticks)
{
    uint32_t i;

    taskENTER_CRITICAL();
    s_can_enabled = (enabled == 1U) ? 1U : 0U;
    s_can_valid = (ticks != NULL) ? (valid_mask & 0x0FU) : 0U;
    s_can_protect = protect_mask & s_can_valid;
    for (i = 0U; i < 4U; i++)
    {
        s_can_ticks[i] = (ticks != NULL) ? ticks[i] : 0U;
    }
    taskEXIT_CRITICAL();
}

/* 只发布快照，不改变原状态保护、VM及喂狗决策 */
void Safety_MotionFeedbackReport(uint32_t enabled,
    uint32_t speed_valid_mask, uint32_t position_valid_mask,
    const volatile TickType_t *speed_ticks,
    const volatile TickType_t *position_ticks)
{
    uint32_t i;
    taskENTER_CRITICAL();
    s_motion_feedback_enabled = (enabled == 1U) ? 1U : 0U;
    s_motion_speed_valid = (speed_ticks != NULL) ?
                          (speed_valid_mask & 0x0FU) : 0U;
    s_motion_position_valid = (position_ticks != NULL) ?
                             (position_valid_mask & 0x0FU) : 0U;
    for (i = 0U; i < 4U; i++)
    {
        s_motion_speed_ticks[i] = (speed_ticks != NULL) ? speed_ticks[i] : 0U;
        s_motion_position_ticks[i] = (position_ticks != NULL) ? position_ticks[i] : 0U;
    }
    taskEXIT_CRITICAL();
}

/* 只由监督任务调用，不等待CAN，不操作接收FIFO */
static void Safety_CanGuardUpdate(uint32_t vm_request)
{
    TickType_t now;
    TickType_t ticks[4];
    uint32_t enabled;
    uint32_t valid;
    uint32_t protect;
    uint32_t fresh = 0U;
    uint32_t i;

    g_can_guard_ready = 0U;
    if (g_safety_fault != 0U)
    {
        /* 已锁定故障时保留首次CAN上下文，避免自身关电导致二次误报 */
        return;
    }
    if (vm_request != 1U)
    {
        /* 正常撤销供电不是通信故障，下次请求重新计上电等待 */
        s_can_requested = 0U;
        s_can_was_ready = 0U;
        return;
    }

    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    enabled = s_can_enabled;
    valid = s_can_valid;
    protect = s_can_protect;
    for (i = 0U; i < 4U; i++)
    {
        ticks[i] = s_can_ticks[i];
    }
    taskEXIT_CRITICAL();

    if (s_can_requested == 0U)
    {
        s_can_power_tick = now;
        s_can_requested = 1U;
        s_can_was_ready = 0U;
    }
    for (i = 0U; i < 4U; i++)
    {
        /* 监督自己重算年龄，不能只相信采集任务留下的有效mask */
        if (((valid & (1UL << i)) != 0U) &&
            ((TickType_t)(now - ticks[i]) <=
             pdMS_TO_TICKS(SAFETY_CAN_FRESH_MS)))
        {
            fresh |= 1UL << i;
        }
    }

    protect &= fresh;
    if (protect != 0U)
    {
        g_can_guard_bad_nodes = protect;
        g_can_guard_cause = 3U;
        g_safety_fault |= SAFETY_FAULT_CAN_PROTECT;
        return;
    }
    if ((enabled == 1U) && (fresh == 0x0FU))
    {
        s_can_was_ready = 1U;
        g_can_guard_ready = 1U;
        return;
    }

    if ((s_can_was_ready != 0U) ||
        ((TickType_t)(now - s_can_power_tick) >=
         pdMS_TO_TICKS(SAFETY_CAN_BOOT_MS)))
    {
        g_can_guard_bad_nodes =
            (enabled == 1U) ? (0x0FU & ~fresh) : 0x0FU;
        g_can_guard_cause = (enabled == 1U) ? 2U : 1U;
        g_safety_fault |= SAFETY_FAULT_CAN_DATA;
    }
}

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

/* 调用者持短临界区；不等待CAN，关电在本轮或下一轮Safety输出执行 */
static void Safety_WheelTestCut(uint32_t cause)
{
    if (cause != 0U) g_wheel_test_state = WHEEL_TEST_CUT;
    g_wheel_test_active = 0U;
    g_wheel_test_cut_vm = 1U;
    g_wheel_test_cause = cause;
}

/* 每20ms由Safety推进；即使CAN任务停止推进，截止时间仍生效 */
static void Safety_WheelTestUpdate(uint32_t vm_request)
{
    taskENTER_CRITICAL();
    s_wheel_test_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (vm_request != 1U)
    {
        if (g_wheel_test_active == 1U)
            g_wheel_test_state = WHEEL_TEST_CANCELLED;
        g_wheel_test_active = 0U;
        g_wheel_test_cut_vm = 0U;
        s_wheel_test_used = 0U;
    }
    else if (g_wheel_test_active == 1U)
    {
        if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
            (g_safety_ready != 1U) || (g_can_guard_ready != 1U) ||
            (g_motion_diag_fault_latched != 0U) ||
            (g_motion_auto_stop_state != MOTION_STOP_IDLE) ||
            (g_motion_auto_stop_cut_vm != 0U))
            Safety_WheelTestCut(4U);
        else if (g_wheel_test_state == WHEEL_TEST_PREPARING)
        {
            if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                pdMS_TO_TICKS(WHEEL_TEST_PREP_MS)) Safety_WheelTestCut(6U);
        }
        else if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                 pdMS_TO_TICKS(WHEEL_TEST_CUT_MS))
            Safety_WheelTestCut((g_wheel_test_state == WHEEL_TEST_DONE) ? 0U : 3U);
    }
    taskEXIT_CRITICAL();
}

/* 先占用本次供电会话，准备超时也由Safety独立关VM */
uint32_t Safety_WheelTestPrepare(void)
{
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((s_wheel_test_vm_requested == 1U) && (s_wheel_test_used == 0U) &&
        (g_wheel_test_active == 0U) && (g_wheel_test_cut_vm == 0U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
        (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U))
    {
        s_wheel_test_used = 1U;
        s_wheel_test_start_tick = xTaskGetTickCount();
        g_wheel_test_session++;
        g_wheel_test_state = WHEEL_TEST_PREPARING;
        g_wheel_test_active = 1U;
        g_wheel_test_cause = 0U;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void Safety_WheelTestPrepareFailed(void)
{
    taskENTER_CRITICAL();
    if ((g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_PREPARING)) Safety_WheelTestCut(6U);
    taskEXIT_CRITICAL();
}

uint32_t Safety_WheelTestArm(uint32_t *session)
{
    uint32_t accepted = 0U;
    if (session == NULL) return 0U;
    taskENTER_CRITICAL();
    if ((s_wheel_test_vm_requested == 1U) && (g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_PREPARING) &&
        (g_wheel_test_cut_vm == 0U) && (g_rtos_fault == 0U) &&
        (g_safety_fault == 0U) && (g_safety_ready == 1U) &&
        (g_can_guard_ready == 1U) && (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U))
    {
        if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
            pdMS_TO_TICKS(WHEEL_TEST_PREP_MS)) Safety_WheelTestCut(6U);
        else
        {
            /* 转动时间从本次速度发送前另起算，不包含准备阶段 */
            s_wheel_test_start_tick = xTaskGetTickCount();
            *session = g_wheel_test_session;
            g_wheel_test_state = WHEEL_TEST_STARTING;
            accepted = 1U;
        }
    }
    taskEXIT_CRITICAL();
    return accepted;
}

/* 四轮分笔启动前复核本会话，过期或撤VM后不得补发启动 */
uint32_t Safety_WheelTestStartAllowed(uint32_t session)
{
    uint32_t allowed = 0U;
    taskENTER_CRITICAL();
    if ((session == g_wheel_test_session) && (g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_STARTING))
    {
        if ((s_wheel_test_vm_requested != 1U) || (g_rtos_fault != 0U) ||
            (g_safety_fault != 0U) || (g_safety_ready != 1U) ||
            (g_can_guard_ready != 1U)) Safety_WheelTestCut(4U);
        else if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                 pdMS_TO_TICKS(WHEEL_TEST_RUN_MS)) Safety_WheelTestCut(3U);
        else allowed = 1U;
    }
    taskEXIT_CRITICAL();
    return allowed;
}

void Safety_WheelTestStartReport(uint32_t session, uint32_t result)
{
    taskENTER_CRITICAL();
    if ((session == g_wheel_test_session) && (g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_STARTING))
    {
        if (result != 1U) Safety_WheelTestCut(1U);
        else if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                 pdMS_TO_TICKS(WHEEL_TEST_CUT_MS)) Safety_WheelTestCut(3U);
        else g_wheel_test_state = WHEEL_TEST_RUNNING;
    }
    taskEXIT_CRITICAL();
}

uint32_t Safety_WheelTestTakeStop(uint32_t *session)
{
    uint32_t taken = 0U;
    if (session == NULL) return 0U;
    taskENTER_CRITICAL();
    if ((g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_RUNNING))
    {
        TickType_t age = (TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick);
        if (age >= pdMS_TO_TICKS(WHEEL_TEST_CUT_MS)) Safety_WheelTestCut(3U);
        else if (age >= pdMS_TO_TICKS(WHEEL_TEST_RUN_MS))
        {
            g_wheel_test_state = WHEEL_TEST_STOPPING;
            *session = g_wheel_test_session;
            taken = 1U;
        }
    }
    taskEXIT_CRITICAL();
    return taken;
}

void Safety_WheelTestStopReport(uint32_t session, uint32_t result)
{
    taskENTER_CRITICAL();
    if ((session == g_wheel_test_session) && (g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_STOPPING))
    {
        if (result != 1U) Safety_WheelTestCut(2U);
        else if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                 pdMS_TO_TICKS(WHEEL_TEST_CUT_MS)) Safety_WheelTestCut(3U);
        else g_wheel_test_state = WHEEL_TEST_DONE;
    }
    taskEXIT_CRITICAL();
}

void Safety_WheelTestAbort(void)
{
    taskENTER_CRITICAL();
    if (g_wheel_test_active == 1U) Safety_WheelTestCut(5U);
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

/* 只由监督任务调用；这一步只显示禁止原因，不发送CAN */
static void Safety_MotionPermissionUpdate(uint32_t vm_request)
{
    uint32_t blocked = MOTION_BLOCK_POLICY_PENDING;
    uint32_t enabled;
    uint32_t speed_valid;
    uint32_t position_valid;
    uint32_t speed_fresh = 0U;
    uint32_t position_fresh = 0U;
    uint32_t i;
    TickType_t now;
    TickType_t speed_ticks[4];
    TickType_t position_ticks[4];

    /* 一次取出一致快照；CAN任务停住时，本任务仍会继续算年龄 */
    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    enabled = s_motion_feedback_enabled;
    speed_valid = s_motion_speed_valid;
    position_valid = s_motion_position_valid;
    for (i = 0U; i < 4U; i++)
    {
        speed_ticks[i] = s_motion_speed_ticks[i];
        position_ticks[i] = s_motion_position_ticks[i];
    }
    taskEXIT_CRITICAL();

    if ((vm_request == 1U) && (enabled == 1U))
    {
        for (i = 0U; i < 4U; i++)
        {
            /* 有效位确认本会话成功采样；无符号相减支持tick回绕 */
            if (((speed_valid & (1UL << i)) != 0U) &&
                ((TickType_t)(now - speed_ticks[i]) <=
                 pdMS_TO_TICKS(SAFETY_MOTION_DIAG_FRESH_MS)))
            {
                speed_fresh |= 1UL << i;
            }
            if (((position_valid & (1UL << i)) != 0U) &&
                ((TickType_t)(now - position_ticks[i]) <=
                 pdMS_TO_TICKS(SAFETY_MOTION_DIAG_FRESH_MS)))
            {
                position_fresh |= 1UL << i;
            }
        }
    }
    g_motion_speed_fresh_mask = speed_fresh;
    g_motion_position_fresh_mask = position_fresh;
    Safety_MotionDiagLatchUpdate(vm_request, speed_fresh, position_fresh);
    if (g_motion_diag_fault_latched != 0U)
    {
        blocked |= MOTION_BLOCK_DIAG_LATCHED;
    }
    if (speed_fresh != 0x0FU)
    {
        blocked |= MOTION_BLOCK_SPEED_FEEDBACK;
    }
    if (position_fresh != 0x0FU)
    {
        blocked |= MOTION_BLOCK_POSITION_FEEDBACK;
    }
    if (vm_request != 1U)
    {
        blocked |= MOTION_BLOCK_VM_OFF;
    }
    if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
        (g_safety_ready != 1U))
    {
        blocked |= MOTION_BLOCK_PLATFORM;
    }
    if (g_can_guard_ready != 1U)
    {
        blocked |= MOTION_BLOCK_CAN;
    }

    /* 平台/CAN就绪也不放行，不能用静态1秒反馈阈值允许运动 */
    g_motion_block_reason = blocked;
    g_motion_allowed = 0U;
}

/* 只有监督任务调用；故障锁定后，正常心跳恢复也不能自动开VM */
void Safety_Update(uint32_t vm_request, uint32_t can_test_result)
{
    TickType_t now;
    TickType_t last[3];
    uint32_t seen;
    uint32_t bad = 0U;
    uint32_t i;

    g_safety_count++;
    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    seen = s_seen_mask;
    for (i = 0U; i < 3U; i++)
    {
        last[i] = s_last_tick[i];
    }
    taskEXIT_CRITICAL();

    if (s_started == 0U)
    {
        s_start_tick = now;
        s_started = 1U;
    }

    if (((seen & 0x01U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_APP]) >
         pdMS_TO_TICKS(SAFETY_APP_MS)))
    {
        bad |= 0x01U;
    }
    if (((seen & 0x02U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_RX]) >
         pdMS_TO_TICKS(SAFETY_RX_MS)))
    {
        bad |= 0x02U;
    }
    if (((seen & 0x04U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_KEY]) >
         pdMS_TO_TICKS(SAFETY_KEY_MS)))
    {
        bad |= 0x04U;
    }

    /* 回环结果0表示还在启动，其他非1结果表示已失败 */
    if ((can_test_result != 0U) && (can_test_result != 1U))
    {
        g_safety_fault |= 0x10U;
    }
    if ((s_was_ready != 0U) ||
        ((TickType_t)(now - s_start_tick) >=
         pdMS_TO_TICKS(SAFETY_STARTUP_MS)))
    {
        g_safety_fault |= bad;
        if (can_test_result == 0U)
        {
            g_safety_fault |= 0x08U;
        }
    }
    /* 在最后决定VM允许之前，先纳入CAN故障判断 */
    /* 短时测试主动关电后不把随后的掉线记成原CAN故障 */
    Safety_CanGuardUpdate(((g_wheel_test_cut_vm == 1U) || (g_chassis_auto_cut_vm == 1U)) ? 0U : vm_request);
    g_safety_ready = 0U;
    if ((g_safety_fault == 0U) && (bad == 0U) &&
        (can_test_result == 1U))
    {
        s_was_ready = 1U;
        g_safety_ready = 1U;
    }

    /* 更新诊断并发布停止请求，失败或超时追加关VM条件 */
    Safety_WheelTestUpdate(vm_request);
    ChassisAuto_SafetyUpdate(vm_request);
    Safety_MotionPermissionUpdate(((g_wheel_test_cut_vm == 1U) || (g_chassis_auto_cut_vm == 1U)) ? 0U : vm_request);
    Safety_MotionStopUpdate(((g_wheel_test_cut_vm == 1U) || (g_chassis_auto_cut_vm == 1U)) ? 0U : vm_request);

    if ((g_safety_ready == 1U) && (vm_request == 1U) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_wheel_test_cut_vm == 0U) &&
        (g_chassis_auto_cut_vm == 0U))
    {
        Power_Enable();
    }
    else
    {
        Power_Disable();
    }
    /* 本轮任务/CAN检查与输出决策完成，只有健康才继续喂狗 */
    if ((g_safety_ready == 1U) && (g_rtos_fault == 0U))
    {
        Watchdog_Feed();
    }
}


/* 将新读到的3A供原CAN监督使用，不伪造采样时间 */
extern volatile uint32_t g_can_poll_result[4];
extern volatile uint32_t g_can_poll_flags[4];
extern volatile TickType_t g_can_poll_tick[4];
extern volatile uint32_t g_can_poll_count[4];

volatile uint32_t g_chassis_auto_request = 0U;
volatile uint32_t g_chassis_auto_dry_run = 1U;
volatile uint32_t g_chassis_auto_mode = 1U;
volatile uint32_t g_chassis_auto_state = CHASSIS_AUTO_IDLE;
volatile uint32_t g_chassis_auto_segment = 0U;
volatile uint32_t g_chassis_auto_active = 0U;
volatile uint32_t g_chassis_auto_cut_vm = 0U;
volatile uint32_t g_chassis_auto_session = 0U;
volatile uint32_t g_chassis_auto_result = 0U;
volatile uint32_t g_chassis_auto_cause = 0U;
volatile uint32_t g_chassis_auto_io_result = 0U;
volatile uint32_t g_chassis_auto_last_address = 0U;
volatile uint32_t g_chassis_auto_start_mask[3] = {0U};
volatile uint32_t g_chassis_auto_stop_mask[3] = {0U};
volatile int32_t g_chassis_auto_driver_rpm[4] = {0};

static uint32_t s_vm_requested, s_used, s_mode;
static uint32_t s_motor_index, s_monitor_cursor, s_monitor_between;
static uint32_t s_status_valid, s_speed_valid, s_run_started, s_stop_due;
static uint32_t s_status[4];
static TickType_t s_status_tick[4], s_speed_tick[4];
static TickType_t s_total_tick, s_phase_tick, s_run_tick;
static ChassisMotorCommand s_commands[3][4];

/* 以下带Locked的函数只在短临界区内操作内存，不等待CAN */
static void Auto_CutLocked(uint32_t cause)
{
    g_chassis_auto_active = 0U;
    g_chassis_auto_cut_vm = 1U;
    g_chassis_auto_state = CHASSIS_AUTO_FAULT;
    g_chassis_auto_cause = cause;
    g_chassis_auto_result = 2U;
}

static void Auto_EnterLocked(uint32_t state, TickType_t now)
{
    g_chassis_auto_state = state;
    s_phase_tick = now;
    s_motor_index = 0U;
    s_monitor_between = 0U;
}

static uint32_t Auto_BaseHealthy(void)
{
    return ((g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
        (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_wheel_test_active == 0U) && (g_wheel_test_cut_vm == 0U)) ? 1U : 0U;
}

static uint32_t Auto_AllStatusSinceLocked(TickType_t now)
{
    uint32_t i;
    if (s_status_valid != 0x0FU) return 0U;
    for (i = 0U; i < 4U; i++)
        if (((TickType_t)(now - s_status_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_status_tick[i]) > (TickType_t)(now - s_phase_tick)))
            return 0U;
    return 1U;
}

static uint32_t Auto_AllSpeedSinceLocked(TickType_t now)
{
    uint32_t i;
    if (s_speed_valid != 0x0FU) return 0U;
    for (i = 0U; i < 4U; i++)
        if (((TickType_t)(now - s_speed_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_speed_tick[i]) > (TickType_t)(now - s_phase_tick)))
            return 0U;
    return 1U;
}

static uint32_t Auto_ZeroSpeedLocked(void)
{
    uint32_t i;
    for (i = 0U; i < 4U; i++) if (g_chassis_auto_driver_rpm[i] != 0) return 0U;
    return 1U;
}

/* 独立时限和反馈检查，CAN停推进时由Safety继续调用 */
static void Auto_CheckLocked(TickType_t now)
{
    uint32_t i, state = g_chassis_auto_state;
    TickType_t run_age;
    if (g_chassis_auto_active != 1U) return;
    if (!Auto_BaseHealthy()) { Auto_CutLocked(2U); return; }
    if ((TickType_t)(now - s_total_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_TOTAL_MS))
    { Auto_CutLocked(6U); return; }
    if (state <= CHASSIS_AUTO_VERIFY_ON)
    {
        if ((TickType_t)(now - s_total_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_PREP_MS))
            Auto_CutLocked(6U);
        return;
    }
    for (i = 0U; i < 4U; i++)
    {
        if (((s_status_valid & (1UL << i)) == 0U) ||
            ((s_speed_valid & (1UL << i)) == 0U) ||
            ((TickType_t)(now - s_status_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_speed_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)))
        { Auto_CutLocked(3U); return; }
        if ((s_status[i] & 0x0DU) != 1U) { Auto_CutLocked(2U); return; }
    }
    if ((state == CHASSIS_AUTO_SETTLE) &&
        ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_SETTLE_MS)))
    { Auto_CutLocked(7U); return; }
    if (!s_run_started || (state == CHASSIS_AUTO_SETTLE)) return;
    run_age = (TickType_t)(now - s_run_tick);
    if (run_age >= pdMS_TO_TICKS(CHASSIS_AUTO_RUN_MS)) s_stop_due = 1U;
    if ((run_age >= pdMS_TO_TICKS(CHASSIS_AUTO_RUN_MS + CHASSIS_AUTO_STOP_MS)) &&
        (g_chassis_auto_stop_mask[g_chassis_auto_segment] != 0x0FU))
    { Auto_CutLocked(6U); return; }
    if ((s_mode == 0U) && (state == CHASSIS_AUTO_RUN) &&
        (run_age >= pdMS_TO_TICKS(500U)))
    {
        for (i = 0U; i < 4U; i++)
        {
            int32_t raw = g_chassis_auto_driver_rpm[i];
            /* 不把1RPM量化为0误判为停转；只拒绝明显超速和新采到的反向 */
            if ((raw < -(int32_t)CHASSIS_AUTO_SPEED_LIMIT_RPM) ||
                (raw > (int32_t)CHASSIS_AUTO_SPEED_LIMIT_RPM)) { Auto_CutLocked(8U); return; }
            if (((TickType_t)(now - s_speed_tick[i]) <= run_age - pdMS_TO_TICKS(500U)) &&
                (((s_commands[g_chassis_auto_segment][i].direction == 0U) && (raw < 0)) ||
                 ((s_commands[g_chassis_auto_segment][i].direction == 1U) && (raw > 0))))
            { Auto_CutLocked(8U); return; }
        }
    }
}

void ChassisAuto_Init(void)
{
    uint32_t i;
    g_chassis_auto_request = 0U;
    g_chassis_auto_dry_run = g_chassis_auto_mode = 1U;
    g_chassis_auto_state = CHASSIS_AUTO_IDLE;
    g_chassis_auto_segment = g_chassis_auto_active = g_chassis_auto_cut_vm = 0U;
    g_chassis_auto_session = g_chassis_auto_result = g_chassis_auto_cause = 0U;
    g_chassis_auto_io_result = g_chassis_auto_last_address = 0U;
    s_vm_requested = s_used = s_mode = 0U;
    s_motor_index = s_monitor_cursor = s_monitor_between = 0U;
    s_status_valid = s_speed_valid = s_run_started = s_stop_due = 0U;
    s_total_tick = s_phase_tick = s_run_tick = 0U;
    for (i = 0U; i < 3U; i++) g_chassis_auto_start_mask[i] = g_chassis_auto_stop_mask[i] = 0U;
    for (i = 0U; i < 4U; i++)
    { s_status[i] = 0U; s_status_tick[i] = s_speed_tick[i] = 0U; g_chassis_auto_driver_rpm[i] = 0; }
}

uint32_t ChassisAuto_Begin(uint32_t dry_run)
{
    uint32_t i, j, accepted = 0U;
    taskENTER_CRITICAL();
    if ((dry_run <= 1U) && s_vm_requested && !s_used && !g_chassis_auto_active &&
        !g_chassis_auto_cut_vm && Auto_BaseHealthy())
    {
        for (i = 0U; i < 3U; i++)
        {
            if (!Chassis_BasicTestCommands(&g_chassis_geometry, i, s_commands[i])) break;
            /* 三个基本动作每轮原为1RPM，保持其方向组合，统一放大到5RPM。 */
            for (j = 0U; j < 4U; j++)
                s_commands[i][j].speed_raw = (uint16_t)CHASSIS_AUTO_TARGET_RPM;
        }
        if (i == 3U)
        {
            s_used = 1U; s_mode = dry_run; g_chassis_auto_mode = dry_run;
            g_chassis_auto_session++;
            g_chassis_auto_segment = 0U;
            g_chassis_auto_result = g_chassis_auto_cause = 0U;
            s_total_tick = xTaskGetTickCount();
            s_status_valid = s_speed_valid = s_run_started = s_stop_due = 0U;
            s_monitor_cursor = 0U;
            for (i = 0U; i < 3U; i++) g_chassis_auto_start_mask[i] = g_chassis_auto_stop_mask[i] = 0U;
            g_chassis_auto_active = 1U;
            Auto_EnterLocked(CHASSIS_AUTO_DISABLE, s_total_tick);
            accepted = 1U;
        }
    }
    if (!accepted) g_chassis_auto_result = 9U;
    taskEXIT_CRITICAL();
    return accepted;
}

void ChassisAuto_SafetyUpdate(uint32_t vm_request)
{
    taskENTER_CRITICAL();
    s_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (!s_vm_requested)
    {
        if (g_chassis_auto_active)
        { g_chassis_auto_state = CHASSIS_AUTO_CANCELLED; g_chassis_auto_result = 3U; }
        g_chassis_auto_active = g_chassis_auto_cut_vm = 0U;
        s_used = 0U;
    }
    else Auto_CheckLocked(xTaskGetTickCount());
    taskEXIT_CRITICAL();
}

void ChassisAuto_Abort(void)
{
    taskENTER_CRITICAL();
    if (g_chassis_auto_active) Auto_CutLocked(9U);
    taskEXIT_CRITICAL();
}

static void Auto_Monitor(uint32_t session)
{
    uint32_t cursor = s_monitor_cursor, i = cursor / 2U, result;
    uint32_t is_speed = cursor & 1U;
    int32_t rpm = 0;
    CanRxMsg reply = {0};
    if (is_speed) result = CAN1_ReadSpeed((uint8_t)(i + 1U), &reply, &rpm);
    else result = CAN1_ReadStatus((uint8_t)(i + 1U), &reply);
    taskENTER_CRITICAL();
    if ((session == g_chassis_auto_session) && g_chassis_auto_active)
    {
        g_chassis_auto_last_address = i + 1U; g_chassis_auto_io_result = result;
        if (result != 1U)
        {
            if (is_speed) s_speed_valid &= ~(1UL << i);
            else { s_status_valid &= ~(1UL << i); g_can_poll_result[i] = result; }
            Auto_CutLocked(3U);
        }
        else if (is_speed)
        {
            g_chassis_auto_driver_rpm[i] = rpm;
            s_speed_tick[i] = xTaskGetTickCount(); s_speed_valid |= 1UL << i;
        }
        else
        {
            s_status[i] = reply.Data[1]; s_status_tick[i] = xTaskGetTickCount();
            s_status_valid |= 1UL << i;
            g_can_poll_result[i] = 1U; g_can_poll_flags[i] = reply.Data[1];
            g_can_poll_tick[i] = s_status_tick[i]; g_can_poll_count[i]++;
            if ((reply.Data[1] & 0x0CU) != 0U) Auto_CutLocked(2U);
        }
        s_monitor_cursor = (cursor + 1U) % 8U;
    }
    taskEXIT_CRITICAL();
}

/* 仅CAN任务调用：一轮至多一笔有界交易，不在临界区内等待 */
void ChassisAuto_CanStep(void)
{
    uint32_t state, session, index, result, expected, i;
    TickType_t now;
    CanRxMsg reply = {0};
    taskENTER_CRITICAL();
    now = xTaskGetTickCount(); Auto_CheckLocked(now);
    if (!g_chassis_auto_active) { taskEXIT_CRITICAL(); return; }
    state = g_chassis_auto_state; session = g_chassis_auto_session;
    if ((state == CHASSIS_AUTO_START || state == CHASSIS_AUTO_RUN) && s_stop_due)
    { Auto_EnterLocked(CHASSIS_AUTO_STOP, now); state = CHASSIS_AUTO_STOP; }
    if ((state == CHASSIS_AUTO_DISABLE) || (state == CHASSIS_AUTO_ENABLE) ||
        (state == CHASSIS_AUTO_START))
    {
        if (s_monitor_between)
        { s_monitor_between = 0U; taskEXIT_CRITICAL(); Auto_Monitor(session); return; }
    }
    if ((state == CHASSIS_AUTO_VERIFY_OFF) || (state == CHASSIS_AUTO_VERIFY_ON))
    {
        expected = (state == CHASSIS_AUTO_VERIFY_ON) ? 1U : 0U;
        if (Auto_AllStatusSinceLocked(now) &&
            ((state == CHASSIS_AUTO_VERIFY_OFF) || Auto_AllSpeedSinceLocked(now)))
        {
            for (i = 0U; i < 4U; i++) if ((s_status[i] & 0x0DU) != expected) break;
            if (i != 4U) Auto_CutLocked(1U);
            else if ((state == CHASSIS_AUTO_VERIFY_ON) && !Auto_ZeroSpeedLocked())
                Auto_CutLocked(7U);
            else if (state == CHASSIS_AUTO_VERIFY_OFF) Auto_EnterLocked(CHASSIS_AUTO_ENABLE, now);
            else { s_run_started = s_stop_due = 0U; Auto_EnterLocked(CHASSIS_AUTO_START, now); }
            taskEXIT_CRITICAL(); return;
        }
        taskEXIT_CRITICAL(); Auto_Monitor(session); return;
    }
    if (state == CHASSIS_AUTO_SETTLE)
    {
        if (Auto_AllSpeedSinceLocked(now) && Auto_ZeroSpeedLocked() &&
            ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_GAP_MS)))
        {
            if (g_chassis_auto_segment == 2U)
            {
                g_chassis_auto_state = CHASSIS_AUTO_DONE;
                g_chassis_auto_result = 1U; g_chassis_auto_cause = 0U;
                g_chassis_auto_active = 0U; g_chassis_auto_cut_vm = 1U;
            }
            else
            {
                g_chassis_auto_segment++; s_run_started = s_stop_due = 0U;
                Auto_EnterLocked(CHASSIS_AUTO_START, now);
            }
            taskEXIT_CRITICAL(); return;
        }
        taskEXIT_CRITICAL(); Auto_Monitor(session); return;
    }
    if (state == CHASSIS_AUTO_RUN)
    { taskEXIT_CRITICAL(); Auto_Monitor(session); return; }
    index = s_motor_index;
    if (index >= 4U) { Auto_CutLocked(6U); taskEXIT_CRITICAL(); return; }
    if ((state == CHASSIS_AUTO_START) && !s_run_started)
    { s_run_tick = now; s_run_started = 1U; }
    taskEXIT_CRITICAL();

    if (state == CHASSIS_AUTO_DISABLE)
        result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 0U, &reply);
    else if (state == CHASSIS_AUTO_ENABLE)
        result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 1U, &reply);
    else if (state == CHASSIS_AUTO_START)
    {
        ChassisMotorCommand command = s_commands[g_chassis_auto_segment][index];
        result = (s_mode == 1U) ? 1U : CAN1_SetEmmSpeedRaw(command.address,
            command.direction, command.speed_raw, 0U, &reply);
    }
    else if (state == CHASSIS_AUTO_STOP)
        result = (s_mode == 1U) ? 1U : CAN1_StopMotor((uint8_t)(index + 1U), &reply);
    else return;

    taskENTER_CRITICAL();
    if ((session == g_chassis_auto_session) && g_chassis_auto_active &&
        (g_chassis_auto_state == state))
    {
        g_chassis_auto_last_address = index + 1U; g_chassis_auto_io_result = result;
        if (result != 1U)
            Auto_CutLocked((state == CHASSIS_AUTO_START) ? 4U :
                           ((state == CHASSIS_AUTO_STOP) ? 5U : 1U));
        else
        {
            if (state == CHASSIS_AUTO_START)
                g_chassis_auto_start_mask[g_chassis_auto_segment] |= 1UL << index;
            if (state == CHASSIS_AUTO_STOP)
                g_chassis_auto_stop_mask[g_chassis_auto_segment] |= 1UL << index;
            s_motor_index++;
            if (s_motor_index == 4U)
            {
                if (state == CHASSIS_AUTO_DISABLE) Auto_EnterLocked(CHASSIS_AUTO_VERIFY_OFF, xTaskGetTickCount());
                else if (state == CHASSIS_AUTO_ENABLE) Auto_EnterLocked(CHASSIS_AUTO_VERIFY_ON, xTaskGetTickCount());
                else if (state == CHASSIS_AUTO_START) Auto_EnterLocked(CHASSIS_AUTO_RUN, xTaskGetTickCount());
                else Auto_EnterLocked(CHASSIS_AUTO_SETTLE, xTaskGetTickCount());
            }
            else if (state != CHASSIS_AUTO_STOP) s_monitor_between = 1U;
        }
    }
    taskEXIT_CRITICAL();
}
volatile uint32_t g_can_feedback_enable, g_can_disable_request, g_can_enable_request;
volatile uint32_t g_can_jog_request;
volatile uint32_t g_can_poll_enable;
volatile uint32_t g_can_position_request;
volatile uint32_t g_can_speed_request;
volatile uint32_t g_can_status_request;
volatile uint32_t g_can_stop_request;
volatile uint32_t g_can_version_request;
volatile uint32_t g_can_zero_request;
volatile uint32_t g_motion_diag_fail_stop_once;
volatile uint32_t g_motion_diag_hold_report;
volatile uint32_t g_motion_diag_hold_stop_service;
volatile uint32_t g_vm_test_request;
static uint32_t s_can_jog_preparing;
static uint32_t App_CanEnableTestReady(void)
{
    return ((g_vm_test_request == 1U) &&
            (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
            (g_can_poll_enable == 1U) &&
            (g_wheel_test_cut_vm == 0U) &&
            (g_chassis_auto_cut_vm == 0U) &&
            (g_motion_diag_fault_latched == 0U) &&
            (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
            (g_motion_auto_stop_cut_vm == 0U) &&
            (g_motion_diag_hold_report == 0U) &&
            (g_motion_diag_hold_stop_service == 0U) &&
            (g_motion_diag_fail_stop_once == 0U) &&
            (g_can_stop_request == 0U) &&
            (g_can_disable_request == 0U)) ? 1U : 0U;
}

/* 该测试占用控制入口；拒绝同时挂起的其他人工交易，避免事后补发 */
static uint32_t App_CanJogOtherRequest(void)
{
    return ((g_can_stop_request != 0U) || (g_can_disable_request != 0U) ||
            (g_can_enable_request != 0U) || (g_can_zero_request != 0U) ||
            (g_can_speed_request != 0U) || (g_can_position_request != 0U) ||
            (g_can_version_request != 0U) || (g_can_status_request != 0U)) ? 1U : 0U;
}

static void App_CanJogClearRequests(void)
{
    g_can_stop_request = 0U;
    g_can_disable_request = 0U;
    g_can_enable_request = 0U;
    g_can_zero_request = 0U;
    g_can_speed_request = 0U;
    g_can_position_request = 0U;
    g_can_version_request = 0U;
    g_can_status_request = 0U;
}

static uint32_t App_CanJogReady(void)
{
    return ((App_CanEnableTestReady() == 1U) &&
            (g_can_feedback_enable == 0U) &&
            (App_CanJogOtherRequest() == 0U)) ? 1U : 0U;
}


static void app_can(void){
        if (g_chassis_auto_active == 1U)
        {
            g_chassis_auto_request = 0U;
            if (g_vm_test_request != 1U)
            {
                App_CanJogClearRequests();
                g_can_jog_request = 0U;
                ChassisAuto_SafetyUpdate(g_vm_test_request);
            }
            else if ((App_CanJogReady() != 1U) || (g_can_jog_request != 0U))
            {
                App_CanJogClearRequests();
                g_can_jog_request = 0U;
                ChassisAuto_Abort();
            }
            else ChassisAuto_CanStep();
        }
        else if (g_chassis_auto_request == 1U)
        {
            g_chassis_auto_request = 0U;
            if ((App_CanJogReady() == 1U) && (g_wheel_test_active == 0U) &&
                (s_can_jog_preparing == 0U) && (g_can_jog_request == 0U) &&
                (g_chassis_auto_dry_run <= 1U))
                (void)ChassisAuto_Begin(g_chassis_auto_dry_run);
            else g_chassis_auto_result = 9U;
        }
}
static unsigned int calls_disable,calls_enable,calls_start,calls_stop,calls_status,calls_speed,checks,bad_call;
static unsigned int starts[3][4],stops[3][4],directions[3][4];
static int fail_disable,fail_enable,fail_start_seg,fail_start_addr,fail_stop_seg,fail_stop_addr,fail_status,fail_speed;
static int reverse_feedback,hold_feedback,force_protect,late_vm_off;
static unsigned int frame_delay;
static void publish(void){unsigned int i;uint32_t valid=0,protect=0;for(i=0;i<4;i++){if(g_can_poll_result[i]==1&&(TickType_t)(test_tick-g_can_poll_tick[i])<=1000){valid|=1U<<i;if(g_can_poll_flags[i]&12)protect|=1U<<i;}}Safety_CanReport(g_can_poll_enable,valid,protect,g_can_poll_tick);}
static void safety_tick(void){Safety_Report(0);Safety_Report(1);Safety_Report(2);Safety_Update(g_vm_test_request,1);}
static void delay_frame(void){unsigned int t;for(t=0;t<frame_delay;t+=20){test_tick+=20;safety_tick();}if(late_vm_off){g_vm_test_request=0;safety_tick();}}
static uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t en,CanRxMsg *r){uint32_t code=1;if(!power_enabled)return 5;if(a<1||a>4||en>1)bad_call=1;r->Data[0]=0xF3;if(en){calls_enable++;if(fail_enable==a)code=4;}else{calls_disable++;if(fail_disable==a)code=4;}if(code==1){plant_enabled[a-1]=en;if(!en)plant_rpm[a-1]=0;}delay_frame();return code;}
static uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *r){calls_status++;if(!power_enabled||fail_status==a){delay_frame();return 5;}r->Data[0]=0x3A;r->Data[1]=(uint8_t)(plant_enabled[a-1]|(force_protect==a?8:0));delay_frame();return 1;}
static uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg *r,int32_t *rpm){calls_speed++;if(!power_enabled||fail_speed==a){delay_frame();return 5;}*rpm=(hold_feedback==a)?1:plant_rpm[a-1];if(reverse_feedback==a)*rpm=-*rpm;r->Data[0]=0x35;delay_frame();return 1;}
static uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t d,uint16_t v,uint8_t acc,CanRxMsg *r){unsigned int seg=g_chassis_auto_segment;uint32_t code=1;calls_start++;starts[seg][a-1]++;directions[seg][a-1]=d;if(a<1||a>4||d>1||v!=CHASSIS_AUTO_TARGET_RPM||acc!=0)bad_call=1;if(!power_enabled||!plant_enabled[a-1])return 8;if(fail_start_seg==(int)seg&&fail_start_addr==a)code=5;plant_rpm[a-1]=d?-(int32_t)v:(int32_t)v;r->Data[0]=0xF6;delay_frame();return code;}
static uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *r){unsigned int seg=g_chassis_auto_segment;calls_stop++;stops[seg][a-1]++;if(a<1||a>4)bad_call=1;if(!power_enabled)return 5;if(!(fail_stop_seg==(int)seg&&fail_stop_addr==a))plant_rpm[a-1]=0;r->Data[0]=0xFE;delay_frame();return (fail_stop_seg==(int)seg&&fail_stop_addr==a)?5:1;}
static void pump(unsigned int ms,int can){unsigned int t;for(t=10;t<=ms;t+=10){test_tick+=10;if(t%20==0)safety_tick();if(can&&t%50==0){app_can();publish();}}}
static void base(TickType_t tick){unsigned int i;Safety_Init();g_rtos_fault=0;test_tick=tick;g_vm_test_request=g_can_poll_enable=1;g_can_feedback_enable=0;g_can_jog_request=0;s_can_jog_preparing=0;App_CanJogClearRequests();g_motion_diag_hold_report=g_motion_diag_hold_stop_service=g_motion_diag_fail_stop_once=0;calls_disable=calls_enable=calls_start=calls_stop=calls_status=calls_speed=bad_call=0;memset(starts,0,sizeof(starts));memset(stops,0,sizeof(stops));memset(directions,0,sizeof(directions));fail_disable=fail_enable=fail_start_addr=fail_stop_addr=fail_status=fail_speed=reverse_feedback=hold_feedback=force_protect=late_vm_off=0;fail_start_seg=fail_stop_seg=-1;frame_delay=0;for(i=0;i<4;i++){g_can_poll_result[i]=1;g_can_poll_flags[i]=1;g_can_poll_tick[i]=tick;g_can_poll_count[i]=0;}publish();safety_tick();}
static void trigger(unsigned int dry){g_chassis_auto_dry_run=dry;g_chassis_auto_request=1;app_can();publish();}
static void until_state(unsigned int target,unsigned int budget){unsigned int i;for(i=0;i<budget&&g_chassis_auto_active&&g_chassis_auto_state!=target;i+=50)pump(50,1);}
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %d state%u cause%u result%u seg%u tick%u sf%u starts%u stops%u\n",__LINE__,g_chassis_auto_state,g_chassis_auto_cause,g_chassis_auto_result,g_chassis_auto_segment,test_tick,g_safety_fault,calls_start,calls_stop);return 1;}}while(0)
int sequence_checks(void){unsigned int i,j;uint32_t old;TickType_t origin;
const unsigned int expected[3][4]={{0,1,0,1},{0,0,1,1},{1,1,1,1}};
base(100);pump(500,1);CHECK(!g_chassis_auto_active&&!calls_start&&!calls_disable);
base(100);trigger(0);pump(40000,1);CHECK(g_chassis_auto_state==CHASSIS_AUTO_DONE&&g_chassis_auto_result==1&&g_chassis_auto_cut_vm==1&&!g_chassis_auto_active&&!power_enabled&&g_safety_fault==0);CHECK(calls_disable==4&&calls_enable==4&&calls_start==12&&calls_stop==12&&!bad_call);for(i=0;i<3;i++){CHECK(g_chassis_auto_start_mask[i]==15&&g_chassis_auto_stop_mask[i]==15);for(j=0;j<4;j++)CHECK(starts[i][j]==1&&stops[i][j]==1&&directions[i][j]==expected[i][j]);}
old=g_chassis_auto_session;trigger(0);CHECK(g_chassis_auto_result==9&&g_chassis_auto_session==old&&calls_start==12);g_vm_test_request=0;pump(100,1);CHECK(!g_chassis_auto_cut_vm);g_vm_test_request=1;for(i=0;i<4;i++){g_can_poll_result[i]=1;g_can_poll_tick[i]=test_tick;g_can_poll_flags[i]=1;}publish();safety_tick();trigger(1);CHECK(g_chassis_auto_active&&g_chassis_auto_session!=old);
base(100);trigger(1);g_chassis_auto_dry_run=0;pump(40000,1);CHECK(g_chassis_auto_mode==1&&g_chassis_auto_result==1&&!calls_start&&!calls_stop&&!power_enabled);for(i=0;i<3;i++)CHECK(g_chassis_auto_start_mask[i]==15&&g_chassis_auto_stop_mask[i]==15);
for(i=1;i<=4;i++){base(100);fail_disable=i;trigger(0);pump(5000,1);CHECK(g_chassis_auto_cause==1&&!power_enabled&&!calls_start);base(100);fail_enable=i;trigger(0);pump(8000,1);CHECK(g_chassis_auto_cause==1&&!power_enabled&&!calls_start);}
for(i=0;i<3;i++)for(j=1;j<=4;j++){base(100);fail_start_seg=i;fail_start_addr=j;trigger(0);pump(40000,1);CHECK(g_chassis_auto_cause==4&&!power_enabled&&calls_start==i*4+j);base(100);fail_stop_seg=i;fail_stop_addr=j;trigger(0);pump(40000,1);CHECK(g_chassis_auto_cause==5&&!power_enabled&&calls_stop==i*4+j);}
for(i=1;i<=4;i++){base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);CHECK(g_chassis_auto_state==CHASSIS_AUTO_RUN);fail_speed=i;pump(1500,1);CHECK(g_chassis_auto_cause==3&&!power_enabled&&calls_start==4);base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);reverse_feedback=i;pump(2000,1);CHECK(g_chassis_auto_cause==8&&!power_enabled);}
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);origin=test_tick;pump(1500,0);CHECK(!power_enabled&&g_chassis_auto_result==2);CHECK(test_tick-origin==1500);
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);g_vm_test_request=0;pump(100,1);CHECK(g_chassis_auto_state==CHASSIS_AUTO_CANCELLED&&g_chassis_auto_result==3&&!power_enabled&&!g_chassis_auto_cut_vm);old=calls_start;pump(500,1);CHECK(calls_start==old);
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);g_can_enable_request=1;pump(100,1);CHECK(g_chassis_auto_cause==9&&!power_enabled&&!g_can_enable_request);
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);g_chassis_auto_request=1;pump(100,1);CHECK(g_chassis_auto_active&&!g_chassis_auto_request);pump(40000,1);CHECK(calls_start==12&&g_chassis_auto_result==1);
base(100);g_can_jog_request=1;trigger(0);CHECK(g_chassis_auto_result==9&&!g_chassis_auto_active);base(100);g_wheel_test_active=1;trigger(0);CHECK(g_chassis_auto_result==9&&!g_chassis_auto_active);
base(0xFFFFF000U);trigger(0);pump(40000,1);CHECK(g_chassis_auto_result==1&&!power_enabled&&calls_start==12);
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);force_protect=3;pump(1500,1);CHECK(!power_enabled&&g_chassis_auto_result==2);
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);late_vm_off=1;pump(100,1);CHECK(!power_enabled&&!g_chassis_auto_active);old=calls_start;pump(1000,1);CHECK(calls_start==old);
/* Immediate user cancellation is consistent even before the next Safety tick. */
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);g_vm_test_request=0;app_can();CHECK(g_chassis_auto_state==CHASSIS_AUTO_CANCELLED&&g_chassis_auto_result==3);pump(50,0);CHECK(!power_enabled);
/* Each monitored stream is independently age checked, not only CAN task liveness. */
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);s_speed_tick[2]=test_tick-1001;safety_tick();CHECK(g_chassis_auto_cause==3&&!power_enabled);
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);s_status_tick[1]=test_tick-1001;safety_tick();CHECK(g_chassis_auto_cause==3&&!power_enabled);
/* Stop zero must be newly sampled; persistent motion cannot advance the segment. */
base(100);trigger(0);until_state(CHASSIS_AUTO_SETTLE,15000);CHECK(g_chassis_auto_state==CHASSIS_AUTO_SETTLE);hold_feedback=4;pump(2500,1);CHECK(g_chassis_auto_cause==7&&!power_enabled&&g_chassis_auto_segment==0&&calls_start==4);
/* Overspeed and prepare-time unexpected motion fail closed. */
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);plant_rpm[0]=(int32_t)CHASSIS_AUTO_SPEED_LIMIT_RPM+1;pump(1500,1);CHECK(g_chassis_auto_cause==8&&!power_enabled);
base(100);hold_feedback=2;trigger(0);pump(8000,1);CHECK(g_chassis_auto_cause==7&&!calls_start&&!power_enabled);
/* Exercise independent total/preparation/stop deadlines with fresh synthetic frames. */
base(100);trigger(1);until_state(CHASSIS_AUTO_RUN,5000);s_total_tick=test_tick-45000;safety_tick();CHECK(g_chassis_auto_cause==6&&!power_enabled);
base(100);trigger(1);s_total_tick=test_tick-10000;for(i=0;i<4;i++)g_can_poll_tick[i]=test_tick;publish();safety_tick();CHECK(g_chassis_auto_cause==6&&!power_enabled);
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);s_run_tick=test_tick-10500;safety_tick();CHECK(g_chassis_auto_cause==6&&!power_enabled);
/* A CAN return after withdrawal cannot advance state or reopen VM. */
base(100);trigger(0);until_state(CHASSIS_AUTO_START,5000);late_vm_off=1;pump(100,1);CHECK(g_chassis_auto_state==CHASSIS_AUTO_CANCELLED&&!g_chassis_auto_active&&!power_enabled);CHECK(calls_start<=1);
/* Slow transactions still leave the independent Safety deadlines operational. */
base(100);trigger(0);until_state(CHASSIS_AUTO_RUN,5000);frame_delay=1100;pump(100,1);CHECK(!power_enabled&&!g_chassis_auto_active);
/* Original bounded four-wheel jog and original diagnostic stop remain independent. */
base(100);CHECK(Safety_WheelTestPrepare()==1);CHECK(Safety_WheelTestArm(&old)==1);Safety_WheelTestStartReport(old,1);test_tick+=10000;for(i=0;i<4;i++)g_can_poll_tick[i]=test_tick;publish();safety_tick();CHECK(Safety_WheelTestTakeStop(&old)==1);Safety_WheelTestStopReport(old,1);test_tick+=500;for(i=0;i<4;i++)g_can_poll_tick[i]=test_tick;publish();safety_tick();CHECK(g_wheel_test_state==WHEEL_TEST_DONE&&g_wheel_test_cut_vm==1&&!power_enabled&&!g_chassis_auto_active);
base(100);g_motion_diag_fault_latched=16;safety_tick();CHECK(g_motion_auto_stop_state==MOTION_STOP_PENDING);{uint8_t address;CHECK(Safety_MotionStopTake(&address,&old)==1);Safety_MotionStopReport(address,old,4);}safety_tick();CHECK(g_motion_auto_stop_cut_vm==1&&!power_enabled&&!g_chassis_auto_active);
printf("PASS %u checks (actual Safety, auto executor, math and main dispatch; mocked CAN/plant/tick)\n",checks);return 0;}



#define APP_CHASSIS_KEY_TEST_ENABLE 1
#define APP_START_KEY_DEBOUNCE_MS 20U
#define APP_CHASSIS_KEY_READY_MS 5000U
static uint32_t g_can_test_result=1;
typedef struct{uint32_t GPIO_Pin,GPIO_Mode,GPIO_PuPd;} GPIO_InitTypeDef;
#define RCC_AHB1Periph_GPIOC 0
#define GPIO_Pin_13 0
#define GPIO_Mode_IN 0
#define GPIO_PuPd_UP 0
#define GPIOC 0
#define ENABLE 1
static uint8_t pin_raw=1;
static void RCC_AHB1PeriphClockCmd(int a,int v){}
static void GPIO_StructInit(GPIO_InitTypeDef *s){memset(s,0,sizeof(*s));}
static void GPIO_Init(int a,GPIO_InitTypeDef *s){}
static uint8_t GPIO_ReadInputDataBit(int a,int b){return pin_raw;}

/* PC13低电平表示按下，先确认释放再接受按下事件 */
static uint8_t g_key_last_raw = 1U;
static uint8_t g_key_release_seen = 0U;
static TickType_t g_key_change_tick = 0U;

/* 初始化启动按键输入，使用上拉 */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    g_key_last_raw = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
#if APP_CHASSIS_KEY_TEST_ENABLE
    g_key_release_seen = 1U; /* 开发测试：上电已按下也接受，稳定20ms后仅一次 */
#else
    g_key_release_seen = 0U;
#endif
    g_key_change_tick = 0U;
}

/* 稳定20ms产生一次事件；长按不重复，再次按下需先稳定释放 */
uint8_t Key_GetEvent(void)
{
    uint8_t raw;
    TickType_t now;

    raw = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    now = xTaskGetTickCount();

    if (raw != g_key_last_raw)
    {
        g_key_last_raw = raw;
        g_key_change_tick = now;
    }

    if ((TickType_t)(now - g_key_change_tick) <
        pdMS_TO_TICKS(APP_START_KEY_DEBOUNCE_MS))
    {
        return 0U;
    }

    if (raw != 0U)
    {
        g_key_release_seen = 1U;
    }
    else if (g_key_release_seen != 0U)
    {
        g_key_release_seen = 0U;
        return 1U;
    }

    return 0U;
}
#if APP_CHASSIS_KEY_TEST_ENABLE
/* 0等按键、2等CAN就绪、3测试中、4完成、5取消/失败。
 * 本开发固件每次上电仅一次；再次按键不会重启，复位后才允许重测。 */
volatile uint32_t g_chassis_key_state = 0U;
volatile uint32_t g_chassis_key_cause = 0U; /* 1按键取消、2门控、3就绪超时、4测试失败/超时 */
static TickType_t s_chassis_key_tick = 0U;

static void App_ChassisKeyStop(uint32_t cause)
{
    taskENTER_CRITICAL();
    g_chassis_auto_request = 0U;
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    g_chassis_key_state = 5U;
    g_chassis_key_cause = cause;
    taskEXIT_CRITICAL();
}

/* 只提交请求；不等待、不调用CAN、不直接接通电源。 */
static void App_ChassisKeyStep(uint32_t event)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t state = g_chassis_key_state;
    if (event != 0U)
    {
        if ((state >= 1U) && (state <= 3U))
        { App_ChassisKeyStop(1U); return; }
        if (state != 0U) return;
        if ((g_vm_test_request != 0U) || (g_rtos_fault != 0U) ||
            (g_safety_fault != 0U) ||
            (g_chassis_auto_active != 0U) || (g_chassis_auto_request != 0U) ||
            (g_chassis_auto_cut_vm != 0U) || (g_wheel_test_active != 0U) ||
            (g_wheel_test_cut_vm != 0U) || (s_can_jog_preparing != 0U) ||
            (g_can_jog_request != 0U) || (App_CanJogOtherRequest() != 0U) ||
            (g_can_feedback_enable != 0U) || (g_motion_diag_fault_latched != 0U) ||
            (g_motion_auto_stop_state != MOTION_STOP_IDLE) ||
            (g_motion_auto_stop_cut_vm != 0U))
        { App_ChassisKeyStop(2U); return; }
        g_vm_test_request = 1U;
        g_chassis_key_state = 2U;
        s_chassis_key_tick = now;
        return;
    }
    if ((state == 0U) || (state >= 4U)) return;
    if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
        ((state == 3U) && (g_safety_ready != 1U)) || (g_can_feedback_enable != 0U) ||
        (g_can_poll_enable != 1U) || (g_can_jog_request != 0U) ||
        (App_CanJogOtherRequest() != 0U) ||
        (g_motion_diag_hold_report != 0U) || (g_motion_diag_hold_stop_service != 0U) ||
        (g_motion_diag_fail_stop_once != 0U))
    { App_ChassisKeyStop(2U); return; }
    if (state == 2U)
    {
        if (g_vm_test_request != 1U)
        { App_ChassisKeyStop(2U); return; }
        if ((TickType_t)(now - s_chassis_key_tick) >= pdMS_TO_TICKS(APP_CHASSIS_KEY_READY_MS))
        { App_ChassisKeyStop(3U); return; }
        if ((g_can_test_result == 1U) && (App_CanJogReady() == 1U))
        {
            taskENTER_CRITICAL();
            g_chassis_auto_result = 0U;
            g_chassis_auto_dry_run = 0U; /* 按键明确选择实转；Watch入口默认仍为演练 */
            g_chassis_auto_request = 1U;
            g_chassis_key_state = 3U;
            s_chassis_key_tick = now;
            taskEXIT_CRITICAL();
        }
    }
    else if (state == 3U)
    {
        if ((g_chassis_auto_request == 0U) && (g_chassis_auto_active == 0U) &&
            (g_chassis_auto_result != 0U))
        {
            if (g_chassis_auto_result == 1U)
            { g_chassis_key_state = 4U; g_vm_test_request = 0U; }
            else App_ChassisKeyStop(4U);
        }
        else if ((g_vm_test_request != 1U) ||
            ((TickType_t)(now - s_chassis_key_tick) >= pdMS_TO_TICKS(50000U)))
        { App_ChassisKeyStop(4U); }
    }
}
#endif


static void key_base(TickType_t tick){base(tick);g_vm_test_request=0;safety_tick();g_chassis_key_state=g_chassis_key_cause=0;s_chassis_key_tick=0;g_can_test_result=1;}
static void key_pump(unsigned ms,int can){unsigned t;for(t=10;t<=ms;t+=10){test_tick+=10;if(t%20==0)safety_tick();App_ChassisKeyStep(0);if(can&&t%50==0){
/* The original app_can mock omits idle polling; simulate the real one-frame
 * status poll while powered and before the automatic executor takes over. */
if(power_enabled&&!g_chassis_auto_active&&!g_chassis_auto_request){unsigned a=(t/50)%4;CanRxMsg r={0};g_can_poll_result[a]=CAN1_ReadStatus((uint8_t)(a+1),&r);g_can_poll_flags[a]=r.Data[1];g_can_poll_tick[a]=test_tick;}
app_can();publish();}}}

int main(void){unsigned old,i;sequence_checks();old=checks;
/* Actual GPIO key: boot held is accepted once at 20ms, never retriggers held. */
key_base(0);pin_raw=0;Key_Init();test_tick=19;CHECK(Key_GetEvent()==0);test_tick=20;CHECK(Key_GetEvent()==1);App_ChassisKeyStep(1);CHECK(g_vm_test_request==1&&g_chassis_key_state==2);test_tick=200;CHECK(Key_GetEvent()==0);
/* Stable release is needed only for a second press, which cancels the test. */
pin_raw=1;CHECK(Key_GetEvent()==0);test_tick+=20;CHECK(Key_GetEvent()==0);pin_raw=0;CHECK(Key_GetEvent()==0);test_tick+=19;CHECK(Key_GetEvent()==0);test_tick++;CHECK(Key_GetEvent()==1);App_ChassisKeyStep(1);CHECK(g_chassis_key_state==5&&!g_vm_test_request);
/* First normal press: no initial release or artificial countdown needed. */
key_base(0);pin_raw=1;Key_Init();pin_raw=0;CHECK(Key_GetEvent()==0);test_tick=20;CHECK(Key_GetEvent()==1);App_ChassisKeyStep(1);CHECK(g_vm_test_request==1&&g_chassis_key_state==2&&!calls_start);
key_pump(45000,1);CHECK(g_chassis_key_state==4&&g_chassis_auto_result==1&&!power_enabled&&!g_vm_test_request);CHECK(calls_start==12&&calls_stop==12&&!bad_call);App_ChassisKeyStep(1);key_pump(10000,1);CHECK(g_chassis_key_state==4&&calls_start==12&&!power_enabled);
key_base(100);key_pump(60000,1);CHECK(!g_vm_test_request&&!calls_start&&!calls_disable&&g_chassis_key_state==0);
/* An early press waits for startup readiness internally without another press. */
key_base(0);g_safety_ready=0;App_ChassisKeyStep(1);CHECK(g_chassis_key_state==2&&g_vm_test_request==1);App_ChassisKeyStep(0);CHECK(g_chassis_key_state==2);key_pump(45000,1);CHECK(g_chassis_key_state==4&&calls_start==12&&!power_enabled);
/* Cancel during health preparation, queued and moving phases. */
key_base(100);App_ChassisKeyStep(1);App_ChassisKeyStep(1);key_pump(100,1);CHECK(!calls_start&&!g_vm_test_request&&!power_enabled);
key_base(100);App_ChassisKeyStep(1);g_can_guard_ready=1;App_ChassisKeyStep(0);CHECK(g_chassis_auto_request==1);App_ChassisKeyStep(1);key_pump(100,1);CHECK(!g_chassis_auto_request&&!calls_start&&!power_enabled);
key_base(100);App_ChassisKeyStep(1);key_pump(3500,1);CHECK(calls_start>0&&g_chassis_auto_active);App_ChassisKeyStep(1);key_pump(100,1);CHECK(!power_enabled&&!g_vm_test_request&&g_chassis_key_state==5);i=calls_start;App_ChassisKeyStep(1);key_pump(10000,1);CHECK(calls_start==i&&!power_enabled);
key_base(100);App_ChassisKeyStep(1);test_tick+=5000;App_ChassisKeyStep(0);CHECK(g_chassis_key_state==5&&g_chassis_key_cause==3&&!g_vm_test_request&&!calls_start);
key_base(100);g_safety_fault=1;App_ChassisKeyStep(1);CHECK(g_chassis_key_state==5&&!g_vm_test_request&&g_safety_fault==1);
key_base(100);g_can_enable_request=1;App_ChassisKeyStep(1);CHECK(g_chassis_key_state==5&&!g_vm_test_request&&g_can_enable_request==1);g_can_enable_request=0;
key_base(100);App_ChassisKeyStep(1);g_motion_diag_hold_report=1;App_ChassisKeyStep(0);CHECK(g_chassis_key_state==5&&!g_vm_test_request);g_motion_diag_hold_report=0;
key_base(100);App_ChassisKeyStep(1);key_pump(3500,1);fail_speed=2;key_pump(1500,1);CHECK(g_chassis_key_state==5&&!power_enabled&&!g_vm_test_request&&g_chassis_auto_result==2);
key_base(100);g_chassis_key_state=3;g_vm_test_request=1;g_chassis_auto_result=9;App_ChassisKeyStep(0);CHECK(g_chassis_key_state==5&&!g_vm_test_request);
key_base(100);g_chassis_key_state=3;g_vm_test_request=1;g_chassis_auto_request=1;s_chassis_key_tick=test_tick;test_tick+=50000;App_ChassisKeyStep(0);CHECK(g_chassis_key_state==5&&!g_vm_test_request&&!g_chassis_auto_request);
key_base(0xfffff000U);App_ChassisKeyStep(1);key_pump(45000,1);CHECK(g_chassis_key_state==4&&!power_enabled&&calls_start==12);
key_base(100);App_ChassisKeyStep(1);key_pump(3500,1);CHECK(g_chassis_auto_active&&g_chassis_auto_mode==0);plant_rpm[0]=7;key_pump(1000,1);CHECK(g_chassis_auto_active);plant_rpm[0]=8;key_pump(1000,1);CHECK(!power_enabled&&g_chassis_auto_cause==8);
printf("PASS %u immediate-key checks, %u total\n",checks-old,checks);return 0;}
