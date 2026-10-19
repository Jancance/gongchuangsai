#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 初始化提示次数：0静音，1默认，可改为3 */
#define APP_INIT_BEEP_COUNT    2U

/* 单次鸣叫和两次之间的间隔，单位：毫秒 */
#define APP_BEEP_ON_MS         200U
#define APP_BEEP_GAP_MS        200U
/* 启动按键连续稳定时间，单位：毫秒 */
#define APP_START_KEY_DEBOUNCE_MS    20U

/* 开发专用PC13脱机测试；设0恢复原扫码启动，非正式比赛模式 */
#define APP_CHASSIS_KEY_TEST_ENABLE 1U
/* 本次只读姿态验收，强制VM请求0并忽略启动按键；验收后显式恢复 */
#define APP_HWT_STATIC_TEST 0U
/* 无线Watch显式请求：只运行5RPM逆时针10秒，默认VM关闭。 */
#define APP_HWT_TURN_TEST 1U
#if APP_HWT_STATIC_TEST && APP_HWT_TURN_TEST
#error HWT_test_modes_must_be_exclusive
#endif
#define APP_HWT_BAUD 115200U
#define APP_CHASSIS_KEY_READY_MS 5000U

#endif

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
#define CHASSIS_AUTO_FIRST_SEGMENT (APP_HWT_TURN_TEST ? 2U : 0U)

/* 仅用于1RPM三段调试，时间/反馈阈值未推广为正式运动限值 */
/* 此次可视观察目标5RPM；大于7RPM中止，仅开发测试候选阈值 */
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
            g_chassis_auto_segment = CHASSIS_AUTO_FIRST_SEGMENT;
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

volatile uint32_t g_hwt_version_result=1,g_hwt_valid_mask=3,g_hwt_update_tick;
volatile uint32_t g_hwt_cal_active,g_hwt_cal_end_tx_result,g_hwt_version_state,g_hwt_version_request,g_hwt_a7_request,g_hwt_angle_count;
volatile float g_hwt_yaw_deg;
static uint32_t g_can_test_result=1;
static unsigned hwt_service=1,sensor_phase;
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
static void App_HwtTurnSafety(void);
static void safety_tick(void){App_HwtTurnSafety();Safety_Report(0);Safety_Report(1);Safety_Report(2);Safety_Update(g_vm_test_request,1);}
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
#if APP_CHASSIS_KEY_TEST_ENABLE && !APP_HWT_STATIC_TEST
/* 0等按键、2等CAN就绪、3测试中、4完成、5取消/失败。
 * 本开发固件每次上电仅一次；再次按键不会重启，复位后才允许重测。 */
volatile uint32_t g_chassis_key_state = 0U;
volatile uint32_t g_chassis_key_cause = 0U; /* 1按键取消、2门控、3就绪超时、4测试失败/超时 */
static TickType_t s_chassis_key_tick = 0U;

#if APP_HWT_TURN_TEST
volatile uint32_t g_hwt_turn_request = 0U; /* 1开始一次实转，2取消；默认0 */
volatile float g_hwt_turn_yaw_start = 0.0f, g_hwt_turn_yaw_end = 0.0f;
volatile float g_hwt_turn_yaw_delta = 0.0f;
static float s_hwt_turn_last_yaw;
static uint32_t s_hwt_turn_angle_count;
/* 采样任务与数据均须新鲜，不把旧valid_mask当成新数据。 */
static uint32_t App_HwtTurnHealthy(void)
{
    return ((g_hwt_version_result == 1U) && (g_hwt_valid_mask == 3U) &&
        ((uint32_t)xTaskGetTickCount() - g_hwt_update_tick <= 100U) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}
#endif

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
#if APP_HWT_TURN_TEST
/* 只由Watch开始；PC13只取消活动测试。重复请求不续时、不排队。 */
static void App_HwtTurnStep(uint32_t request, uint32_t key_event)
{
    uint32_t state = g_chassis_key_state;
    if (((request == 2U) || (key_event != 0U)) && (state >= 2U) && (state <= 3U))
    { App_ChassisKeyStop(1U); return; }
    if (((state == 2U) || (state == 3U)) && !App_HwtTurnHealthy())
    { App_ChassisKeyStop(5U); return; }
    if ((request == 1U) && (state == 0U))
    {
        if (!App_HwtTurnHealthy()) { App_ChassisKeyStop(5U); return; }
        g_hwt_turn_yaw_start = g_hwt_turn_yaw_end = g_hwt_yaw_deg;
        g_hwt_turn_yaw_delta = 0.0f;
        s_hwt_turn_last_yaw = g_hwt_yaw_deg;
        s_hwt_turn_angle_count = g_hwt_angle_count;
        App_ChassisKeyStep(1U);
        return;
    }
    if (((state == 2U) || (state == 3U)) &&
        (s_hwt_turn_angle_count != g_hwt_angle_count))
    {
        float delta = g_hwt_yaw_deg - s_hwt_turn_last_yaw;
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        g_hwt_turn_yaw_delta += delta;
        g_hwt_turn_yaw_end = s_hwt_turn_last_yaw = g_hwt_yaw_deg;
        s_hwt_turn_angle_count = g_hwt_angle_count;
    }
    App_ChassisKeyStep(0U);
}
/* Safety任务也独立检查，Watch直接改VM不能绕过显式测试会话。 */
static void App_HwtTurnSafety(void)
{
    if ((g_chassis_key_state == 2U) || (g_chassis_key_state == 3U))
    {
        if (!App_HwtTurnHealthy()) App_ChassisKeyStop(5U);
    }
    else g_vm_test_request = 0U;
}
#endif
#endif


static void turn_base(TickType_t time){g_chassis_key_state=0;g_hwt_version_result=1;g_hwt_valid_mask=3;g_hwt_update_tick=time;
g_hwt_cal_active=g_hwt_cal_end_tx_result=g_hwt_version_state=g_hwt_version_request=g_hwt_a7_request=0;g_hwt_angle_count=0;g_hwt_yaw_deg=0;hwt_service=1;sensor_phase=0;
base(time);g_vm_test_request=0;safety_tick();g_chassis_key_state=g_chassis_key_cause=0;s_chassis_key_tick=0;g_hwt_turn_request=0;g_hwt_turn_yaw_delta=0;g_can_test_result=1;}
static void sensor_tick(void){if(hwt_service){g_hwt_update_tick=test_tick;sensor_phase+=10;if(sensor_phase>=100){sensor_phase=0;g_hwt_angle_count++;if(plant_rpm[0])g_hwt_yaw_deg+=0.5f;if(g_hwt_yaw_deg>180)g_hwt_yaw_deg-=360;}}}
static void turn_pump(unsigned ms,int can){unsigned t;for(t=10;t<=ms;t+=10){test_tick+=10;sensor_tick();if(t%20==0)safety_tick();App_HwtTurnStep(0,0);
if(can&&t%50==0){if(power_enabled&&!g_chassis_auto_active&&!g_chassis_auto_request){unsigned a=(t/50)%4;CanRxMsg reply={0};g_can_poll_result[a]=CAN1_ReadStatus(a+1,&reply);g_can_poll_flags[a]=reply.Data[1];g_can_poll_tick[a]=test_tick;}app_can();publish();}}}
int main(void){unsigned i,previous;TickType_t start_tick;
turn_base(100);turn_pump(10000,1);CHECK(!power_enabled&&!g_vm_test_request&&!calls_start&&!calls_disable&&g_chassis_key_state==0);
/* An idle physical key cannot start motion, and manually forcing VM is denied. */
App_HwtTurnStep(0,1);CHECK(!g_vm_test_request&&g_chassis_key_state==0);g_vm_test_request=1;safety_tick();CHECK(!g_vm_test_request&&!power_enabled);
turn_base(100);App_HwtTurnStep(1,0);CHECK(g_chassis_key_state==2&&g_vm_test_request==1);turn_pump(18000,1);
CHECK(g_chassis_key_state==4&&g_chassis_auto_result==1&&g_chassis_auto_cause==0&&!power_enabled&&!g_vm_test_request&&!bad_call);
CHECK(calls_disable==4&&calls_enable==4&&calls_start==4&&calls_stop==4&&g_chassis_auto_segment==2);
for(i=0;i<4;i++){CHECK(starts[0][i]==0&&starts[1][i]==0&&starts[2][i]==1&&directions[2][i]==1&&stops[2][i]==1);}
CHECK(g_chassis_auto_start_mask[2]==15&&g_chassis_auto_stop_mask[2]==15&&g_chassis_auto_start_mask[0]==0&&g_chassis_auto_start_mask[1]==0);
CHECK(g_hwt_turn_yaw_delta>40&&g_hwt_turn_yaw_delta<60&&g_hwt_turn_yaw_end==g_hwt_yaw_deg);
previous=calls_start;App_HwtTurnStep(1,0);turn_pump(10000,1);CHECK(calls_start==previous&&!power_enabled&&g_chassis_key_state==4);
/* Duplicated Watch requests cannot cancel, queue or extend the active run. */
turn_base(100);App_HwtTurnStep(1,0);turn_pump(5000,1);CHECK(g_chassis_auto_state==CHASSIS_AUTO_RUN);start_tick=s_run_tick;App_HwtTurnStep(1,0);
CHECK(g_chassis_auto_active&&s_run_tick==start_tick);turn_pump(15000,1);CHECK(g_chassis_key_state==4&&calls_start==4&&!power_enabled);
/* Cancel on Watch or PC13 during preparation and motion. */
for(i=0;i<3;i++){turn_base(100);App_HwtTurnStep(1,0);if(i)turn_pump(5000,1);App_HwtTurnStep(i==2?0:2,i==2?1:0);turn_pump(100,1);CHECK(g_chassis_key_state==5&&g_chassis_key_cause==1&&!power_enabled&&!g_vm_test_request);}
/* Invalid/stalled HWT, calibration conflicts and read conflicts stop even through Safety alone. */
for(i=0;i<6;i++){turn_base(100);App_HwtTurnStep(1,0);turn_pump(5000,1);CHECK(g_chassis_auto_active);
 if(i==0){hwt_service=0;test_tick+=110;}
 if(i==1)g_hwt_valid_mask=0;
 if(i==2)g_hwt_cal_active=1;
 if(i==3)g_hwt_cal_end_tx_result=2;
 if(i==4)g_hwt_version_request=1;
 if(i==5)g_hwt_a7_request=1;
 safety_tick();CHECK(!g_vm_test_request&&!power_enabled&&g_chassis_key_cause==5&&g_chassis_key_state==5);}
turn_base(100);g_hwt_version_result=0;App_HwtTurnStep(1,0);CHECK(!g_vm_test_request&&!calls_start&&g_chassis_key_cause==5);
turn_base(100);App_HwtTurnStep(1,0);turn_pump(5000,1);plant_rpm[0]=-8;turn_pump(1000,1);CHECK(!power_enabled&&g_chassis_auto_cause==8);
turn_base(100);App_HwtTurnStep(1,0);turn_pump(5000,1);fail_status=2;turn_pump(1200,1);CHECK(!power_enabled&&g_chassis_auto_result==2);
turn_base(100);App_HwtTurnStep(1,0);turn_pump(5000,1);turn_pump(1500,0);CHECK(!power_enabled&&g_chassis_auto_result==2);
turn_base(100);fail_start_seg=2;fail_start_addr=2;App_HwtTurnStep(1,0);turn_pump(6000,1);CHECK(!power_enabled&&g_chassis_auto_result==2&&calls_start==2);
turn_base(100);fail_stop_seg=2;fail_stop_addr=2;App_HwtTurnStep(1,0);turn_pump(18000,1);CHECK(!power_enabled&&g_chassis_auto_result==2);
/* Independent run stop bound when CAN task is stalled, refreshed feedback isolates the lease. */
turn_base(100);App_HwtTurnStep(1,0);turn_pump(5000,1);start_tick=s_run_tick;test_tick=start_tick+10500;g_hwt_update_tick=test_tick;
for(i=0;i<4;i++){s_status_tick[i]=s_speed_tick[i]=test_tick;g_can_poll_tick[i]=test_tick;}publish();safety_tick();CHECK(!power_enabled&&g_chassis_auto_cause==6);
/* Angle accumulation unwraps +/-180 while a real segment is active. */
turn_base(0xfffff000U);g_hwt_yaw_deg=179;App_HwtTurnStep(1,0);turn_pump(18000,1);CHECK(g_chassis_key_state==4&&!power_enabled&&calls_start==4&&g_hwt_turn_yaw_delta>40&&g_hwt_turn_yaw_delta<60);
printf("PASS %u wireless-turn checks (actual Safety, kinematics, executor, coordinator; mocked CAN/plant/HWT/tick)\n",checks);return 0;}
