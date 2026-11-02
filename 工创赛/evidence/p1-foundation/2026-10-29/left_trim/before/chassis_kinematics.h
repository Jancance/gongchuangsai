#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdint.h>

/* X形45度麦轮；数组按CAN地址1左后、2右后、3左前、4右前 */
#define CHASSIS_WHEEL_COUNT 4U
#define CHASSIS_LEFT_REAR 0U
#define CHASSIS_RIGHT_REAR 1U
#define CHASSIS_LEFT_FRONT 2U
#define CHASSIS_RIGHT_FRONT 3U

typedef struct
{
    float wheel_diameter_mm;
    float track_width_mm; /* 左右轮心距 */
    float wheelbase_mm;   /* 前后轮心距 */
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

/* 用户确认轮径77、左右轮心距212、前后轮心距206mm；轮胎实际行程仍待校准。 */
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
/* 航向控制轮速转四轮指令：正值逆时针、负值顺时针，不发送CAN。 */
/* 前进基础速度见app_config，修正限2RPM，四轮始终逻辑前进。 */
/* 0前进、1向左、2向右；非法模式或修正清零命令，纯计算不发CAN。 */
uint32_t Chassis_TranslationHoldCommandsAtSpeed(uint32_t mode, uint32_t base_rpm,
    int32_t correction_rpm, ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);
/* S曲线纯命令编码：通常0..15RPM，专用20cm/s构建0..52RPM；后端仍核单条许可。
 * 修正限到min(基速,2)，零速不原地纠偏。
 * 原Hold接口仍为3..15RPM；新函数不发帧、不授权运动。 */
uint32_t Chassis_ProfileCorrectionRpm(uint32_t base_rpm, int32_t requested, int32_t *effective);
uint32_t Chassis_TranslationProfileCommands(uint32_t mode, uint32_t base_rpm,
    int32_t correction_rpm, ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);
uint32_t Chassis_TranslationHoldCommands(uint32_t mode, int32_t correction_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);
uint32_t Chassis_TranslationHoldSelfTest(void);
uint32_t Chassis_ForwardHoldCommands(int32_t correction_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);
uint32_t Chassis_HeadingCommands(int32_t command_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);
/* 前进/横移距离试验：符号/32位幅值先以int64作差，再换算四轮逻辑行程。
 * 输出均值与轮间最大差，不含打滑补偿或世界坐标。 */
/* mode=0前进/1左移/2右移，投影后正值为目标方向；无横移打滑补偿。 */
uint32_t Chassis_TranslationPositionMm(uint32_t mode,
    const uint32_t start_sign[4], const uint32_t start_raw[4],
    const uint32_t sign[4], const uint32_t raw[4], float *mean_mm, float *spread_mm);
uint32_t Chassis_ForwardPositionMm(const uint32_t start_sign[4], const uint32_t start_raw[4],
    const uint32_t sign[4], const uint32_t raw[4], float *mean_mm, float *spread_mm);
/* 独立前进距离比例：210cm三次实测均值214.583333cm，2026-10-26。
 * 校准进度=名义编码进度*该系数；目标仍是用户输入的mm。
 * 仅用于距离任务，横移、轮速和原始编码/四轮差异不变；提速后须复核。 */
#define CHASSIS_FORWARD_DISTANCE_SCALE 1.02182540f
uint32_t Chassis_CalibrateDistanceMm(uint32_t mode, float encoder_mm, float *progress_mm);
uint32_t Chassis_DistanceRpm(float remaining_mm, uint32_t maximum_rpm);
uint32_t Chassis_DistanceSelfTest(void);
#endif
