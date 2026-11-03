#ifndef VEHICLE_PARAMETERS_H
#define VEHICLE_PARAMETERS_H
#include <stdint.h>
/* ================= 现场只改这一个文件，保存→Rebuild→Download =================
 * 调参单位：底盘距离cm、平移cm/s；机械臂Z为旧3200脉冲/圈的虚拟脉冲，
 * 伸缩mm仅沿用32脉冲/mm粗标；DM/舵机为角度deg、DM速度deg/s。
 * 本文件是工程E调试输入，坐标、路径、夹持和负载须现场逐段验证。
 */
#ifndef VEHICLE_INTEGRATED_ENABLE
#define VEHICLE_INTEGRATED_ENABLE 1U
#endif
#ifndef VEHICLE_TEST_STAGE
#define VEHICLE_TEST_STAGE 1U       /* 1~8：本次只调哪一大步骤。 */
#endif
#ifndef VEHICLE_RUN_ALL
#define VEHICLE_RUN_ALL 0U          /* 0单段；1从第1段连续执行八段。 */
#endif
#ifndef VEHICLE_REHEARSAL
#define VEHICLE_REHEARSAL 0U        /* 八段现场分别通过后才设1，整场180秒。 */
#endif
#ifndef VEHICLE_SCAN_BYPASS
#define VEHICLE_SCAN_BYPASS 1U      /* 当前按用户要求跳过扫码；不伪造任务码/计数。 */
#endif
#define VEHICLE_STAGE_TIMEOUT_MS 120000U
#define VEHICLE_REHEARSAL_MS 180000U

/* ---------- 上电停放：按键前DM/伸缩不自动运动 ---------- */
#ifndef STARTUP_GRIPPER_ANGLE_DEG
#define STARTUP_GRIPPER_ANGLE_DEG 140.0f
#endif
#ifndef STARTUP_CARRIER_ANGLE_DEG
#define STARTUP_CARRIER_ANGLE_DEG (-1.0f) /* 未定=-1不输出；填0~270后生效。 */
#endif
#define STARTUP_DM_SPEED_SCALE 0.5f /* 300×0.5=150deg/s，逆时针约180→360(显示0)。 */
/* 伸缩：回零开启时，按键后先按既有30RPM参数回零，确认本次完成才建参考。
 * 首次TEST_ONLY=1只测试回零，结束撤VM且不运行DM/底盘；无自动写回零参数。
 * HOME_ENABLE=0才使用手工ZERO/VALID流程；手推/失电/故障会使参考失效。
 */
/* 用户已确认缩回挡块及30RPM回零可用；先单次回零，再执行启动准备。
 * 改速度需另核驱动菜单并验证；不自动写参数，不把普通106RPM当回零速度。 */
#ifndef STARTUP_EXTENSION_HOME_ENABLE
#define STARTUP_EXTENSION_HOME_ENABLE 1U
#endif
/* 首次仅测伸缩回零，不启动DM或底盘；实物确认后改0再联调。 */
#ifndef STARTUP_EXTENSION_HOME_TEST_ONLY
#define STARTUP_EXTENSION_HOME_TEST_ONLY 1U
#endif
#define STARTUP_EXTENSION_HOME_RPM 30U
#define STARTUP_EXTENSION_HOME_TIMEOUT_MS 11000U
#define STARTUP_EXTENSION_REFERENCE_VALID 0U
#define STARTUP_EXTENSION_ZERO_DEG 0.0f
#define STARTUP_EXTENSION_EXTEND_SIGN (-1)
#define STARTUP_EXTENSION_TARGET_MM 0.0f  /* 这里改按键后的固定伸出量，当前0~50粗标。 */
#define VEHICLE_EXTENSION_RPM 106.0f /* 应用保护上限；普通动作速度改下方逐行参数。 */
#define STARTUP_EXTENSION_RPM 106.0f /* 按键启动时的伸缩速度，可在上限内调低。 */
#define VEHICLE_EXTENSION_PULSES_PER_MM 32.0f /* 粗标，精修需实测。 */
#define VEHICLE_BETWEEN_LEGS_MS 0U /* 已有停车核验，去掉重复的诊断停顿。 */
#define VEHICLE_X_ACCEL_RPM_S 45055U
/* 轮径/轮距/轴距为既有工程输入，非本次重新标定；常规现场先改逐段距离。 */
#define VEHICLE_WHEEL_DIAMETER_MM 77.0f
#define VEHICLE_TRACK_WIDTH_MM 212.0f
#define VEHICLE_WHEELBASE_MM 206.0f
/* 升降机械顶部对应驱动角；未确认时禁止Z动作，不影响只调底盘路线。 */
#define VEHICLE_LIFT_REFERENCE_VALID 0U
#define VEHICLE_LIFT_TOP_DEG 0.0f
#define VEHICLE_LIFT_DOWN_SIGN 1
#define VEHICLE_LIFT_MAX_TARGET_PULSES 105000.0f
#define VEHICLE_LIFT_RPM 1846.4f
#define VEHICLE_DM_SPEED_DEG_S 300.0f /* 既有应用上限；逐行速度独立配置。 */
#define VEHICLE_DM_ZERO_RAW_DEG (-296.544037F) /* 既有逻辑左零基准，非本轮机械零点证明。 */
#define VEHICLE_GRIP_OPEN_DEG 180.0f
#define VEHICLE_GRIP_CLOSE_DEG 135.0f /* 上电140与动作闭合135分开。 */
#define VEHICLE_CARRIER_REFERENCE_VALID 0U /* 调好三槽角后再填1，不拿历史0度当当前对齐。 */
#define VEHICLE_CARRIER_SLOT1_DEG 0.0f
#define VEHICLE_CARRIER_SLOT2_DEG 120.0f
#define VEHICLE_CARRIER_SLOT3_DEG 240.0f
#define VEHICLE_CARRIER_ARM_DEG 35.1079407f
#define VEHICLE_SOURCE_ARM_DEG 180.0f
#define VEHICLE_SOURCE_Z_PULSES 105000.0f
#define VEHICLE_CARRIER_Z_PULSES 28000.0f
#define VEHICLE_SAFE_Z_PULSES 0.0f
/* 后续工位实测后在这里填；-1拒绝执行，不能套用原料区高度。 */
#define VEHICLE_COARSE_Z_PULSES (-1.0f)
#define VEHICLE_STORAGE_Z_PULSES (-1.0f)
#define VEHICLE_STACK_Z_PULSES (-1.0f)
/* 工程运行范围与故障限值保留，普通调参不能用删除判断来绕过。 */
#define VEHICLE_MIN_ROUTE_CM 1.0f
#define VEHICLE_MAX_ROUTE_CM 210.0f
#define VEHICLE_MAX_TRANSLATION_CM_S 20.0f
#define VEHICLE_AXIS_TIMEOUT_MS 0U /* 0保留动作总等待无限；通信/服务间隔及整段截止仍保留。 */
#ifndef VEHICLE_MATERIAL_OBSERVATION_REQUIRED
#define VEHICLE_MATERIAL_OBSERVATION_REQUIRED 1U
#endif
/* 真实成功观察接口尚待视觉/执行反馈接入；0仅用于无物料路线调试，
 * 不能把0写成已完成夹持/载台库存/正确计数。正式自主不允许旁路。 */

typedef struct {uint32_t direction;float distance_cm,speed_cm_s;} VehicleDriveRow;
/* 方向：0前进、1左移、2右移、3后退；相对本段起始车头，当前不转底盘朝向。 */
#define VF(cm,speed) {0U,cm,speed}
#define VL(cm,speed) {1U,cm,speed}
#define VR(cm,speed) {2U,cm,speed}
#define VB(cm,speed) {3U,cm,speed}
#define VSTOP {0U,0.0f,0.0f}
#ifdef VEHICLE_PARAMETER_TABLES
static const VehicleDriveRow vehicle_drive[8][8]={
 /* 1 启停区1→扫码区：已经观察方向正确，距离/停车仍要量。 */
 {VF(15.0f,5.0f),VL(105.0f,10.0f),VSTOP},
 /* 2 扫码区→原料区：以下2~8是场地图推导候选，未实车验收。 */
 {VF(90.0f,10.0f),VR(96.0f,10.0f),VSTOP},
 /* 3 原料区→粗加工区 */
 {VL(192.0f,10.0f),VSTOP},
 /* 4 粗加工区→暂存区 */
 {VR(96.0f,10.0f),VF(96.0f,10.0f),VSTOP},
 /* 5 暂存区→原料区 */
 {VB(96.0f,10.0f),VR(96.0f,10.0f),VSTOP},
 /* 6 原料区→粗加工区 */
 {VL(192.0f,10.0f),VSTOP},
 /* 7 粗加工区→暂存区 */
 {VR(96.0f,10.0f),VF(96.0f,10.0f),VSTOP},
 /* 8 暂存区→本轮启停区（用户已确认）。区2仅镜像1/8段的最后接入横移。 */
 {VB(96.0f,10.0f),VB(90.0f,10.0f),VR(105.0f,10.0f),VB(15.0f,5.0f),VSTOP}
};
#endif

/* ---------- 各段机械臂动作：每一行都可改目标、速度和停留 ----------
 * opcode：0结束，1升降目标(虚拟脉冲)，2DM角度，3伸缩目标mm，
 * 4夹爪角度，5载盘角度，6停留ms，7原料→载台实物观察，
 * 8载台→工位放稳观察，9工位→载台观察。观察缺失时不记成功、不运输。
 * 舵机无独立速度命令，speed列填0；settle_ms只表示等待，不冒充反馈。
 * 下面使用三件循环：item=0/1/2自动选对应槽位；同批实例仍需任务反馈。
 */
typedef struct {uint32_t op;float target,speed;uint32_t settle_ms;} VehicleArmRow;
#define VA_END {0U,0.0f,0.0f,0U}
#define VA_Z(p,rpm) {1U,p,rpm,0U}
#define VA_DM(d,dps) {2U,d,dps,0U}
#define VA_EXT(mm,rpm) {3U,mm,rpm,0U}
#define VA_GRIP(d,wait_ms) {4U,d,0.0f,wait_ms}
#define VA_SLOT(wait_ms) {5U,-2.0f,0.0f,wait_ms} /* -2=本件槽位，其他数值为直接角。 */
#define VA_WAIT(ms) {6U,0.0f,0.0f,ms}
#define VA_OBSERVE(kind) {kind,0.0f,0.0f,0U}
#ifdef VEHICLE_PARAMETER_TABLES
static const uint32_t vehicle_arm_enable[8]={0U,1U,1U,1U,1U,1U,1U,0U};
#endif
#ifdef VEHICLE_PARAMETER_TABLES
static const uint32_t vehicle_arm_repeat[8]={1U,3U,3U,3U,3U,3U,3U,1U};
#endif
#ifdef VEHICLE_PARAMETER_TABLES
static const VehicleArmRow vehicle_arm_rows[8][32]={
 /* 第1段只调底盘；4310/伸缩并行准备由公共启动流程完成。 */
 {VA_END},
 /* 第2段：原料取一件→放对应车载槽→空爪安全高度→回运输角0。 */
 {VA_SLOT(1500U),VA_GRIP(VEHICLE_GRIP_OPEN_DEG,500U),VA_DM(VEHICLE_SOURCE_ARM_DEG,300.0f),
  VA_EXT(STARTUP_EXTENSION_TARGET_MM,106.0f),VA_Z(VEHICLE_SOURCE_Z_PULSES,1846.4f),VA_GRIP(VEHICLE_GRIP_CLOSE_DEG,500U),
  VA_Z(VEHICLE_SAFE_Z_PULSES,1846.4f),VA_DM(VEHICLE_CARRIER_ARM_DEG,300.0f),VA_Z(VEHICLE_CARRIER_Z_PULSES,1846.4f),
  VA_GRIP(VEHICLE_GRIP_OPEN_DEG,500U),VA_Z(VEHICLE_SAFE_Z_PULSES,1846.4f),VA_EXT(STARTUP_EXTENSION_TARGET_MM,106.0f),VA_DM(0.0f,300.0f),VA_OBSERVE(7U),VA_END},
 /* 第3段：载台→粗加工放稳→再取回载台；不接触地面后推移。 */
 {VA_SLOT(1500U),VA_DM(VEHICLE_CARRIER_ARM_DEG,300.0f),VA_Z(VEHICLE_CARRIER_Z_PULSES,1846.4f),VA_GRIP(VEHICLE_GRIP_CLOSE_DEG,500U),
  VA_Z(0.0f,1846.4f),VA_EXT(-2.0f,106.0f),VA_DM(180.0f,300.0f),VA_Z(VEHICLE_COARSE_Z_PULSES,1846.4f),VA_GRIP(VEHICLE_GRIP_OPEN_DEG,500U),VA_Z(0.0f,1846.4f),
  VA_OBSERVE(8U),VA_WAIT(500U),VA_Z(VEHICLE_COARSE_Z_PULSES,1846.4f),VA_GRIP(VEHICLE_GRIP_CLOSE_DEG,500U),VA_Z(0.0f,1846.4f),
  VA_DM(VEHICLE_CARRIER_ARM_DEG,300.0f),VA_Z(VEHICLE_CARRIER_Z_PULSES,1846.4f),VA_GRIP(VEHICLE_GRIP_OPEN_DEG,500U),
  VA_Z(0.0f,1846.4f),VA_EXT(STARTUP_EXTENSION_TARGET_MM,106.0f),VA_DM(0.0f,300.0f),VA_OBSERVE(9U),VA_END},
 /* 第4段：第一批暂存，不把第四段粗加工位置当同色码垛位置。 */
 {VA_SLOT(1500U),VA_DM(VEHICLE_CARRIER_ARM_DEG,300.0f),VA_Z(VEHICLE_CARRIER_Z_PULSES,1846.4f),VA_GRIP(VEHICLE_GRIP_CLOSE_DEG,500U),
  VA_Z(0.0f,1846.4f),VA_EXT(-2.0f,106.0f),VA_DM(180.0f,300.0f),VA_Z(VEHICLE_STORAGE_Z_PULSES,1846.4f),VA_GRIP(VEHICLE_GRIP_OPEN_DEG,500U),VA_Z(0.0f,1846.4f),
  VA_EXT(STARTUP_EXTENSION_TARGET_MM,106.0f),VA_DM(0.0f,300.0f),VA_OBSERVE(8U),VA_END},
 /* 第5段：第二批取料。可单独改本段每个动作，不影响第2段。 */
 {VA_SLOT(1500U),VA_GRIP(180.0f,500U),VA_DM(180.0f,300.0f),VA_EXT(STARTUP_EXTENSION_TARGET_MM,106.0f),
  VA_Z(VEHICLE_SOURCE_Z_PULSES,1846.4f),VA_GRIP(135.0f,500U),VA_Z(0.0f,1846.4f),VA_DM(VEHICLE_CARRIER_ARM_DEG,300.0f),
  VA_Z(VEHICLE_CARRIER_Z_PULSES,1846.4f),VA_GRIP(180.0f,500U),VA_Z(0.0f,1846.4f),VA_EXT(STARTUP_EXTENSION_TARGET_MM,106.0f),VA_DM(0.0f,300.0f),VA_OBSERVE(7U),VA_END},
 /* 第6段：第二批粗加工，可独立改高度和停留。 */
 {VA_SLOT(1500U),VA_DM(VEHICLE_CARRIER_ARM_DEG,300.0f),VA_Z(VEHICLE_CARRIER_Z_PULSES,1846.4f),VA_GRIP(135.0f,500U),VA_Z(0.0f,1846.4f),
  VA_EXT(-2.0f,106.0f),VA_DM(180.0f,300.0f),VA_Z(VEHICLE_COARSE_Z_PULSES,1846.4f),VA_GRIP(180.0f,500U),VA_Z(0.0f,1846.4f),VA_OBSERVE(8U),
  VA_WAIT(500U),VA_Z(VEHICLE_COARSE_Z_PULSES,1846.4f),VA_GRIP(135.0f,500U),VA_Z(0.0f,1846.4f),VA_DM(VEHICLE_CARRIER_ARM_DEG,300.0f),
  VA_Z(VEHICLE_CARRIER_Z_PULSES,1846.4f),VA_GRIP(180.0f,500U),VA_Z(0.0f,1846.4f),VA_EXT(STARTUP_EXTENSION_TARGET_MM,106.0f),VA_DM(0.0f,300.0f),VA_OBSERVE(9U),VA_END},
 /* 第7段：码垛高度独立；真实同色底层和落稳确认接入后才可全自动。 */
 {VA_SLOT(1500U),VA_DM(VEHICLE_CARRIER_ARM_DEG,300.0f),VA_Z(VEHICLE_CARRIER_Z_PULSES,1846.4f),VA_GRIP(135.0f,500U),VA_Z(0.0f,1846.4f),
  VA_EXT(-2.0f,106.0f),VA_DM(180.0f,300.0f),VA_Z(VEHICLE_STACK_Z_PULSES,1846.4f),VA_GRIP(180.0f,500U),VA_Z(0.0f,1846.4f),VA_EXT(STARTUP_EXTENSION_TARGET_MM,106.0f),VA_DM(0.0f,300.0f),VA_OBSERVE(8U),VA_END},
 {VA_END}
};
#endif
/* 对应每个工位/每件的伸缩目标；未示教=-1，先填实测再做带物料动作。
 * 原料区目前沿用启动固定伸出量；粗加工/暂存/码垛可分别配置。 */
#ifdef VEHICLE_PARAMETER_TABLES
static const float vehicle_station_extension_mm[8][3]={
 {0,0,0},{STARTUP_EXTENSION_TARGET_MM,STARTUP_EXTENSION_TARGET_MM,STARTUP_EXTENSION_TARGET_MM},
 {-1,-1,-1},{-1,-1,-1},{STARTUP_EXTENSION_TARGET_MM,STARTUP_EXTENSION_TARGET_MM,STARTUP_EXTENSION_TARGET_MM},
 {-1,-1,-1},{-1,-1,-1},{0,0,0}};
#endif
#endif
