# 连续地图坐标：最小观测接线设计

日期：2026-11-17。设计基线：本地主任务阶段提交 `97a603c5`；后续产品修改由实现分工完成。本设计只读源码并新增本文档，现场暂停，不下载、reset、发车或联网。本阶段只接坐标估计和生命周期，不改速度、运动准入、VM、STOP，不把地图坐标用于闭环到点。

## 1. 选定的最小方案

在 `app_odometry.c` 新增一个静态、长期存在的 `ChassisOdometry` 地图context，复用当前真实轮位置和HWT数据及现有里程积分实现。地图context独立于原local context：换PLAN、换后端session、ResetRecords和local SetPose都不重置它。约新增1.4KB静态context，最终以构建RAM与链接map为准，不放CAN任务栈。

首版地图源固定 **RAW_WHEEL_HWT，calibration_scope=0，nav_applied=0**。这里RAW指原始轮反馈经现有基础轮径/前距尺度与HWT积分，不表示未进行任何单位/几何换算。既有四向nav残差仍按原debug单段tuple生效并保留原输出，地图首版不使用它，不扩大scope、不累计nav overlay。这样避免local重建原点、换scope或校准消失导致全场反跳，也不把未验证的任意距离校准带入地图。

不使用名义动作距离累加，不将目标点写成当前位置，不假设停车时车辆绝对未动。地图是会漂移、带源时刻与有效性的估计，不是绝对定位。

## 2. 当前真实调用与采集路径

下文简写路径相对 `3.stm32f407工程/CHASSIS/USER/`。

| 源码证据（修改前行号） | 当前行为 | 本阶段接线 |
|---|---|---|
| `main.c:158-160` | 唯一CAN任务调用 `AppChassis_CanStep()`，航向模式每轮vTaskDelay10ms，另加CAN事务耗时 | 新context只由同一CAN任务更新，不创建任务/定时器/额外CAN查询 |
| `app_chassis_service.c:86-122,161-162` | 先服务动作/机构和既有反馈查询，末尾 `AppMotion_StepOdometry()` | 地图在现有末尾消费本轮真实cache；控制链顺序不变 |
| `app_motion_runtime.c:118-122,982-993` | local inputs接VM/jog/combined/route heading/runtime route id；初始化与Step通过AppOdometry | 保留旧inputs与local调用，添加独立地图请求/只读快照入口 |
| `app_odometry.c:145-173` | 每次local开始可SetPose；读取后端samples，再积分 | 地图不复制local的重锚逻辑，也不因后端cache临时mask不足立即失连续性 |
| `app_motion_runtime.c:781-784`，`stage1_runtime.inc:29-30` | route/单段重开清local记录与reference | 地图不受这些记录重置影响 |
| `chassis_auto_test.c:50-55,1220-1229` | 后端成功位置查询写sign/raw/ms/mask；读口只复制cache | 地图可用该实际raw样本，但不依赖session-local cache长期存在 |
| `chassis_auto_test.c:852-855` | 后端初始化会清samples；新序列调用初始化时同样影响cache | mask短暂清空只表示这一cache暂缺，不证明车辆位移断档 |
| `chassis_auto_test.c:1232-1253` | 原OdometryPoll只在inactive、请求VM且power.active、健康时读一个位置，失败可触发现有保护 | 不改变这条函数的许可条件、失败处理或调用次数 |
| `app_chassis_service.c:26-35` | combined1/2才交替插入专用位置轮询，普通idle走App_CanFeedbackStep | 单靠后端cache不能覆盖整段抓放等待 |

### 2.1 必须复用现有idle position cache

`app_can_diagnostics.c:291-296` 的 `App_CanFeedbackStep()` 在 `g_can_feedback_enable && g_can_poll_enable && (g_vm_test_request || g_startup_axes_vm_request) && g_safety_ready` 时执行反馈轮询。它已经在330-336行成功读取并记录每轮 `g_can_feedback_position_result/sign/raw/tick`，不需要新发查询。

其298-348行每个节点依次状态→速度→位置，再换节点。4轮约12次既有交易；10ms任务间隔意味着理想约120ms加事务耗时，不是实测上界。DM/机构事务和异常可能使实际间隔变大，必须检查真实时戳，不承诺每轮总能小于400ms。

**不能直接使用 `g_can_feedback_position_valid_mask` 作为地图采集mask。** Refresh在365行附近仍按 `g_vm_test_request==1` 形成安全监督mask，尽管采集本身允许startup VM保持。地图应读取实际成功result/sign/raw/tick，并用自己的源年龄和积分条件判断；安全监督的既有mask不改。

`startup_axes.c:46-47` 的Holding为state4且startup VM请求1；Begin在80行置请求1，取消/故障在45/87行等清请求。机构保持期间既有反馈轮询因此有机会持续取得真实轮位置；请求1不等于物理供电或源鲜度已实测，只有成功查询及时间戳构成观测证据。

实现可在同一CAN owner内直接复制cache，或给diagnostics加一个仅复制的getter。每个轮子从后端cache与diagnostic cache选时间更新、合法的真实样本；backend的ms已是毫秒，diagnostic TickType_t只乘一次 `portTICK_PERIOD_MS`。相同时间戳且数值不同不能被静默覆盖；同源重复样本可复用。时间比较保留uint32回绕语义，不用当前发布tick替换源采样tick。

## 3. 地图frame与初始参考

固定项目地图：+X从启停区2指向1，+Y朝场地内部/粗加工；车体前/左为正，航向逆时针为正；地图0°朝+X、90°朝+Y。`vehicle_tuning.h:14-35` 给启停区1 `(2250,150)`、区2 `(150,150)`，单位mm。使用FieldMap_Park(4/0)或对应宏，避免第三份硬编码表；地图版本随快照发布。

只在本轮明确起点处调用 `ChassisOdometry_SetPose(map_x,map_y,90°,fresh_raw_HWT_yaw)`，由已有heading_offset实现HWT与地图对齐。原始HWT角度保留。初始位置是抽签起点加人工按标记摆车的工程参考，不宣称传感器测得绝对起点。

TaskA在真实新轮启动入口先锁定start_id与run_id，再发地图请求；CAN owner独占消费并写context。请求本身不发车；stage1仅在既有开局准备阶段等待本轮有效地图基准，沿用原截止时间。中途stage单测没有确认地图起点时传start_id=0，新epoch清除旧valid并记录NO_START/invalid，不阻塞该独立测试，不能采用该阶段名义工位坐标冒充实车位置。

## 4. 最小接口与状态机

建议接口（命名以实现统一为准）：

- `AppOdometry_RequestMapRun(run_id,start_id)`：只向单槽邮箱提交真实新轮，start_id1/2表示已锁存启停区；0显式无地图起点。重复run不得清GAP，过期run不得覆盖新轮。不能由每个PLAN或session自动调用。
- `AppOdometry_ReadMap(AppMapPose *out)`：临界区复制小型只读快照；不给外部context指针。
- 新flag `APP_CONTINUOUS_MAP_ENABLE` 默认0、候选1，用于开/关构建验证；不扩运动入口。关闭时原local/nav行为保持。

快照最少含 `ChassisOdometryStatus pose`、run_id/start_id/map_version、begin_ms、state/reasons、source=RAW_WHEEL_HWT、calibration_scope=0/nav_applied=0、四轮源ms、HWT angle_ms。可附request/result与source_epoch帮助诊断；没有实际用途的框架字段不加。

| 状态 | 进入/更新 | 出口与有效性 |
|---|---|---|
| OFF/NO_RUN | 上电、无请求或功能关闭 | 没有地图参考；valid0，不显示“地图原点0已确认” |
| NO_START | 新run来自中途stage，start_id0 | 只记录本轮无地图起点，valid0；不会自动套阶段目标 |
| WAIT_BASELINE | 新run1/2由CAN消费；取得fresh HWT并设置地图参考 | 等本轮开始后完整、新鲜四轮/HWT源；尚未based时valid0 |
| TRACKING | **在底盘尚inactive时**建立完整第一基准，后续持续真实位置/HWT积分 | 跨动作、PLAN和session保留原点、历史与累计坐标 |
| GAP | 已有真实基准后源过期/拒绝/持续时隙丢失，或基准未立而底盘先active；明确供电/关键故障造成不可观测 | 保留最后坐标和真实sample_ms，valid0/continuity_lost；同run不自动恢复，不写目标补空白 |

### 4.1 第一基准竞态

开始时不能拿开局前旧cache或运动后的第一包直接作为“零位移”基准。要求用于第一次基准的轮/HWT时刻属于本轮（至少不早于begin_ms），仍新鲜，且底盘尚未active；初始参考设置与首次完整基准可以分轮完成。启动轴准备期间既有位置轮询可以提供这些观测，但目前没有实车证明总能及时完成。

stage1新轮在既有VehicleMission state1或独立stage1 state7开局准备中，等待匹配run_id的地图TRACKING且based/valid后才发布第一段动作；准备等待使用既有阶段限值（独立60s、集成120s），原进入运动后重设运动预算保持，处理已处于Holding时立即启动的真实调用顺序。这是每轮一次的坐标基准初始化，不是运动中停车或每段门禁；不放宽或取消原运动检查，关闭开关与stage2..8独立测试不受此等待影响。四轮mask齐全但异时历史无法插值时仍未ready。若第一基准尚未建立却底盘已经active，地图转NO_BASELINE/GAP，不能丢掉起步前位移后再假valid。

### 4.2 连续性、停止与恢复

- 地图只喂确实存在的wheel/HWT样本。后端mask临时清空不能直接调用地图Invalidate；已有独立历史在年龄内仍可保留。轮源400ms、HWT100ms等当前估计边界由 `chassis_odometry.h:11-12` 与Update实际执行，本轮不擅自放宽。
- `ChassisOdometry_Update` 对common源时刻插值，status.sample_ms是实际共同采样时刻，age是now减它；getter/遥测不能把now写成新源时刻。
- `chassis_odometry.c:60-65,149-155` 已有sticky continuity_lost。供电断、故障或真实源gap后不靠local SetPose、ResetRecords、plan切换或再次BEGIN同run清掉它。
- local Watch pose只影响原local坐标；它不能重建地图、清地图GAP或把未测坐标升级为地图观测。地图保持独立的raw积分与原始参考；若实现选择显式标REFERENCE_CHANGE，应注明是额外观测失效策略，不能影响运动控制。
- 新一轮仅在真实新的启动/摆车事件由新run建立。它表示新参考，不是对上轮失踪位移的恢复。重启丢失RAM后同样无旧地图连续性；不从保留目标或上次显示位置自行续接。
- 取消/故障后仍记录能获得的真实源与最后有效点，但不新增清故障、喂狗、VM保持或自动重试。软件供电请求、PowerCommandStatus、实际灯/电压分别记录；本轮未读现场，不能称物理VM连续。

## 5. 最小修改范围

1. `app_config.h`：一个地图观测开关及合法值检查。
2. `app_odometry.h/c`：独立静态context、请求邮箱、只读快照、raw样本择新/真实时效、初始参考与状态机；原local/raw/nav代码仅在必要接线处增调用，输出和校准scope不改。
3. `app_motion_runtime.c`内实际新轮入口（`vehicle_mission.inc`，必要时 `stage1_runtime.inc`）：run_id锁存后发一次地图请求；stage1用真实锁存start_id，中途stage传0。PLAN/route安装点不发新run，不重置地图。
4. `app_can_diagnostics.h/c`仅在确需封装时加纯cache读取getter；也可在CAN owner直接复制已有全局，避免新增CAN交易。`app_chassis_service.c`只在现有owner调用链确需显式接线时改；不改调度、查询/失败条件和VM。

不需要改 `field_map.c`几何、不需要改 `app_heading_runtime.c`或原航向控制、不需要新通用融合框架；HWT对齐在独立map context中完成。Keil新增source不是必需，避免无理由改工程。

## 6. MATLAB前置与后续验证

对应产品编码之前，本机MATLAB至少验证：两起点与90°对齐、HWT±180跨界、直行/横移/L形真实轮增量、local reset/plan/session换组而map不跳、真实诊断cache在startup保持期提供连续源、短暂mask清空在鲜度内不永久GAP、真实>400ms轮gap/100ms航向过期保持invalid、首基准之前启动/旧cache拒绝、开始源时刻/回绕、manual localpose不清mapgap、重复/过期run不重建，新run是新参考、中途stage无起点invalid。

之后用实际C或提取实际新函数验证同一矩阵，开/关候选Keil构建、额外RAM/栈证据、编码GBK/CRLF/非ASCII保持检查；精确说明桩与实际执行范围。地图源不是目标，须做源静止而目标改变不移动map、raw动而目标不变map仍更新两类检查。

本阶段仅软件/离线交付，不能证明整场始终有效。首次基准能否在既有启动过程取得、机构保持期间源最大间隔、断电后是否需要新增合法定位源，均待独立现场证据；无法观测就明确invalid，不伪造完整轨迹。后续实车另获单次动作授权，先保存首故障，不自动循环。

## 7. 规则检查与交付边界

已读项目规则基线相关需求：R FIELD-01/02/06、RUN-01/02、PWR-01、MECH-01/02；E LOC-02/03/05/06/07、FIELD-08、MOT-02/04、RUN-06、CAL-02；Q FIELD-05、OBS-03、QR-04保留待技术答疑确认。本阶段没有修改R规则，不把地图估计当绝对定位或实车验收。

交付必须明确：地图首版RAW_ONLY、nav_applied0；源时刻与cache生效路径；新轮/PLAN/local原点各自生命周期；首基准竞态与gap无自动恢复；仅观测，不控制到点；MATLAB、actual C、构建、下载、实车各自状态与证据。
