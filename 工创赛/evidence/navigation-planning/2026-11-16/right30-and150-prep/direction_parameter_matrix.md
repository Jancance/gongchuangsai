# 方向参数及右移30/150cm候选独立源码审查

日期：2026-11-16。审查基线：本地HEAD `607368de0ce5eeb087f675b9183fbee4cb761f35`；这是右移150cm候选的本地checkpoint，不代表已推送。指定6.1-sol服务403后按用户许可由当前模型独立审查，不能称为6.1-sol high审查。本记录只写证据目录，不改产品/根交接，不fetch/pull/push，不连接硬件。

## 1. 权威入口和规则边界

- 唯一日常工程 `3.stm32f407工程/gongxun-f407.uvprojx`；活动源为 `CHASSIS/USER/`。工程759/779/869/894行及2113/2133/2223/2248行分别绑定活动motion_profile/chassis_kinematics/app_motion_runtime/app_translation_runtime，旁侧旧USER及handoff不作为本次判据。
- 已检查基线：R RUN-01（自主正式入口与调试隔离）、PWR-01（内置单电源）；E LOC-02/03/05/07、MOT-02/04、RUN-06（定位时效、四向及工况验证、联合约束、状态出口）。Q FIELD-05/OBS-03/QR-04继续**待技术答疑确认**。软件候选及本表均为E，不新增正式规则，不签整场/斜向/任意工况已验。
- MATLAB仍须先于对应实现。下列独立解析仅供与真实MATLAB结果对照，不能替代用户电脑上的MATLAB前置验证，更不能替代实车。

以下源路径除特别注明，均相对 `3.stm32f407工程/CHASSIS/USER/`；行号为本次读取当前源的LF/rg行号。

## 2. BASELINE40候选方向矩阵

| 项目 | 前进 | 左移 | 右移 | 后退 |
|---|---|---|---|---|
| direction / action kind | 0 / 4 | 1 / 5 | 2 / 6 | 3 / 7 |
| 已有调试PLAN | 8，2100mm | 9，2100mm | 10，2100mm；12，1500mm | 11，2100mm |
| 本次新增工况 | 无 | 无 | 13，300mm（软件准备通过，晚点和1500一起测；不自动发车） | 无 |
| RPM预算 | 100 | 102 | 102 | 100 |
| 参考速度上限 | 400mm/s | 400mm/s | 400mm/s；300mm自动退峰 | 400mm/s |
| 距离换算倍率 | 1.02182540 | BASELINE40不套旧route左倍率 | 1 | 继承纵向1.02182540；**不是后退独立标定** |
| 航向保持 | 共用P/D及滤波、限斜率 | 同左 | 同左 | 同左 |
| 前进专用横偏环/航向前馈 | 仅PLAN8+mode0启用 | 关闭 | 关闭 | 关闭 |
| BASELINE40左2%后向trim | 关闭 | 宏开且PLAN9+mode1精确通过才启用 | 关闭 | 关闭 |
| 地面证据范围 | 既有2100工况，不能推任意速度/长度 | 既有2100、当前左trim工况，不能推右 | 既有2100三趟；1500与300尚待实测 | 既有2100三趟；比例仍沿用纵向，不能称已独立校准 |

来源：`app_entry.h:14-20`、`app_entry.c:7-15,29-46`、`motion_trial40.h:4-26`、`app_config.h:40-54`。初写本表时13尚待增加；随后已独立核对root相对607368de的产品增量，13精确为右102RPM/300mm，8..12旧tuple保持。详见同目录`review_current_model_independent.md`。

## 3. 各参数实际作用范围

### 3.1 倍率、方向与轮速映射

- `chassis_kinematics.h:113`定义 `CHASSIS_FORWARD_DISTANCE_SCALE=1.02182540`；`chassis_kinematics.c:439-447`只对mode0/3乘此倍率，mode1仅在`APP_LEFT_TRIM_ACTIVE`旧route分支乘`APP_LEFT_DISTANCE_SCALE=0.99761905`，右mode2不乘。纵向前后共用是当前代码事实，不是后退标定结论。
- 轮径来自 `3.stm32f407工程/USER/vehicle_parameters.h:60`的77mm；`chassis_kinematics.c:10,27-32`建立几何，`chassis_auto_test.c:1854-1863`使用按方向校准后的`πD/60`作为mm/s/RPM。
- `motion_trial20.h:28-44`：BASELINE40位置跟随预算2RPM、Kp1；参考上限为 `min(factor*(rpm−2),400)`。因此100/102是预算，不等于每个时刻实际RPM；短距离也不等于真正达到400mm/s。
- `chassis_auto_test.c:534-572`位置误差P目标经临界阻尼二阶滤波、RPM限幅和末段1RPM靠近；不是新增轮速PID。`heading_controller.c:209-217`是航向PD。本次没有改任何驱动器内部PID，也没有从代码检查推断驱动器内部实参。
- `chassis_auto_test.c:616-635`投影矩阵四行分别前/左/右/后，先按轮方向恢复逻辑轮程再按模式投影；不得把前后符号或左trim复制成右向补偿。

### 3.2 航向PD、前横偏环及左2%

- 四向共用 `heading_controller.c:198-260`：`u=1*heading_error−0.25*yaw_rate`，误差超过10°拒绝；误差/角速均≤0.3置零，限±2RPM。HoldStep在新鲜帧上用80ms角速度低通、10RPM/s斜率及0.1RPM迟滞量化，间隔≥100ms拒绝；`app_translation_runtime.c:264-283`仅新sequence调用。
- `chassis_auto_test.c:1956-1962`启用前横偏修正的充分条件是PLAN8、mode0、原白名单、route_reference=0、profile有效、12779ms、单动作。其误差项 `−0.75*e*cos(delta)`见238-260行；航向项`−0.5*(πD/60)*tan(delta)`见270-283行，后乘当前baseRPM（1380行）。限12.1mm/s、横向斜率20mm/s²，轮预算见285-303行及`chassis_kinematics.c:286-323`。这不是可直接推广四向的“通用横偏PD”；没有该处独立误差D项。
- BASELINE40左trim的宏与比例见`app_config.h:42-48`；加减速先除1.02见`motion_trial40.h:14-25`；命令精确锁PLAN9+mode1+非route+白名单见`chassis_auto_test.c:1423-1427`。`chassis_kinematics.c:274-284`按`round(.02*baseRPM)`给四轮共同叠后向分量，保留航向项；右/后不走此函数。
- 旧route左trim是另一分支：`app_config.h:89-97`（默认关闭，比例0.01670644及distance_scale0.99761905），`motion_route_trial.h:44-48`与`chassis_kinematics.c:326-352`（≤52RPM）及`chassis_auto_test.c:1429-1430`。不能和BASELINE40左2%混记，不能将旧route左标定移到右/后。

### 3.3 A/J与速度参考

- BASELINE40名义A150mm/s²、J300mm/s³，普通S候选A40/J80，`app_config.h:49-54,98-108`。
- prepare实际扣除位置跟随余量：`Aeff=A−factor*2*C*omega`，`Jeff=J−factor*6*C*omega²`；BASELINE40 `C=2,omega=1`，见`chassis_auto_test.c:1854-1868`。左2%先除1.02再扣自己的factor，不得把右A/J照填左。
- 正式积分器仅一套 `motion_profile.c:66-123`，输入distance和limits，短程无匀速时重新求peak。方向策略在上层，曲线没有另造右30控制器。

| BASELINE40方向条件 | factor (mm/s/RPM) | Aeff (mm/s²) | Jeff (mm/s³) |
|---|---:|---:|---:|
| 前/后：沿用纵向倍率 | 4.119704268027363 | 133.52118292789055 | 250.56354878367165 |
| 左：当前2%宏开 | 4.031710572106901 | 130.93198124098416 | 245.73712019354073 |
| 右（及左2%宏关） | 4.031710572106901 | 133.8731577115724 | 251.6194731347172 |

表值为源码常数的独立双精度解析展示；固件float32和真实MATLAB回执有各自数值容差，不把这些数值另写成驱动器PID或实车能力。

## 4. 300mm独立解析对照值（非MATLAB替代）

右factor=`π*77/60=4.031710572106901 mm/s/RPM`；`Aeff=133.8731577115724 mm/s²`，`Jeff=251.6194731347172 mm/s³`。400mm/s参考的最低完整加减速距离约`1407.9796088487733mm`。

300mm大于达到Aeff所需`2*Aeff*(Aeff/Jeff)^2=75.79176493661268mm`，但小于达到400参考所需距离，因此是**加速度饱和、无匀速的对称S曲线**：

- `tj=Aeff/Jeff=0.5320460934273423s`
- `ta=(sqrt(tj²+4D/Aeff)−3tj)/2=0.7223569376094857s`
- `peak=Jeff*tj*(tj+ta)=167.93089480786773mm/s`
- `duration=4tj+2ta=3.5728982489283405s`
- 对比1500：在400上限下仍有约0.230051s匀速、总约7.269949s，2100约1.730051s匀速/8.769949s；后两值已有上一候选真实MATLAB记录，见`../right-short-distance-prep/final_handoff_note.md:4`。300与1500不能宣称同实际峰速，额外长度只用于增加可观察工况，不能单凭距离趋势唯一认定倍率/滑移。

## 5. 保护、停止和源时间戳

- `motion_trial20.h:15-26`针对白名单BASELINE40保留反馈130RPM和按130+30RPM计算跳变量门限；300不能仅因短就放松这些边界。run12779ms是会话时限，非预期运动duration。
- `app_heading_runtime.c:107-120`分别检查sample、angle及rate源帧age（HS_SOURCE_MS）；`app_translation_runtime.c:72-102`仍核源、session、VM、ready、run和stop截止。序号变化与到达时刻并非任意伪造新鲜。
- `chassis_auto_test.c:581-661`缺四轮位置有界等待、每轮400ms时效、跳变/回退/越程门限、使用**最老共同源时刻**插值轮程后再方向校准；首次reached冻结progress不可冒充完整地面行程。
- 在线XY `chassis_odometry.c:125-174`同样共同源时刻插值四轮与yaw，F乘纵向倍率、L不乘，再以中点/sinc旋转积分；`sample_ms`保存该共同源时刻。`chassis_odometry.h:11-12`轮400ms、yaw100ms。`app_odometry.c:99-100`传HWT angle_ms；不能把失鲜invalid的last-valid位置伪装当前健康值。
- 前横偏有效性/age/session二次核查见`chassis_auto_test.c:238-268,1369-1386`，失效目标归零是前横偏行为，不放松主运动故障条件。本次不改STOP、VM、看门狗或时效阈值，不用软件零速替代真实停稳。

## 6. 通用API已有什么、尚缺什么

- 数学层 `MotionProfile_Prepare(plan,distance_mm,limits)`已经参数化距离与速度/A/J；它不接方向、不持有发车许可。
- `MoveDistanceStart(direction,max_rpm,distance_cm)`已存在（`app_motion_runtime.c:384-387`），但100/102RPM受到精确PLAN/距离白名单及motion-list submit限制（347-364行），**不是比赛可任意输入方向/距离/峰速的正式通用入口**。普通timed Move仍只接受0..MOVE_RIGHT（364行）；后退注释为route-distance candidate（`motion_control.h:11`）。
- 显式route的`MotionRoute_Build(direction,mm,speed,...)`已有方向/距离/速度参数，但`motion_route_trial.h:13-38`将各向最大速度硬限制≤20cm/s、speed≥12.1mm/s、RPM≤52，受debug/custom开关限制，不能宣称现已把400mm/s工况全面推广。
- 普通非BASELINE40距离白名单 `motion_distance_limits.h:7-21`仅方向0..2、3..15RPM、100..1000mm，另有特定前2100/15和前2100/52例外；BASELINE40只精确工况放行。`app_config.h:9-20`禁止BASELINE40与competition/integrated/stage1混开。
- 未来用户要求应记录为：一个方向/距离/峰速接口，按方向选**经实测限定**的倍率/航向/补偿/速度能力，再复用同一个curve prepare与begin流程。保留prepare/Begin分离及提交时session、source、许可复核；prepare成功不等于允许发车。未验方向工况明确未验，不能自动套前横偏/左2%或称后退倍率已标定。
- 本轮只增右300精确case，不进行通用controller重构，不开放比赛入口，不改变旧8..12语义。最终新增代码已复核旧8..12 tuple、13精确tuple、BF1/selected action/KEEP_VM/POWERED_DONE和独立client一致性；软件回执与新旧镜像SHA核验见同目录独审报告，硬件仍待另行授权执行。

## 7. 当前结论及未闭环

方向隔离在现有活动源中清楚：共用航向PD和S曲线，前横偏仅8、左2%仅9，右10/12及后11均不误套。右300应继续该隔离，且参考峰值约167.931mm/s，不能标成已达到400。1500及300的真实板身份、实车主距/旁偏/轮速与重复性未闭环；任意方向/长度/峰速、斜向及完整比赛仍未验。本结论是当前模型源码独立审阅，不是6.1-sol high，不是硬件验收。
