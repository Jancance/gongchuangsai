# 左移经验旁偏残差接入导航：最小软件计划

日期2026-11-17；规划角色GPT-6.1-sol xhigh。父代理提供当前HEAD `90da4385`、右1%软件提交 `03528388` 未下载、现场保持暂停。本代理只读活动源和现有证据，仅维护本计划，不操作硬件、COM、探针、Git或网络，不修改产品或其他证据。

本修订以同目录 `root_formal_contract.json` 为正式合同。此前本文件的“残差均值/位移均值”、左PLAN19范围、改原baseline/capture/perimeter、增加owner输入结构的建议全部被此合同替代，不作为实施要求。用户已最终明确固定lambda=.5，不再从样本选择权重。正式生命周期MATLAB已实际通过，见第6节；只授权其后四个指定产品文件的软件实施，硬件仍暂停。

## 1. 明确的实现目标

按用户新决定，把左移中轮+HWT仍未解释的经验前后旁偏先纳入导航坐标输出，不再只做报告。保留原始轮+HWT坐标，采用用户最后明确固定的工程权重lambda=0.5；这是对小残差的候选增益，不是统计置信度，不将全程距离乘0.5。不改HWT实际读数、已有左电机2%后移、右1%前移、主向距离比例、控制/停止/定位valid保护。硬件暂停不变。

当前没有证据证明残差全是轮滑。它还包含尺量、摆位、共同源末窗与最终测量错窗等未识别部分。本候选是该工况下的工程残差模型E，不能把补偿后的坐标称独立测得的真位移或已通过实车定位验收。

## 2. 三趟均值训练及完全隔离的评分

唯一训练集是 `localization-left210-field-run1/2/3` 的新左210三趟，使用 `localization-left210-three-run-analysis/score_summary.json` 中同口径轮+实际HWT模型及用户尺量。全部左150三趟只评分，不进入本次系数计算，不根据其结果调lambda或系数后声称留出通过。左150历史已经看过，不冒称是此前从未看到的前瞻盲测。

约定前F/左L为正，已有raw包含实际HWT逐段旋转。每趟先用正左原始位移归一化，再对三个比例取简单算术均值：

```text
k_i = (ground_F_i - full_HWT_raw_F_i) / positive_raw_L_i
k_1 = 0.0207056643465753
k_2 = 0.01631057365918832
k_3 = 0.0167003774830163
k = mean(k_1,k_2,k_3) = +0.017905538496259973
lambda = 0.5
lambda*k = +0.008952769248129987
```

上述定义及数值已由正式MATLAB验证并与ROOT合同绑定。不得换成“残差均值/位移均值”，也不能按第三趟择优或回归新gamma。分母采用正左原始位移，不能改用已经校正的nav_L；禁止旧两趟gamma/beta、主向缩放或右/前/后样本混入。

lambda只作用上述F残差。新三趟左210均属于训练，不再将其第3趟称此新系数的独立留出。正式MATLAB给出左150三趟旁向表观MAE为7.10616851883877mm、最坏10.218473021313045mm；L保持原模型不变。该评分不用于重新选择lambda。lambda=0/1只作敏感性背景，不得替换用户固定的50%。

以后每个同工况组按用户决定测试3次，分别保留原始值和每趟误差，再报告三次有符号误差均值，并列绝对误差/范围以免正负抵消被隐去。不得自行剔除一次，不在本轮暂停期间启动任何一趟，也不为凑均值自动追加第四趟。

## 3. 唯一注入点：AppOdometry raw到nav纯投影

`ChassisOdometry s_odometry`及 `chassis_odometry.c` 的轮历史、时基、HWT解缠/实际角、前向倍率、状态与保护均原封不动，始终只产生raw。候选只在 `AppOdometry_Step` 调用 `ChassisOdometry_Update` 之后、发布快照之前生成nav副本。

在一次成功自动SetPose建立的固定参考上锁存原点O=(x0,y0)、路径起始前方向phi及scope。当前raw平移先作同参考投影：

```text
dx = raw.x - x0; dy = raw.y - y0
Fraw = cos(phi)*dx + sin(phi)*dy
Lraw = -sin(phi)*dx + cos(phi)*dy
cF = lambda*k*max(Lraw,0)
nav = raw                         // 完整复制status，随后仅改XY
nav.x = raw.x + cos(phi)*cF
nav.y = raw.y + sin(phi)*cF
```

这等价于Fnav=Fraw+cF、Lnav=Lraw。phi固定为成功SetPose时的路径前方向，**不用当前车体yaw旋转整个累计残差**；当前yaw已经用于raw逐段积分。不能额外加行程*sin(欠角)、不能减现电机2%命令、不能把地面残差再当独立加速度积分。`max(Lraw,0)`用净左位移，不累计abs增量；回弹时连续回撤，不把往返晃动重复累加为路程。

`heading_deg/sample_ms/age_ms/updates/valid/reasons/continuity_lost/local_origin`逐项与raw保持一致；不对valid作OR、不清reason、不因残差修正变健康。不增加停车条件或故障复位。

## 4. scope、参考与invalid合同

默认新宏例如 `APP_LEFT_NAV_RESIDUAL_ENABLE=0`；候选隔离构建=1，enable须baseline40 debug3。固定lambda=.5和冻结k，不增加运行时调参、在线拟合、通用残差框架。

scope只允许已各有三趟证据的独立左PLAN9/18、单动作count1、MOTION_LEFT_DISTANCE、102RPM、对应2100/1500mm完整tuple，且run_ms=0、reference_mm_s=0、relative_deg=0、非runtime route。PLAN19左30未三次验证，明确排除。通过现有只读运动观察信息核完整tuple，不改owner文件、`AppOdometryInputs`结构或原控制数据通路。

scope/frame **只在成功自动SetPose建新reference时锁存**，frame可借用现baseline固定参考；不要每tick按live PLAN或combined_state开/关补偿。锁存参考后，即使DONE、STOP、invalid、anchor_valid暂清或选择下一计划，也不得把已积累的cF突然删除。下一个成功SetPose显式建立新原点时才重置cF=0并锁存新scope；前/右/后/turn/route/左30的新reference为inactive，nav与raw完全一致。

- Init无参考：nav=raw，未补偿。
- 成功自动SetPose：raw/nav在新原点重合，cF=0；更新现baseline参考字段后锁存scope。第一次完整轮基准前不能凭空积分。
- ResetRecords：原raw anchor与新nav anchor立即无效，原raw记录清空行为保持，但**不可立即清掉旧scope/frame导致旧坐标回跳**。等待后续SetPose成功后再切换参考；原点重建引起的显式任务坐标重置与无参考地掉补偿是两回事。
- 手动SetPose请求失败：不改变残差scope/frame；保持原anchor_invalid处理与返回值，raw/nav anchor均为0。成功：显式新raw姿态，旧scope终止，nav=新raw；原baseline仍无自动动作锚时保持anchor_valid=0，ICM按原逻辑清历史。
- raw invalid/gap：保raw原冻结行为和所有invalid标志；用同一锁存参考对冻结raw继续纯投影，所以nav也冻结在已补偿值。**不能写成“if valid才校正、else直接raw”**，该写法会在失鲜时跳回未补偿坐标。重复sample不累加；恢复前若raw要求SetPose，nav不能偷恢复。
- lambda/宏/系数在候选运行中不改变；若以后需要改，必须在新reference边界，不本次增加动态混合状态机。

## 5. 原API保持、导航出口与ICM轮锚

正式合同保留 `s_odometry`、`s_baseline40.pose`、`AppOdometry_ReadBaseline40`、`g_perimeter_records[].pose`的原raw语义和行为。capture banks、perimeter/OP1、前向controller仍读原raw，均不修改；不把已有raw出口静默改成nav。

新增独立静态nav缓存，使用同一个现有 `AppOdometryBaseline40`类型（64B），新增 `AppOdometry_ReadNavigation40`读取它；原struct不增加字段。feature-off该getter直接alias原raw getter。feature-on时新nav缓存的pose是本计划的纯投影，原metadata逐项复制；`g_chassis_odometry`/OD1统一发布这份nav.pose。原raw诊断直接经现raw getter保留，无需新增raw全局或raw getter。相关快照按现临界区发布/读取，比较须匹配sample_ms/reference。

重要ABI限制：`AppOdometryBaseline40`嵌在 `app_fusion_capture.c` 的CaptureScratch，后者有固定schema/RAM上限。**不追加raw_pose、不扩大struct、不改任何RAW/context或capture布局。** capture源文件及其两处原getter调用完全不动，维持原轮/HWT/ICM原件与OD bank raw语义。新nav静态缓存独立分配，构建核map而非改capture bank。

`app_icm_observer.c`仅将原轮锚getter改成 `AppOdometry_ReadNavigation40`一次，随后原209..226行得到已校正导航轮锚，后续预测自然受一次残差影响；**禁止在observer末端或IO1序列化再加lambda*k*L**。其F/右R坐标、实际HWT重力旋转、cal、NO_LEVER、有效性和reference/session清历史保持。nav轮锚仍是同源锚，不能把IO1接近nav称独立真值证明。

前向controller始终读取原raw baseline，代码/API/目标和电机命令保持。前向新reference的nav scope也为inactive，nav==raw。导航残差不回写原raw或前向控制；验收直接比较实际前向控制输出及原raw快照，不靠只读调用图推断。

## 6. 最少产品改点与MATLAB前置

ROOT只批准四个产品文件：`app_config.h`（默认关宏/固定k与lambda）、`app_odometry.h/.c`（同型新nav getter/cache、纯投影发布、scope/frame及参考生命周期）、`app_icm_observer.c`（仅一次换nav getter）。不修改owner、输入结构、capture、perimeter、ChassisOdometry、ICM算法/结构、控制器、运动trim或保护。保原六dirty、GBK/CRLF，默认宏0的行为不变。

正式前置MATLAB已实际完成并由ROOT确认通过：`medium/matlab_receipt.json` exit_code=0、formal_lifecycle=true、hardware_accessed=false、product_written=false；MATLAB R2020a脚本SHA256 `3B6358D300CC6A937D13EA38364B2E1204B43F2AFE7CE885FE868EFCC9DB4FD9`，与ROOT合同一致。`medium/matlab_summary.json`状态 `PASS_FORMAL_LIFECYCLE_FIXED_HALF_GAIN`，包含20个scope cases、125个生命周期断言、解析投影最大误差9.095e-13mm、float发布最大误差约0.0002453mm。该证据允许其后四文件软件实施，不等于产品实现/编译已通过。正式验证要求为：

1. 直接从三趟210证据求每趟归一化比例再取简单均值、正号k与用户固定lambda=.5，冻结输入/系数hash；证明150三趟不进入训练。三趟210标训练评分；150标本次拟合留出，主距不变，列每趟和三次均值，不把训练改善冒称验证，不从敏感性表重新选权重。
2. 对phi=0/37/89.8/170/-170度、非零原点、纯左/零距/右净位移/回弹、跨±180实际HWT生成raw，核F增量=cF、L/heading不变、只一次注入。双精度解析目标误差≤1e-8mm；float实际C另用适当浮点容差。
3. 生命周期序列覆盖Init→SetPose→valid样本→重复sample→DONE/STOP→invalid/gap→ResetRecords等待→失败手动SetPose→成功新reference。验证同参考不凭空跳变、invalid标志逐字段不变、重复不累加、旧scope不因live PLAN变化关闭、新reference归零、前后右转/route无效scope。
4. 用与现observer相同的两锚F/右变换核nav锚只含一次cF、原raw getter/capture/perimeter/controller均保持raw；跟踪reference/session变化清旧锚。用静止/恒加速度解析例确认没有末端再补一次；不要求所有真实IO1帧变valid。

按上述已通过正式MATLAB及ROOT批准编写四文件后，仍须做实际C on/off对照、无效/生命周期、g_chassis_odometry与nav.pose一致、原raw API/记录/状态不变、observer一次注入、前向controller输出不变、默认和候选Keil0错0警告、map/capture结构尺寸不变核验。物理暂停；不下载/复位/发车。

## 7. 规则及仍未闭环

本计划已核基线R RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04、RUN-06。Q FIELD-05/OBS-03/QR-04保持待官方技术答疑。本轮方案及lambda均E，不更改正式规则。

仍未闭环：半权残差实车效果、left30仅外推未验证、同步地面真值和测量误差界、ICM平面杆臂/动态重力及有效窗、跨动作/整场绝对定位，后退尚未测。软件接入不等于上述已解决；既有硬件暂停状态及右1%未下载保持。
