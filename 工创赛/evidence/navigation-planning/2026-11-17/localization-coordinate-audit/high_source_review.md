# 定位坐标链独立只读审查

日期：2026-11-17。角色：GPT-6.1-sol high 独立审查。源基线：`24afd6f3a119f2eb1c88f05e3224344bb4df5713`，分支 `codex/chassis-navigation-plan`。

## 结论

**当前源审查未发现需要为“实际转89.8度而非命令90度”修改二维坐标公式的确定错误。** 当前实际产品已经用 HWT 的实际航向、共同源时刻及四轮增量计算二维坐标。无需另加固定0.2度补偿，否则会重复计入角差。

当前产品不是可直接用于整场的跨任务统一定位：同一组合路线内部有连续观测路径，但新任务重锚、终态位置轮询结束后失去连续性、ICM仅短窗观察器等边界仍存在。应先验证这些边界和模型，不能据此删除连续性保护或直接部署融合。

本次仅新增本报告及 `independent-c` 离线例证，不修改产品、参数、保护、原dirty或历史证据；不执行硬件、串口、下载、复位、Git写或网络同步。MATLAB数值验证由本任务另一角色执行。

## 权威与范围

已读根目录及 `工创赛/AGENTS.md`、根《长期记忆.md》与交接最新内容，以及 `工创赛/docs/competition_requirements_2027.md` 相关条目。实际工程为 `3.stm32f407工程/gongxun-f407.uvprojx`，其文件表指向 `CHASSIS/USER` 下的本次三个模块，未采用旧 `USER` 或冻结工程作为当前实现。

- R：RUN-01、PWR-01、MECH-01/02，仅核对本次离线工作不会变更正式自主入口、电源及展开边界。
- E：LOC-01～07、MOT-01～04、RUN-06。本报告的坐标、时间、融合和验收判断均属于工程判断，不是正式竞赛精度规则。
- Q：FIELD-05、OBS-03、QR-04继续待技术答疑确认。

用户已明确 HWT 读数作为可靠转角基准，接受约0.2度欠角；不研究其漂移，也不以地面直角复测为本轮前置。早期摆位误差和源末窗早于物理末点的限制继续有效，不据此拟合固定误差或声称地面精度。

## 已核真实链路

1. 工程文件表中 `app_odometry.c`、`chassis_odometry.c`、`app_icm_observer.c` 均取自 `CHASSIS/USER`。
2. `heading_source.c` 从 HWT 已接收角帧重建 `angle_ms`，`app_odometry.c:98-107` 将 `source.yaw_deg/source.angle_ms` 与四轮位置样本送入纯里程计；不是命令转角，也不是任务DONE角。
3. `chassis_odometry.c:125-140` 选择四轮/HWT最新样本中最老的共同时间，再以历史包围插值。`sample_ms` 是该共同时间，`age_ms` 是观察时刻减去该时间；它不是即时实车位置。
4. `chassis_odometry.c:157-171` 从前/左为正的四轮位移解算 `dx/dy`，按实际HWT增量作中点旋转和 `sinc` 修正，再累加场地X/Y。`heading_offset` 只建立场地与HWT之间的零点差。跨±180度先展开再计算增量。
5. `CHASSIS_FORWARD_DISTANCE_SCALE=1.02182540` 仍作用于车体前向 `dx`。这是现有工程系数，不因本轮源审查就认定有误，也不能用不可靠旧尺量重新拟合。

该中点/sinc形式对一个采样间隔内近似恒定车体平移速度和角速度成立；不是任意变化运动的精确连续积分。需要MATLAB覆盖联合运动，但不能把离散近似本身当成已经造成实车误差的证据。

## 坐标符号无已证实冲突

`chassis_odometry.h` 明确车体前/左/逆时针为正，场地X/Y固定；麦轮逆解 `chassis_kinematics.c` 与里程计的 `dy=(lr-rr-lf+rf)/4` 一致。

ICM当前轴映射为 `aF=-sensorY`、`aL=-sensorX`、`aU=-sensorZ`，符合已确认安装方向。`app_icm_observer.c:204-211` 输出的是**路线起始方向的前/右坐标**，并非当前车体前/左：

```text
theta = HWT_yaw + heading_offset - path_heading
a_path_front = cos(theta)*aF - sin(theta)*aL
a_path_right = -sin(theta)*aF - cos(theta)*aL
p_path_front = cos(path_heading)*dX + sin(path_heading)*dY
p_path_right = sin(path_heading)*dX - cos(path_heading)*dY
```

因此 `position_r_mm` 与场地Y/车体左向不能直接同号相加。现有两套变换互相一致，尚无充分依据去翻转某个轴。后续残差融合必须先变换到同一个坐标系和参考点，并匹配同一时刻。

## 跨动作与跨任务的确定边界

### 一条组合路线内部

`app_chassis_service.c:26-35` 只在组合状态1/2时交替执行闲时位置轮询；`chassis_auto_test.c:1229-1252` 在已供电、非active等条件下继续补轮位置。动作开始的 `Auto_PositionReset` 清动作进度历史，但未清 `s_odometry_samples` 或 `s_odometry`。所以不能笼统说“每个动作都会把二维位置清零”。同一有效参考内可以跨动作积分，实际是否连续还受轮/HWT样本时效和轮询调度影响。

### 新任务与终态

`AppOdometry_ResetRecords` 清 `s_perimeter_odom_started` 并使baseline anchor无效；`App_RuntimeRouteInstall` 和 `App_Stage1Rearm` 调用它且清路线参考。下一次组合状态1时，`app_odometry.c:82-95` 会重新 `SetPose`：当前阶段1/运行时路线设局部 `(0,0,0)`，旧perimeter分支设 `(2100,2100,-90)`。这不是任意整场地图上的持续绝对位姿。

组合状态3之后不再经 `App_ComboIdlePoll` 更新轮位置；在没有其他位置更新路径的该链中，400ms后 `continuity_lost` 锁存，位姿冻结直至明确 `SetPose`。即使HWT之后继续变化，也不能将冻结二维坐标当连续有效定位。

这两点是现有生命周期边界，不足以证明原观察器实现错误。若未来目标改成跨独立任务连续导航，需要另行设计明确的场地锚定及任务间观测策略；不应直接去掉失效判定来伪造连续坐标。

## ICM合用的实际边界

`app_icm_observer.c` 受 `APP_BASELINE40_ACTIVE` 限定，默认产品配置为关闭；本轮实测候选是否启用须查其独立冻结配置，不能用默认宏代替候选事实。读输出的当前产品调用仅串口观察，没有回写里程计，也没有给予运动许可。

其 `Obs_Predict` 使用相邻轮坐标锚点和这段ICM加速度，预测到较新的ICM时刻。它依赖轮位置，不是独立惯导位置，也不能把与轮位置相近当成独立真值验证。动作session或参考变化时清短窗，不应直接拼接每趟局部输出。

当前存在明确披露的限制：

- `ICM_OBS_NO_LEVER` 始终置位，未补传感器偏心旋转加速度。真实安装偏心存在，旋转时不应直接将传感器加速度作为车心加速度使用。
- 重力仅通过陀螺传播；校准后数据缺口会锁存 `ICM_OBS_GRAVITY_GAP`，之后仍可在 `reasons==0` 时发布 `valid=1`。所以该valid仅表示当前短窗数值条件通过，不能解释为重力方向已完整恢复或可部署健康。
- HWT采用因果旧样本保持，100ms上限是准入条件，不是已测时间同步精度；轮位置时标是CAN读回接收时刻，不是驱动内部已知采样时刻。
- 短窗共用轮/HWT输入，ICM与轮残差存在相关性；不能用该残差独立估出融合占比或绝对精度。

已知杆臂可作为离线模型输入使用；高度未测时z=0只能保留为二维假设。当前无需为仅观察输出修改产品；如果后续要接入控制，必须先完成相应MATLAB前置及同时间/同坐标/同参考点验证。

## 本阶段建议与未闭环

本阶段结论为 **SOURCE_REVIEW_PASS_WITH_LIFECYCLE_AND_OBSERVER_LIMITS**，不是整场定位验收。保留现有代码，先检查MATLAB的实际89.8度、跨180度、二维联合轨迹及现有数据残差分解；不叠固定0.2度，不拟合旧尺量，不把源末窗当物理末点。

没有直接可原样运行且指向当前纯里程计的旧回归入口：`工创赛/scripts/p1_odometry_check.py` 指向旧 `3.stm32f407工程/USER` 并抽旧main适配器；提取审查脚本也固定历史输出目录。若补实际C证据，应在本阶段全新目录摘取必要案例、编译当前 `CHASSIS/USER` 实际源并记录哈希，不覆盖旧证据。

按root追加授权，已在全新 `independent-c` 目录补最小实际C例证。`run_actual_c.py` 逐字节复制当前 `chassis_odometry.c/.h`，仅对几何及轮方向extern提供从当前源提取的常量stub。ARMCC Cortex-M4编译/链接和Unicorn运行通过 **5类、416断言**，详见 `independent-c/report.json` 与 `build.json`：

- HWT实际转89.8度后，输入经现系数校正为1000mm车体前移，输出场地 `(3.490511,999.992249)` mm、朝向89.800003度；实际保留了欠角的坐标影响。
- 场地朝向170度跨+180至-170度后再前移100mm，得到 `(-98.479935,-17.364681)` mm。
- 双向纯转的中心X/Y均维持零。
- 位置缺口后位姿冻结，新的fresh样本不能偷偷恢复。
- 明确SetPose后按新参考恢复，旧历史未被拼入。

该C证据验证纯里程计数学和其失效语义，不包含app生命周期、真实任务调度、传感器物理延迟或地面精度；不替代MATLAB前置。

## MATLAB最终收口独审核验

主MATLAB最终状态为 `PASS_DIAGNOSTIC_WITH_LIMITATIONS`，R2020a。最终 `medium/audit_localization.m` SHA256为 `4fdde01261bc8d67b208fdff6aa61df3df57d6c858143ffdb6e9ed3211d3a0d9`，与 `attempt7_script_hash.json` 一致。已读最终脚本、summary、LOO逐项名单和原始转向decoded，确认以下口径：

- 8个解析例通过，最大误差5.68e-14mm。300mm前移→实际转89.8度→再300mm前移，应得 `(301.047195,299.998172)` mm；强行多补0.2度得到 `(300,300)`，反而偏离约1.047mm。这只是几何例证，不是实车精度。
- 22趟离线轨迹拆分为车体前向分量旋转贡献与车体左向分量旋转贡献后相加；采用逐时刻HWT航向，不以最终yaw一次旋转整段累计位移。
- 在线OD主方向同源比较仍保留原 `<0.01mm` 检查。旁向不是同样精度的数值复现：最终最大绝对差约0.1321mm，早期双轴0.01mm检查已失败并保留。在线按共同源时刻分段积分，离线回顾性插值到5ms后积分，还对在线OD做时间插值；二者不是逐步同构算法。不能将旁向差说成全二维0.01mm复现，也不能据此认定产品坐标错误。
- 独立Python重新读取最终summary和 `loo_heldout.csv`，逐项确认16个留出样本没有混入训练名单，按同方向/同版本重算固定值和比例系数及留出残差，最大差4.62e-14mm。没有发现留出真值泄漏，但选模型仍基于这批数据，不能当独立前瞻验证。
- 当前16趟综合旁向残差MAE：不补偿7.329mm、固定值LOO7.760mm、比例LOO6.695mm；分方向并不普遍改善。右向仅2个当前版长距样本，前/后/左混有30/150cm，原摆位未知且尺量末点不同窗。结果只能用于拒绝贸然补偿或提出后续验证，不能作为部署收益/绝对定位精度。
- 独立Python从4份原始转向decoded重算PRE窗口静态陀螺均值、原生edge_cycle时间积分和同窗HWT角差，末角最大差4.26e-13度。ICM减HWT末角约 `+0.1303,+0.1216,-0.1150,-0.1220` 度，未把末点强行对齐。逐时刻最大差约2.26～2.45度，包含时序差及积分误差，不能全归因为传感器角度误差。首raw.service_ms与物理采样边沿的固定偏差未测。
- 上述HWT角差取共同RAW源窗约5.08～5.49秒，不是约20秒尾段最终停稳角，不能与此前最终角混用。

最终结论：**PASS_SOURCE_AND_OFFLINE_DIAGNOSTICS_WITH_LIMITATIONS**。保留当前坐标公式与产品控制，不部署新ICM融合或经验旁偏补偿。源审查、MATLAB诊断和实际C五类例证相互支持“实际航向已经进入坐标变换”，不构成地面精度或整场定位验收。

未闭环：实测同时间位置真值；跨独立任务统一场地锚定；杆臂/姿态/采样时序验证；经验旁偏残差的重复性与独立留出；正式比赛入口及LOC-07整体验收。没有硬件或实车新增结论。

## 新单次左移执行脚本的追加只读核对

按root追加范围，比较 `localization-left150-field-run1/execute_once.py` 与已成功 `turn90_90-field-repeat-ccw2/execute_once.py`：仅3处改动，分别为绑定plan20→18、Controller20→18、运行计划名 `turn_ccw90_90`→`left150_40`。没有增加START循环、自动重试、下载或放宽硬件/故障检查；保留一次reset和一次START，以及新目录独占创建回执。

当前 `execution_binding.json` 绑定已装候选 `79e74f1f...fa2f0d` 的原下载回执，plan18/左1500mm/参考400mm/s，`execution_permitted/site_confirmation/root_release` 均为false，脚本因此不可直接发车。未执行脚本、未连接硬件。仍应由root在新横移空间和测量条件确认后按已授权单次范围释放。

源 `App_SerialMotionSafety` 的3000ms仅用于 `AppDebugTurn90Plan`；plan18仍为1000ms。此次脚本复用不授权改保护或宣称plan18已有3秒保活。新增实车数据可检验本批输入，但单次结果本身不足以部署残差补偿。
