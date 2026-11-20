# 连续地图导航补偿与航向继承独立审查

日期：2026-11-17。最终复审包含 tracking-run 小型API；前一版624计数报告完整保存在 `INDEPENDENT_REVIEW_before_tracking_run.md`，不作为最终映像结果。审查基线 `555086e2b0361945cf02bf07cab556d528d39dc5`，分支 `codex/chassis-navigation-plan`。审查者独立阅读五源实际 diff、合同、MATLAB/ARM C 夹具与完整构建证据，并另行装载最终 ARM AXF 重放；仅写审查报告及旧版副本，未修改产品、调用硬件、下载、复位、发车或执行 Git 写操作。

## 结论

**PASS_WITH_LIMITATIONS：当前软件改动未发现剩余阻塞缺陷，可保留本地检查点；未部署、未实车验收。**

地图补偿使用独立默认关闭开关。四向距离段按成功接受的动作标签处理；普通 route 不再受旧单段 debug tuple 门禁限制，超出原样本条件明确标为外推。现有四向主/旁系数、50%权重和原航向 PD 保持。动作融合、斜行/边走边转执行、地图闭环到点及整场精度未在本阶段完成。

## 实际源码核查

- 最终增加 `AppOdometry_MapTrackingRun()`，以小型原子身份读替代 runtime 两处128B地图副本。核对其 pending run、TRACKING/valid、轮/HWT源龄以及 stale sticky GAP 判定与原 ReadMap 一致；不积分、不复制或修改XY。实际 C 已增有效run、getter锁存GAP后返回0、地图关闭返回0断言；API独自首次触发stale锁存分支为源码核查，未冒充该组合已动态覆盖。该改动减少相关深调用栈局部副本，不据此宣称运行余栈已验。
- `app_odometry.c` 保留独立 raw context。修正输入为本段净 raw 位移在段首实际 heading 上的投影正部；当前段纯重算，换段固化最后发布修正，未把 corrected XY 回灌积分器。四向主/旁符号与合同一致，不依据命令距离直接生成位移，不以绝对路程累计短时反向样本。
- 首个共同 `sample_ms >= action.begin` 只建立净段原点，使用该共同样本的实际地图 heading；旧共同样本不建立新段基准。动作边界与关闭后的停车尾保留 raw，分别公开 BOUNDARY/TAIL；定时或显式 turn 不增长经验修正并公开 UNSUPPORTED。新 epoch 即使同毫秒或跳号也重建净段基准。
- `MotionGetState()` 将已接受 `request=1` 视为 BUSY，避免上一动作遗留 DONE 提前关闭新标签。关闭后当前标签不重新激活。状态读取、补偿计算和发布处于同一发布临界区，getter 不积分补偿。
- GAP 分支先恢复同 run 的整份已发布快照，再将 raw status 的 XY 恢复为快照 raw XY，最后统一发布 raw+correction。raw、corrected、sample 与修正元数据共同冻结；同 run 不恢复有效，新 run 清本轮累计并忽略旧 run 标签。局部 ResetRecords/SetPose 不清地图累计。
- `App_MoveSubmit`、`TurnStart`、`App_PerimeterTurnStart` 三处只在成功接受分支发布标签；失败路径不增加 epoch。此项是实际源码与完整链接核查，ARM 单元夹具直接调用标签 API，未动态执行这三个接受入口。
- REFERENCE 分类已收紧为原 2100mm/RPM/debug tuple、相应 trim、route_speed=0 且控制距离比例关闭；300/1500mm及新比例版本为 EXTRAPOLATED。原先将执行合法性误当采样参考范围的问题已修正，实际 C 覆盖其分类。
- 四向直线原本均运行 `App_ForwardHoldStep -> HeadingController_HoldStep -> ChassisAuto_UpdateForwardHold`，本次未新增四套控制器或修改增益。中央 CombinedStart 从同 run 最近成功完成的计划 target 建立新参考，中央完成 hook 仅在真实后端 DONE/result/ownership/stop 条件合格后提交，覆盖绕过 RuntimeRouteInstall 的 VehicleMission_Load；失败预写 target 不成为继承值。
- 独立 Move 在成功提交前准备自身参考，并原子捕获已完成前段，防止完成后下一直接 Move 抢占吞掉残差。独立 turn 清旧列表 route flag；其 API 继续表示相对转角，不能将连续直接 `TurnStart(90)` 称作已实现绝对 90/180 规划。正式列表仍由固定参考加名义累计偏移生成目标。
- raw/corrected 的 heading 都保留真实 HWT+地图 offset，不置名义角；各新段补偿采用实际首共同样本 heading。现有 ±2RPM、10°有效范围、0.3°死区及停止后不再更新的控制边界不等于每个短直段末误差必归零。

## MATLAB 与实际 ARM C

已核本机 MATLAB 回执：overlay 1303、heading lifecycle 924 检查通过，产品编辑前完成并记录 product_unchanged。模型证明代数和生命周期合同，不证明车辆动态、采样延迟或毫米精度。

审查者未重跑会覆盖证据的生成脚本；另行按 ELF 的 SHF_ALLOC 节装载 PROGBITS、清零 NOBITS，每个映像新建 Unicorn 实例。最终五组均到达预期 stop，返回失败行号 0：

| 映像 | 独立重放断言 | SHA256 |
|---|---:|---|
| map_cal | 627 | `5e6ece2b448b812e07ca81888ad5d28bde5633391077566c9c6bd5b82d87efa3` |
| map_cal_reference | 627 | `2778efa47554b5009a872daf8fde27dbf8a095c4e64de17d158d681f0bd88e61` |
| map_cal_legacy | 627 | `a22aa13865ac68041c538448d286b12d39984c061c5817c5f367e3bfed0532d2` |
| map_raw | 349 | `1bf44624026993a9440239340379e2745fe8bc7499d598ebe5f96163b53c9320` |
| map_off | 4 | `3bb0edac1685a4bab3a233c420d79b02ab416b081d3528f192c73a85b00292af` |

夹具真实编译 AppOdometry、ChassisOdometry、kinematics、heading_controller 与 continuation header；RTOS/tick/critical、MotionGetState、heading/cache/power 为桩。覆盖两起点、四向、反向净进度、无新标签、首共同样本、实际87/180度、跨段历史、同毫秒多个标签/跳号、旧标签新 run、局部 reset、尾流、REFERENCE/EXTRAPOLATED、源异常与 getter 抢占 GAP。未在此夹具动态执行完整 app_motion_runtime、VehicleMission、RuntimeRouteInstall 或 RTOS 调度；记录其 SHA 不代表已经执行该源。首轮虚构过快输入触发源保护的失败已留档，产品源限未放宽。

七源 SHA 与重放回执一致，其中五个变更源为：

| 源 | SHA256 |
|---|---|
| app_odometry.c | `9ab3f2b7dd307be356dfb7825ab320704e7cf7118d79a2362c37e358881888e2` |
| app_odometry.h | `a02dc8aa4c367107f77b16131508c5e77520a69b862775b71f24bffa48c8d0cd` |
| app_config.h | `2070531ab2bea306fd69174566905a55c93f2e25eadde6d363c1c693f5b9ff20` |
| app_motion_runtime.c | `02a046ce73411eec8fb003c93fc8694351c59552e06c166ce507064ea0f8b45a` |
| route_heading_reference.h | `b2c536225fcaae8943fc3032d614c41048fa8468e86ef92c361c4aacbe7ff8a7` |

## 完整构建与资源边界

独核最终 19:50:20–19:50:48 的四份构建回执，均显式 `ICM45686_SAMPLE_ONLY=0`、0错误0警告；24个冻结文件 hash 与回执一致。integrated_on 实际链接地图/三处接受 hook/中央 Begin/Completed，未仅靠 sample-only 裁入口构建。最终 AXF：

- integrated_on：`82010a29789f5d3d0e7f484418b1b18c70fc9233f1e9030dc14de2e516cbb3cb`
- integrated_off：`85ad6c279f3fa9dfcdb605a3acabbe08dbcc4a1e766b5d8879596e37255147c1`
- map_off：`e3840bb578cb6de39418dd4338ce2fddc30d0e11f92e7e21c417cb8e9c6265f9`
- baseline_on：`2ff770a79ee2b0cc7e4d21db72637c1065fd7f3992c105f9430d40c81275199e`

ZI 状态/快照拆为独立节，由原链接区域安排，固定 SRAM/CCM 基址与容量未放宽。integrated_on 对补偿关闭差56B静态RW+ZI，对地图全关差1832B。五源非ASCII字节保持、均CRLF且无裸LF；限定产品 diff-check 通过。调用图仍含 Unknown Stack Size，CAN栈384字及静态调用图不能替代真实 high-watermark；未签运行余栈安全。

## 规则和未闭环

已核 R：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02。E：LOC-02/03/05/06/07、FIELD-08、MOT-01/02/03/04、CAL-02；本阶段补偿、参考继承和覆盖标记均为工程实现。Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。

未闭环：新组四向与L形/转向后续段精度、跨速度/距离/载荷外推、真实轮/HWT延迟、停车尾、双起点整场、扫掠车道/障碍、运行余栈，以及后续动作融合和地图闭环。地图不是无漂移绝对定位。本审查未读取当前板端镜像/供电/VM，未下载或发车，不继承历史硬件状态。
