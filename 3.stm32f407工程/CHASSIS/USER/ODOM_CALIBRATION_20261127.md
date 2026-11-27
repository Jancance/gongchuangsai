# 无雷达里程计四向距离比例（2026-11-27，本地候选）

## 用途与合并边界

本记录对应本地 `codex/chassis-navigation-plan` 工作树的无雷达里程计标定。它只覆盖编码器车体增量到毫米的四向距离比例，不修改航向 PID、速度曲线、轮径、轮距、机械动作或竞赛规则。

与队友版本整合时，以方向明确的 `CHASSIS_ODOM_*_DISTANCE_SCALE` 四个宏及 `chassis_odometry.c` 的 X/Y 分向应用为本项完整变更单元。若队友同时改了 `chassis_kinematics.h`、`chassis_odometry.c` 或 `app_config.h`，逐项合并，不整文件覆盖。旧宏 `CHASSIS_FORWARD_DISTANCE_SCALE` 保留为前进宏的兼容别名；旧可选调试 trim `APP_LEFT_DISTANCE_SCALE` 仅同步左移值，默认 trim 仍关闭。

## 拟合来源

系数按同方向、同趟编码器主向距离与地面实尺的比值拟合。编码器距离来自四轮位置计数、逻辑方向和 77 mm 轮径、65536 位置单位/圈；地面尺量来自用户已记录的 2026-11-26/27 单趟实测。

| 方向 | 地面尺量 | 编码器距离 | 比值 | 样本边界 |
|---|---:|---:|---:|---|
| 前进 | 1705、1710 mm（区间端点） | 1661.9 mm | 1.02743847（实尺中点 1707.5 mm） | 2026-11-27 前进三趟中用户记录的实测范围；日志 `forward_kp30_live_20261127`。比例仍需同趟尺量关联复核。 |
| 后退 | 1689、1683 mm | 1662.0716、1661.7672 mm | 1.01448963（两趟比值均值） | 2026-11-26 两趟；实尺偏向相反，不用偏向数据拟合距离。 |
| 左移 | 1630 mm | 1661.7 mm | 0.98092315 | 2026-11-27 单趟用户实尺记录。 |
| 右移 | 1640 mm | 1661.8 mm | 0.98688169 | 2026-11-27 单趟用户实尺记录。 |

倒车编码器和尺量证据在 `_speed_upgrade_work_20261125/left120_baseline_20261126/ANALYSIS_20261126.md`、`analysis_metrics.csv`、`reverse01.measurement.json` 和 `reverse02.measurement.json`。前进/横移原始 COM17 遥测位于 `_speed_upgrade_work_20261125/forward_kp30_live_20261127/` 与 `_speed_upgrade_work_20261125/lateral_live_20261127/`。原始日志未读回确认板上镜像；实尺与本轮原始记录的逐趟对应仍有不确定性，故本表是本地工程候选，不是跨速度、负载或场地的最终标定。

HWT yaw/rate 用于描述过程偏航和姿态旋转，不用于拟合 X/Y 距离比例。比例作用于车体坐标增量，之后才使用 HWT yaw 转到场地坐标。不同速度可能改变滑移与实际比例；当前数据没有同方向多速度、同条件的成对尺量，不能据此建立速度相关系数。下一次各方向直线测试应保存每趟实尺、四轮首末位置计数、方向、速度档、载荷、起点与终点测量参考，并在同速度至少重复后再决定是否更新对应系数。

## 后续直线航向参数建议

目前直线试验航向增益仍为前进 `Kp/Kd=2.75/0.25`、左移 `1.50/0.30`、右移 `2.50/0.30`；本次没有改这些参数。最近三趟前进 HWT 相对航向峰值约 5.19～6.43°、绝对角速度峰值约 34.72～42.72°/s，末端航向误差约 0.70～2.28°，并伴随用户记录的稳定右偏。左移单趟相对航向到 +2.25°、峰角速度 16.84°/s；右移单趟到 −1.46°、峰角速度 9.70°/s。偏航幅度本身不足以断定 PID 不合理，但前进现值尚未消除过程右偏。

下一轮先固定速度、起点、载荷、地面参考和方向，各做一趟可配对的基线；优先只对前进 P 做小步 MATLAB 量化影子比较，再决定是否值得一次实车对照，D 暂时保持 0.25。历史 `Kp=3.00/Kd=0.25` 影子回算虽有整数修正差异，但独立评审结论为不部署；不直接恢复该候选，也不同时改 P、D、速度表或轮速限幅。左右移分别保留现值为基线，先补同条件重复尺量，不共用前进增益。

## 代码与验证

- `chassis_kinematics.h`：定义四个方向系数；保留既有 `CHASSIS_FORWARD_DISTANCE_SCALE` 作为前进别名。
- `chassis_odometry.c`：X 正增量乘前进系数，X 负增量乘后退系数；Y 正增量乘左移系数，Y 负增量乘右移系数；再按 HWT 航向旋转到场地坐标。
- `chassis_kinematics.c`：无雷达距离进度 mode 0/1/2/3 分别使用前进/左移/右移/后退比例；`Chassis_DistanceSelfTest()` 同步核对四向比例。
- `app_config.h`：可选 `APP_LEFT_DISTANCE_SCALE` 更新为左移拟合值；`APP_LEFT_TRIM_ENABLE` 默认仍为 0。
- MATLAB R2020a 脚本 `_speed_upgrade_work_20261125/calibration_precheck_20261127.m` 检查比例重算、四个符号方向和 0°/90°坐标变换，结果见 `matlab_calibration_precheck_20261127.log`。此检查是代数/坐标映射前置验证，不证明真实地面轨迹。
- ARM C 单元 `validation/odom_calibration_20261127/unit.c` 和 `run_unit.py` 编译并执行目标分支的 `chassis_odometry.c`、`chassis_kinematics.c`；`Chassis_DistanceSelfTest()` 通过，并覆盖前/后/左/右四种轮符号在 0°/90° 航向下的累计位姿，共8种情形、16项断言通过。结果 `_speed_upgrade_work_20261125/odom_calibration_replay_20261127/result.json` 绑定夹具 SHA256 `9334a340df72712aa848f0118d02ce1a3e3ac060`、里程计源 SHA256 `0aae740c80750c6bb80a47b685de0e6d4da73c5c0b0b06c3f4decaff7b6e0551` 和运动学源 SHA256 `b0c1ccd5d79c51ed26cbcb72444c4c8ed1553660c0607791537df2e985accd79`。Cortex-M3 ARMCC 产物在 Unicorn 中回放，按 SHF_ALLOC 初始化 PROGBITS/NOBITS；几何、轮方向和 HWT 为受控输入，不是板端/实车测试。
- 以 GitHub `origin/main` 为基线重建：`keil_odom_calibration_integrated_20261127.log` 与 `keil_odom_calibration_offline_20261127.log` 均为0错误、0警告。活动 AXF 仅为隔离 worktree 的构建产物，不随标定源提交；未下载、未复位、未发车。

需求核对：R `RUN-01/02/04`、`FIELD-02/06`、`PWR-01`、`MECH-01/02`；E `LOC-02/05/06`、`MOT-01/02/03/04`。Q `FIELD-05`、`OBS-03`、`QR-04` 继续待技术答疑确认。比例是工程标定选择，不改规则。
