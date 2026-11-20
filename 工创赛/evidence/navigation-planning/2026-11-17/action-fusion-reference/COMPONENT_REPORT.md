# 独立参考模块组件交付（2026-11-18）

本阶段仅新增 CHASSIS/USER/fusion_reference.c/.h，调用者拥有 plan/state；没有 RTOS、CAN、电源、机械、视觉或运行安装调用。root 负责统一工程登记、整机构建、交接和本地提交。本组件未访问硬件、未下载/reset/发车、未联网。既有 map50%、四向控制倍率、trim 和 heading PD 未改。

## 当前结果

- 本机 MATLAB R2020a 最新实际 exit0：9034 检查，脚本 SHA256 `23073cb5d4914ac30c3b0064cff64d277043f20d2d0cb9b55150047df2ef4f76`。回执 matlab_receipt.json；实际时间 2026-11-18 00:22:19→00:22:26。历史 8695、9010、9028 不覆盖后来新增的行为，分别留副本。
- 最终真实 ARM C：1364 断言通过，包含 96 组 MATLAB budget_vectors.csv 逐项浮点对照；actual_c_checks.json 为最终结果。使用实际 fusion_reference.c 和 chassis_kinematics.c，经 ARMCC Cortex-M3 soft-float 编译/链接后在 Unicorn 执行，无 RTOS 或执行器桩。此项不是 F407 实时或实车验收。
- 最终 C SHA256 `07a9439f0435cb8ce35fc045b0a74f17e07e0362549336859760ae5921d02fb2`；H `8e0d855ae91a5ae6bc856aaf6af2300cd24ccdf8d91d251294087466fa0ab961`。两源 ASCII/CRLF，未转码既有文件。
- ARM 编译/链接日志没有警告。ABI sizeof：FusionPlan 1408B，FusionInput 1012B，FusionState 120B；均调用者分配，不是栈余量证明。
- 1314 是补齐容量/自交等矩阵前的历史回执，actual_c_checks_1314.json 保留；独审另有自有测试计数，不能混为同一总数。

## 已实现行为

1. 固定有向场地折线、32 点/32 调用方障碍盒容量，超容量拒绝；2400mm 场界和四黄区内置，调用方不能漏掉。所有几何坐标均以底盘中心为参考，参数是显式模型输入。
2. 矩形 body 在 heading 预算区间的支持函数极值生成保守 AABB，对平移作连续线段扫掠；同时核实际姿态 footprint、回线短段和 desired 速度短预览。保守检查可误拒合法路径，不能作为整场物理保证。
3. 当前段 signed 进度、横偏、沿原折线 remaining。只遍历相邻同向共线段，可在一次源间隔跨多个小节点；遇折角停止，绝不 global nearest 跳段。同向共线保持非零 desired，非共线仅在位置窗口和真实停稳后换向。
4. 计划绝对 heading 独立于真实 heading；90 在起点，180 拆头尾两次 90，当前保守可行则优先 CW。实际 heading 转换世界/车体速度，heading 误差由后续适配复用原 PD，本组件没有第二套 heading PID。
5. backward pass 将后续低速/停止要求传播到前面的共线段。速度是沿程 desired cap，横向有独立 cap，总 desired 平移不超 400mm/s；先扣 heading wheel reserve，再用实际 Chassis_Inverse 限轮并回算 body/world，不把未兑现的原请求当输出。
6. 实际速度按 vt/vn 独立预算，末端横向15mm/s回线可跨新鲜样本持续，沿程400/横向100等严重异常仍请求恢复。终点 remaining0、横偏30或未真实停稳均不判 DONE。
7. run/start/map/layout/path/direction、source健康/时间/非有限输入校验；坏输入不推进状态，恢复后可继续。重复源 sample 返回完整缓存参考，不消费阶段；同 timestamp 内容冲突拒绝，stale 校验先于缓存复用。没有新增永久锁存。

## 首错和修复证据

- 初始 MATLAB 两个夹具错误（减速节点索引、所谓合法斜段的 body 已碰黄区）留 matlab_attempt1/2；修正夹具，没有放宽几何。
- 后续独审发现重复源参考变化和共线端点球约束；先补可调用 MATLAB state_step/cache/有界邻段归一化并实际9028通过，再改相应 C。前草稿 draft_before_cache_* 保留。
- 末端沿程 cap0 被实际速度模长误用，MATLAB 和实际 ARM C 均真实重现；lateral_failure_* 保存首失败。先 MATLAB9034通过，再改 C 为 vt/vn 分开校验；最终1364通过。没有通过放大 margin 掩盖。
- ARM 夹具首轮的 size 符号被链接器移除、脚本读取默认 GBK 和 MATLAB 日志解码问题，保 actual_c_attempt1_* 记录；这些不是产品通过结果。

## 明确未完成

始终 model_only=1、deployment_allowed=0、calibration_verified=0，无运行调用、运动许可、A/J执行或真实制动保证。acceleration_hint 的平方根 cap 不是 jerk-limited 轨迹；preview 也不是当前速度/源延迟/当前加速度下的停距保证。后续执行适配必须处理这些约束。

转向几何使用完整旋转圆的外接框，不能区分 CW/CCW 的空间：180 当前在该保守模型下可行时 CW，否则拒绝；不能称已实现精确 CCW fallback。曲线 blend、不停顿折角、实际有向转向扫掠、雷达路径/地图 adapter、运输包络/停车点冻结和双起点整场均未闭环。

混合动作只标记 MIXED/未验证，没有改既有 map50% 或按两个车体分量重复套系数。原纯四向经验补偿如何覆盖小幅混合纠偏，须后续对应 MATLAB/执行适配验证，不能据此静默取消用户已定主向补偿。

已核 R FIELD-01/02/06、OBS-01/04、RUN-01/02/07、MECH-01/02；E LOC-02/05/06/07、MOT-02/03、CAL-02；Q FIELD-05/OBS-03/QR-04 继续待技术答疑。MATLAB、实际 C、整机构建和实车证据分别记录。
