# ICM 独立短窗观测：本地候选完成

2026-11-15。最终设计以 `high_plan.md` 及用户追加的 **20 ms（50 Hz）预测**为准。产品已冻结，待独立 high 检查和协调者本地提交。没有下载、复位、串口命令、发车、Git 提交或网络同步；板上仍是先前 `3966ed37…1113d`，不继承其历史实时 VM 状态。

## 已实现

- `app_icm_observer.c/.h` 由现有 ICM owner 消费新样点；200 个运动前、VM 命令关闭时的连续有效样本均值只是静止假设，不是六面标定。均值用 int32 原始 counts 累加，避免 200 次 float 加法人为产生径向残差；运动先开始则本次复位周期保持 WAIT_CAL。
- 三轴 gyro 减起前均值，二阶传播并归一化重力；使用已发布且不晚于该 IMU 样点的 HWT 航向，按零阶保持投影到路径前 F/右 R。128 个 `{tick,aF,aR}`，无全程 I/J；20 ms 最多预测一次，不追赶补跑。共同编码器锚 Δ=1..250 ms，预测龄 0..200 ms，并要求连续历史完整覆盖。
- 丢序、丢 edge、skipped/race/transfer 计数变化、数据/参考/HWT/连续性失效只清自己的历史/锚点、发布无效原因；重力跨断点的 GRAVITY_GAP 诊断保持。参考/session 改变当次立即发布 invalid，不能留下旧会话 valid。
- IO1 为 27 个整数及 CRC16-CCITT，版本 1；时间/计数为 unsigned 字段，18..25 为 signed milli 单位。无法计算的数据使用 `INT32_MIN`，不把失效当零。成功状态帧后 phase40..55 ms 才尝试，500 ms **尝试**节流，忙就跳过，位于原所有诊断调用最后；原 IR1 不变。parser 验证 CRC、字段、符号、flags、窗口与 sentinel。
- 仅修改新模块、main hook、serial IO1、正式唯一 uvprojx 双目标条目及离线 parser。未加任务/栈/优先级、动态分配、控制接口或停车条件。

## 实际验证

1. 本机 MATLAB R2020a：`observer_local_matlab.m` 最终 **268 项 PASS，exit0**；1 ms 锚输入 float 量化按 `eps*(|p0|+|p1|)/Δ` 独立解析项计入界限。最大 double p/v 误差约 `5.83e-13 mm / 9.12e-12 mm/s`，single 约 `0.0005045 mm / 0.0059275 mm/s`，均为合成数值。`observer_schedule_matlab.m` **exit0**，20 ms 正常节拍和 100 ms 延迟后不补跑通过。
2. `check_observer_arm.py` 真正编译并执行当前 C 核、当前 IO1 发送函数：**98 用例、395 断言 PASS**。覆盖零运动、正负 F/R、常加速度/jerk、1/22/100/200 ms 锚、异步裁段、250/251 与 200/201 ms 边界、tick 回绕、非单位静止均值/非零 gyro、重复与丢序、HWT future 不回填、session/参考/连续性、缺历史后恢复及 DONE 旧锚失效。实际 ARM 生成的 WAIT_CAL/UINT_MAX、2100 mm 与回绕 IO1 均经当前 Python parser 通过；忙 TX 连续调用没有重试。外设/时钟/getter/TX 为合成桩，执行器为 Cortex-M3 soft-float 指令/Unicorn，不是 F407 实时测量。
3. `build_observer.py` 从唯一工程生成隔离副本：baseline40 integrated、baseline40 offline、baseline-off integrated 全部 **0 Error / 0 Warning**。最终完整 SHA 绑定在 `build_summary.json`、`observer_arm_summary.json`、`observer_validation_summary.json`。
4. 1937 项实施前源 manifest 只有获准的 main/serial/uvproj 三个既有项变化；其他控制源保持原 SHA，parser 单独有 before/after。原 GBK 字节及 CRLF 保留；uvprojx 原 LF 保留，新 C/H 为 ASCII/CRLF。
5. baseline-off allocated **尺寸相同，非逐字节相同**：RO 仅两字节不同，为 main 新增 4 行后，`App_RtosAssertFailed` 的两个源码行号立即数从 401/471 到 405/475。已有反汇编与字节明细；其余 allocated 字节及 RAM 完全一致。没有为行号一致性保留转递 include/空宏技巧。
6. 链接 callgraph：ICM task 已知静态链 592 B，原分配 1024 B；Step 自身 328 B、其链 504 B。IO1 已知链 376 B 加 printf unknown；这些数值不含中断/上下文额外栈，不能取代真实余栈。集成 Code/RW/ZI 为 106680/3064/121304 B（旧 102328/2952/119096 B）。

## 失败与旧尝试保留

- 旧 `validate_observer.m` 的全程 J 线性内插失败原件、log、receipt 保留；没有放宽其断言来伪称通过。
- 新局部脚本首轮 204 项和改严因果后 205 项证据分别保存；最终加 1 ms 时首次 single 界漏算锚位置量化，失败脚本/log/receipt 保存为 `.delta1failed`。修正的是独立数值误差界，局部积分核未改。
- 初轮实际 C 检查定位 float 累加静止均值产生约 −0.01758 mm/s² 径向数值偏差，以及参考改变后旧 valid 可存活到下个节拍，两项均已修改并纳入最终复测。
- 行号保持的短暂试验已撤回；只采用最终清晰 include + `#if` hook 的源码/构建绑定。旧中间 AXF 哈希不可当最终候选。

## 交付与未闭环

`checkpoint_delta.py` 已生成 `checkpoint_sources.json`，以 base64 保存四件既有文件精确 before/after 和新 C/H after，避免 Git EOL 转换破坏原字节；`integration.patch` 只表达此次六件增量。协调者负责独立临时目录复放、reverse check、根交接及本地 commit。既有四件 dirty 不应整件混入本次提交；该交付是精确增量恢复点，不是 checkout 即可重建所有既有未提交成果的完整 checkpoint。

未闭环：真实 200 Hz 完整性/执行耗时/任务余栈/通信负载、三轴实物符号与完整标定、长期姿态漂移、安装左90/后20 mm及未知高度的杆臂未补偿、振动/滑移/沿程地面真值。本模块只补旧轮/HWT锚到最新 IMU 样点的短时间外推，不能消除或认证整趟侧滑。

已核 R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06。所有新增数值均是独立观测工程边界，不是官方精度或停车规则；Q FIELD-05/OBS-03/QR-04 仍待技术答疑确认。
