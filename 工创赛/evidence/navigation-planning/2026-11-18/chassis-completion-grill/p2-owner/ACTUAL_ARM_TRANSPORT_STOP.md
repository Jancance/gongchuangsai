# P2 CAN 回读与 STOP 应答边界

产品改动基于 `8d7138f59bf360aca8dea3e3e1af5efc091ef373` 的本地工作区。CAN 复用已有 0x43 系统快照事务，可返回设备地址、原始绝对圈计数及方向、带符号 0.1 RPM、状态标志及主控请求/收齐应答时刻，失败清零并保持无效。新接口当前无运行调用者。已有 STOP 流程再次检查成功应答返回时的请求年龄，达到 500 ms 时沿用故障原因 6 和 VM 切断请求，且不记入该轮 ACK 位。

冻结前置见 [MATLAB 报告](REPORT.md)和 [SHA 清单](FROZEN_MATLAB_PRE_PRODUCT.json)。本机 R2020a attempt6 记录 78 项 owner 检查通过和六条合成位姿模型轨迹。原报告及冻结输入保持原样。

| ARM Cortex-M4 + Unicorn | 断言及结果 | 证据 |
|---|---|---|
| 当前 CAN 源，online/未确认、online/确认、offline/未确认、offline/确认 | 2150、2153、2150、2150，全部通过。覆盖旧快照接口、新结构字段和时间戳、32 位回绕、坏序号、非法字段、包长、地址、超时及参数为空。 | [回放结果](transport-arm-attempt5/actual_can_report.json)、[输入哈希](transport-arm-attempt5/inputs.json) |
| 旧 STOP 源，online/offline | 每配置 51 项，8 项预期失败。500 和 501 ms（含回绕）均复现 ACK 被纳入掩码及未请求断 VM。 | [基线反例](stop-arm-attempt2/actual_can_report.json) |
| 当前 STOP 源，online/offline | 每配置 51 项，全过。499 ms 接受，500/501 ms 拒绝，含回绕、首次期限、重复请求不刷新、失败应答及等待应答中取消。 | [回放结果](stop-arm-attempt3/actual_can_report.json)、[输入哈希](stop-arm-attempt3/inputs.json) |

旧 transport attempt4 的两条失败来自夹具对模拟读时钟次数少计 1 次，旧 STOP attempt1 缺少 `StartupAxes_Holding` 链接桩。两份旧记录保留。这两次修正只涉及测试夹具。

| 产品源码 | SHA-256 |
|---|---|
| `can.c` | `fc5a8ae25238c33b1e687539ccfee3cf348a03c7e91a893f092be3b6e09c04c6` |
| `can.h` | `b472223a92b63841b6b4e81168c140bb742be98d2c4572de6cc9b7e10cb7dbe6` |
| `chassis_auto_test.c` | `c86d93fce27db82889d762e546ca323624306fb35861064757d07d8310a84194` |

已核 R RUN-01/04/07、PWR-01 和 E LOC-03/05、MOT-02/03、RUN-05/06。FIELD-05、OBS-03、QR-04 待技术答疑。500 ms 是底盘软件停车期限。回放使用 RTOS 时钟、CAN 外设或停车回复桩，板端 CAN 时序、唯一运行 owner、轮速与 HWT 共同时间点、独立停稳证明和真实 VM/物理停车仍需单独验证。此记录不包含最新整机 Keil 构建，未接设备、下载或发车。
