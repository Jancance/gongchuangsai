# 区2 CAN 限时回读

`CAN1_ReadEmmSystemFeedbackBounded` 读取 `0x43` 五包系统状态并返回原始位置计数、轮速与标志。`CAN1_ReadStatusBounded` 单独读取 `0x3A`。两者要求会话已就绪，邮箱最晚在槽起点 +2 ms 提交，成功帧最晚在 +6 ms 完成校验，含两个边界。时间为主控观察时刻。驱动协议没有事务序号，运行调用方须避免重试产生同轮同功能歧义，并校验运行与电源身份。

- MATLAB R2020a 前置验证 7820 项。模型、基线 SHA256 和运行摘要在本目录。
- ARMCC Cortex-M4 与 Unicorn 回放四配置共 9825 项，含既有速度、诊断与 STOP 路径。首轮回放复现 `0x43` 在 +5/+6 ms 跨 tick 时提前超时，修正后按轮询前时刻决定是否继续等候，按接收和解码时刻验收。`arm_replays.json` 保留修正前后结果。
- 最终同一源码另复跑 `../f6/f6_review_fifo_replay.py` 的 8577 项，包括 F6 槽末无关帧排队及 STOP 边界。两套回放包含重复断言。
- 编译与回放输入 SHA256 为 `can.c` `14ef82bf11d9d34b859ae41577542fc57c565c404247096379d1646fddb0c2c9`，`can.h` `d96a2b8e6f26027a28444b557e982b05dff051307533365402bb3c837d86534a`。可执行本目录的 `can_reads_arm.py` 复跑。

已核规则 R RUN-01/04/07、PWR-01。工程 E LOC-03/05、MOT-04、RUN-05/06。Q FIELD-05、OBS-03、QR-04 待技术答疑。现场 RTOS 时序、实际 CAN 回复迟到与总线负荷仍待实测。新入口待接唯一 CAN owner 与导航停稳判定。

## 五帧与最终发布复核

独审复现后，本机 MATLAB R2020a 补验 9469 项通过，覆盖分帧长度、编号、解码和诊断时间及计时回绕。补验通过后，限时 `0x43` 增加编号 0～4 和 DLC 8/8/8/8/2 校验。诊断结束后复核截止时刻，在同一临界区提交有效反馈。`0x3A` 也在同一临界区完成最终时刻判定和时间写入。调用方仍需在获取本次运行状态后复核时效和事务归属。

最终工作树 SHA256 为 `can.c` `e423949a111d5944d824a31cbd36374edea5bc7e5e04cac95f1859058ea02373`、`can.h` `220939ec31350bc498fd4f4af5914fd7d71ae47c2b0fa8cd4cdbbf61eead70db`。最终同源主回放 9825 项、原独审夹具复跑 9917 项、F6 和 STOP 回归 8577 项均通过，三套存在重复断言。独审夹具包含六帧失败、诊断后过期失败和回绕时接收时刻为 0 的成功场景。证据见 `arm_delivery_fix_replays.json`、`matlab_can_reads_delivery_receipt.json` 和 `matlab_delivery_fix_result.txt`。
