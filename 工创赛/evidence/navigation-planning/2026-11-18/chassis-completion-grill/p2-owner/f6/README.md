# 区2限时 F6

- Cortex-M4 回放修正旧 0x43 空 FIFO 桩时钟与 Offline 零速夹具。独审核出槽末无关帧挡住及时 ACK。修复前本机 R2020a 通过 136 项，源码随后仅在限时入口有界读取 FIFO。
- 最终工作树 CAN 源 SHA 前缀 `0574ec714`、`47936e6e`。主回放四组共 8545 项，高档独审含槽末排队及 STOP 边界四组共 8577 项。两套计数有重叠。连续定位设置下 Keil Integrated、Offline 均 0 错误、0 警告。证据见 `工创赛/evidence/navigation-planning/2026-11-18/chassis-completion-grill/p2-owner/f6/`。
- 限时 F6 记录邮箱受理、TX 完成、有效 ACK，准入 +2 ms、成功 +6 ms 含边界。驱动 ACK 无事务代号，后续 CAN owner 须核运行、会话、供电与取消，并保留部分施加掩码。
- 规则 R RUN-01/04/07、PWR-01。工程选择 E LOC-03/05、MOT-02/03、RUN-05/06。Q FIELD-05、OBS-03、QR-04 待技术答疑。下一步限时 0x43/0x3A、PC13 与实反馈/独立停稳、圆角联合 MATLAB 和整链回放。现场暂停，Git 仅本地。

回放使用 ARMCC Cortex-M4 和 Unicorn，CAN 与 FreeRTOS tick 由夹具注入。工作区可运行本目录的 f6_independent_replay.py 和 f6_review_fifo_replay.py。模型边界不包含现场 RTOS 抢占、总线负载及驱动旧 ACK 使用寿命。
