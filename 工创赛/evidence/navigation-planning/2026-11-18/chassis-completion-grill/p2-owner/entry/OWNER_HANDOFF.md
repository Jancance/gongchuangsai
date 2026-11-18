# 区 2 CAN 批次时隙与停稳交接

提交批次时，临界区内校验生产者传入的绝对时刻仍等于 owner 公布的下一 phase 0 时刻，并且距当前为 1～80 ms。回绕后时刻 0 有效。沿用 run、session、power epoch、待发送事务及 generation 检查，四轮 ACK 齐全后发布。

释放下一段前核对区 2 run/session、同一个活跃供电 epoch、最新轮速与状态、STOP ACK 后四轮停稳，再复用底盘 DONE 成功释放检查。释放保留 VM 请求，下一次 Begin 核发新 session。

- 产品 C/H 编辑前，本机 MATLAB R2020a 运行 `validate_zone2_owner_handoff.m`，退出码 0，输出 `ZONE2 OWNER HANDOFF PASS MATLAB 2020a: 53248 scenarios, 20 accepted times, 4 completed releases`。脚本 SHA256 `993a86137c9c76c9d7a23bba35bef07e10c4b1dfac6c1d6246858a88dbe93da5`。C/H 前后哈希分别为 `2034bce1f6d2684e22ac31f28618e5c738a2887919c2a55dc07f0496e1588877` 和 `7508d86b05e319c63568c9587035254de07a983a28fdd23e0297dd5aabf41379`，验证前后一致。MATLAB 输出用户配置目录访问提示，批处理校验完成。
- Cortex-M4/Unicorn 3,921 条断言通过，覆盖跨零绝对批次、错时隙、截止时刻、错代、四 ACK、DONE 前拒绝、停稳失败拒绝、错 run/session、epoch 改变、重复释放及新会话。见 [actual_can_report.json](owner-handoff-arm-attempt2/actual_can_report.json) 和 [inputs.json](owner-handoff-arm-attempt2/inputs.json)。首次使用模拟供电时刻 0 并在启动期间跨回绕，产生 4 条失败。最终回放让预备流程使用正常启动时间，再在批次准入场景设置跨零时钟。完整跨回绕启动未被此新增向量覆盖。
- R 核 FIELD-02/06、RUN-01/04、PWR-01。E 核 LOC-05、MOT-02/03、RUN-05/06。FIELD-05、OBS-03、QR-04 仍待技术答疑确认。真实调度、CAN 回包和带载停车续行待现场验证。

## 单次轮反馈超时

正常更新过四轮后，运动中第一次 0x43 读取失败可在下一常规轮询重读该轮，不增发 CAN 事务。恢复期间轮样最大年龄 150 ms，四轮都取得有效新反馈后恢复 130 ms。期间再次失败时取消放宽并恢复轮询轮转，过期反馈仍触发 STOP。停稳证据继续用 130 ms。

- 改代码前，本机 MATLAB R2020a 执行 `validate_zone2_wheel_retry.m`，退出码 0。1,152 个调度场景通过，含 768 个重复失败停车场景。正常/恢复反馈年龄最大为 116/146 ms，已建立历史时共同可插值时间的最大年龄 136 ms，恢复后为 116 ms。第一次模型断言误要求连续故障场景也能持续插值，修正后只对单次超时验证仍可插值。
- Cortex-M4/Unicorn 4,456 条断言通过。单次丢包保留原时间戳，重读取得新反馈后推进至第 5 批、STOP 计数 0。连续两次失败保留原因 3 并完成四轮 STOP。记录见 [actual_can_report.json](owner-wheel-retry-arm-1/actual_can_report.json) 和 [inputs.json](owner-wheel-retry-arm-1/inputs.json)。
