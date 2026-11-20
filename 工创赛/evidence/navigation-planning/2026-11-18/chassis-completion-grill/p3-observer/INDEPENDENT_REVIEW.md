# 区2雷达事务独立复核

2026-11-18，6.2 Sol 高档。单独编译工作树实际 `radar_observer.c`、`radar_client.c` 和 `link_protocol.c`，ARMCC Cortex-M4 指令由 Unicorn 执行。夹具使用原始协议字节、CRC、出线请求序号和 Observer 公共接口，RTOS 时钟、USART/DMA、舵机和升降由夹具提供。未连接硬件。

| 修前重现 | 修后检查 |
|---|---|
| 断言225：取消后的 STOP 仍待确认时，重配置已可受理。 | STOP 排队且前帧占 DMA 时，重配置和扫描均未受理。重复 STOP 和早于发出的 ACK 保留当前 STOP，实际发出并收到应答后恢复准入。 |
| 断言242、251：扫描请求尚未发出时收到旧 ACK、GRID 和锁定 STATUS，收拢后发布序号10的占用位集。 | 同一输入保持等待，实际出线后的 ACK、锁定 STATUS 和序号11的空 GRID 齐全后开始回升，收拢后发布空位集。 |
| 断言269、271：活动扫描的同一次 Poll 里先读到区1 HELLO、再读到区2 HELLO，仍发布地图。 | 区变更锁存错误，当次事务进入故障阶段 99。STOP 无 ACK 时，1499 ms 保持等待，1500 ms 超时后仍保留首因和结果失效状态，链路可为后续新 run 恢复准入。 |

修前原夹具执行103项，五项失败。修后同夹具执行94项，全部通过。增加前帧占 DMA、重复 STOP、早 ACK 和超时边界后，修后最终执行102项，全部通过。不同控制分支的断言数随实际路径变化。

其余覆盖包括时钟跨 tick 和 uint32 回绕、扫描 STATUS 溢出、空 GRID、下降与回升先 Release、180°收拢等待边界、PATH 实发序号与错误节点/方向、取消立即使已发布地图和路线失效。

运行 `python independent_run.py replay01`，使用 Keil ARMCC5 和当前 Python 的 Unicorn。可用 `--arm-bin` 指定编译器路径，使用 `--radar-source` 对已有旧源码快照执行当前夹具。临时构建与运行回执生成到仓库内 `工创赛/tmp/zone2_observer_independent/replay01`。档案包含 [修前结果](independent_initial_result.json) 和 [修后结果](independent_final_result.json)。

已核规则 R FIELD-02/06、RUN-01/04/07、MECH-01/02，事务身份、超时和发布顺序属 E LOC-03/04/05/06、RUN-05/06。Q FIELD-05、OBS-03、QR-04 仍待技术答疑。GRID 和 STATUS 的协议字段无法将每个已发命令之后才到达的旧帧完全绑定至运行，ACK 帧头回显语义仍待实物协议证据。STOP 超时仅证明通信有界退出，雷达物理停转、实际机构到位及整车导航运行仍需现场验证。
