# 区 2 整链 Cortex-M4F 回放

用 Keil ARMCC 和 Unicorn 执行 `run_zone2_chain.py`。默认跑完整八站，`--shadow` 验证零速入口，`--one-miss` 和 `--one-status-miss` 分别在运动中对 3 号电机丢弃一次 `0x43` 及 `0x3A`，`--drop-radar` 模拟开跑后持续未回复 HELLO，`--drop-f6` 模拟一批 F6 的第三轮已发送但 ACK 缺失。各模式独立运行，程序输出每轮隔离目录，内含 ARMCC 日志、AXF、回放收据及输入源 SHA256。

回放执行实际 `AppChassis_Zone2CanStep` 函数体、区 2 应用、CAN owner、里程计发布与积分、路径及运动参考。雷达客户端及观察器解析现有二进制协议编码的握手、扫描图和 PATH 回复，使用空占用图及六条事先指定的有向 PATH。CAN 在事务函数边界替换，按轮速积分位置脉冲。HWT 在 `App_HeadingSourceRead` 发布边界模拟独立角度及角速度，时戳按 10 ms 刷新。雷达 UART、机构、VM、调度、车轮响应均为夹具模型。

独立车体积分在每段固定轮速期间用双精度计算平面刚体位姿，使用现有前向距离比例，输入只有实际下发的带符号轮速及持续时间。按车体朝向，用分离轴检查四个黄色区域、用实体外廓检查场地四壁，并在每个保持区间使用从区间中点出发的平移与转动最大位移上界。额外 5 mm 与 1° 不确定度采用统一平移上界检查，负下界表示该方法未证明净空。空图仅覆盖该固定场景，给定时间及车体净空为模型量。

| 收据 | 回放结果 |
| --- | --- |
| [190 mm/s 基线](zone2_chain_190_deadline.json) | 180.010 秒时在末段，已完成 7 站、6 次 PATH、2 次复用。 |
| [250 mm/s 首次回放](zone2_chain_250_micro_edge.json) | 第 4 段起点到格心约 0.00590 mm，触发参考规划最小 0.01 mm 边长拒绝。 |
| [短边修复后首次整链](zone2_chain_first_full.json) | 175.990 秒，8 站、6 次 PATH、2 次复用、32 条 STOP。 |
| [首次独立扫掠](zone2_chain_first_sweep.json) | 32307 个轮速保持区间，名义模型最小净空 8.2326 mm。统一扰动上界下最小 −0.08588 mm。 |
| [0x43 单次丢包修复前](zone2_chain_one_miss_before.json) | 1630 ms 注入一次丢包，1690 ms 提前停车。 |
| [0x43 单次丢包修复后](zone2_chain_first_one_wheel_miss_pass.json) | 同样在 1630 ms 丢包一次，完成 8 站，174.950 秒。 |
| [单次 0x3A 丢包](zone2_chain_first_one_status_miss.json) | 1590 ms 丢包一次，完成 8 站，175.990 秒。 |
| [持续雷达失联](zone2_chain_first_stale_radar.json) | 5100 ms 判地图失效，40 ms 内四轮 STOP 并撤销 VM 请求。 |
| [运动中 F6 部分 ACK](zone2_chain_first_partial_f6.json) | 可能已执行、发送完成和 ACK 掩码分别为 0x7、0x7、0x3。四轮 STOP，无虚假到站。 |

端点复核和 0x43 有界补读合入后，以下同版回放均通过。终点小偏差恢复的专项注入在 MATLAB 验证中记录。

| 最终收据 | 断言 | 结果 |
| --- | ---: | --- |
| [完整八站](zone2_chain_final_active.json) | 116906 | 175.990 秒，8 站、6 次 PATH、2 次复用、32 条 STOP，地图返航坐标 149.957、149.834 mm，航向 90.0005°。 |
| [单次 0x43 丢包](zone2_chain_final_one_wheel_miss.json) | 116202 | 174.950 秒，八站全部完成。 |
| [单次 0x3A 丢包](zone2_chain_final_one_status_miss.json) | 116906 | 175.990 秒，八站全部完成。 |
| [持续雷达失联](zone2_chain_final_stale_radar.json) | 3309 | 发出四轮 STOP，撤销 VM 请求，无后续轮速下发。 |
| [运动中 F6 部分 ACK](zone2_chain_final_partial_f6.json) | 926 | 发出四轮 STOP，撤销 VM 请求，无虚假到站。 |
| [影子入口](zone2_chain_final_shadow.json) | 1497 | 1 次 PATH，4 条 STOP，零非零轮速下发。 |

核对 R FIELD-01/02/06、OBS-01/04、RUN-01/04/07、PWR-01、MECH-01/02。模型覆盖 E LOC-03/04/05 的软件路径与接收策略，FIELD-05 和 OBS-03 待技术答疑确认。舵机与升降释放、真实 VM、电机跟踪、HWT 硬件、占用图质量、RTOS 时序及地面全程验收均待实车。实测净空、赛道真实车宽/附件和有障碍路线仍开放。
