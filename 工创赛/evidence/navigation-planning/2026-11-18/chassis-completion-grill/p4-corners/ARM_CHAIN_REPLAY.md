# 区 2 整链 Cortex-M4F 回放

用 Keil ARMCC 和 Unicorn 执行 `run_zone2_chain.py`。默认跑完整八站，`--shadow` 验证零速入口，`--one-miss` 和 `--one-status-miss` 分别在运动中对 3 号电机丢弃一次 `0x43` 及 `0x3A`，`--drop-radar` 模拟开跑后持续未回复 HELLO，`--drop-f6` 模拟一批 F6 的第三轮已发送但 ACK 缺失。各模式独立运行，程序输出每轮隔离目录，内含 ARMCC 日志、AXF、回放收据及输入源 SHA256。

回放执行实际 `AppChassis_Zone2CanStep` 函数体、区 2 应用、CAN owner、里程计发布与积分、路径及运动参考。雷达客户端及观察器解析现有二进制协议编码的握手、扫描图和 PATH 回复。默认使用空占用图及六条事先指定的有向 PATH。CAN 在事务函数边界替换，按轮速积分位置脉冲。HWT 在 `App_HeadingSourceRead` 发布边界模拟独立角度及角速度，时戳按 10 ms 刷新。雷达 UART、机构、VM、调度、车轮响应均为夹具模型。

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

启动时序夹具从主工程取得雷达配置和 2 ms 任务周期，按 4200 半步 × 733 μs 向上取整模拟每趟 3079 ms，两趟实际由 2 ms 观察器节拍确认。按键前完成模拟握手，扫描 ACK 在下一次服务处理，`--startup-timing` 参数为传出扫描请求到网格和锁定状态入接收队列之间的等待。CAN 按 10 ms 运行，同一时刻先执行 CAN。实际中断、任务占用及串口传输时长待实测。

| 历史 3.20 秒转向及 180 秒中止收据 | 地图就绪 | 首段开始跟踪 | 截止结果 |
| --- | ---: | ---: | --- |
| [网格结果立即返回](zone2_chain_startup_0ms.json) | 8.664 s | 10.160 s | 180.002 s 执行超时分支，完成 7 站，189415 项回放检查通过。 |
| [网格结果延迟 1000 ms](zone2_chain_startup_1000ms.json) | 9.664 s | 11.200 s | 180.002 s 执行超时分支，完成 7 站，189129 项回放检查通过。 |

零结果等待时，前 7 站均比简化夹具迟 8.640 s。将这段差值加到简化夹具的 175.990 s，推算需 184.630 s。第八段从 156.160 s 开始，单参考轨迹需要 26.240 s。

### 2.08 秒转向候选

先用 MATLAB R2020a 验证正常转向轮端限值和扫掠，再将八次 90° 原地转向的参考时长由 3.20 秒改为 2.08 秒，模型共节省 8.96 秒。候选模型轮端峰值界为 80.32 RPM、140.67 RPM/s、1295.40 RPM/s²，航向速度余量为 5.32°/s。轮端模型所用反馈修正和时序假设见本机 [MATLAB 脚本](validate_zone2_turn_timing.m) 和 [回执](zone2_turn_timing.json)。故障识别后再延迟 100 ms 的条件模型下界为 +6.24 mm，200 ms 为 −11.57 mm，停车拖尾仍待测。

历史同台扫描 14 帧的观测间隔约 2.313 秒，按相邻采样点估计的上界约 2.453 秒。以下为仍有旧 180 秒中止条件时的回放，用时取夹具记录最终检查的 10 ms 时间格。

| 回放输入 | 结果 |
| --- | --- |
| [扫描等待 2453 ms](zone2_chain_turn2080_scan2453_with_cutoff.json) | 178.870 秒，八站、六次 PATH、两次复用及 32 条 STOP，187339 项断言。 |
| [同样等待并单次 0x43 漏包](zone2_chain_turn2080_scan2453_0x43_with_cutoff.json) | 179.830 秒，八站，188369 项断言。 |
| [同样等待并单次 0x3A 漏包](zone2_chain_turn2080_scan2453_0x3a_with_cutoff.json) | 178.870 秒，八站，187339 项断言。 |
| [扫描等待 4500 ms](zone2_chain_turn2080_scan4500_with_cutoff.json) | 180.000 秒进入旧中止分支，已完成七站。 |

Keil Integrated 和 Offline 构建均为 0 错误、0 警告。独审用 2313 ms 加单次 0x3A 漏包复跑八站完成，确认标称最小净距约 8.239 mm，姿态扰动下界约 −0.079 mm。单次漏包在首段注入，专项证据限于该时机。 [构建回执](zone2_turn2080_keil_with_cutoff.json)

### 取消整轮 180 秒自动中止

取消 `AppZone2_CanPermit` 和 `AppZone2_Step` 中按整轮时间拒绝轮速和中止导航的判断。比赛计分时间仍按 RUN-07 比较。启动和准备等待、分段停稳判断、CAN 和定位健康、同场图时效、正常回区停车各走原有流程。对应 MATLAB R2020a 前置模型通过 410 项断言，包含跨 180 秒、局部超时及健康条件。 [模型](validate_zone2_no_global_deadline.m) · [回执](zone2_no_global_deadline_matlab.json)

| 当前末版回放 | 结果 |
| --- | --- |
| [扫描等待 6000 ms](zone2_chain_no_cutoff_scan6000.json) | 182.380 秒完成，180 秒后共 44 条非零轮速，190158 项断言。 |
| [Sol high 独审，扫描等待 7003 ms](zone2_chain_no_cutoff_scan7003_review.json) | 183.420 秒完成，180 秒后共 96 条非零轮速，190990 项断言。 |
| [等待 2453 ms 并单次 0x43 漏包](zone2_chain_no_cutoff_scan2453_0x43.json) | 179.820 秒完成，188369 项断言。 |
| [等待 2453 ms 并单次 0x3A 漏包](zone2_chain_no_cutoff_scan2453_0x3a.json) | 178.860 秒完成，187339 项断言。 |
| [持续雷达失联](zone2_chain_no_cutoff_stale_radar.json) | 原故障路径发出四轮 STOP，3309 项断言。 |
| [运动中部分 F6 ACK](zone2_chain_no_cutoff_partial_f6.json) | 原故障路径发出四轮 STOP，926 项断言。 |
| [影子入口](zone2_chain_no_cutoff_shadow.json) | 无非零轮速下发，1497 项断言。 |

完成时间取回放记录的状态转换时刻。完成用例均验证八站、六次 PATH、两次复用、32 条 STOP 和最终回区，故障码为 0。独审检查了两个总时钟判断的删除范围；对应旧版 7003 ms 等待的回放在 180.002 秒中止。该独审候选计入位姿误差后的净距下界仍为 −0.079 mm，实际尾程没有测量。Integrated 与 Offline 重新构建均为 0 错误、0 警告。 [构建回执](zone2_no_global_deadline_keil.json)

核对 R FIELD-01/02/06、OBS-01/04、RUN-01/04/07、PWR-01、MECH-01/02。模型覆盖 E LOC-03/04/05 的软件路径与接收策略，FIELD-05 和 OBS-03 待技术答疑确认。舵机与升降释放、真实 VM、电机跟踪、HWT 硬件、占用图质量、RTOS 时序及地面全程验收均待实车。实测净空、赛道真实车宽/附件和有障碍路线仍开放。
## 占用格 5 的绕行与拒绝

同一轮模拟雷达扫描发布 5 号占用格（X 1920–2400 mm、Y 480–960 mm）。第 2 段请求 2→14。旧回复 `[4,14]` 穿过占用格，绕行回复 `[12,14]` 展开为 2→7→12→13→14。MATLAB R2020a 验证两个回复的占用判定、车体扫掠和八段模型。另验证格 7 占用时从外圈回区的备选。 [MATLAB 5 号格回执](zone2_occupied5_matlab.json) [MATLAB 7 号格回执](zone2_occupied7_matlab.json)

| 回放 | 结果 |
| --- | --- |
| [绕行](zone2_chain_occupied5_detour.json) | 111504 项断言，167.980 秒完成八站，按简化开局计时。 |
| [绕行独审，包含开局时序](zone2_chain_occupied5_detour_timed_review.json) | 187599 项断言，扫描等待 2453 ms，179.100 秒完成八站。实际仿真轨迹经过约 (1199.99, 899.91) mm，离 5 号占用格最小 48.705 mm。 |
| [同图回传旧路径](zone2_chain_occupied5_bad_path.json) | 12185 项断言，首站停稳后拒绝第 2 段。从首站停稳到回放结束没有新非零轮速，VM 请求已撤销。 |

本次沿用 `421f0ad7` 固件及其双目标构建，空图八站和零速影子模式回归通过。实物雷达识别和现场返回的路线待验，整个车道的扰动净距下界仍为 −0.085 mm。核对 R FIELD-01/02/06、OBS-01/04、RUN-01/04/07，E LOC-03/05/06、MOT-03、OBS-07。Q FIELD-05、OBS-03 保持待技术答疑。
