# 2026-11-10 抓取提速现场记录

> 最新状态：用户已要求退回原pzq，仅松爪等待改50ms。当前板载9ef9041c…，本次一轮三件软件完成，第三次松爪17.445秒、回0°18.666秒，无故障/待STOP；三槽实际放稳尚未新确认。下文候选和停点保留为历史，最新见[PZQ_RELEASE50_ROLLBACK_2026-11-11.md](PZQ_RELEASE50_ROLLBACK_2026-11-11.md)。

> 最新板载455e5ea1…：伸缩200RPM/加减速600RPM/s，升降2500RPM/45055RPM/s；下降行程一半再伸出。2026-11-11只运行一轮连续三件，半程放行实测51.68%，左件9.421秒因持续堵转而停止，距目标约0.84mm。用户确认停稳、第一件在槽、左件在盘、无碰擦，驱动屏幕Clogging Protect。STOP回复E2，掩码2仍保留；不能直接重跑。详[PICKUP_HALF_DESCENT_2026-11-10.md](PICKUP_HALF_DESCENT_2026-11-10.md)。

当前活动任务：连续三件抓放提速、伸缩末端收敛与50ms松爪转臂衔接。每次均由用户确认现场后启动一轮；底盘与第三段未启动，S形选项关闭。

## 版本与改动

- 原版第一档：升降2100 RPM、加减速45055 RPM/s；伸缩200 RPM、加减速600 RPM/s。左侧第二件两次位置检查失败。
- 制动候选：仅第一档伸缩减速度改400 RPM/s。MATLAB32项、实际C构帧/白名单/停止检查通过；两个Keil目标0错0警告。一次三件完成后，复测在左侧越位60.875虚拟度再次停止，不能称其已稳定。
- 历史有界收敛候选：仅第一档、节点6、156/160mm向外伸出、200/600/400参数允许60～90虚拟度范围内最多150ms自行收敛；超90度或持续达150ms停止。最终到位仍需45度窗口和3次零速检查，驱动故障和升降检查保留。MATLAB199项与实际C边界/原故障/STOP检查、双目标构建通过。物理效果按下表逐轮记录。
- 原pzq文件保留；历史braking/settle/single-speed工程不覆盖。当前板载工程为tmp/pickup_half_descent_20261110/project。节点6配置读取修正另在tmp/pickup_driver_fifo_fix_20261111/project，尚未部署；这不代表堵转已解决。既有handoff/pickup_speed_trials_20261110包为初始离线包，其旧AXF哈希不能替代当前板载镜像。

## 每轮结果

| 记录 | 版本 | 档位 | MCU结果 | 第三次松爪 / 回0° | 故障时间 | 实物放稳确认 |
|---|---|---|---|---|---|---|
| [200930](../evidence/integration/2026-11-10/speed-trials/integration13-three-immediate-once-20261110-200930.json) | 原第一档200/600/600 | 1 | 停止 | 未完成 | 6.967s，索引1 | 未确认三件放稳 |
| [201728](../evidence/integration/2026-11-10/speed-trials/integration13-three-immediate-once-20261110-201728.json) | 原第一档200/600/600 | 1 | 停止 | 未完成 | 7.580s，索引1 | 未确认三件放稳 |
| [203008](../evidence/integration/2026-11-10/speed-trials-braking/integration13-three-immediate-once-20261110-203008.json) | 第一档200/600/400 | 1 | 完成 | 16.896s / 18.083s | — | 未确认三件放稳 |
| [203519](../evidence/integration/2026-11-10/speed-trials-braking/integration13-three-immediate-once-20261110-203519.json) | 第一档200/600/400 | 1 | 停止 | 未完成 | 7.977s，索引1 | 未确认三件放稳 |
| [204349](../evidence/integration/2026-11-10/speed-trials-settle/integration13-three-immediate-once-20261110-204349.json) | 第一档200/600/400，末端有界收敛 | 1 | 完成 | 16.581s / 17.936s | — | 用户已确认 |
| [204708](../evidence/integration/2026-11-10/speed-trials-settle/integration13-three-immediate-once-20261110-204708.json) | 第一档200/600/400，末端有界收敛 | 2 | 停止 | 未完成 | 7.979s，索引1 | 未确认三件放稳 |
| [211343](../evidence/integration/2026-11-10/smooth-handoff/integration13-three-immediate-once-20261110-211343.json) | 分段伸缩/50ms衔接 | 2 | 停止 | 未完成 | 7.936s，索引1 | 未确认三件放稳 |
| [212343](../evidence/integration/2026-11-10/smooth-tail/integration13-three-immediate-once-20261110-212343.json) | 分段伸缩/低速末段有界收敛/50ms | 2 | 停止 | 未完成 | 6.871s，索引1 | 未确认三件放稳 |
| [213749](../evidence/integration/2026-11-10/single-speed/integration13-three-immediate-once-20261110-213749.json) | 单速/提前闭爪/50ms | 1 | 停止 | 未完成 | 7.145s，索引1 | 未确认三件放稳 |
| [222451](../evidence/integration/2026-11-10/second-diag/integration13-three-immediate-once-20261110-222451.json) | 单速第二档/FD目标与速度诊断 | 2 | 停止 | 未完成 | 6.891s，索引1 | 未确认三件放稳 |
| [014905](../evidence/integration/2026-11-10/half-descent/integration13-three-immediate-once-20261111-014905.json) | 修订第二档2500/200RPM、半程伸出 | 2 | 停止 | 未完成 | 9.421s，索引1 | 未确认三件放稳 |

索引0/1/2分别是下部/左侧/右侧。时间从三件链启动计，不含建立参考；松爪命令不是物料落稳时戳。角度为当前代码的虚拟角，毫米仅用现行315.5脉冲/mm换算；未新尺量，主机采样最大值不是连续峰值。

## 接续与边界

- 当前实际板载版本以最新成功Flash全量读回及对应candidate.json为准（455e5ea1，半程200/600/600版，173068字节）；下载授权与撤承托后启动确认分别记录。
- 新增--repeat-ready-once仅从无故障的完整三件DONE、有效参考、空爪、运输姿态和底盘空闲开始一次；不复位、不写参考有效标志，须先看到新RUNNING才能接受新的DONE。11项离线入口检查通过。断电/手移轴后重新建真实参考。
- 第二档已运行5次、第三档0次；原伸缩加减速为1200/2000 RPM/s。第二档越位199.575虚拟度（约5.62mm）后停止，不能靠第一档90度短时窗口放行。用户确认停稳、第一件在槽、左件在盘、无碰擦。
- 最新诊断：FD接受ACK02，驱动最终目标raw−55368与软件目标折算差约0.0063mm；0x35采样峰值324.9RPM，末段42.3→47.1→107RPM后越位，说明不能只改启动时机就宣称解决跟随误差。STOP后目标会改变，不用它替代原FD目标。
- 上条324.9RPM/越位为历史300RPM轮次；最新200RPM轮次目标正确、半程启动有效，但最终flags5且驱动屏幕Clogging Protect，升降已到位、伸缩未到位。未关闭堵转保护、未写PID或清故障。
- 未闭环：完整行程速度/加减速跟随、载荷/供电/驱动参数、重复性、越位消除、视觉/动态原料盘、底盘第三段。全部新增本地保存，未上传GitHub。
- 参数、收敛窗口和调试流程属于工程选择E。既有R SRC-01/02、RUN-01/02/04、PWR-01、MECH-01/02未改，正式自主和尺寸验收不由这些主机调试轮次代替；官方Q仍待技术答疑确认。
