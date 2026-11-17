# 连续地图位姿独立审查（最终）

日期：2026-11-17。基线 `97a603c5476906d2f1fbe956a9418de5877af9e1`，分支 `codex/chassis-navigation-plan`。本审查者只读产品、Git及测试证据，仅写本文；未下载、reset、发车、读取硬件或执行Git写操作。

## 结论

**PASS_WITH_LIMITATIONS：本阶段地图观测接线可保存为本地软件检查点，未发现剩余阻塞缺陷；未部署、未实车验收。**

已复核实际七源diff、唯一CAN owner、TaskA真新轮入口、最终ARM C夹具、15:36完整运行时构建及冻结hash。地图是独立RAW_WHEEL_HWT估计，默认 `APP_CONTINUOUS_MAP_ENABLE=0`，仅隔离候选设1；不是坐标闭环到点、动作融合或新增串口命令。按用户本阶段范围，保存检查点后停止，动作融合留待讨论。

## 实现核查

- `AppOdometry_MapStep` 在现有Step中由唯一CAN task更新静态context；diagnostic getter只复制既有成功cache，不新建查询、任务或轮询，不改安全mask。backend毫秒不再换算，diagnostic tick只乘一次；共同 `pose.sample_ms` 保留源时刻，不以发布now冒充新数据。
- 两起点为 `(2250,150)/(150,150)` mm、heading90°，本轮首个HWT设固定offset；复用真实四轮异时插值、中点+sinc、65536/rev与原前距基础尺度。RAW_ONLY表示无新nav经验overlay，`calibration_scope=nav_applied=0`，不等于完全未标定编码值。
- 真新轮邮箱run/start/begin原子提交；半区序比较拒绝同/旧run。pending新轮getter立即返回新run invalid，owner下一轮才重建；start0为NO_START。PLAN/session/local SetPose/ResetRecords不重建地图。
- 本轮begin之后且非未来的真实源，必须在底盘inactive时建立真正可插值的based+valid；mask15不等于ready。整车state1与独立stage1 state7只在首段前等匹配run，stage2等中途单测start0不被阻塞，宏关保留原路径。
- 新准备等待各自有界：独立入口用原60s MOVE_TIMEOUT数值限制准备，集成入口由原state1阶段/排练预算限制。既有起步/Load重设运动计时保持，不能声称准备+运动共用一个未重置总截止，也不代表官方等待/整场用时通过。
- 择新、重复、同戳异值、旧包、未来/非法源与uint32回绕有明确处理。源龄/拒绝、未立基准先active、已based后已知软件断电/enable_count改变或关键fault会sticky GAP；同run fresh源、重复BEGIN与local重置不能恢复。active/ODR/epoch不是物理VM电压实测。

## 独审推动闭环的两项缺陷

1. 原getter只将返回副本变GAP，owner遇fresh源仍可在同run重新valid。实际C失败留 `attempt2_read_stale_failure.json`；最终以run绑定 `s_map_gap_run` 邮箱锁存，getter与owner发布均在临界区复核，同号请求不清锁，新run才清。
2. owner抓邮箱之后，getter抢占锁GAP，原发布虽invalid仍可能推进XY/sample_ms跨过gap。实际C失败留 `attempt3_interleave_failure.json`；最终publish恢复同run上份published.pose后再Gap。中段 `Power_ReadCommandStatus` 桩注入抢占、随后真实积分的用例确认sample100/y150保持。wheel_ms/angle_ms等诊断可已更新，最后有效位姿时刻以pose.sample_ms/valid为准。

新run若在旧owner步骤中提交，旧步骤最多发布旧run，getter识别pending后返回新run invalid；下轮重建。gap邮箱绑定run，旧锁不误伤新轮。未声称穷举全部RTOS调度。

## 验证与实际范围

- MATLAB：最终回执为本机R2020a、13:52:54–13:53:00、exit0，脚本SHA `787526d732f0563c60d190812d9335ad31282f35da6a88533ac0e59788faf26f`；最终146检查。仅产品前置合成生命周期模型，未执行实际C，不能证明实车精度/总线时延。
- 实际C：阅读 `check_map_c.py` 与 `actual-c/unit.c`；编译真实app_odometry/chassis_odometry/chassis_kinematics。RTOS tick/critical、heading/cache/power及运行上下文为桩。最终map_on106、map_on_nav106、map_off3均PASS，记录的5源SHA与当前一致。覆盖两起点、L形、异步首基准、cache择新/异常时间、源龄、gap、pending run、power epoch、local reset与getter抢占。真实入口准备分支是源码+完整构建核查，未在该夹具运行完整TaskA/CAN/机构任务。
- 审查者另行只读装载三个ARM AXF，按ELF alloc段含ZI清零，在Unicorn中独立重新执行，均到stop且failed_line0，计数106/106/3；不只采信实现者PASS文本。AXF SHA依次：`77bb6bc5d5823b76ca094d33bc18dd11d1df90f796ba59e35a39e4b99f9b5edf`、`d44883e11162391d3d2aca3eba4c1efc29cb82eb8ebf1892004bf0feb6258ec5`、`ccca7f5b8163073fe180622412cd7d8bdb1d1d2c5d41d1a7c6f40ab1a72eba65`。
- manual范围：L形主用例的request置1后立即0未执行；真正成功manual SetPose发生在poweroff令map GAP分支，证实不清gap。有效map状态下manual成功组合未动态执行；context无耦合由diff静态确认，不冒充动态覆盖。
- Keil：15:36四配置integrated_on/off、standalone_on、baseline_on都显式 `ICM45686_SAMPLE_ONLY=0`，日志0错0警；独核全部冻结hash。集成on真实链接VehicleMission+MapStep+ReadMap，独立on链接Stage1Service+MapStep+ReadMap，集成off去除map context/调用。baseline_on仅兼容构建，未有真新轮入口，不能当自动启动地图。早先15:34 sample-only裁剪构建留 `build_attempt1_sample_only_defaults/`，不作为完整产品通过证据。
- 最终集成on AXF：`4bb2a46561c2cc864569834e79e31ea9ebc33a803824f4426e852345d87ac6be`；off：`a1ae9aab605c94cf809135ecd0e2d3b3fb65280ba4e4296399047f68dd8ff2f5`，其余见build_receipts。
- 内存/栈：集成on/off总RW+ZI差1704B，1456B context及两个各100B快照均static；map静态调用链约404B，getter frame16B。总调用图有Unknown Stack Size，不据此签运行余栈或栈安全。
- 七源非ASCII字节序列保持，全部CRLF无裸LF；原六dirty的SHA与preparation逐一相同。七源 `git -c core.whitespace=cr-at-eol diff --check` 通过；保留CRLF时默认diff-check会误报CR尾空白。原nav/capture/控制/VM/STOP实现未被本diff改写。

最终app_odometry.c SHA `d78a8522edbade0b0814bef86de9c6900dc029622b3fcd3f55934fba2575420c`；七源清单见implementation_sha256.json。

## 规则与未闭环

已核R：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02；E：LOC-02/03/05/06/07、FIELD-08、MOT-02/04、RUN-06、CAL-02。Q：FIELD-05/OBS-03/QR-04继续待技术答疑确认。地图观测不替代车道包络、障碍、同区返回、一键自主、抓放/载台及整场用时验收。

未闭环：真实首基准时间、机构保持/实际CAN最大源间隔、轮/HWT延迟、运行余栈、摆车误差/滑移/累计漂移、双起点路线与故障实车、地图消费端动作融合。本轮未下载、复位、读硬件或发车；不声称当前板端镜像/物理供电已复核。保存软件检查点后停止，不自动恢复现场或推进融合。
