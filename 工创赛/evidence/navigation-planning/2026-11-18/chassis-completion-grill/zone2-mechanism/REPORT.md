# 区二开局扫描升降

- 活动工程 `3.stm32f407工程/gongxun-f407.uvprojx`，基线 `5bbc3ee8`。普通扫描默认展开45°、下行4200半步、立即释放、等待并扫描一次、回升4200半步、立即释放、收拢180°、待收拢定时结束后请求路径。每个升降动作分别Enable；任务观察到相同序列、完成态、成功结果和精确步数后的同次服务立即Release，回升Release失败直接进故障，禁止发收拢指令。
- 自动扫描每半步仍用733 µs，单程名义3078.6 ms，ceil 3079 ms。保持现有普通扫描等待参数（展开1000 ms、下行后500 ms、收拢1000 ms），保持取消/异常保相。校准及独立8559步诊断上行保留原有收拢后释放时序。当前验证保留旧雷达观察入口，上电Ready触发一次扫描属于旧观察模式，一键比赛入口在后续导航集成验证。
- MATLAB R2020a 56场景306断言通过，退出0。前置源文件散列与准备快照一致。MATLAB为理想计数与事件模型，模拟雷达等待为示例值。证据 `pre_matlab_sources.json`、`zone2_mechanism_model.m`、`matlab_run.log`、`matlab_summary.json`、`matlab_exit.txt`。
- ARMCC/Unicorn 执行真实Observer产品状态机，71场景33051断言通过（包含旧候选计算的全范围断言）。检查两程步数、间隔和方向、收拢指令发出瞬间线圈已释放、单程仅释放一次、错序列/短步/异常/取消/拒绝释放；UART初始化体及client/lift/servo为夹具边界。实际main Radar Task函数仅限制无限循环，默认沿用保存校准6场景18断言通过，恢复显式校准调用6场景12断言通过，Observer/client为边界替身。证据 `run_arm.py`、`observer_arm_result.json`、`main_arm_result.json`。
- R核对FIELD-01/02/06、OBS-01/04、RUN-01/04/07、PWR-01、MECH-01/02。E核对LOC-03/04/05、RUN-05/06，本轮动作步数及间隔为工程软件合同。Q FIELD-05、OBS-03、QR-04待技术答疑确认。开局动作是否满足出发后展开及停滞判定、实体机构包络/行程/负载时序、雷达实际扫描和收拢到位待现场验收。
- 实施源仅 `CHASSIS/USER/main.c`（GBK/CRLF，SHA256 `04d331328e5cb335cfafc4f1cfc3d78e28b78215d0de3954612a48d5b226e4cf`）和 `CHASSIS/HARDWARE/RADAR_V6/radar_observer.c`（UTF-8/CRLF，SHA256 `066dde3da8becd07454ea2726401f7f8a7e25dc295710530442468f8e27d252e`）。未访问硬件或下载。

- Keil Vehicle_Integrated / Vehicle_Offline 均 0 错误、0 警告。6.2-sol high 独立 ARM 仿真通过，真实 Observer 59 场景 32944 次检查，真实 lift/TIM7 38 场景 21196 次检查。绕过真实外设的范围见 independent 证据。
