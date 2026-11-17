# 连续地图坐标接入软件报告

日期：2026-11-17。基线97a603c5。仅本地软件实现与离线验证；本阶段没有fetch/push、下载、reset、发车或硬件访问。动作融合尚未开始。

## 实现范围

七个产品源文件：app_config.h、app_odometry.h/c、app_can_diagnostics.h/c、stage1_runtime.inc、vehicle_mission.inc。默认APP_CONTINUOUS_MAP_ENABLE=0，候选构建显式1；功能关闭保留旧路径。

地图使用独立静态ChassisOdometry context，唯一CAN owner更新；TaskA只提交真实新run并读取完整快照。跨PLAN、后端session、ResetRecords、local SetPose和nav scope保持独立。起点1/2参考为(2250,150)/(150,150)mm、朝向90°；初始点来自已知摆车参考，不是传感器绝对定位。

源固定RAW_WHEEL_HWT、nav_applied=0、calibration_scope=0。RAW仍包含既有基础轮径/前向尺度换算；四向经验nav残差继续只在原local/debug scope应用。缓存来源为真实后端位置及既有diagnostic success cache，按真实源时戳择新；新增getter只复制cache，不增加CAN交易或改变VM/STOP策略。

新轮先拒绝开局前cache，inactive时真正based且valid才可发布首段。集成stage1 state1和独立stage1 state7只等一次；中途stage测试以start0清除旧地图valid、不等待地图ready。mask15但异步历史不能插值时仍等待。

失效后保留最后发布坐标和sample_ms，同run不自动恢复。getter检测过期通过run绑定邮箱通知CAN owner锁存；已复现并修复getter过期后fresh恢复，以及CAN中段被getter抢占导致坐标跨gap推进两项问题。真正新run清邮箱，旧snapshot不会使新run失效。

新增准备等待的边界准确为：独立stage1使用既有60000ms数值，集成state1受既有120000ms阶段限值（rehearsal开启时另有180000ms整轮限值）。原开始运动/Load时重设运动阶段时钟保持，因此不宣称含准备的总预算完全不变，也不把这些工程超时等同比赛规则允许停滞时间。

## 验证结果

- 本机MATLAB R2020a在产品编码前实际执行，13:52:54开始、13:53:00完成，146检查通过。证据matlab_receipt.json、matlab_summary.json及verify_map_lifecycle.m。它是合成生命周期模型，不是实际C或实车证据。
- actual ARM C使用ARMCC编译、Unicorn执行真实app_odometry.c、chassis_odometry.c、chassis_kinematics.c，map_on/map_on_nav各106断言、map_off3断言通过。RTOS时钟/临界区、源cache/HWT与供电提供者为桩；AppDebugPlan40Allowed从真实源码提取。覆盖双起点、L形真实编码、异步共同源基准、cache择新/重复/冲突/未来/非法、plan/local重置隔离、旧run与回绕、真实stale/GAP/已知供电epoch和故障、getter及中段抢占的sticky与lastpoint。结果与最终app_odometry SHA一致。它不证明RTOS全调度、真实CAN时延或实车精度。
- Keil统一工程派生四个离线候选：integrated_on/off、standalone_on、baseline_on，全为0错0警告，仅-r从未-f。集成on/off除地图开关外配置相同。baseline继承此前control-distance-ratio冻结定义；独立入口为integrated0/stage1=1。
- 明确给四配置设置ICM45686_SAMPLE_ONLY=0，使真实主入口参与链接。首次按默认采样模式构建虽0错0警告但主入口被裁掉，已留build_attempt1_sample_only_defaults，不用于集成通过结论。
- frozen/保存最终候选工程、AXF、map、编译器callgraph、link命令、source_sha256及source_options_reference.uvoptx。options是原本机选项的参考副本，构建定义以冻结候选工程/receipt为准。
- 原六处dirty的SHA保持；七源既有非ASCII字节顺序与CRLF保持；git diff --check以cr-at-eol兼容既有换行通过。final_checks.json、implementation_sha256.json为精确哈希与结果。

| 最终配置 | Code | RO | RW | ZI | AXF SHA256 |
|---|---:|---:|---:|---:|---|
| integrated_on |143000|6860|3612|123876|4bb2a46561c2cc864569834e79e31ea9ebc33a803824f4426e852345d87ac6be|
| integrated_off |141276|6860|3592|122192|a1ae9aab605c94cf809135ecd0e2d3b3fb65280ba4e4296399047f68dd8ff2f5|
| standalone_on |142048|6816|3524|123668|9eec7017df9fbedaf2c5da8aa048099cbf7054cacdb881d58f760fbbb03fa816|
| baseline_on |114208|6656|3088|191528|d4c2da482236c7eb2e9b3f708fe1f9d654bdf7fa7b83ab445842a4156f72cb14|

集成on/off静态RAM差1704B（RW+20、ZI+1684）；map context本体1456B为静态，不分配在CAN任务栈。编译器integrated_on callgraph：MapStep自身栈136B、链260B，AppOdometry_Step链396B，AppMotion_StepOdometry链404B，ReadMap自身16B/链32B。CANtask分配保持384 words=1536B；其编译器链报告920B+Unknown。编译器结果含unknown/cycles等限制，不能代替中断嵌套与实际high-watermark验收；未更改任务栈分配。

首次缺include的编译失败、read-stale复现失败、抢占lastpoint复现失败分别保留attempt1_compile_failure.log、attempt2_read_stale_failure.json、attempt3_interleave_failure.json。nav配置首次缺真实准入函数依赖的link诊断保留test_dependency_failure_summary.json，最终真实函数已补入测试，不增加产品桩。

## 规则、限制与交付停止点

R已核FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02；E涉及LOC-02/03/05/06/07、FIELD-08、MOT-02/04、CAL-02。Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。地图估计不等于车道包络、随机障碍、载台流程或比赛时间验收，不把E写成R。

未闭环：实际摆车误差、首基准建立时间、机构保持期间真实源最大间隔、轮/HWT时延、地面滑移与累计漂移、双起点整场、现场故障/供电、栈high-watermark。功能未部署、未签署实车通过。

本阶段完成坐标生命周期接入、实际C与整机软件证据后停止；根交接与本地提交由主执行者收尾。下一步先与用户讨论动作融合方案，不继续编写融合或改变速度/运动控制。
