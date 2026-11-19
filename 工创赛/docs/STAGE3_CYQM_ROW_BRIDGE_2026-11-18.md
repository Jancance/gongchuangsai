# 2026-11-18 创源启明R模式第三段观测桥

> 07:56最新：夹爪正确通道只读显示TIM4_CH4/PB9的1537µs配置、计数运行/输出启用；用户处理“松动或未供电”后明确确认抓住红料，未据此唯一归因供电或接线。原IT4回顶075450、DM180转运075521完成，无故障、未松爪/换版。当前红料在2号环上方，等待用户提供本次物料底面到纸面的新垂直间隙，再分段示教放料Z。37°本boot配置已写，默认值双构建候选未烧录。下文无动作记录为此前首故障历史，已由本段接续，不删除。

用户要求参考视觉开源代码。已直接读取提供的`D:/新建文件夹4/工创赛/代码/香橙派视觉开源代码/cyqm_vision_gongxun-main.zip`，SHA256 `2c87d78be58db113d81e8b8b5c565b575a64e722994fb37a9955bd02f5af9233`。原`code/main.py:338-355`在三环detected/stable后发送中环中心与实际连线角；`turntable_serial.py:345-374`为CC03像素/float角度帧。当前vendor/ring_station_detector.py与ZIP原件逐字节相同，SHA256 `9ac6728b5710835f76224b3cdcd8ce6d5d7c401af8d0b338e84b0da589bb6cc5`。沿用其Canny、椭圆/同心聚类、三环筛选和原三帧中值/质量判定，不复用作者旧相机点位/底盘矩阵。R三环与S叠放基准线流程分开。

## 实现和验证

仅本地overlay `output/stage3_row_bridge_20261118/`：region_desktop.py、ring_row_alignment.py及stage3-row-config.json。原第三段Pipeline保留，本轮选择CYQM R模式，输入原detector的centers_px[1]/station_yaw_deg。本车右5/前5/逆时针2°实测响应与用户确认(960,540)/摆正目标经MATLAB前置175场景通过后接入：先角度，再2×2平移逆解，适配已有O31 CRC协议与原200ms/pose/run围栏。名义A2用于原2mm/1°残差口径；calibration_verified和physical_placement_verified始终false，UI明确工程对位响应启用、落点待验，不把工程响应当实物全链路精度。

顺序采集worker跳过中间帧时，前一次已消费read完成时间仍是本帧read开始的保守下界；仅要求同generation且sequence增加，旧epoch/重复/换worker仍重置历史。实际旧程序偶尔能遇到连续帧而发invalid包，并非任何时候都完全不发。首次部署真实新图可检测并发valid O31，但同pose16的主控s3_seq=6367，Pi进程重启新序号较低，旧包拒收生效。追加MATLAB序号验证后，region_desktop从最近12个会话的有界尾部恢复同run/pose且CRC有效的实际已发送序号；不改写MCU计数或健康标志。六项本地集成测试通过（真实最终两图、三帧回调、旧epoch/失效/重复/换generation/STOP及序号恢复；测试重复文件只测软件控制流，不称实车新帧）。

证据均在`evidence/integration/2026-11-18/stage3-resume/`：row-bridge-matlab.json/log、row-sequence-matlab.json/log、row-bridge-upstream-comparison.json、row-bridge-acceptance.json；部署脚本`scripts/deploy_stage3_row_bridge_20261118.py`默认local，显式deploy要求本轮新鲜MCU与Pi证据，当前未配放料参数不会自动放料。

## 已部署和真实主控接纳

第一次07:33:40备份并替换region、新adapter、第三段config，保留原66项manifest文件和外部HSV。序号修正第二次07:38:16仅替换region，保留另68项文件。备份分别为Pi `/home/orangepi/vision_tools/stage3-row-backup-20261118-073340`、`...-073816`。原viewer正常TERM/恢复，文件与父目录fsync、os.sync、manifest核对完成；无主控复位/下载/伪造到位请求。最终主5609、相机5616、exact session `captures/run-20261118-073817-354473765`，69项manifest全匹配，原source入口/60°/20mm/HSV保持。

真实第5帧：中环(960.06671,538.87003)、row -0.49771°，发送`O31,2,16,6371,1,390,-48,0,399,-482,167,57C6`；第7帧MCU state4/pose17/align_done1，第8帧state7/config_ready0。原始记录见row-bridge-acceptance.json；SWD `readonly-status-20261118-073901-765264.json`全197368字节Flash仍匹配a5e、state7/fault0。这个到位标志来自真实三帧观测与MCU判断。本次接桥时底盘已经对准，没有额外自动离位纠偏试验，不夸大为全部动态闭环已验。

## 后续单件准备与当前阻断

用户确认车载取料高度仍约20mm。已选择center-only（红料槽2→2号环），载台120°、DM36.5°、预收160°、Z8003.4668p依次软件完成，实读Z8009.7783p、DM36.47928°、X0。用户确认爪尖在可夹位置，并先指定36.75°后更正为37°；最终RAM carrier37/pick8003.466797已写，配置文件carrier37-pick-config.json。

低Z时尝试DM36.75被原互锁拒绝g_it_rejected6，日志stage3-live-20261118-074500.json，未转动。主代理没有在获得这一失败结果后停住，却继续发了close请求；这是执行顺序错误，已向用户说明并保留首证。close原请求随后软件完成且拒绝清0（正常新命令路径，非手工清错），日志074525；软件完成不能替代夹持。用户最新明确“夹爪完全没有合拢动作”，没有夹牢或抬起证据，未继续升降/转臂/放料。

当前默认37°另有隔离固件候选`tmp/stage3_carrier37_20261118/project`，仅stage3_cycle.inc的36.5f→37.0f，2052其他源文件逐字节保留、编码/换行保持，双Keil0错0警。在线cb753e79cba1f93443f003a497d119a7f5632068bb71b633471db88eb909f256，Offline01121d1e2bef01def411c7b8ef790fa15aabced30e861681e0438af502c4f020；见`evidence/integration/2026-11-18/stage3-carrier37/candidate.json`。未烧录、未替换外部实际工程，当前板仍a5e，RAM37不等于掉电默认已持久化。

夹爪诊断必须按当前源码PB9/TIM4_CH4。最初误读历史TIM2，`gripper-no-close-readonly-20261118-075010.json`已标无效通道，不作无PWM证据；随后读取正确TIM4/PB9，结果以最新gripper-tim4-readonly记录为准。当前下一步定位PWM寄存器/引脚与实物不动之间的差异，不盲目重复夹紧、更改角度或解除低位互锁。

R检查RUN-01/03、FIELD-06、PWR-01、MECH-01/02；工程适配E CAL-02/03/05、VIS-15、RUN-06。官方Q继续待技术答疑确认。根main无提交/remote，隔离bare fetch已核21522db，非根目录已同步。仅本地与已授权Pi部署/实车调试，无GitHub提交推送。未闭环：夹爪实动作/夹牢、顶部37°复核、新地面Z/单件落点、左右和三件、自主离位纠偏、冷启持久性。
