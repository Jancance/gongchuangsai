# 上电双鸣等待软件初始化与香橙派串口握手

用户2026-11-17明确要求：最开始上电两声应在香橙派与主控串口真实通信成功、软件初始化就绪后再响。此处初始化不包含按PC13后才做的机构回零、机械参考标定或实际移动。本改动只控制启动提示，不新增PC13门禁、停车、故障或保护，不改变纠偏完成时原80ms/80ms双鸣。

本地独立候选 `tmp/boot_ready_beep_20261117/project` 基于镜像30994工程，原基线全部2051源码保持；编辑前bare fetch成功，FETCH_HEAD仍21522db，根仓库无提交/remote，不强行pull。主代理已于22:50:48完整读回30994的194292字节并记录state0/fault0，见`evidence/integration/2026-11-17/stage2-extension-mirror/flash-readback-20261117-225048.json`。该既有下载事实不代表本新提示版已下载或机械动作通过。

## 行为与范围

固件仅原main.c和integration10_complete.inc两处局部接入，加boot_ready_link.h/inc两个文件；不改工程目标/全局参数。

- main所有原Init及6个xTaskCreate成功后、scheduler前置main完成位；TaskA执行原HeadingDryRun、条件CRC/RuntimeRoute初始化、条件平移自检后置app完成位。原UART诊断task不再立即响，明确记录它未运行的自检为skip，初始化段尾置自己的完成位。
- 提示所需mask为127：main完成、app实际自检成功、VM前CAN内部回环通过、安全任务ready且无既有故障、watchdog初始化成功、screen初始化成功、在线已实际调用的RadarServo与StartupPose初始化结果正确。carrier配置-1表示原设计不输出位置脉冲，result0合法；不为了变1强转载盘。机械valid0、轴未回零、未进行雷达测距/HWT物理姿态验收不阻塞提示。Offline明确SKIPPED，不伪造外设成功，不启RNG/UART提示输出或蜂鸣。
- USART6仍由原Vision_Poll唯一任务处理，使用独立BH1挑战和BE1回显，CRC16-CCITT。当前boot的64位nonce由F407现有硬件RNG非阻塞取两个32位word；未ready或RNG当前错误时仅不发挑战/不响，不构造假nonce或新故障。成功发送的序号才生效，每500ms可重发，收到CRC正确、nonce及当前已发序号完全匹配的BE1才锁存本次双向握手完成。随便收到字节、只打开串口、旧boot/旧序号回包不能放行。
- TaskA只在软件ready、本boot握手完成、蜂鸣器空闲且无正在/待启动业务时，以短临界区提交原上电两声，一次boot只提交一次。用户提前按PC13不被拦截，启动提示延后，不能覆盖原纠偏完成双鸣。原SourceCycle state5和全部source_mcu_cycle.inc、PC13派发、原蜂鸣器驱动及镜像运动源码逐字节不变。
- 握手是启动时一次链路闭环，不宣称持续连接健康、曝光时间、算法或机械验收。未增加掉线停车或新的运行保护。

Pi仅必要两代码增量：`output/stage23_boot_ready_overlay_20261117/`中的source_fast_cycle_view_20261115.py及新boot_ready_handshake.py，另有manifest-delta.json说明。唯一串口owner在原逐行接收最前处理BH1并BE1回显，preview也回显握手；原OC观察发送仍严格受preview_only禁止。CONFIG明确boot_handshake_tx_enabled=true与observation_tx_enabled=false，窗口显示“预览：BE握手可发，OC禁发”，不把“串口一切TX均禁止”与握手混淆。

此目录是增量而非完整部署包；不得用原66项完整manifest或本地旧body-profile覆盖现场已部署新J。最小部署脚本`scripts/deploy_boot_ready_pi_20261117.py`默认local-only，只有--deploy才连接；核新IP helper、当前3417preview唯一owner/cwd/ttyUSB0后备份代码、原manifest及进程信息，TERM并确认设备释放，再原子替换两代码及现有manifest对应两keys、os.sync，原start.sh恢复preview。固定实际HSV、settle JSON、实际argv配置/profile在前后均核SHA；现场3aee新J、阈值、reference、HSV、300ms/5帧/2px、20mm/5mm及autostart trial不改。主代理独占执行部署/下载，本编码代理未连接任何设备。

## 软件验证与镜像

证据目录：`evidence/integration/2026-11-17/boot-ready-beep/`。

- 本机MATLAB实际1073检查于23:03:46通过后才写实现，matlab-result.json和matlab.log保留前置事实，覆盖初始化门槛、nonce/序号、RNG、忙TX、回绕与单次/业务双鸣共存模型。
- 真C直接编译实际boot_ready_link.inc执行109检查：CRC、旧nonce/未发序号、RNG错误/未ready、忙TX、拆包/文本二进制分派、嵌入CR拒绝、每项初始化门槛、单次提示与原业务双鸣共存；Offline目标明确skip且无RNG/UART/蜂鸣输出。见actual-boot-ready.c及两个实际运行log。
- Pi实际echo模块和从真实主入口抽取的接收dispatcher执行10检查：正确CRC回包、拒绝非法/溢出、短写报错、先处理握手而不进入业务分派、preview独立回显。不是仅测试另写的协议模型。
- 双Keil重构建均0错误0警告：在线AXF `fb2bc55bbdc32846fef06a40c31e969361b491ec274323c1755d45dd6290022d`，Flash段195828字节；Offline `aa1af4627a17a0d649d011d80818e11b3c93653325750b178f235a78a0c2d401`。日志为Vehicle_LiftRuler-build.log及Vehicle_LiftRuler_Offline-build.log。
- source-manifest.json记录2053源码；candidate.json记录4固件delta、2Pi代码与双AXF/Flash段摘要，并保留extend_sign/home_direction=1兼容镜像host校验。encoding-line-endings.json证实两原固件文件非ASCII字节完全不变、原CRLF/混合裸LF保持，没有批量转码。Pi脚本按UTF8/原CRLF保存。

R检查RUN-01、PWR-01、MECH-01/02，正式规则不变；E检查RUN-06，提示与握手属于工程实现；官方Q继续“待技术答疑确认”。本地编译和模型不代替真实串口及听到两声：尚待主代理核新镜像全Flash、g_boot_*初始化mask/nonce/tx/rx/单次提示记录及用户实际听感，并确认preview无OC。镜像方向/原点/65mm往返、首抓/三件及旧stall原因仍各自待验。编码代理未probe/SSH/下载/复位/发动作，未提交推送GitHub；部署结果以后续主代理证据为准。
