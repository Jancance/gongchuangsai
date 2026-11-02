## 2026-11-01 四台X静态检查通过，用户改为直接210cm，已完成一次

用户先要求X静态和单轮低速，随后明确改为直接210cm，并确认左前轮已装回紧固、同一起点对齐、相同载物、路径停车余量清空及可随时断电。未执行单轮低速：tmp/x_lf_lowrpm_20261101仅准备编译（0错5条未使用变量/函数警告），未下载/未发车，不称已通过。

静态证据：x_static_low_speed_20261101/static/hardware_raw.json，使用已修1A长度的独立只读诊断bef65b…，四台9项共36/36回复成功。均X闭环、校准/编码器ready，P_Pul均PUL_OFF，CAN500k/6B/Receive/保护Enable；PID126640/126640/15600/26未改，限流3A/带宽1000Hz一致，无过流过热堵转标志。读取后VM关闭，reset_iwdg0；行驶下载前再读静态程序仍完成且无IWDG/主控故障。没有通过复位清故障以假造通过。

新独立tmp/x_vehicle210_20261101基于旧X离线候选，只改3源：zdt_x_config.h显式APP_X_OFFLINE_ONLY0释放本次已授权运行入口；chassis_auto_test.c将X加速度参数明确为1000RPM/s（不依赖手册未明确的X加速度0语义）；main.c修正遥测字符TM2->TM6。航向P1/D0.25/死区0.3/I关闭、逐轮下发、210cm/20cm/s外层轨迹及54份源保持，Safety实现/头、故障锁存、STOP/VM/通信与姿态保护原样。旧Emm和X离线候选均不覆盖。新增p1_x_vehicle210_20261101.py仅允许mode3/default/route-trial单次210路径，原X离线主机禁动仍保留；TM6/BT6单位0.1RPM与Emm身份隔离。

验证：构建0错0警告。实际CAN/codec/保护检查401断言通过；X与历史错误处理主机11项通过。首次模型因旧fixture要求accel0失败（保留models失败报告），fixture改成严格要求本次1000，未放宽其他断言；实际C14场景103977断言通过，models_explicit_accel。模型未模拟厂家闭源内部X环，不当实车证据。初行驶候选待机仍TM2，主机拒绝发车；定位为字符数组标签漏改，旧59be18…镜像及idle_monitor保留，非电机运动指令错误。修正后新待机TM6身份/自检/无故障/VM0正确。没有通过放宽主机身份检查绕过问题。

最终下载SHA cac3b1900b8ca76426838c61ea5f6582ccbd0e4111c04ebc18fb5acfdb980903，Flash90128字节回读一致，VM默认0。首次且仅一次run_once/20261101_175748_935900：software_complete，elapsed22092ms，编码器归一反馈2100.013mm，stop_mask15、四轮0RPM、VM请求/PE2均0，trip/cause/主控CAN/RTOS/安全故障均0。261有效TM6/0无效，时间严格递增无回退，126位置帧/17批次，最大航向误差0.862度，四轮反馈峰值61/59.9/50/54.4RPM，均低于原80保护。注意PE3原轮距汇总2055.635mm未乘既有距离标定系数，不能直接当尺量误差，运动状态2100.013与尺量分开。

现场用户反馈“右偏2.5cm 走210cm”，记录实际前进210cm（未单独量化欠/超），相对原同航向Emm右2.65cm减少0.15cm；单次差异不能认定X显著改善。原诊断Emm曾右2.25cm，重复波动存在。本次结束灯灭/停稳和全过程抖动观察正在询问，不代用户确认。未重复发车。静态/编译/模型/实际遥测/尺量各自分开。当前板上已为本次X行驶候选，驱动保持X及四台脉冲口关闭，不能刷回Emm行驶程序而不切回驱动。

证据：x_vehicle210/vehicle_review.json、prepare.json、build_tm6.log、transport_checks、models_explicit_accel、vehicle_tm6/download_and_readback.json、idle_tm6、run_once。相关R FIELD-01/02/06、RUN-01、PWR-01及E LOC-02/03/05、MOT-02/04、RUN-06沿用核对，R/Q未变。源原编码CRLF保持，fetch确认HEAD=origin/main；所有成果仅本地，未提交推送。下一步先收齐是否抖动的现场观察，再决定保留X或进一步对照，不自动改航向环/内环参数。

