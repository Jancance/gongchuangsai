# 独立直线PC13无反应：调度门修复

用户报告按PC13小车无反应。只读COM14约8秒：151条有效记录，另有采集结束半帧截断1条。VM2/LT1均run1/state7/cause2，Queue state2说明请求已发布；底盘session0、state0/result0、零非零速度样本、VM请求0/PE2关闭、全局CAN/RTOS/安全故障0。HF3为10641ms取消快照，源新鲜有效。日志能证明按键已进入本轮且之后进入取消出口，但不能仅据此还原用户按键次数或说PC13硬件故障。证据straight210/no-motion-01/。

源码明确阻塞：App_CanTask只在`(!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding())`时调用App_CombinedTestStep。独立测试保留VEHICLE_INTEGRATED_ENABLE=1以使用任务入口，却不启动机构，Holding始终0；因此底盘请求留在队列，根本未调度。原模型直接调用底盘dispatch，漏掉真实主任务这一条件；此前10场景通过不能扩写为已验证真实调度可启动。

本轮将原条件提为App_VehicleCombinedAllowed：保留三个机构单项模式互斥、机械臂不忙、启动轴不处于1/2/3准备状态；只有独立直线开关1且启动轴状态0时允许不依赖机构持位的底盘调度。原集成流程仍沿用原持位条件，状态5/6且无持位不放行。底盘自身准入、航向新鲜度、反馈/停止及故障撤VM保护未删，距离/速度/系数保持210cm/40cm/s/1.02182540；不新增机构动作。

新模型直接从实际main.c取该函数；旧版则提取原if表达式执行，旧版复现starts0/无轮命令并最终到截止，新增场景断言失败。新版10场景26722断言0失败，含真实门槛允许独立状态0、拒绝准备态1/2/3、拒绝故障/取消态5/6、机械臂忙和原持位可调度等检查。任务/完整CAN后端和真实门判断共同执行；其余main调度、HWT/轮响应/外设仍为模型，不是整固件板端或地面测距验证。

候选与安装后实际最终源码均重验，正式/离线目标各0错误0警告。只改CHASSIS/USER/main.c，其余1867源码/工程SHA保持。证据`evidence/integration/2026-11-04/straight-dispatch-fix/`的before/after/actual-final、manifest及构建/Flash/UART记录。独立版恢复原联调时需同时核对本轮main的最新SHA，再恢复straight210/before保存的原参数/任务/main；不能仅以较早测试版main SHA当当前文件。

下载只以Programming Done/Verify OK/Application running与flash-result.json为准；只读LT1核对2100000/400/1021825、VM2 run0待机后，用户再单按PC13。助手不发串口或RAM运动命令。第一次真实位移与误差仍待测，不能将此次软件修复当直线测距或补偿已闭环。

最终下载已Programming Done/Verify OK/Application running，烧录后只读136条、坏帧0，VM2共8条确认run0/state0/cause0；LT1共8条确认210cm/400mm/s。镜像SHA与原始日志在flash-result.json、flash-attempt1.log及after-flash-uart/。未触发本轮运动，当前需用户在直线净空、同一地面测量基准下单按复测。原无反应记录与模型覆盖缺口保留，不把软件/Flash通过当新实车启动成功。

已核R RUN-01/02/04、FIELD-01/02/06、PWR-01、MECH-01/02；E CAL-02、LOC-02/05/07、MOT-02/04、RUN-06。本次调度互锁修复为E，R不变，FIELD-05/OBS-03/QR-04等Q仍**待技术答疑确认**。未闭环：本版板端启动及地面210cm/横偏/重复性、补偿适用性与原联调恢复、旧偶发撤VM。
