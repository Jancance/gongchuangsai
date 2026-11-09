# 扫码后撤VM：第1→2段准入交接修复

用户报告“扫到码就断vm的电了”，随后要求继续。只读本轮锁存257条、坏帧0、发送命令0，证据`evidence/integration/2026-11-04/stage12-chain/scan-vm-fault-01/`。

同版run1第1段state6/cause7、联合取消state6/cause2；EX3首故障0、保持使能1次/3样本/hold1，未发现伸缩失能首因。TMQ显示已生成两条动作、后端回IDLE/result0、STOP mask15、四轮命令0、VM请求0/PE2关闭，CAN/RTOS/安全故障0。代码中第2段Queue将后端成功重置后调用App_SerialIdleReady，单次不满足立即返回失败并主动撤VM；这与本轮锁存相符。

HF3取消快照：source_valid0、angle_age99ms/rate_age100ms、sample_age94ms、UART错误0，恰到原100ms严格小于准入边界。但旧版未记录Queue入口具体拒绝位，不能把稍后取消快照当成该入口同一时刻测量，故本轮物理首因仍未完全闭环。

## 修复（E）

原App_SerialIdleReady条件改为相同条件的App_SerialIdleBlocked位掩码，Ready仍等于mask0，原100ms航向新鲜度、角速度±3deg/s、所有权、停止/健康等门槛保持。原位掩码如下：

| 位值 | 原检查条件未通过 |
|---|---|
| 1 | Emm菜单确认 |
| 2 | 组合状态/请求或动作表有效位 |
| 4 | 前进保持/VM请求/供电持位条件 |
| 8 | 后端active/result/cut_vm |
| 16 | 平移/串口协议自检 |
| 32 | 航向源健康、新鲜度或校准/版本操作互斥 |
| 64 | 运行资源独占 |
| 128 | CAN控制故障锁存 |
| 256 | 角速度不在±3deg/s |

只在合法路线已准备、后端已按原停车条件完成重置后，允许mask仅含32/256时进入待准入状态。新参数VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS=1000，范围100～2000ms；每次服务重查完整原准入条件，mask0才发布动作。等待期间不发布底盘动作、不请求底盘VM，已确认的机构保持仍供电；取消/平台/机构故障立即停止，任何非32/256拒绝位立即失败，持续未就绪到1秒截止仍cause7撤VM。不是放宽源100ms门槛、忽略故障或无限重试，也不在运行中用过期数据继续走。

VM2在原VM1字段后附queue_state、queue_stage、queue_first_mask、queue_last_mask、queue_wait_ms、queue_attempts，首/末拒绝位在失败后保留。queue_state0无等待/1待准入/2已发布/3截止或取消；等待计时仅覆盖本次交接。采集器兼容VM1/VM2并从固件stage确认标签。VG3编码器交接与(2110,1200)目标保持，依然固定启停区1、仅第1→2段、不抓料/不进3。

## 同一最终版本检查

`evidence/integration/2026-11-04/stage12-admission/`保存原件、manifest、before/after/actual-final结果、候选/实际构建及下载记录。旧版模型实际主准入检查被简化，这次直接从main.c抽取真实App_HeadingRuntimeSource、App_HeadingRuntimeExclusive、App_SerialIdleReady及新版Blocked执行；传感器/外设状态为模型。旧版加入真实判断后在新增准入场景40项断言失败，复现cause7交接失败条件，不当作实车唯一根因证明。

最终候选和实际安装源码各执行60场景、880024断言0失败：原40场景以及100ms边界短暂/700ms/999ms拒绝后恢复、持续拒绝到截止、角速度等待、资源冲突立即拒绝、等待中取消/安全故障、搜索后待准入、99ms可准入。每个待准入步检查无底盘动作请求/active/底盘VM请求。输入开关1/2的模型都固定本次工程起点1，不宣称双起点实物。真实任务、联合/伸缩、底盘CAN后端和主准入函数一起执行；DM/PWM/轮系/传感器/部分调度仍为模型，不能替代板端完整调度或物理验收。

安装3文件：USER/vehicle_parameters.h、CHASSIS/USER/main.c、vehicle_mission.inc；其余1862源码/工程SHA保持。候选和实际Vehicle_Integrated/Vehicle_Offline构建均0错0警告，实际源码再次整组通过。原采集11项回归通过，VM2与VG3另由整组输出CRC往返解析验证。无Git提交/推送。

## 部署与复测

下载结果单独见flash-result.json及Flash原始日志，只有Programming Done/Verify OK/Application running后再只读确认VM2本轮run0待机；不把构建通过当部署。助手不通过串口或RAM触发运动。完整实物复测须从启停区1、车头+Y开始，不能在当前扫码点直接重跑；PC13一次，采集扫码→等待（若有）→第2段→偏置工位停车同轮日志。

本次最终下载已Programming Done/Verify OK/Application running成功，镜像SHA256为`64448f108e37dbc12a477617c377173122d90c0edf8ea8c3f547b36247c79b50`。烧录后只读171条、坏帧0，VM2共10条确认stage1/run0/state0/cause0、queue_state0待机；未触发新轮运动。证据flash-result.json、flash-attempt1.log与after-flash-uart/。当前等待用户从启停区1单次复测，不能把模型/Flash成功写成串联实物已通过。

如果仍失败，VM2可区分持续源/角速度门槛、资源冲突和其他准入项，结合HF3再定位。当前100ms航向边界为何在交接时出现、UART调度/显示调用影响及真实传感器间隔未完整确认，不称物理根因已闭环。

已核R QR-01/05/06/07、DSP-01/02、RUN-01/02/04/07、FIELD-01/02/06、PWR-01、MECH-01/02；E RUN-05/06、LOC-02/03/05/06/07、MOT-01～04、CAL-02～05、MECH-06。本次待准入与诊断为E，R不变，FIELD-05/OBS-03/QR-04等Q仍**待技术答疑确认**。未闭环：本候选同版真实串联、故障实际响应/重复性、偏置落点/车道/负载、毫米视觉和抓取、后续各段与180秒。
