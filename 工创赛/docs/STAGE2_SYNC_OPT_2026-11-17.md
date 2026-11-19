# 第二段准备缓存与统一放行（本地候选，2026-11-17）

本轮目标由用户经 grill-me 确认并明确授权继续：第二段减少串行和额外等待；第三段暂停等待队友机械模型。全程仅本地软件，无 probe、SSH、相机、串口、烧录、复位或机构动作。实际供电、夹持和位姿未知。

## 基线与范围

- 当前最后已烧固件：`97744c768b7f7d23afeb626300f700ccb9fba0490d473077db3c7e0f8267ee11`，不把本轮候选记成已部署。
- 只读基线：`tmp/stage3_parallel_preclose_20261116/project`；候选：`tmp/stage2_sync_opt_20261117/project`。
- 根 git 无 commit/remote，保留全部 untracked。隔离 bare `tmp/p1_upload_git_20261022.git` 本轮 fetch 成功，FETCH_HEAD `21522db5efa1a08cebb33ca2bf7b52601c4db809`；这不是根目录已 pull 或主线已同步，不提交、不推送。
- 仅四处源码差异：`pzq_commit_overlap.inc`、`vehicle_source_parallel.inc`、`dm_can2_motion.c`、`USER/vehicle_parameters.h`。编码/换行按原字节保留，复刻 prepare 脚本生成的完整源码清单与候选一致。

## 用户工程决定 E 与实际实现

1. 首次 X65mm 的真实完成立即进入视觉纠偏：原 `source_mcu_cycle.inc` prep5 → state2 已满足，不新增等候或修改其流程。DM180、原到位容差、每次独立120°和5稳定帧保持。
2. 左右 commit 后，phase4 使用 DM/X/Z 三轴组7；下部使用 X回0/Z下降组6。所有对应目标先准备，X/Z命令设缓存位，成功后只广播一次 CAN1 `FF 66 6B`；同一个 owner 调用内将 DMgo 置1。取消旧下部伸缩提前量。速度、加速和物理目标不变。
3. 末5mm 原140°提前闭爪保留；下降到位立即进入原 phase5。左右 X收回/Z上升先缓存统一放行，DM仍等原真实 Z净空/X≤30mm条件才转槽盘；下部 X已回0，原无X位移组保持。
4. 完整 DM/X/Z 放料姿态都完成后，同调用检查载盘原700ms窗口：已满立即210°，未满保留剩余真实等待。不把舵机PWM命令当实际角度反馈。
5. 210° PWM确认生效后，重新取当前RTOS tick计时，等待100ms（原300ms）。第1/2件 DM180/X65/Z观察位缓存统一放行；第3件原DM0/X0收回逻辑不变。
6. DM fast-retarget 仅新 S2 scope 在 state3 发真实当前位置 hold，缓存最终目标，等待有效反馈tick变化且仍新鲜后才准备成功；不伪增读取轮次。此时真实已使能，保留 STOP所需 `s_enable_possible=1`。S2 scope 已在state6等待 STOP应答时不重复提交request3，以保留原100ms应答机会；普通DM/S3路径不变，未增加保护或故障门限。

“统一放行”是控制指令层语义。CAN1广播与 DMgo 同调用；真实DM目标帧沿原 main 的下一次DM服务发送，未声称两总线物理零差。原main完整车配置存在 DM服务 → VSP服务 → DM服务，两次服务调度未改；实际总线/电机启动差仍须实车测量。原 `start_skew_ms` 包括缓存传输耗时，不能拿它当物理启动差，`sync_tick`只记录广播发送时刻。

## MATLAB前置与验证证据

证据目录：`evidence/integration/2026-11-17/stage2-sync-opt/`。

- 原控制契约模型在本机 `D:/MATLAB1/bin/matlab.exe` 实际通过400检查/28场景，完成时间 **2026-11-17 00:56:51**，早于首次对应源码准备。初次结果由本聊天工具 stdout 保留；当前JSON已被补充运行覆盖，不能用当前JSON冒充初次文件。
- S2 STOP应答延迟补充模型在 **01:20:44** 通过420检查，早于该scope STOP修复；当前 `matlab-result.json` 属这次补充结果。模型检查准备/缓存失败、STOP、完整轴到位、夹紧/释放、100ms回绕、三件和S3非作用域；不模拟真实运动动力学或总线实时性。
- 实际C owner：837检查，三位置/三槽、commit后相机丢失、完整pose松爪、700ms真窗口、PWM生效函数延迟、100ms回绕、末件行为。
- 实际VSP缓存/SPF组：315检查，组7/组6、缓存/广播/准备失败、取消、广播准入。
- 完整实际 `dm_can2_motion.c`（仅替换外设传输/时钟）：12检查，真实fastprepare → hold pending → hold反馈 → go → 最终目标，STOP回复及非S2原fast行为。
- 联合实际VSP_Prepare + 完整DM + VSP_Start + SPF：368检查，包括真实准备未完成时取消、缓存失败、广播失败、STOP回复延迟0/12/24/36ms；确认STOP mask真实清完。不使用fake ready、fake健康或修改读取轮次。
- 继承S3真实IT3/owner/dispatcher：6286检查，保留并行160°、原600ms窗口、返回600p、CRC/时效、teach、STOP与区域互斥。S3关键文件和原启动/观察/CAN/升降源码逐字节一致，见 `candidate.json`。
- 最终双Keil已通过，在线/Offline均 **0错误0警告**；在线AXF `8865228bf72580627a316ea7eb1e4672b2fbec9a45869d70ca2d9e33e3fceb4e`，Offline `f833ab775dc6703d07d4d84b822b2b217fad60d15850112b79c52d47afa20bb4`。以本目录 `Vehicle_LiftRuler-build.log`、`Vehicle_LiftRuler_Offline-build.log` 与 `candidate.json` hash为准；早期构建不充当最终构建。源码可重复生成，完整 `source-manifest.json` 记录SHA256。

## 既有第二段入口与部署边界

本轮没有改默认区域：`g_region_dispatch_area` 上电0、`g_region_dispatch_test_enable=1`，直接PC13仍会走原第三段调试入口。不能声称刷入此候选后直接PC13就是第二段。

生产区域接入沿用 `RegionDispatch_Arrive(1U)`，它在互斥条件允许时切到原料区并直接发布 `g_source_cycle_request=1U`，会启动已有第二段流程。已有现场调试方式是待机/无故障/无动作时由后续获授权的部署流程仅配置 `g_region_dispatch_area=1U`，再由用户PC13走现有 `SourceCycle_KeyEvent()`；这里记录入口，不执行配置或发车。不新增按键、串口启动命令，不改变队友区域状态机/QR/第三段默认。

下一步仅在第三段持料/机械条件已由用户明确处理、重新确认实物前提和部署授权后，选用候选版本，测左右三轴、下部双轴、第1/2件回观察实际启动差，验证抓牢/放稳与100ms释放余量。当前未实车验收，未更新实车正本或根仓库固件副本。

## 已核需求与开放项

R：SRC-01/02、RUN-01/02、PWR-01、MECH-01/02；E：RUN-06、CAL-02/05、VIS-15。本轮是用户时序工程选择E，R未改；官方未明确Q继续“待技术答疑确认”。软件完成不能替代单电源、尺寸、自动展开、放稳或整场规则验收。

未闭环：实际夹持/供电/位姿、跨CAN启动差、真实机械扫掠/负载/释放100ms余量、完整三件实车、第三段新机械模型与重新标定；根git同步未完成。没有新增物理成功结论。
