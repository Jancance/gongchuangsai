# 到名义扫码点后撤VM、不搜索：航向参考修复

## 本次实车观察与日志

用户确认新版现在可以运动、能到指定停车位置，但无码后撤VM，没有横移搜索。只读COM14采集262条、CRC/字段错误0、串口运动命令0，原件在 `evidence/integration/2026-11-04/stage1-extension-diagnosis/arrived-no-search-uart/`。

任务run1/stage1/state6/cause8；伸缩EX1 first_reason0，没有本次伸缩首故障。底盘推算横移1051.887mm、distance_reached1、distance_fault_reason0、四轮零速、STOP mask15，CAN/RTOS/Safety全局故障0。随后后端变为IDLE/result0，但组合state4/cause3；首次故障快照是phase0/backend IDLE/session1，HWT有效且无串口错误。旧字段中的取消并不证明用户再次按键。

到位及位移均是软件/编码器证据；用户“到指定位置”是本次功能观察，不补写地面毫米精度。上次伸缩监视故障本次未复现，不以first_reason0宣称其物理原因已查清。

## 原因

第一段成功启动时已经建立s_route_heading_reference，valid=1。到点无码后，任务排队第一次搜索，后端完成重入、回到IDLE。主调度把搜索当作一个新动作组，又调用严格的RouteHeading_Init；该函数拒绝对valid=1的参考再次初始化，于是App_CombinedTestStop(3)取消，任务上报cause8并撤VM，尚未发出搜索运动。

旧代码在整组模型中复现HEADING_REFUSED valid1 search1，两个起点第一次搜索均state6/cause8，与本次实车故障链一致。不是跟随、末端或无码搜索耗尽保护导致。

之前整组模型的部分主调度采用适配实现，没有执行实际航向参考初始化判定，漏覆盖了这一条件。本轮已加入实际route_heading_reference.h中的判断，并保留旧版失败复现，不能把先前模型通过当作该条件已覆盖。

## 修复

- route_heading_reference.h新增Ensure/Reset：同一轮可复用有效、范围合法的原参考；未建立时从新鲜yaw初始化；原严格Init保留，不任意覆盖坐标系。
- main.c在整车模式使用Ensure，使名义路线、两次搜索及同轮子路线保持同一车头+Y参考，不在每次搜索重新定方向。
- vehicle_mission.inc只在新一轮有效启动时Reset，防止同次上电多轮运行沿用上一轮航向原点。

只改上述3文件，其他1862源码/工程SHA保持；45cm/s、搜索15/30cm、3秒斜移末端等待、通信/停稳保护和EX1诊断均保持，没有删除保护来绕过错误。

## 同版本整组验证

证据目录 `evidence/integration/2026-11-04/stage1-search-heading/`：

- before/、before-repro.log/json：未修改实际旧源码，复现首次搜索航向参考拒绝。
- after/：修复副本18场景通过；actual-final/：安装并构建后的实际源码18场景、16320断言0失败。
- 同一可执行流程包含真实任务、真实完整CAN后端、真实联合启动/伸缩监视控制器及实际航向参考函数。DM驱动/PWM、HWT、UART/扫码/屏幕、部分主调度、CAN/RTOS/轮系仍为模型，不是完整ARM固件执行或整车实物验收。
- 覆盖双起点：名义点有码、第一次/第二次搜索途中有码、无有效码退出、非法码、取消、CAN故障、伸缩监视失败，以及同次上电第二轮新航向参考建立。最后一项确认run2从新yaw建参考，搜索仍复用本轮原点。
- 副本与实际正式/离线目标均0错误0警告，原件与前后SHA见manifest/before-install；无Git提交或推送。

复现当前实际整组：`python -X utf8 scripts/check_stage1_whole_flow_20261104.py`。默认启用真实联合/伸缩控制器和航向参考，输出至stage1-whole-flow-current，避免覆盖旧版14/16场景历史证据。

## 最新部署

本轮延续已授权现场排查完成烧录，Flash日志Programming Done/Verify OK/Application running；镜像SHA见flash-result.json。最初10秒日志轮询未等到结束，随后已核对实际Flash成功记录，不把轮询截止当作下载失败。只读采样136条、坏帧0，VM1 run0/state0/cause0待机，未触发路线运动。

需要用户将车从扫码点放回PA1对应启停区、车头+Y、两侧搜索范围净空，再PC13一次。不能在扫码点就地重跑整段。新搜索物理动作尚未复测，不宣称实物已通过。

## R/E/Q与开放项

核R QR-01/05/06/07、DSP-01/02、RUN-01/02/04、FIELD-02/06、PWR-01、MECH-01/02；E RUN-06、LOC-03/05/07、MOT-01/02/04。航向参考生命周期和模型修复是E，R不变，FIELD-05/OBS-03/QR-04等Q待技术答疑确认。新版搜索实车、真实落点/读码可靠性、上次伸缩异常物理子因、位姿更新与第2段衔接仍开放。
