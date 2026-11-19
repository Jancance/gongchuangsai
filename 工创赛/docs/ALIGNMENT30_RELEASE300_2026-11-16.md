# 单次纠偏上限30mm、槽盘松爪等待300ms

## 最新追加：仅原料区观察开爪220°，最终候选构建通过未烧录

用户后续明确希望识别物料时开爪210→220，经grill-me核对后选择“只改观察开爪”，170°预收、140°夹紧与槽盘放料210°保持。当前单键MCU准备序列`{1,2,7,6,3,3}`实际以Integration10 request6开爪，调用点为`CHASSIS/USER/integration10_complete.inc`的`jiazhua(request==5U?140.0f:210.0f)`。该首次观察分支改220；前两件放料后的phase7组完成、再次回到原料区观察时也写220，槽盘release210与300ms保持，末件最终停放不额外改220。只改request6会导致后两次观察仍210，已在本地最终候选前补齐，不把该中间版本当最终交付。不盲改`STARTUP_GRIPPER_ANGLE_DEG`或`VEHICLE_SOURCE_PREP_GRIP_DEG`，避免扩大为上电及其他流程参数修改。先运行本机MATLAB`validate_observe220_20261116.m`，16项名义角/PWM/分支与各观察访问检查通过（220°名义2130µs，非实物角度反馈）；对应新隔离目录`tmp/source_alignment30_release300_observe220_20261116/project`，证据`alignment30-release300-observe220/`。

30mm/300ms原候选已完成：独立`tmp/source_alignment30_release300_20261116/project`只改三个文件，实际低层准入166/MCU截断1394/单件owner668/末件951/故障627/锁轮65/组10检查通过，双目标构建0错误0警告；高档独立审查无阻断发现。在线c1736448，Offline7070e561。其证据`source-alignment30-release300/`保持，不把后续220°候选混作同一个镜像。

最终220°候选只在30mm/300ms基线上再改两个固件文件：首次观察request6，以及前两件phase7真实完成后的观察开爪。实际C分支13、返回观察PWM失败133、三位置三槽owner698、末件968（内含MCU整轮667）、六阶段故障633、锁轮65/组10，共2520断言通过。明确299ms不回程、300ms放行，返位组未完成时不提前220；220命令被拒绝则原Low_Fail7，不能发布DONE；槽盘210仅在原37°放料点，槽3最终仍210。高档独立审查最终两文件版本无阻断发现，取代首次观察单点中间结论。

最终在线AXF SHA256 `1b55358e7ecf5a875e36385ad16a2ab31baee159a80d5db6a1de45360ff6ad6c`，Offline `2e56424570aa39a3ef39a8ded108529dbd3f18392fa658108a97484430b06de2`，双目标完整构建0错误0警告。新工程`tmp/source_alignment30_release300_observe220_20261116/project/gongxun-f407.uvprojx`；准备/构建脚本`scripts/prepare_alignment30_release300_observe220_20261116.py`、`scripts/build_alignment30_release300_observe220_20261116.py`，证据`evidence/integration/2026-11-16/alignment30-release300-observe220/`的candidate/checks/final-invariants与日志。30mm/300ms参数、45mm/s、170°/140°/210°非观察动作及既有停止/VM保持；五帧Pi清单24项字节相同，无需换视觉包。本候选尚未烧录/实测，车上仍96613b64。

## 授权与本轮范围

用户先要求查明当前单步上限，并询问槽盘是否松爪后等200ms才转4310；若是则改300ms。核对当前已烧录96613b64源码：MCU在最大误差轴按误差四舍五入、范围1..20mm，低层同样只准入1..20mm；phase6槽盘松爪从`Gripper_PulseApplied()`确认PWM生效起算200ms，随后才提交phase7包括4310回程的并行组。用户后续明确单次上限20→30mm，因此本轮两项一起修改。

30mm是单轴每次纠偏的上限，不是固定步长。误差8mm仍提交约8mm，5mm画面定位容差不变，每次只选择误差最大轴。速度继续45mm/s、全局A/J保持；不放宽零回缩到位窗，不新增保护/故障。300ms仅槽盘放料后、松爪PWM生效到回程组放行，原料区抓取不增加等待，170°预收/五帧视觉/初态分支和机构同步保留。

## 前置验证与实施

先使用本机MATLAB：`scripts/validate_carrier_release300_20261116.m`完成71检查，含299/300ms边界、PWM加载时刻与32位计时回绕、取消不能放行；`scripts/validate_source_alignment30_20261116.m`在真实方向标定/轮径/曲线及追随预留下扩大至1..30mm，完成4506检查、1536门控场景。前置均通过后，再由6.1 Sol medium实现隔离候选，6.1 Sol high独立审查；本轮不声称主聊天模型自行切换。

证据目录：`evidence/integration/2026-11-16/carrier-release300/`与`source-alignment30-release300/`。本轮只扩大单次步长边界，不假定纠偏次数会减少，也不把原记录的3秒未达标结论改为通过。源区45mm/s及真实STOP/新鲜零速条件不变；普通行驶仍原1000ms等待。

## 当前状态与未闭环

当前车上仍是已烧录96613b64（20mm/200ms）、Pi五帧新分支实际自启包。本轮新候选未烧录、未部署视觉、未发车、未上传。实际30mm纠偏响应、300ms放料稳定性、3秒纠偏/25秒整段和零回缩重复性待新鲜实车验证。此前第三件归位故障与用户重启后的成功轮次保持分开记录，见`STOPPED5_INITIAL_BRANCH_FAST_ALIGNMENT_2026-11-16.md`。

已核R：RUN-01/02/04、SRC-01/02、FIELD-06、PWR-01、MECH-01/02；E：SRC-04、CAL-05、VIS-12/14/15、RUN-06、MOT-02/04及本轮30mm/300ms用户选择。上限、等待与速度是工程参数，不是官方规则；Q仍待技术答疑确认。通过既有隔离bare仓库fetch完成，远程仍21522db，根本地未跟踪成果未覆盖，无提交/推送。
