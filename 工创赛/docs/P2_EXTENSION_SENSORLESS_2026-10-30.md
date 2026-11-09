# 28伸缩轴手动缩回碰撞回零候选

> 2026-10-30第二次触挡自行停止确认（E，最新）：用户明确“是再次缩回、真正触挡后自行停止，进入下一步”。结合第一次回零及中间伸出退让，接受回零→离挡→再次触挡自行停止的实物功能复现，不再重复该项。第二次未提供新Watch，软件完成状态保留待核，不沿用第一次state8/result1当本次结果。证据`evidence/p2-arm/2026-10-30/extension-sensorless-board/second-home-observation.json`。当前转车载三槽盘实际接口确认，见`P2_CARRIER_RESUME_2026-10-30.md`；28有效行程/毫米映射/取放伸缩点、毫米重复精度、止挡承载/保持、掉电和抓放仍待验。无固件/编译/下载或助手硬件命令。核R RUN-01/02/03、PWR-01、MECH-01/02，E RUN-06/LOC-03/CAL-05；官方Q保持待技术答疑确认，下方“第二次尚未执行”为历史。

> 2026-10-30首次回零后退让确认（E，最新）：用户明确“伸出、离开挡块、停稳”，接受本次实际离挡及最终停稳；本轮无新运动Watch，不补写state/result或精确位移，未将160脉冲换算毫米。证据`evidence/p2-arm/2026-10-30/extension-sensorless-board/retreat-after-first-home.json`。下一项为下述第二次缩回回零，尚未执行；无需改固件/重编译/下载。核R RUN-01/PWR-01/MECH-01/02，E RUN-06/LOC-03/CAL-05；重复原点精度、全行程/毫米、止挡承载、掉电恢复及抓放仍待验，官方Q保持待技术答疑确认。

## 历史步骤：第二次缩回回零（已获实物确认，软件状态待核）

保持程序运行、VM就绪、`g_lift_state=4`，其他轴动作请求0、`g_dm_point_enable=0`。普通伸出动作已物理停稳，开始前确认运动Watch未失败锁存，正常完成应为`g_extension_motion_state=7/result=1`；若为state9或result非1，先回图排查，不连续请求。确认缩回通路及止挡无异常后，**仅写`g_extension_home_request=1`一次**，不同时触发普通运动。

观察是否再次缩回、真正接触同一止挡后自行停止。触挡持续顶压、未触挡提前停或其他异常不计通过；需要停止写`g_extension_home_request=2`并保持程序运行，必要时按既有外部停机流程处理，不等超时。停稳后提供本次新Watch：

```text
g_extension_home_state
g_extension_home_result
g_extension_home_flags
g_extension_home_start_result
g_extension_home_reported
g_extension_home_abort_result
g_extension_home_stop_result
```

软件完成候选为state8/result1/reported1，须结合本次真实触挡/自行停止的实物反馈。不要把首次留存的state8当成第二次完成。第二次功能检查即使成功，也不独立证明毫米重复精度。

> 2026-10-30首次实物回零功能确认（E，最新）：用户明确“是缩回、真正触挡后自行停止”；Watch为home_state8/result1/flags3/start_result1/reported1、abort_result0/stop_result0、home_request0/VM请求1。结合本次口述，接受首次缩回触挡、自行停止及软件回零完成；abort/stop为0表示未调用退出/强制停车，不是停车失败。原图、SHA及字段见`evidence/p2-arm/2026-10-30/extension-sensorless-board/first-home-acceptance.json`，本轮实际home c/h、extension.c、config及main五处SHA均与最终安装记录一致，无固件修改/编译/助手烧录或硬件命令。替代下方“首次实物回零待验”的历史状态；重复回零功能/原点精度、止挡承载/长期保持、毫米/全行程、掉电恢复和带载抓放仍待验。核R RUN-01/PWR-01/MECH-01/02，E RUN-06/LOC-03/CAL-05；官方Q保持待技术答疑确认。

日期2026-10-30，E类工程实施。以下实现和首次试验步骤保留历史；当前状态及下一项以本节最新记录为准。

## 历史步骤：低速伸出，留出复测间隙（已获实物确认）

保持现固件及程序运行，无需重新编译/下载。VM就绪、`g_lift_state=4`，其他轴动作请求0、`g_dm_point_enable=0`，确认伸出方向通路无干涉后，**仅写`g_extension_motion_request=2`一次**。沿用160脉冲、5RPM、加速20、3秒整体超时；脉冲不换算毫米，不自动加大步长或延时。

观察是否实际伸出、离开缩回止挡并最终停稳，回以下Watch：

```text
g_extension_motion_state
g_extension_motion_result
g_extension_move_result
g_extension_flags
g_extension_delta_raw
g_extension_speed_rpm
g_extension_stop_result
```

正常软件完成候选state7/result1/move_result1/speed0，stop_result0表示未发强制停车。需要停止时写`g_extension_motion_request=3`并保持程序运行；停止未确认按既有外部停机流程处理。本轮用户已确认离挡/停稳，未提供本次运动Watch，不补造软件结果。第二次回零按上方当前下一项执行，不连续触发或自动往返；第二次回零功能确认不等于毫米重复精度验收。

## 本机输入与当前状态

已确认协议方向0缩回、1伸出。用户确认无原点开关、机械限位可以作为原点；本轮回零参数截图read_result1且电机未动，接受只读功能成功。实读mode0、direction0、rpm30、timeout10000ms、detect_rpm300、detect_current800mA、detect_time60ms、auto_home0。原图及字段在`evidence/p2-arm/2026-10-30/extension-home-read-board/board-acceptance.json`。

用户先反馈仍在行程中间，随后补充“有3.5cm”，换算为离缩回止挡35mm的用户输入，不是3.5mm，也不是独立标定。此前3～5mm是询问中的首次试验起点选项，不是已执行摆位或正式要求。本轮按当前35mm余量准备单次、有界、人工观察的回零候选；不换算35mm为脉冲，不保证10秒一定覆盖，也不据机械限位存在推定承载/碰撞停止已通过。

## 回零入口与边界

实际工程`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`新增独立`extension_home.c/h`，默认`g_extension_home_request=0`。写1才准备一次缩回端无开关碰撞回零；写2退出回零并停车。与`g_extension_home_read_request=1`的只读操作明确分开，后者不会启动运动。

菜单mode0是配置值，不是错误。厂家X系列V2资料7.4.2规定启动命令可选回零模式，本次显式发节点6扩展帧0x600、数据`9A 02 00 6B`选择多圈无限位碰撞模式；不修改菜单、不写阈值/上电自动回零、不发单圈零点设置或松轴命令。方向0和30RPM等均来自本轮6号实读；数值虽与35历史配置相同，不当作35参数可直接移植或28阈值适用已验收。

每次接单重新读Emm选项、6号回零参数、驱动及回零状态和静止速度；只匹配本轮dir0/30RPM/10000ms/300RPM/800mA/60ms/auto0，配置变化拒绝，不自动改回。预备最多2500ms；驱动配置10秒，主控启动后11秒上限；服务间隔超过500ms失败。忙时启动请求不排队、不重置计时。

本轮先清除旧6号完成通知，排空启动前接收队列后才捕获新6号9A/9F通知；原5号通知条件保持，不能把5号或格式不符的帧当6号完成。启动ACK丢失也按可能已启动处理。仅默认空闲标志不算成功，必须见本轮运行或完成通知，回零状态无失败且速度0，才给软件driver-reported完成。

中止和启动后的失败均先发`9C 48 6B`退出，再发普通立即停止。若退出回复条件不满足但停止ACK成功，额外读取新鲜回零闲置状态和速度0确认已结束。启动后退出/停止未确认时仍占用互锁，不放行升降、旋转或普通伸缩新动作；最终复核修正了失电许可持续无效时的重复停止尝试，失败锁存后仅显式request2才重试。失败前未发送启动时不伪造已运行。整个流程不以发送停止尝试代替物理停止。

唯一CAN任务保持；升降/旋转忙时拒绝回零，回零活动时拦截其他轴新启动和外部重读，原停止入口保留。普通伸缩停止请求3在回零期间转为退出回零处理。其他160脉冲/5RPM小步、5号升降/回零和CAN2逻辑不改参数。

## 下一单项：用户首次实物缩回回零

1. 保持当前可观察的缩回通路和机械止挡条件，其他轴动作请求0，`g_dm_point_enable=0`。本次仅28伸缩，不抓放或同时旋转/升降。
2. 退出旧调试并重新打开上述实际工程，加载本轮新加入的回零模块，下载已编译版本后F5。VM请求1，等待约3秒和`g_lift_state=4`。
3. **仅将`g_extension_home_request=1`写一次**，保持程序运行，观察是否沿已确认方向缩回、是否真正接触止挡后自行停止。读取请求变量不替代该启动请求。
4. 若触挡仍持续顶压、方向不对、异常或需要停止，立即写`g_extension_home_request=2`并让程序继续执行；必要时按既有外部停机流程处理，不等待10秒。未触挡提前停止也如实记录，不判原点成功。
5. 停稳后发下面Watch和“是否真实触挡、自行停稳”的实物反馈。软件成功候选为state8/result1/reported1，仍不独立证明已碰挡块/原点重复性。

```text
g_extension_home_state
g_extension_home_result
g_extension_home_flags
g_extension_home_start_result
g_extension_home_reported
g_extension_home_abort_result
g_extension_home_stop_result
```

| 字段 | 含义 |
|---|---|
| home_state | 0空闲，1～7准备/回零，8驱动报告完成，9失败锁存，10人工中止 |
| home_result | 1驱动完成，2中止，3超时，4通信，5驱动状态，6互锁，7参数变化，8服务间隔超限 |
| home_reported | 仅本会话驱动报告，掉电/失败/中止撤销；不当精密原点或运动许可 |
| start_result | 1为启动命令获允许应答，不单独代表触挡或回零完成 |
| abort/stop_result | 未发为0；中止后分别保存退出/停车结果，停止未确认需处理 |

## 验证与证据

初次8处更新：EXTENSION c/h、新home c/h、CAN c/h、main和项目引用，805个其余源/项目SHA保持；最终只在home.c修正停止锁存分支，其余812源/项目SHA保持。旧版本、初始manifest和锁存修正前文件均保留，最终实际SHA与验证副本一致。

最终192项合成检查通过：64项原小步/调度/传输、41项参数读取、87项真实回零状态机/任务分支/CAN启动退出和通知捕获。包括启动不动作、fresh配置匹配、双机构互锁、忙时不排队、超时/失电/中止/服务超间隔、未见运行证据不成功、旧/错节点通知不成功、退出/停车失败保留占用、失电失败后不自动重复停车且显式中止可重试、原5号通知隔离。测试没有真实运动/电流/接触力输入。

副本及实际最终Build均0错误0警告，Code58264/RO608/RW2200/ZI26016。`evidence/p2-arm/2026-10-30/extension-sensorless/`保存两阶段备份、manifest/installed/review、输入/协议说明、192项日志及最终构建；`installed-before-stop-latch-fix.json`保存首次候选，不冒充最终状态。生成/安装脚本`prepare_extension_sensorless_20261030.py`，最终一文件修订信息在manifest，独立验证器`validate_extension_sensorless_20261030.py`及既有验证器sensorless配置。

助手未烧录或发硬件命令。原点首次触挡停止、重复性、限位承载/长期保持、实际行程/毫米、掉电恢复、带载/固定抓放均未闭环。

R已核RUN-01/PWR-01/MECH-01/02，E已核RUN-06/LOC-03/CAL-05。本次仅工程候选，官方Q继续待技术答疑确认，不新增正式规则或把软件报告当实物验收。
