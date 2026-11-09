# 28夹爪伸缩轴接续（E）

> 2026-10-30第二次缩回触挡自行停止确认（E，最新）：用户明确再次缩回、真正触挡后自行停止并要求进入下一步。接受首次回零→伸出离挡→再次回零的实物功能复现，不重复同项；第二次无新Watch，软件完成状态待核，不补造8/1或毫米精度。证据extension-sensorless-board/second-home-observation.json。转车载三槽盘实际引脚/供电确认，见P2_CARRIER_RESUME_2026-10-30.md；28全行程/毫米/取放点位、重复精度、承载/掉电及抓放仍开放，不因转下一单项而放行串联动作。无固件/编译/下载或助手硬件命令。核R RUN-01/02/03、PWR-01、MECH-01/02，E RUN-06/LOC-03/CAL-05；官方Q保持待技术答疑确认。

> 2026-10-30首次回零后伸出退让确认（E，最新）：用户明确“伸出、离开挡块、停稳”，接受本次离挡/最终停稳，无新Watch，不补造软件state/result或毫米距离。证据extension-sensorless-board/retreat-after-first-home.json。下一单项同固件/VM及35就绪、其他轴闲置，确认普通运动未失败锁存及缩回通路后，home_request1一次做第二次缩回触挡回零；异常需停2，回本次实物触挡/自行停止及home专用Watch，不用第一次留存状态替代第二次。详见P2_EXTENSION_SENSORLESS_2026-10-30.md当前下一项。无固件/编译/下载或助手硬件命令；重复精度/全行程/毫米/承载/掉电及抓放待验。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05，官方Q保持待技术答疑确认。

> 2026-10-30首次缩回触挡回零通过（E，最新）：用户明确“是缩回、真正触挡后自行停止”；Watch home8/result1/flags3/start1/reported1、abort0/stop0/request0/VM1，与本次软件完成一致。接受首次触挡自行停止功能，不升级为重复精度/完整行程/限位承载/掉电恢复或抓放验收。原图及字段在extension-sensorless-board/first-home-acceptance.json；本轮实际五处相关源SHA与最终安装一致，无固件/编译/助手烧录或硬件命令。下一单项同固件、VM/35就绪且其他轴闲置，确认伸出通路后motion_request2一次（160脉冲/5RPM/3秒），观察离挡与停稳并回运动Watch；需要停3，确认后才安排第二次回零，不自动往返。详见P2_EXTENSION_SENSORLESS_2026-10-30.md当前下一项。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05；官方Q保持待技术答疑确认，下方首次回零待验为历史。

> 2026-10-30本机参数及回零候选（E，最新）：用户读参result1、mode0/dir0/30RPM/10000ms/300RPM-800mA-60ms/auto0、电机未动，符合只读行为；距缩回机械止挡口述3.5cm=35mm。已新增独立手动02碰撞回零，`g_extension_home_request`默认0、1启动/2退出停车，fresh参数匹配、运行/完成证据/速度0判驱动完成、超时/失败锁存及未确认停止互锁，保持原参数及160小步。初始8处更新（805保护源）加最终home.c停止锁存修正（812其余源保持），192合成检查和副本/实际Build0错误0警告Code58264，未烧录/助手发动作。下一单项下载最终新版、VM/35就绪，home_request1一次观察真实缩回触挡及自行停止，需停2；详见P2_EXTENSION_SENSORLESS_2026-10-30.md及extension-sensorless/review.json。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05；首次物理回零/重复原点/行程/抓放待验，官方Q保持待技术答疑确认。

> 2026-10-30机械限位原点候选及只读入口（E，最新）：用户明确“没有限位开关，但是可以限位，可以作为原点”，本轮按缩回机械止挡候选处理，更新旧PE10开关实装前提但保留历史，不当原点已验收。已在实际工程仅新增节点6的22回零参数只读入口，默认0、显式read1、三帧原包及独立解码、运动忙/供电互锁；不添加回零/写参/置零命令，不套35阈值。5处更新、806其余源/项目SHA保持，105合成检查及副本/实际Build0错误0警告Code56124，未烧录/助手发硬件命令。下一单项下载新版、VM/35就绪，`g_extension_home_read_request=1`一次，回result及展开params；详见P2_EXTENSION_HOME_READ_2026-10-30.md及evidence/p2-arm/2026-10-30/extension-home-read/。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05；本机回零参数/止挡条件/原点及行程待验，官方Q保持待技术答疑确认。

> 2026-10-30双向小步功能确认（E，最新）：用户确认请求2“伸出并停稳”，Watch运动7/1/move1/flags3/delta0xfffffffffffff2da（−3366）/speed0/stop0；结合前项请求1缩回，双向各一次功能确认。方向映射1=缩回、2=伸出；不再重复同项或改参，未据此验收原点/全行程/毫米/带载。下一项确认缩回端原点开关实装状态，历史PE10为规划，35不装开关的决定未自动覆盖28，尚不发回零。证据evidence/p2-arm/2026-10-30/extension-jog160-board/extend-functional-acceptance.json及原图。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05，官方Q保持待技术答疑确认，无固件或助手硬件命令变化。

> 2026-10-30缩回方向及最终停稳确认（E，最新）：用户明确缩回停稳，Watch运动7/1/move1/flags3/delta0x43（67）/speed0/stop0，接受请求1=缩回的本次功能，不当精确行程/毫米标定。下一单项同固件/160脉冲/5RPM/3秒超时，请求2一次确认反向伸出及停稳，回当前运动Watch；不重复缩回/下载。详见P2_EXTENSION_JOG160_2026-10-30.md及evidence/p2-arm/2026-10-30/extension-jog160-board/retract-functional-acceptance.json。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05，反向/原点/行程/抓放待验，官方Q保持待技术答疑确认，无固件或助手硬件命令变化。

> 2026-10-30步长扩大到160（E，最新）：16脉冲软件反馈7/1/move1/flags3/delta0xE6（230）/speed0/stop0已收到，用户“好像缩回”只作方向暂述；用户要求加大步长。本次仅实际extension_config.h的16改160，速度5RPM/加速20/3秒超时不变，64项合成检查及副本/实际Build0错误0警告，其他810源/项目SHA保持；未烧录/助手发动作。当前下一单项下载新版，request1一次观察明确伸/缩及停稳、回运动变量；详见P2_EXTENSION_JOG160_2026-10-30.md及evidence/p2-arm/2026-10-30/extension-jog160/。方向、毫米/原点/行程/抓放待验，核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05，官方Q保持待技术答疑确认。

> 2026-10-30首次小步入口已交付（E，最新）：用户确认“在中间，两侧都有余量”。已将节点6独立16脉冲/5RPM/加速20/3秒超时的相对小步接入实际工程，默认请求0；重读选项/状态/起点、忙时拒绝、位置变化+到位+静止判完成、失败锁存及停止，升降/旋转互锁保持。7处更新、804个其余源SHA保持，64项合成检查及副本/实际Build0错误0警告Code55212；原底盘/节点5CAN控制代码保持，无烧录/助手硬件命令。此E实现将下方“EXTENSION仅查询”扩展为显式小步候选，原只读能力/源码备份保留，不静默放开原地址保护。下一单项下载新版/F5，VM1等待35就绪后`g_extension_motion_request=1`一次，回实际伸/缩、停稳和状态/结果/反馈；需停写3，不猜方向/毫米、不回零或松轴。详见[P2_EXTENSION_JOG_2026-10-30.md](P2_EXTENSION_JOG_2026-10-30.md)，证据extension-jog/review.json及构建/主机日志。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05，方向/原点/行程/抓放待实测，官方Q保持待技术答疑确认。

> 2026-10-30 6号只读通信通过（E，最新）：用户两张原始Watch截图显示state4/error0，version/status/options_result均1、read_request0。三帧均扩展数据帧ExtId0x600、IDE4/RTR0；版本DLC6：1F 00 CA 21 14 6B，状态DLC3：3A 03 6B，选项DLC4：1A 00 06 6B。长度/命令/尾6B与现有查询严格校验一致，选项Data2 bit1符合Emm分支判据；DLC外Data4=FF/Data5=F8不算有效载荷，不误判错帧。接受本次CAN1节点6三项只读识别功能，不重复该项，不当伸缩方向/原点/行程或抓放验收；03为驱动状态标志，不证明机构已到目标。原图SHA/字段/检查见evidence/p2-arm/2026-10-30/extension-readonly-board/board-acceptance.json。下一项准备独立28低速有界相对小步；已向用户确认当前机构是否在中间且两侧有余量，答复前不触发运动、不套35的3000RPM/5000脉冲或mm映射。实际源已只读检查，当前EXTENSION仍仅查询、无运动入口，本轮未改固件/编译/烧录/发硬件命令。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05，官方Q继续待技术答疑确认。下方未通信/等待回图为历史。

> 2026-10-30参数口述确认（E，最新）：用户回复“没有问题就是这些参数”，接受地址6/CAN1_MAP/500000bit/s/固定6B的本机参数口述确认，不重复索要同组参数；未提供菜单照片，不虚构独立接线观察或通信通过。小板装好接CAN1后，沿用现固件的EXTENSION只读入口，VM就绪且g_lift_state4、其他机构运动/回零请求0时，g_extension_read_request1一次，运行约1秒，回state/error/version_result/status_result/options_result及三个reply原帧。预期state4/error0/三个result1；失败不连续重发，不试运动。来源及证据evidence/p2-arm/2026-10-30/extension-resume/menu-confirmation.json，实际源及main调用就绪/互锁条件已只读核对；无代码变化/编译/下载/助手硬件命令。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05，通信/方向/原点/行程/抓放仍待验，官方Q保持待技术答疑确认。

> 2026-10-30恢复（E，最新）：用户明确CAN小板“已到货”，替代下方缺板暂缓状态；安装、接线、实际菜单配置及6号通信尚未确认。按用户“下一步就这样”接续，旋转保留取料177.833679°/放料粗调33.8401489°，物理精调后置且不当最终对齐通过。已核实际EXTENSION两源SHA与原只读安装记录一致，本轮无代码变更/编译/下载/助手硬件命令。下一单项断电安装及CAN1接线确认，并核28菜单地址6、CAN1_MAP、500000bit/s、固定6B；不套达妙CAN2的1Mbps，不擅改现有CAN1总线。配置确认后才显式read_request1一次读6号版本/状态/选项，暂不发运动/回零。证据evidence/p2-arm/2026-10-30/extension-resume/intake.json；核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05。实际通信、方向/原点/行程和固定抓放未闭环，官方Q继续待技术答疑确认。下方保留历史。

## 用户明确暂缓：CAN小板未到货

2026-10-28用户明确28电机CAN小板未到，要求先做下一项、做完后回来继续28。当前仅用途（夹爪伸缩）/地址6及主机只读代码准备完成；未取得28实机CAN通信，不将缺板记为软件故障，也不要求再次下载或查询6号。保留现有只读模块且g_extension_read_request保持0（默认0，无自动查询）。不临时更换串口/脉冲方案，不删除已做代码与来源。

恢复断点：下一项完成且CAN小板到货后，从确认本台小板安装/接线、通信方式CAN1_MAP及500kbit/s匹配，随后6号1F/3A/1A只读识别开始；不用重新确认已知用途/地址。只读识别成功后再做单轴小步/方向、原点/行程。下一项为机械臂旋转轴实物接口登记（历史计划DM4310），当前型号/装机/接线仍待确认，不能把参考手册1Mbps直接加入现有500k CAN1。已核R RUN-01/PWR-01/MECH-01/02，E RUN-06/LOC-03/CAL-05；官方Q保持待技术答疑确认。

## 当前：地址6确认，只读模块已安装编译

用户明确ID_Addr=6，关闭地址待确认。当前沿用主控现有CAN总线500kbit/s尝试只读识别，不据地址或照片认定28已设CAN1_MAP/500k或已接通。新建实际工程HARDWARE/EXTENSION/extension.c/h，经原唯一CAN任务的机构隔离分支串行调用，不复制35运动驱动、不扩大CAN运动地址范围；35模块源码未改。

仅显式g_extension_read_request1触发6号1F版本/3A状态/1A选项三项查询，默认不上电自动查询、失败不自动重试。供电/平台未就绪或35运动/回零时拒绝读取（error7），忙时不排队。原CAN每项查询有限超时保持；模块不含使能/运动/回零/参数写入。

Watch：g_extension_read_request；g_extension_state（0空闲、1～3查询、4完成、5失败）；g_extension_error；g_extension_version_result/status_result/options_result（1成功）；g_extension_version_reply/status_reply/options_reply原帧。掉电使完成状态失效，原始帧只供历史排查，不作新鲜查询证据。

已完成主机检查：只查询地址6、启动不发命令、供电/升降忙拦截、三次有限查询、失败不重试、忙时请求不重启、供电失效及中途互锁。候选与实际Keil Build均0错误0警告，Code45580/RO588/RW1732/ZI25660。四文件源哈希、原main/uvprojx备份、host-check.log、stage-build.log、actual-build.log、installed.json位于evidence/p2-arm/2026-10-28/extension-readonly/；生成脚本scripts/prepare_extension_readonly_20261028.py，源模板output/extension_readonly_20261028/。已经写实际D盘工程，未烧录或发板端命令。

下一单项只读识别：下载新版/F5，g_vm_test_request1后等35身份state4；保持35的motion/home请求0，g_extension_read_request1一次/F5约1秒，暂停取extension_state/error/三个result与原始回复截图。预期state4和三个result1；失败先保留错误/原帧再检查28实际通信设置，不试运动。角色与地址已确认，型号/固件、协议/物理连接、方向/原点/行程仍待实机证据；官方Q保持待技术答疑确认。本节替代下文地址未知的历史状态。已核R RUN-01/PWR-01/MECH-01/02，E RUN-06/LOC-03/CAL-05。

用户本轮明确28步进电机带动夹爪伸缩；照片可见齿条/齿轮伸缩机构及夹爪，归档evidence/p2-arm/2026-10-28/extension-intake/assembly-overview.jpg、rack-gripper.jpg。轴角色由“28第二轴”细化为“夹爪伸缩轴”，与35升降、夹爪开合舵机和车载三槽盘分开。不据图片推定齿轮参数、脉冲毫米比、通信物理层、地址、伸缩正负方向或极限位置。

当前35顶部两次回零功能复现已通过，后续不重复；伸缩当前只确认角色，未写驱动或发运动。下一唯一输入：28驱动的通信地址（P_Addr/地址菜单当前值）；若没有该菜单或不确定，发该驱动菜单照片。35地址5不套给28，历史引脚/原点规划不等于实装验收。通信方式、型号、回零/行程与安全小步须后续按实物确认。

已检查R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05。证据为用户用途确认及两张照片；伸缩通信/方向/原点/行程/到位/固定抓放未闭环，官方Q保持待技术答疑确认。照片不作为初始尺寸或比赛合规验收。
