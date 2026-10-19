# 无线HWT顺时针对照（2026-10-19）

## 已验证逆时针结果
用户图key_state4/cause0、auto_result1/cause0、vm0、yaw_delta56.920166°、当前yaw56.1291504°、valid3；用户确认逆时针转动10秒自动停稳，随后静置一分钟yaw没变。本次接受单段逆时针随动/停止及用户观察期间稳定，不作绝对精度或长时间零漂声明。
原先未开外部VM开关时发生看门狗复位；用户开VM后重试先被7RPM速度门槛中止，改本轮10RPM门槛后通过。不能据此独立归因全部复位，仍保留既有记录。
已备份当时逆时针AXF：evidence/p1-foundation/2026-10-19/gongxun-f407-ccw-user-pass.axf。

## 当前顺时针版本
APP_HWT_TURN_TEST1、APP_HWT_TURN_CLOCKWISE1、APP_HWT_STATIC_TEST0。仍只执行转动段2，在生成原逆时针四轮命令后按配置反转四轮方向：全1改为全0，对应本轮驱动反馈应为正RPM、航向相对角预计为负。
5RPM目标、10秒共同运行窗口、±10RPM超速门槛、状态/反馈/方向/时限/HWT联锁、默认VM0、PC13取消、每次上电只一次全部保留。原运动学和非本模式三段不变。恢复逆时针可把APP_HWT_TURN_CLOCKWISE改0后重编译，不在Watch运行中切方向。

## 操作
1. 下载最新顺时针固件，确认VM外部开关开启、四轮CAN/供电连接正常、车体周围有转动空间。
2. 无线Debug点击Run，确认version_result1、valid_mask3，再将g_hwt_turn_request置1一次。
3. 应顺时针转动约10秒后自动停稳、关闭VM。约20秒后看key_state4、auto_result1、auto_cause0、vm_request0和yaw_delta。
4. Watch沿用上次列表，无需新增。报告实物方向、是否停稳及相对转角，再静置一分钟看当前yaw是否稳定。
运行中不暂停/单步/复位。取消用PC13或turn_request2，紧急断电。当前固件不自动校准，也不能触发cal_request3。默认请求0，上电不会自己动作。
角度按跨界累计，不直接用两个±180度显示值相减；不要求实际幅度恰等于上次56.92度，未做精确转角闭环或标定。

## 软件验证与未闭环
44项模拟检查通过：真实Safety、运动学、执行器和协调器，模拟CAN/电机/HWT/时间。验证四轮仅方向0/目标5、只有一转动段、10秒停机断VM、顺时针航向为负且跨界正确、默认静止/一次会话/取消、HWT和CAN故障，以及8/10RPM允许、11RPM拒绝、反向拒绝。硬件顺时针尚待用户验证，助手未下载或发物理命令。
Keil ARMCC5.06u7：{'code': 35604, 'ro': 452, 'rw': 1332, 'zi': 25284}，0错误0警告；Objects/codex-hwt101-wireless-clockwise-build.log。
检查R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04；R及官方Q不变。本轮开发E候选门槛不放行正式运动/定位，顺时针动态结果及精度仍待验。
