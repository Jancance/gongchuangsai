# VM静态测量准备：本轮仅切换配置并Rebuild

用户确认+5度失败交易期间VM外部开关保持按下，不能归因松开强制关闭。此前Q2输入22V、开关按下1-2通/松开2-3通；本次F3 E2为条件/参数拒绝，不唯一证明低压。

只读实际main发现SESSION模式App_HeadingRuntimeSafety在闲置/失败会清g_vm_test_request，因此不要在该模式下强写VM/PE2。临时回到既有受Safety_Update监督的通用调试模式，方便下一步设计静态测量。此模式仍包含旧手动动作接口，不是专用的纯电源固件；不得发任何动作请求。

实际工程D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程，退出Debug、VM外部开关保持关闭。只在USER/app_config.h将APP_HEADING_SESSION_TEST从1U改成0U；APP_CHASSIS_KEY_TEST_ENABLE、APP_HWT_STATIC_TEST、APP_HWT_TURN_TEST均保持0U。其他所有代码/参数/请求不改，不删除Safety或直接调用Power_Enable。保存并完整Rebuild，回传完整日志；本轮不下载/不写请求。后续依据构建核对安排负载隔离及受监督静态供电检查，当前不放行硬件操作。

隔离目录tmp/p1_vm_static_mode_20261021/复制实际main/can及候选配置，两完整模块ARMCC编译无诊断，实际三个文件SHA未变。证据vm_static_mode_preparation.json；没有整工程链接、下载或操作硬件，下一步实际完整构建待用户提供。

R已核RUN-01/PWR-01；E已核LOC-03/05/07、RUN-06、MOT-02/04。官方Q仍待技术答疑确认。实际VM_EN/VM电压、拒绝原因、运动/停车/X/Y未闭环。此前正常Power_Init已有通过记录，不重复补初始化或擅自更换器件。