# 黄色失检原始包复核

来源：香橙派独立诊断20261105-140230-265860。经用户启动指定目录HTTP服务后仅取此ZIP，保存于evidence/p2-camera/2026-11-05/yellow-audit/。4740624字节，SHA256=2f59b8e4ef1c72d701c50a1d1822c360d25f8b494f3c2835258ee66b2e5559aa，ZIP内部CRC通过。已通知用户Ctrl+C关闭临时服务。原图已查看，检测器/follow/tracker/camera四模块哈希与原lightblue1发布包吻合。

356帧中FOLLOWING_PIXEL308、SEARCHING4、STALE_RESULT44；有效目标308个均blue、yellow0。first-yellow-with-blue事件实际只是“监视黄色时出现蓝色”的命名，不证明黄色曾检出。帧率为主机检测/显示吞吐，不当曝光帧率。诊断没有逐帧读回曝光，用户设置100及截图有效，但不能给所有原帧补造曝光100。

右侧黄色对应候选出现INSUFFICIENT_NONCLIPPED_PIXELS170次。sample01/02/03右侧圆面bbox约(1469,186,160,170)，usable_fraction均0；bbox中V>=250占93.6%～94.0%，HSV中位H30/S120～121/V255。主代码有效颜色证据使用V<250，导致该候选未进入颜色判别。黄色H/S有证据，但不能仅凭框内中位数宣称颜色算法最终必过。原图红色触底，对应TOUCHES_BORDER拒绝，需将几何视野问题与黄色亮度问题分开。蓝色框为整体投影，不是已标定抓取中心。

下一E对比：独立复制现有视觉程序，摄像机profile只改exposure_absolute50/gain32，保持分辨率、算法、HSV及150ms过期拒绝。使用scripts/prepare_yellow_exposure_trial_20261105.py；脚本拒绝覆盖已存在目录，不开相机/串口。先同场景观察较低曝光是否让黄色有效像素恢复；不直接提高削顶阈值或伪造黄色坐标。该配置尚未在香橙派安装，需用户终端执行。原始版本和曝光166对照保持。

复核脚本scripts/review_yellow_audit_20261105.py输出review.json。临时服务未作为SSH或相机控制通道。只读下载、离线图像分析没有机器人命令。

已核R VIS-01、QR-01、DSP-01、RUN-01/02/04/07、PWR-01、MECH-01/02及E VIS-09/12/15、CAL-02/03、RUN-06。规则不变；曝光和拒绝阈值为E，官方未明确Q仍待技术答疑确认。未闭环：较低曝光新帧、黄色识别与其他色保持、红色完整视野、持续时效、毫米/抓取链。
