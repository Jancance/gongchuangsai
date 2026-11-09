# 第1→2段本轮功能完成，下一项原料区视觉

> 设备连接更新：用户已确认香橙派/相机上电联网。读取现有RealVNC保存连接Orange Pi 5 Pro得到10.252.117.211，与旧工程记录相同；当前电脑10.252.117.235，同网段。TCP22已返回OpenSSH_8.9p1 Ubuntu标识；orangepi用户现有BatchMode认证失败（publickey,password），未执行远端命令/打开相机/启动视觉或运动。主机密钥保存到本轮证据专用known_hosts；待用户提供SSH认证方式。详见evidence/integration/2026-11-05/vision-intake/connection-status.json。

## 本轮实物结果与范围

用户在左上角短路径回0版本部署后反馈“没有问题，进入下一步”。`evidence/integration/2026-11-04/integration4-fix/leftup-return-integrated-run/`末端VM2 run1/stage2/state4/cause0、startup4/0、arm2/0；EX3 hold_enable1/samples3/confirmed1、first_reason0；SP1 state3/cause0/mask7、segment2、encoder推算70.052mm、夹爪命令180°，准备耗时3258ms。源区SF1为空。接受该轮回零、第1→2段和源区并行准备功能完成。用户“没有问题”不扩写为准确70mm尺量、夹爪真实到位传感、负载/重复性/比赛180秒通过。

本轮不因进入下一步重新烧录或要求重复跑已通过项目；保持停车与机构姿态。当前未执行抓料，库存及正确抓放次数不因该准备完成增加。

## 选择的下一项（E）

先检查原料区视觉观察链：实际在线设备与相机身份、实时画面/帧新鲜度、物料颜色/目标实例、任务码匹配和坐标单位。此处原料盘/物料目标不能套用粗加工圆环坐标流程。已有代码包含Mode.MATERIAL_PICKUP及Mode.TURNTABLE，具体选哪个要根据实际拍摄对象和既有现场运行程序确认。

当前本地`vision_app/code/config/project_profile.json`仍tracking_units=pixels、geometry_verified=false、serial_enabled=false、follow_grasp_enabled=false；主相机ID和实际安装几何未确认。毫米接口`software_interfaces/stage2_metric_alignment/profile.unconfigured.json`仍calibrated=false，ID均0、motion_enabled=false，不将旧135°像素数据当毫米，也不凭历史尺寸宣称相机已标定。毫米接口尚未挂接实际串口/main或STM32。需要线上观察和本机变换验证后，才能进入纠偏和抓料。

本轮直接运行：任务码/物料/事务相关33项Python测试通过；`scripts/check_metric_alignment_interface_20261104.py`输出C137检查0失败（另有Python13项记录见其结果）。这是当前源码离线检查，不是实际相机/串口链路通过。

已向用户确认香橙派和相机是否上电联网；当前未获得新鲜IP/设备连接和画面，尚未连接设备或操作相机。后续需实际IP/连接入口，先只读核对部署和观察程序，不猜旧地址或修改摄像机设置。

已核R：QR-01、DSP-01、VIS-01、RUN-01/02/04/07、FIELD-02/06、PWR-01、MECH-01/02；R不变。视觉接口/验证顺序/当前限制均E；官方未明确Q继续待技术答疑确认。未闭环：本机视觉输入与在线握手、几何/毫米和时效、目标对准、单件抓放/载台确认及后续阶段。
