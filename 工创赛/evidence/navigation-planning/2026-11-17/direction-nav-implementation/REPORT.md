# 分方向导航旁残差已接入软件（2026-11-17）

用户授权应用已分析的50%导航候选；本次不加前进0.5%左移电机前馈，不改变主距倍率。新宏APP_DIRECTION_NAV_RESIDUAL_ENABLE默认0，新隔离候选1；原左宏独立保留。仅app_config.h、app_odometry.c变化，GBK/CRLF及非ASCII字节保持，原6dirty不变。

严格成功reference锁存：原左9/18保持，新右10/前8/后11仅单动作2100mm、正确方向/rpm、run/ref/relative0且非runtime route；右另要求现有1%前馈宏启用。150/300等未验证距离不自动套新系数。沿reference净主距计算旁残差，负净距钳0，不按getter次数累积；成功新reference重锁，manual成功清scope、失败保留，ResetRecords同时置raw/nav anchor0但保旧scope/XY，invalid状态不伪造健康。仅nav发布XY改变，原raw/capture/perimeter/控制器保持，ICM现有getter只使用nav锚一次。未增加状态对象或保护停车条件。

MATLAB候选/留一及五组reference旋转检查已在产品编辑前通过。实际AppOdometry/原AppEntry/ICMStep ARM C开、关、left_only各4744断言通过，覆盖新方向、负/零净距、非法距离、路由、valid元数据、旧scope与ICM单次修正。夹具raw生产者/RTOS/时钟为桩，不等于全RTOS/实车。首C尝试新增用例污染后续ICM初始状态，证据保存在attempt1_fixture_state；恢复原左150初始状态后通过，未改产品或断言阈值。没有新独立代理审查，不冒称完成。

隔离Keil启用/全关两候选完整构建0错0警告，frozen/nav_on与nav_off保存镜像和工程。启用镜像fe7ae24bf7a0cda1cac42343301e677be9199bb1a9f3e17564795feb70b3b191，尚未下载；板最后仍3043...8ad3。未复位、未发车、未改旧field绑定。真实栈高水位/时间/地面定位精度待实车验证。

用户后续用途是将估计坐标用于地图。当前独立测试计划仍重建origin、同源数据有延迟且停后可能invalid冻结，因此本次不能声称全场连续即时真值。后续需统一地图原点/初始朝向，连续动作不清位姿，保留sample时间、valid/reasons及失效处理，再做转后直行/L形与跨距验证。原始主距模型当前比尺量平均多估右13.40mm、前5.95mm、后9.66mm，主距并非已准；先保留现倍率，不与旁向和电机控制同时调。

R已核RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04待技术答疑确认。raw模型与地面末点不同步，校准不等于已独立识别纯滑移。软件本地commit，不fetch/push。
