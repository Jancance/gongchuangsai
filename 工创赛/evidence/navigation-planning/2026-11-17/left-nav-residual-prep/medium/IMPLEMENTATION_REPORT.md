# 左向50%导航残差：medium实施与验证

2026-11-17。MATLAB正式合同前置已通过并获root授权后，仅修改app_config.h、app_odometry.c/h、app_icm_observer.c。产品当前冻结，未执行Keil整机构建、Git写入、下载/复位/运动或其他硬件命令；root负责后续high独审、完整构建和总交接。

## 产品行为

新APP_LEFT_NAV_RESIDUAL_ENABLE默认0，候选1；enable>1、enable1但APP_BASELINE40_ACTIVE=0各有编译错误。k=.0179055385f、固定weight=.5f，无主距gamma。成功原SetPose建立参考时，根据实际AppMotion_Live单动作表、真实AppDebugPlan40Allowed和非runtime_route锁存scope，只允许PLAN9左210/PLAN18左150；19左30不适用。要求LEFT_DISTANCE、102RPM、各准确距离、run/reference/relative均0。

navXY=rawXY+reference前轴*(.5*k*max(netPathLeft(raw),0))。原始raw已经按HWT实时实际航向投影；本候选不改HWT公式、不再叠命令角、无主距缩放。scope/frame不随active/DONE/liveplan改变撤回；ResetRecords保scope/XY但立即同时令raw/nav anchor0；失败manual保scope且原raw/nav anchor0，成功manual清scope；新成功参考重新判定。

保留s_odometry、s_baseline40.pose、ReadBaseline40、perimeter、capture和前向lateral控制的raw语义。新增同型ReadNavigation40，featureoff直接alias raw；g_chassis_odometry发布nav.pose，完整ICM observer只换一次getter使用nav轮锚。独立静态nav64B+scope4B，off无两新符号，既有struct/capture ABI不扩大。nav浮点计算在critical外，critical内完整copy raw后仅覆写XY并发布g，不增加64B局部临时。无valid条件退回raw；invalid冻结raw坐标继续同映射，valid/reasons/时间/age/updates/continuity等原样copy，不伪造健康。

初版静态复核发现ResetRecords/manual清raw anchor后nav缓存直到Step尾才更新的窗口；按root确认各加一处nav anchor镜像清零后再执行medium实C。本medium未在修前运行该负例，不冒称有修前实跑失败；根因和修正由root/high证据补充。

## MATLAB前置

正式脚本SHA3B6358D300CC6A937D13EA38364B2E1204B43F2AFE7CE885FE868EFCC9DB4FD9，R2020a exit0，20精确scope例/125生命周期断言及六条实际HWT分解轨迹、非零初始yaw/跨180圆弧投影、负净L/零位移、重复source、invalid元数据与raw/nav消费者分流通过。k三趟均值=.017905538496259973；左150不拟合。固定50%留出旁误差MAE7.10617mm/最坏10.21847mm，仍只是不同窗实尺的表观比较，不是可定位到该精度的承诺。详情REPORT.md和matlab_receipt.json。临时生命周期脚本/输出完整归档provisional_lifecycle_before_contract。

## 实际C与编译边界

check_navigation_c.py/actual_c_checks.json：ARMCC编译真实app_odometry.c、app_entry.c、完整app_icm_observer.c及未改真实kinematics/profile，Unicorn按SHF_ALLOC执行地址加载PROGBITS并清零NOBITS。nav_on/off各4639断言通过。原始里程计生产者、时钟、RTOS临界区、功率状态和动作状态为受控桩；不声称真实调度/CAN/硬件或本步重新验证原始里程计算法。

覆盖648个计划/count/kind/RPM/距离组合及run/reference/relative/runtime_route反例，真实SetPose分支scope锁存、重复source、DONE/liveplan不撤回、perimeter为raw、invalid完整元数据保持、ResetRecords未Step立即双anchor0、manual SetPose内部getter探测双anchor0、失败/成功manual恢复语义、负净L/零位移。完整ICM Step实际经过200样本静止校准与230帧流水，wheel_f/position_f等于仅一次50%残差的nav轮锚，R轴符号和有效状态保持，未再补第二遍。首C夹具漏kin/profile链接依赖，链接错误原样保留attempt1_c_missing_link_dependencies；仅补链接未改真实依赖后通过，非产品失败。

check_compile_guards.py/negative_compile_checks.json：默认0和合法1正编译通过；enable2按Left_nav_residual_enable_invalid、enable1/baseline0按Left_nav_residual_requires_baseline40_debug报错通过。

## 冻结身份与保护

product_source_manifest.json/product_source_diff.txt记录最终4源：config a4fe963a0373d4b4d9c8b1a1dfae62a29872ac16aa41e8a3fe08cb481e0cdea2；odC 9afbd5bb380e0498554622177384f73385be68f0c507d37839ecf8be5069001f；odH bfdd78e9ab9e52ad5ead220f7c1a6fe6878e186f9b085ab1c51207c2340c6a05；icmC 4db507517933936d9452e9ee9be61639325facf9a4e122dee0a6845018095217。GBK和CRLF260/196/42/240保留，git -c core.whitespace=cr-at-eol diff --check通过。

final_source_checks.json确认ReadBaseline40原函数字节完全不变、ICM全文件只有getter一处替换、21项未授权变化之外的源/输入SHA保持。source_before/protected证据继续保留，原6dirty总哈希由root统一核对。本阶段仅本地候选实现与离线证据，未提交/推送。

用户后续要求右1%/前/后各同工况三次并取均值，当前明确后面再测试；没有据此发车、补测试或改本轮只左9/18的scope。R核RUN-01/PWR-01/MECH-01/02；E核LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。