# 2026-11-17 左移旁向经验残差50%接入导航软件

## 已实现与部署状态

用户明确：HWT实际航向可靠；先将它已解释部分以外的左移轮地平移等经验残差归入导航；同工况每组3次取均值，最终固定此前讨论的50%候选权重。此处50%是残差补偿增益，不是统计置信度，也不是总轮距打折。

本阶段完成4个活动CHASSIS/USER源的最小实现、MATLAB前置、medium/high实际C和双配置Keil。候选默认宏关闭，隔离测试配置开启。软件已接入导航坐标发布和ICM短窗锚点；没有下载、复位或发车，板最后安装仍79e74...fa2f0d。右1%前向补偿软件03528388及其冻结3043...8ad3镜像保持；硬件暂停及已备field门关闭保持，不自动恢复试验。

## 计算合同

原始坐标Praw每一小段仍由四轮前/左位移和对应source时间的HWT实际航向，按二维中点+sinc变换累计。转向欠角自然进入此变换，不另加固定0.2度，也不重算/降低HWT信任。raw里程上下文和原始记录不修改。

训练只用新同版左210cm三趟，F前正、L左正。每趟先算ri=(实测F-完整轮/HWT模型F)/模型L，再作等权3次均值：

- ri=0.0207056643465753 / 0.01631057365918832 / 0.0167003774830163。
- k=mean(ri)=0.017905538496259973，产品float常数0.0179055385f；不是另一个曾讨论的均值之比算法。
- lambda=0.5固定，不用左150留出反调或重新选权重。
- Lnet=-sin(phi)*(Xraw-X0)+cos(phi)*(Yraw-Y0)，phi为本参考的前方向，原始Praw已经包含实时HWT。
- cF=lambda*k*max(Lnet,0)，Pnav=Praw+[cos(phi),sin(phi)]*cF。

因此每估计净左移1m修正约8.95277mm前向导航坐标；估计1500/2100mm时约13.429/18.801mm。系数和增益固定，幅度随原始估计净左距变化；不是每趟硬加相同毫米，不自动学习，不取上次修正坐标再次叠加。只调整这项旁向残差，不应用原候选主距gamma，不改变HWT heading或电机动作。校准只适用于当前单段左移参考，不能泛化为所有工况的独立滑移真值。

作用域在成功建立新reference时锁存：APP_BASELINE40活动调试、新APP_LEFT_NAV_RESIDUAL_ENABLE=1、PLAN9左2100或PLAN18左1500、单动作LEFT_DISTANCE/102RPM、相应正确距、run/reference/relative全0、非runtime route。未三次验证的PLAN19左300不启用；前/后/右/转向/路线均不套左系数。scope不因DONE、STOP、live plan字段变化而即时撤销；只有成功新reference重锁，明确成功manual SetPose清scope。

## raw与导航输出分离，避免重复修正

| 数据出口 | 本轮语义 |
| --- | --- |
| s_odometry、AppOdometry_ReadBaseline40 | 原始轮+实际HWT，字节/合同保留 |
| capture的OD序列、轮/HWT原件 | 仍原始，不换成修正值，不变布局 |
| perimeter记录、前向现有controller | 仍原始，不反馈新残差到电机 |
| 新AppOdometry_ReadNavigation40 | 同原64B快照结构，nav.pose含一次旁残差；宏关闭时返回raw |
| g_chassis_odometry/OD1显示 | 发布nav.pose；heading/sample_ms/age/updates/valid/reasons/continuity与raw同源 |
| AppIcmObserver_Step | 唯一改动为读取Navigation40锚点，不在预测端再补一次 |

ICM仍是现有轮锚短窗预测，不是新增独立XY真值或新Kalman滤波；NO_LEVER/动态重力/有效窗限制不因新残差消失。原始里程和新导航可对照，右1%电机前馈与导航残差是两件事。

ResetRecords及manual请求将raw/nav anchor立即同步为0，但保旧scope/XY，不让下一CAN Step之前继续喂旧健康导航锚，也不把坐标退回raw。失败manual保旧scope/frame但保留原raw anchor无效语义；成功manual属于显式新reference才清scope。raw失效冻结时仍对冻结XY作同一仿射投影，元数据原样保持invalid，不伪造健康；重复source/getter不累计。新增nav64B+scope4仅特性开启，投影在critical外只用标量，critical内完整快照发布。

## 验证证据与发现修复

GPT-6.1-sol xhigh规划、medium脚本/实现、high独立实际C角色执行。

MATLAB R2020a正式脚本SHA3B6358D300CC6A937D13EA38364B2E1204B43F2AFE7CE885FE868EFCC9DB4FD9，06:49:36–46 exit0，早于产品写批准。20scope、125生命周期断言以及非零参考/跨180/净左负值夹0/重复幂等/raw不变/invalid元数据等通过。旧provisional缓存模型完整归档且不作为最终合同证据。

训练左210三趟、左150三趟仅回顾性留出评分，数据此前已看过，不宣传事前盲验证。左150旁向表观MAE：无修正20.0601mm、50%7.1062mm、全量7.0494mm；最坏23.7676/10.2185/14.4569mm。50%和全量平均几乎相同而50%最坏较小，是工程候选而非统计精度保证；主L不缩放。模型共同末窗和最终尺量不同步、摆位/尺量误差界未知，数值不签地面厘米/毫米精度。

medium实际ARM C nav开/关各4639断言，含scope648组合、raw/perimeter合同、真实ICM Step校准200样本与230帧一次锚；底层raw producer/RTOS/power接口为桩，边界如其报告。默认0/合法1正编译、非法2/非baseline开启负编译均通过。

high独立编译byte-identical AppOdometry+原始ChassisOdometry+真实ICM Step，关闭176/开启177断言、scope16/observer4通过。rawF=-30.651188mm时，开启navF=-17.221989mm，ICM wheelF与predictF同为后者，证明一次注入；关闭四者同raw。独立重复getter100次、失败/成功manual、新左/非左reference、DONE、负/零Lnet、invalid和Reset即时anchor等通过。

初产品实现曾漏ResetRecords和manual清raw anchor时立即同步nav anchor，root与high检出并在冻结前修复；最终odC SHA9afbd5bb380e0498554622177384f73385be68f0c507d37839ecf8be5069001f。medium未在缺陷版执行C，不伪造该版失败；high的夹具初始record状态/漏采IMU造成gap等首失败按报告保留，修夹具而未放宽产品保护。

## 构建和资源

正式入口仍3.stm32f407工程/gongxun-f407.uvprojx。隔离工创赛/tmp/left-nav-residual-build-20261117/：left_nav_residual启用左nav50%并保右1%候选；left_nav_residual_off关闭左nav、其余同配置。两者Vehicle_Integrated完整Rebuild各0错误0警告。

- nav开AXF c51d57e2234ed0af7cf7a6c1128ca90088592f912147e7a16dcd20a875efef82。
- nav关AXF 34b28982c868b4bbc51a86c05a838c05b4587f70d51103aa3855861c4d9cf354。
- 原right1 frozen3043...8ad3镜像未覆盖，field旧暂停绑定未静默重指向新候选。

新增static68B；AppOdometry_Step frame136→152、静态max260→276B。ICM observer frame328/max504、IcmRaw任务592/1024B、CAN任务1024/1536B保持；调用图有未知间接调用等限制，非实车高水位测量。Baseline40结构仍64B，capture A34020@0x20017300、B33600@0x10007300地址/尺寸保持。见link_layout_stack_checks.json；frozen/nav_on与nav_off保镜像、工程及旧已验证DAPLink选项，均未下载。

## 后续与边界

硬件暂停继续。用户要求以后右1%前向试验、前进、后退每个同方向/距离/版本/载荷工况3次，记录每次原话和位移，取三次残差均值再乘50%作坐标候选；原始3值都保留，补偿后另用新一组验证，不从训练误差宣称效果。右1%是电机动作前馈，不能用此50%增益再自动缩成0.5%，也不套左系数到右/前/后。当前新导航只覆盖左9/18，其他方向待足够完整数据再独立处理。后退仍未测；前210第1只有串口/尺量、RAW缺失，不能计成完整RAW模型。

本阶段只改4源config/odometryc/h/icm.c，原10源、右1%/左2%运动、HWT、时效/STOP/故障处理、原6dirty保持。原GBK/ASCII编码、CRLF和非ASCII字节保留，不fetch/push，每完整阶段本地commit。

R核RUN-01/PWR-01/MECH-01/02；E核LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑。未闭环：新导航的同期地面/途中定位与实际收益、全局连续任务、左30外推、其他方向残差、ICM杆臂/动态重力/物理延迟。软件通过不等同已装、实测或全惯导精度验收。

证据索引：root_formal_contract.json/plan_xhigh.md、medium/IMPLEMENTATION_REPORT.md/matlab_receipt.json/matlab_summary.json/actual_c_checks.json/negative_compile_checks.json、high_review.md、independent-review/actual_c_report.json、build开关回执、link_layout_stack_checks.json、root_preservation_checks.json、before/原始源和frozen/候选。

独立验证夹具的抽取型头文件含生成时额外CR序列，实际C编译/验证即使用这些原字节；保留编译证据与哈希，不为消除格式提示事后转码。产品4源/脚本/文档另经whitespace核查通过，见generated_fixture_whitespace_note.json。
