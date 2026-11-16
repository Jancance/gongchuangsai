# 2026-11-17 左向导航残差：high独立接入合同

状态：PASS_WITH_LIMITATIONS。本报告已完成正式合同、实际C与现有Keil产物独立核验；未下载或实测。仅写本报告及已授权independent-review新证据；不改产品/硬件/串口/探针/Git。读取HEAD90da4385，原6dirty保留，右1%软件03528388尚未装；用户暂停现场保持。

已读根和工创赛AGENTS、根长期记忆/交接及相关基线。R核RUN-01/PWR-01/MECH-01/02；E核LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑。本文全部残差、权重和适用范围属于E，不更改正式规则。

## 一、结论

可以在原始里程之外建立仅左单段调试范围的导航坐标残差层；不能改写原始轮/HWT里程或把修正再回写到积分器。当前已发现ReadBaseline40的控制/采样/observer共享，必须分开raw与nav读取语义。最小安全合同是：原ReadBaseline40保持raw，新增完整ReadNavigation40快照供ICM；g_chassis_odometry发布nav.pose作为导航显示；原电机纠偏与fusion_capture继续读取raw。定位算法、HWT实际航向、左电机2%和右电机1%及STOP/故障保护均不改。

## 二、实际消费者

- app_odometry.c:107..112：ChassisOdometry_Update后，目前g_chassis_odometry和s_baseline40.pose都来自s_odometry.status。
- chassis_auto_test.c:1343..1354通过ReadBaseline40读取，Auto_LateralTarget:238..260从坐标算前向动作的横向PD；s_lateral_enabled:1963..1966只允许前PLAN8/14/15，但原API仍是控制输入，不能悄悄换成nav。
- app_fusion_capture.c:129..145/184..188读取同一ReadBaseline40，把pose/参考写入原始捕获。若原API改corrected，历史和新RAW的OD语义会混淆。
- app_icm_observer.c:182读取同一API；204..211把车体F/L及场地XY转路线F/右R。当前anchor pF=cos(phi)*dx+sin(phi)*dy，pR=sin(phi)*dx-cos(phi)*dy。
- g_chassis_odometry全工程搜索当前只有app_serial_runtime.c:453读取输出OD1，无电机消费者。AppIcmObserver_Read当前只有IO1诊断消费者，也不进入电机控制。g_perimeter_records用于OP1显示，现记录raw，继续保留。

## 三、训练与一次性仿射映射

本轮新候选用三次新左210全部训练，旧的第三趟留出身份不再适用于这个新候选。每次ki=(groundF-modelF)/modelL，按用户要求取三个ki的算术均值，不偷换成总残差/总距离，也不混入左150。

独立端点算术核算：ki=0.0207056643465753/0.01631057365918832/0.0167003774830163；k=0.017905538496259973。lambda=0.5为用户已明确固定的本轮候选，effective gain=0.008952769248129986；不是统计置信度，也不是将总轮距乘0.5，不加入主向gamma。

设raw快照原点为o，固定路线起始航向为phi，d=raw.xy-o：

```text
Lraw_path = -sin(phi)*d.x + cos(phi)*d.y
corrF = lambda*k*max(Lraw_path,0)
nav.x = raw.x + cos(phi)*corrF
nav.y = raw.y + sin(phi)*corrF
nav.heading = raw.heading
```

使用max(净L位移,0)，不取abs、不累计每步绝对值；回弹时修正随净左位移回撤，净右位移不加左向经验项。每次都从不可变raw重新算，不用上一份nav作输入，绝不向s_odometry/wheel_mm/heading_offset写回。相同raw/reference重复调用必须同值，无重复sample漂移。主L保持不变；不按当前HWT角旋这个已在固定路线坐标中的残差，HWT实际姿态已用于原始XY积分。

左150只评分不训练：新半增益预测F=-17.718473/-16.448785/-16.598753mm；相对最终尺量的表观F误差-10.218473/-10.198785/+0.901247mm，主L不变。三150数据此前已看过，不能把这次评分宣传为严格盲测/事前预注册。共同源末与停后尺量不同刻、尺量误差未知，不能称毫米精度或已验证整场定位。

## 四、最小发布与observer合同

1. raw权威仍为s_odometry.status、s_baseline40和ReadBaseline40，capture/前向PD/原perimeter记录继续原调用。新导航快照可用同一AppOdometryBaseline40结构，不增加复杂融合状态；CAN owner基于同一raw pose/reference生成nav，两者按一个临界区发布。
2. 新ReadNavigation40复制完整pose/reference/anchor_valid，ICM仅将182行读源换成此API。observer的既有pF/pR变换和两锚速度自然包含残差；禁止在Obs_Predict输出或IO1显示时再次加corrF。ICM加速度、重力、校准、NO_LEVER等原逻辑不因坐标经验项而改变。
3. g_chassis_odometry可发布nav.pose，因当前只用于OD1显示不会回流电机。raw capture与OD1含义从此不同，必须在候选manifest/报告明确；未来导航消费者应读取完整nav快照并同时检查anchor_valid与pose.valid，不能把单独OD1.valid误当全局定位已锚定。
4. 非左范围、宏关闭时直接复制raw，避免无必要三角运算产生数值差。scope只允许APP_BASELINE40 debug、PLAN9/18、action_count=1、LEFT_DISTANCE/mode1、102RPM和精确2100/1500mm配对、非runtime route、run/reference/relative均为0；默认宏0/候选1，非baseline开启应编译拒绝。
5. 不能用motion.active逐周期开关残差：DONE后active清0会使nav突然退回raw。应在成功SetPose建立本次reference时从不可变动作表锁存scope，持续到下一次成功自动reference重新锁存，或成功manual SetPose清scope；ResetRecords和失败manual不清scope/frame。不要在reference尚未建立时依赖可能仍未提交的backend mode/base字段推断本轮范围。

## 五、有效性、失效和生命周期

- raw valid/reasons/continuity_lost/local_origin/age_ms/sample_ms/updates必须原样保留，不把publish时间now充当新样本，不清gap或假造valid。导航可用必须有当前完整reference及原raw健康，经验映射不能补回失去的连续性。
- raw失效但有限XY冻结时，在同一reference下仍以相同纯仿射映射显示冻结nav XY，并保留valid0；若仅if(raw.valid)才应用，失效时会撤回残差跳到raw，这是应避免的实现缺陷。
- anchor_valid=0时nav也必须anchor_valid=0，observer必须走原ICM_OBS_REFERENCE/clear路径；不能沿用上一参考的修正锚作健康输出。ResetRecords仅同步使raw/nav anchor失效，保旧scope/frame与已补偿XY；manual失败也保旧scope/frame，原raw请求流程仍按既有行为清anchor；manual成功才清scope。不改原raw积分器的既有重置语义。没有参考时仅可明确作为原始/局部读数，不宣称修正导航已就绪。
- AppOdometry_Init必须清新nav快照和scope，不保留前次调用的有效导航锚；原raw初始化语义保持。
- 新ref的origin/path_heading/heading_offset/reference_set_ms必须与nav pose原子一致；新原点L=0时corr=0，不带上次修正。observer现session/ref/continuity切换清历史继续保留；重复同sample_ms不追加第二个锚的分支继续保留。
- NaN/Inf的原点/角度/输入或计算非有限不得写出有效nav锚；只使导航输出不可用并沿既有observer拒绝，不能为此新增电机停车条件或伪造raw健康。

## 六、候选后置验证要求

MATLAB先行后再实现。覆盖phi=0/±90/跨180、非零原点、L=0/正/负/回退、重复输入100次、valid到invalid冻结、新ref/anchor invalid/手动SetPose、所有计划与mode及single/multi/route负例。检查raw输入未变、非左逐字节一致、heading与valid/时标未改、L不变、修正只一次；observer两修正锚的pF正确而pR不多加，重复样本不新增锚、invalid/ref变化清历史。

特别核对至少一个实际app_odometry发布与AppIcmObserver调用路径，不能只验证独立公式；证明前向PD和原始capture仍走rawAPI。保留与前候选相同的实际C/on-off/适当Keil构建证据，但软件通过不代表已下载或实车。右1%和原左2%命令输出及STOP/故障路径应无差异。

前置结论已由下列后置实测软件证据收口；现场暂停，不产生下载/复位/发车授权。

## 正式合同更新与资源核查

ROOT正式合同覆盖早期建议：仅PLAN9左210/18左150，不含19/300；k为逐趟ratio均值.017905538496259973，固定lambda.5；max(netL,0)；保原raw/perimeter/capture/ReadBaseline40；新增同64B结构ReadNavigation40供ICM，g_chassis_odometry=nav.pose，observer仅更换一次getter。ResetRecords只清两锚有效性、保scope/frame等待成功reference；失败manual仅scope保旧，原raw anchor失效语义不改变。旧plan_xhigh中ratio-of-means/19/三出口nav方案须标明已被此合同覆盖。

正式MATLAB receipt为3B6358D300CC6A937D13EA38364B2E1204B43F2AFE7CE885FE868EFCC9DB4FD9，R2020a exit0，20scope/125生命周期；旧provisional invalid/rawchanging cache冻结不能作为本轮纯仿射实现证据。

资源基线：AppOdometryBaseline40=64B（status40+reference24），新增static nav64+scope4合68B，不向原结构追加字段。原observer已有64B wheel局部，只换getter无必要再加64B局部。CAN分配384words=1536B、IcmRaw256words=1024B；旧right1 Keil callgraph AppOdometry_Step frame136/max260B，observer frame328/max504B，IcmRaw任务max592B，CAN任务max1024B。新发布宜只用navX/navY及三角标量，在临界区复制raw snapshot并覆盖XY，避免64B临时对象；三角计算放外面，完整快照在一个临界区内发布。新版真实frame/map仍须构建后核验，旧值不是新版实测余栈。

## 最终独审结果：PASS_WITH_LIMITATIONS

采用ROOT正式合同，冻结app_odometry.c SHA9afbd5bb380e0498554622177384f73385be68f0c507d37839ecf8be5069001f。初审发现ResetRecords只清raw anchor而漏nav anchor的真实缺陷；已由medium在两个原raw anchor失效位置补齐nav anchor立即0。未清scope或撤回XY；本代理未改产品。

独立ARMCC/Unicorn使用byte-identical完整app_odometry.c、chassis_odometry.c、app_icm_observer.c，实际执行AppOdometry与ICM Step；RTOS锁/时间、轮源、电源、动作owner接口作stub，使用实际typedef与当前k/weight宏绑定。off176/on177条断言通过，各16个真实成功reference scope及4个observer场景。覆盖原始/导航getter、100次重复发布、raw perimeter、gOD1导航输出、DONE/live PLAN变化保持scope、负/零netL不加补偿、invalid冻位与所有时效/heading/valid字段保持、ResetRecords立即raw/nav锚0且XY不回跳、manual失败保scope/成功清scope、新左150及新右reference。

实际observer真Step先200样本完成静止校准，再用轮/HWT与ICM源序列。开启时rawF=-30.651187897mm，navF=-17.221988678mm，observer的wheelF与predictF均=-17.221988678mm；关闭时四者全为-30.651187897mm。证实修正只进入一次轮锚，预测末端不二加；同ICM sample重复调用不新增发布，同wheel sample不追加第二锚，raw失效按既有BAD_WHEEL拒绝。此为合成源执行证据，不是新实车或独立地面真值。

ABI与控制：AppOdometryBaseline40实际C sizeof仍64B；原getter及raw/perimeter/capture语义保持。before中其余10个相关源逐字节不变，包含capture/capture头/前控/动作/串口/原里程和右1%运动学及预算；ICM源除一次getter替换外字节相等。capture仍两次raw getter、前控仍raw getter；不通过nav操纵电机。四改源GBK/CRLF、bareLF0，原6dirty哈希保持。git diff --check需以cr-at-eol按原CRLF解释，无需转换文件。

资源实证：独立ELF与主代理完整map均见nav64B+scope4B，无多余64B局部副本。新版Keil AppOdometry frame152B（off136，增16），max276（off260）；observer frame328/max504与IcmRaw任务max592不变，CAN任务max1024不变；原分配CAN1536/IcmRaw1024B。callgraph静态值不是实测余栈或对中断/未解析路径的完整证明。捕获bank仍34020B@0x20017300、33600B@0x10007300，未变结构/固定地址。

独立读取并验证root执行的on/off Keil回执、0错0警告日志及实际AXF哈希：on双宏= c51d57e2234ed0af7cf7a6c1128ca90088592f912147e7a16dcd20a875efef82；off左nav/保右1=34b28982c868b4bbc51a86c05a838c05b4587f70d51103aa3855861c4d9cf354。旧右1冻结3043c4a98aed1b04eb0c80fd4b2ea819b2c70b52a2c0dbe60613a969bca48ad3哈希保持，未覆盖。

独审夹具首版生成字符串语法、用Init冒充ResetRecords、增加DONE步时漏IMU采样导致真实gap保护拒绝，均保存失败脚本/报告；只修独立夹具，未放宽产品或断言。最终证据independent-review/{actual_c_report.json,source_resource_checks.json,build.json,unit.c,run_actual_c.py}。未执行调度竞争、CAN电气、固定地址硬件采集或物理定位验收。

最终未见阻断本地软件候选的确定缺陷。主向不gamma、HWT不改、左2%/右1%/STOP及故障能力保持；三210训练/三150回顾性评分不能签地面精度或全工况。硬件持续暂停，新双宏版与旧右1版均未由本轮下载/复位/发车。
