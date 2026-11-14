# S2.5 四趟停车/纵向距离 MATLAB 前置诊断

日期：2026-11-15。本报告的B0阶段仅保存既有四趟数据的诊断、冻结输入与 MATLAB 脚本/日志，没有增加第五趟。B0没有产品控制、参数、硬件、串口、下载或网络操作。其后root另行greenlight的176B仅观测产品在独立子目录[snapshot-candidate/report.md](snapshot-candidate/report.md)交付，不能把该诊断实现误称停车控制候选通过。Git 仅以 `git show 86ad3415:<path>` 只读提取本地已存在对象，未操作分支、工作区状态或远程；同步与提交由父任务负责。

## 结论

**B0 = FAIL / NOT_IDENTIFIABLE；B1 未运行，不实施停车前视候选。** 这不是说原版没有位置反馈，也不是说不能继续做适用分析。数学复算及四趟时序边界支持：末个完整批的 18～20RPM 与仍在下降的时间参考速度量级一致；原控制允许共同时间编码器进度先到 2100mm，随后直接 STOP，不等待时间曲线归零。四趟到首 STOP ACK 时，参考曲线仍至少约 0.7s 未结束。

但 PM1 快照漏了中间位置更新，且内部 `s_run_tick / follow_tick / follow_rpm / follow_rate`、到位分支的共同时间及批冻结时刻均未记录。不能唯一复原原状态和冻结到位值，不能把“落在可达外界内”冒充“最后 F 逐趟误差≤1RPM 的复现通过”，也不能据此接受唯一的末120mm/0.15s前视候选。

## 冻结输入与实际运行

- 原auto控制源来自本地对象 `86ad3415`，`chassis_auto_test.c` SHA256 = `5742f22ba37b9fc211c5129d6a8dab607fdf8f376e5bc6fcc31070ec18208c1b`，与四趟绑定源一致；排除了本轮浮点计时插桩对行号/源码的影响。**该commit不是S2.5完整checkout**：首版一并提取的其它6份历史依赖不能一概当四趟参数依据，其中app_config/motion_trial20确实没有当前baseline分支。旧原件保留于`frozen_source`，不作为这些数值参数权威；详见下条修正。
- 独审指出依赖绑定缺口后，补充 `frozen_build_bound` 与 `build_bound_source_manifest.json`：当前9份实际依赖逐字节匹配已保存的bounded preparation1731文件清单，kinematics c/h再与S25 build_summary的SHA匹配；S25 build定义明确启用baseline40，auto SHA和AXF也由该build绑定。实际150/300、follow2/Kp1及1RPM floor来自这些正确冻结的app_config/trial20/trial40，而不是历史依赖。数值spec没有变化；定向完整性检查通过。旧输入清单保留`input_manifest_before_dependency_correction.json`，当前清单为补充后的版本。若独立重新准备输入，须在`prepare_inputs.py`之后运行`freeze_build_bound.py`。
- 输入是 `baseline40-field/s25-run1..4` 的原始 `.raw/.jsonl`、XD派生记录、完整批CSV、尺量记录及已核四趟summary。完整路径、字节SHA256和用途见 `input_manifest.json`。
- 冻结参数：前进2100mm、轮径77mm、前进校准1.02182540；参考上限400mm/s；位置跟随Kp1、±2RPM、omega1；10ms更新门槛、dt上限100ms；接近floor **1RPM**；arrival2mm仅在profile已DONE后参与；track50mm；final_wait2000ms；run12779ms。没有使用错误的15RPM floor或把floor升到8RPM。
- 本机实际使用 `E:/Program Files/Polyspace/R2020a/bin/matlab.exe` R2020a。最终进程回执见 `matlab_receipt.json`，结果见 `matlab_results.json`。`matlab_native_stdout.bin` 保留原始stdout/stderr字节，UTF-8可读副本为 `matlab_native_stdout_gbk.txt`；prefs的migratePref权限/格式异常及toolbox_cache警告保留，没有通过改用户配置或假装MATLAB不可用来略过。
- 首跑因脚本自检的变量名 `mean` 遮蔽同名函数而失败；已修正，首失败日志/exit1保留。第一次成功数学诊断后，补充直接PE重算及原始字节日志，再运行最终检查；预补充成功回执同样保留，不计入原控制/B1通过。

## 原控制数学与规则顺序

实际链路是七段时间参考速度前馈，叠加共同时间编码器位置误差的低带宽跟随。并非“没有位置环”。MATLAB按源的single参数/相位状态以及double准备积分复算：

- 校准后 `factor = 4.119704247 mm/(RPM·s)`；它同时用于轮程标定和参考速度/位置跟随换算，不能在末端再乘一次相同比例。
- profile参考速度400mm/s，给位置跟随预留后，参考加速度133.521179mm/s²、jerk250.563553mm/s³；duration8.778662682s；七段边界为0、.532883465、2.995779514、3.528663158、5.25、5.782883644、8.245779991、8.778662682s。
- follower的精确临界阻尼ZOH递推经300个解析点检查，最大误差约8.22e-15；这是递推数学检查，不是原四趟状态恢复。
- `equivalent_progress = common_mean + current_ref_position - observed_ref_position`，所以 `desired = clip(Kp*(observed_ref_position-common_mean)/factor, ±2)`；current position在位置误差中严格消掉。脚本代数检查误差0。补偿是在相同时间比较位置，不是把过期编码器当当前轮程。
- **arrival判断在补偿和follow之前**：若补偿前、已校准的共同时间mean≥2100，立即置reached/finish/stop并return。只有profile已DONE时，2098mm的arrival容差才可触发。不能拿equivalent_progress代替此分支的mean。
- MATLAB仅确认原参考加速度/jerk加follower保守预留约150/300的数学预算；没有高频实际四轮数据，不能据此签真实轮端加减速度、地面jerk或停车效果通过。

## 时间零点和末段参考的可复算边界

TMQ的`elapsed_ms`来自外层动作begin，包含准备/使能/端点读取，不是内部profile elapsed。不能把它或首ACK当s_run_tick。源顺序只给出：最后一个起点PE取样先于run_tick，首个零速target_tx晚于run_tick。四趟内部起算区间分别为22612～22632、23695～23715、23834～23854、27094～27114ms，均宽20ms；这是顺序界，没有把任何端点当实测内部tick。

下面参考F以“最近一次已完成批末ACK至当前批首TX”作为批冻结时刻顺序窗，叠加上述起算窗，读取同一原profile。它描述冻结时刻的时间参考范围；若该调用未能取得共同时间插值而保留旧reference，真实锁存参考还可能更旧，因此不能用这个表代替内部state。±2RPM follower未知与整RPM量化只用于外界一致性检查。

|趟|实发最后完整F RPM|冻结时刻参考F范围 RPM|未知follow下外界F RPM|至少未保存的中间位置更新|
|---|---:|---:|---:|---:|
|1|19|18.189～19.162|16～21|31|
|2|20|19.259～20.231|17～22|31|
|3|20|18.902～19.875|17～22|34|
|4|18|17.476～18.449|15～20|29|

四个记录都在外界内，但都没有可报告的原状态复算误差；JSON用null保留未知，B0 tolerance_pass全部false。原profile在首STOP ACK时刻仍给约14.2～15.6RPM参考，且首ACK以前已经触发arrival。mean已领先当前时间参考至少约16.9～19.1mm；与更老observed reference比较只会更领先，具体值未保存。这给出了“时间参考尚未归零，编码器共同时间进度已到”的机制证据；不等于识别出唯一轮地plant或等效制动距离。

## 信息缺口与起点/末点不能混算

每趟运动期PM1约40个快照。按相邻快照的`sample_sequence`增量，扣除四轮快照中可见的取样变化，可严格统计至少31/31/34/29次更新没有保存其当时轮位置。XD的actual sample约500ms且位置量化，与PM1的32位原计数/位置更新不是同一完整数据流；OD1另有坐标、原点、HWT投影和稀疏采样，均不能补出被省略的四轮各`AUTO_POSITION_HISTORY=16`点内部插值历史。raw标签完整盘点也没有XF等隐藏profile状态帧。

仅用保存PM快照再插值，最后停前可重建的共同mean与到位冻结值分别还差−23.662/−11.307/−24.400/−15.011mm。它们发生在不同时间，**不是原算法的误差，也不是地面滑移量**；恰好显示最新保存快照并未覆盖到真正触发arrival的那次内部状态。脚本明确将其命名为saved_PM插值，没有拿此替代原控制四轮各16点的内部历史。

|趟|到位冻结共同轮程mm|最终PE校准轮程mm|停稳后尺量前进mm|PE－冻结mm|
|---|---:|---:|---:|---:|
|1|2104.655|2114.703|2105|10.048|
|2|2101.789|2114.395|2107.5|12.606|
|3|2102.141|2113.073|2105|10.932|
|4|2102.677|2113.541|2112.5|10.864|

MATLAB从原PE符号/计数、原起点逐轮直接计算终点，和已发布summary误差<1e-6mm；TMQ冻结值及CSV最后完整F也与summary交叉一致。PE各轮端点在首STOP ACK后1034～1067ms，是后采样轮程；arrival时尚有位置采样年龄、轮次不同步及批等待。尺量是停稳后车体测点，测点旋转/方向、轮地滑移与参考不同。不能把PE－冻结全部命名为溜车，更不能统一提前减11mm、再宣称地面更准。

## 最小补充观察建议（E，尚未在本目录实现产品）

要回答“STOP时原参考为什么仍非0”，不需要大环形缓存或新传感器。建议保持控制的三个只读快照：

1. 首次减速且补偿前共同mean≥1980mm，记录进入窗的原状态。
2. 窗内最近一次批冻结。明确它可能被STOP打断，不自动等同于最后完整ACK批。
3. arrival分支设置reached/return之前，记录新sampled reference和仍保留的上次follow状态。

每槽48B：`now_ms/run_tick_ms/common_ms/follow_tick_ms`四个u32（16B）；`common_mean/current_ref_position/current_ref_F/follow_rpm/follow_rate`五个float（20B）；`pending_F/applied_F/flags`三个u32（12B）。三个槽144B，再给session/validmask/freeze与arrival计数等32B头，总预算176B；必须以真实C结构sizeof/map核验，不能用估算替代RAM证据。arrival处follow尚未再推进，follow_tick正可区分最后更新时刻，故本问题无需单独第四个follow槽。

固定静态RAM、无动态分配、无新增UART TX或CAN轮询、只在原事件做有界赋值；异常/正常完成后保持到下一次有效start reset。下一阶段先MATLAB核布局/uint32回绕/reset保持/相位与validflags，再最小产品诊断实现和独审。它可以补上一次末段状态解释，不声称足以完整重建候选未来全轨迹、轮地plant或ground。

父任务随后确认只做这三个事件快照，并要求先做诊断前置验证。已冻结 `snapshot_spec.json` 并由同一本机MATLAB实际运行 `snapshot_diagnostic_checks.m`：字段偏移0/4/…/44、3×48+32=176B；uint32毫秒回绕、混合u32/float32记录字节、有效start reset/拒绝start保持、首次entry保留/latestfreeze覆盖/arrival锁存、arrival前旧follow状态保持、validmask/flags及原profile相位检查均通过。结果为 `PASS_APPLICABLE_DIAGNOSTIC_PREREQUISITE`，见 `snapshot_matlab_results.json`；该结果本身不包含C编译器的sizeof/map、实际M4耗时或硬件捕获。root后续另行greenlight，仅观测产品实现、C/ELF资源与三构建见独立的`snapshot-candidate`最终回执；B0不可辨识/B1关闭结论不变。

## 需求与交付边界

已检查R：RUN-01、PWR-01、MECH-01/02；E：LOC-01/02/03/05/07、MOT-02/04、RUN-06。现有控制/停止出口未变。Q：FIELD-05、OBS-03、QR-04仍待技术答疑确认；本报告的120mm、0.15s、±1RPM、采样与RAM方案全部为E，不是比赛规则。

未闭环：精确原尾段状态、独立轮地响应、STOP request瞬间进度与实际车体停稳同刻数据、地面距离比例及测点偏移。当前结论仅“本地保存数据的数学诊断已实际运行，B0不可辨识，B1没有获得实施许可”；没有签仿真控制候选通过、已下载或实车停车通过。
