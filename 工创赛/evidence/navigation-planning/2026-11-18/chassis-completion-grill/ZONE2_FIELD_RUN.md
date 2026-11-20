# 区 2 雷达底盘现场验收记录

工程为 `3.stm32f407工程/gongxun-f407.uvprojx`。场上载物和全部伸出件计入实测车体。路线格号为 0→2→14→10→22→14→10→22→0。记录每站停稳及最终返回 300×300 mm 区 2。计时从 PC13 启动，包含雷达展开、升降、扫描和收拢。对照 RUN-07 记录是否在 180 秒内结束。

## 本地候选

当前源码的区 2 升降为 977 µs/半步，按 4096 半步/圈估算约 15 RPM。下降、回升各 3800 半步，名义每程约 3.71 秒。GRID_SCAN 在 ACK 后有新增有效圈数才延长等待，最长 15 秒，完整 GRID 到齐即回收。下表是历史候选。

两份候选都从同一主工程的 Integrated 和 Offline 目标构建，四项均为 0 错误、0 警告。现场使用 Integrated，AXF 与同目录 MAP 配对。

| 入口 | `工创赛/tmp/` 下的构建目录 | Integrated AXF SHA256 |
| --- | --- | --- |
| PC13 启动一次两点校准和扫描 | `zone2-field-recovery-calibration-20261119/Vehicle_Integrated/build/` | `3ae6a79684cf8a4d8953c990940e2f9c204af2f4c2a19809e066fa1acf418002` |
| PC13 启动区 2 八段 | `zone2-field-recovery-navigation-20261119/Vehicle_Integrated/build/` | `fc32d3cef56bb1714ee8b3b593e48e211a4c238fe0e88068a95771054c3f0d26` |

同目录 MAP 的 SHA256 分别为 `d459c3e8dfc2cafcf9bd7fa973adc3a0f9eb9899013823aa90aaf595bff84710` 和 `24d6b0b8516a8d1bd967c18d853fb110d13f3c63e35912e3370f68bd779d9cda`。两份 `receipts.json` 在各自构建根目录。以本表为现场候选，较早构建保留作历史。

## 校准扫描与切回八段

1. 正式工程的两个 Keil 目标均已配置区 2 导航宏。现场选 `Vehicle_Integrated`，在「00_现场参数」打开 [vehicle_parameters.h](../../../../../3.stm32f407工程/USER/vehicle_parameters.h#L4)。第 4 行去掉 `//` 即编译校准版，加回 `//` 即编译八段版。每次切换后保存并重编。校准开关由该文件控制。
2. 现场获准后按两点校准规范摆好两根固定校准柱，确认选区 2、升降位于顶部、舵机收拢且有展开空间。校准版上电后待机。按一次 PC13，提交一次校准，成功且机构收拢后再扫一次 GRID。升降每程 3800 半步、步间隔 977 µs，先核起步和实际扫描高度。舵机初始化和升降输出均在按键后开始。舵机初始化会给两个通道下发各自的初始角命令，实体收拢起点需现场确认。
3. 校准和扫图都完成、机构收拢后短响一次。雷达事务失败时，机构状态允许则自动回收，收拢后连续响 3 秒。机构停在中途时用 Watch 查看首错。记录 `g_radar_app_calibration_result` 和 `g_radar_app_scan_result`，值 1 表示请求提交成功。校准由扫描前的 calibration_complete 和保存回执确认。扫描终态核对本次 GRID、stage 8、error 0 与实际收拢。
4. 校准后换八段版，撤下两点校准柱。用户确认首趟直接按正式布置重摆障碍并验绕行。确认起跑姿态后按同一 PC13。八段使用雷达已保存的校准，并重新扫描障碍取得当趟 GRID，然后请求并执行对应 PATH。

现场先确认两点校准事务和保存回执，再用障碍照片与实测位置核对八段版当趟 GRID 的占用格。若有漏检或误检，记下障碍坐标、格号和该次扫描回执。把每段 PATH 与录像对照，并记录八站停稳、整车投影回到区 2 和 PC13 至结束总用时。

也可从仓库根运行以下命令，按现场参数文件的当前状态隔离构建两个目标。每次指定一个尚未存在的输出目录。

```powershell
$build = '工创赛/evidence/navigation-planning/2026-11-18/chassis-completion-grill/p3-observer/build_targets.py'
python $build '工创赛/tmp/zone2-field-new'
```

记录缓存约覆盖前 204.7 秒，若后续仍在运行，满缓存只停止追加周期样本，末条保留终态。记录中的坐标可对比录像，样本周期无法直接证明采样间隔内的最小净距或物理停稳。32 KB RAM 的板端读回、CAN 任务实际栈余量和槽耗时仍待现场核对。

## 单趟记录

1. 登记实际使用的 Integrated AXF 哈希、同一构建的 MAP、雷达校准版本、HWT 安装方向、车体实测外廓、电池和载物情况。确认区 2 起始站位、机头朝向和车道尺寸，记录障碍物坐标和照片。
2. 开启固定机位录像，从按键前拍到车辆和机构都停止。可用已标定的四个场地角点对齐录像和场地坐标。逐站记下停车时刻、车体投影和是否进入下一段。如果提前结束，先记第一次异常、停轮和断电的现场观察。
3. 诊断构建启用 `APP_ZONE2_POSE_TRACE_ENABLE=1` 时，车上记录场地 X/Y、航向、采样时戳、阶段和完成站数。车辆实际停稳后、主控保留供电时，用与板上固件配对的 AXF 及 MAP 读出冻结轨迹。原始二进制一并保留。实际用时和停车位置以录像和人工标记交叉核对。

区 2 起点为 (150,150) mm，车头朝 +Y。模型的 283×254 mm 是历史轮组近似值。静态摆车时应从底盘定位中心量到装机载物后的前、后、左、右四侧最外缘，并实测场地边线和 300 mm 回区线。按当前标称外廓和姿态，前后两边各约 8.5 mm，起步和回区录像应同时拍到这两条边。

## 导出轨迹

下列命令用于现场验收阶段已获准的停车后只读采集。分别替换 AXF 路径、当前探针 UID 和新建证据目录。表格中 X/Y 单位为毫米，航向单位为度，`elapsed_ms` 是启动后的记录时钟，`source_ms` 是导航采用的位姿采样时钟。定位无效时坐标列留空。按源时戳识别重复观测。失败时的终态标记指软件首次进入失败相位，物理停止时刻由录像核对。

采集器核对 MAP 与 AXF 中的两个诊断对象，再比对运行设备的 Flash 与 AXF 加载段。对冻结后的位姿和路线各做两次 RAM 读回，比对两份记录的终态、故障、总用时和完成站数。本次在离线模拟验证了读回一致性、错误固件拒绝和 CSV 解码。

```powershell
python .\工创赛\scripts\capture_zone2_pose_trace.py --axf $installedAxf --probe $probeUid --out $runDir
python .\工创赛\scripts\decode_zone2_pose_trace.py decode (Join-Path $runDir 'pose-trace-read1.bin') (Join-Path $runDir 'trajectory.csv') --compare (Join-Path $runDir 'pose-trace-read2.bin')
```

轨迹包含轮编码器与 HWT 算出的位姿。`route-trace.json` 内的 `grid` 记录当趟实际采用的占用图，`legs` 给出每段交付导航的有向 PATH、命令序号、缓存来源、会话和软件停稳时刻。成功应有八段，`source_leg` 为 1、2、3、4、5、3、4、8。第 6、7 段复用本趟第 3、4 段的应答。穿障等几何拒绝保留合法收到的 PATH，当段无起步和停稳时刻。`observed_elapsed_ms` 是调度器记录收到或读取缓存的时刻。线路上的原始帧需另外采集。

现场把同一次读回目录里的两份原始文件与哈希一并保存。正常完成时另核对最后一站的位姿源时戳。无定位的早期故障可有相同终态字段，配对应以同一次读回为准。用户确认底盘按当趟雷达 PATH 行驶，主控负责把收到的路线转成底盘轨迹并跟踪。

## 记录表

| 项目 | 记录 |
| --- | --- |
| AXF SHA256、装机、载物和起始标线 | |
| 占用图及实物障碍、六次 PATH 及两段缓存 | |
| 机构下降、开始扫描、发布图、收拢、首段起步 | |
| 八站停车时刻与实际静止证据 | |
| 回区最外侧投影、最终总时长 | |
| 最小车道净距、障碍净距、定位对照 | |
| 首个异常及各轮停止顺序和拖尾 | |

规则 R FIELD-01/02/06、OBS-01/04、RUN-01/04/07、PWR-01。工程验收 E LOC-02/03/05–07、MOT-02–04、RUN-05/06。Q FIELD-05、OBS-03、QR-04 待技术答疑确认。当前固定占用图的 ARM 证据属于回放，实物扫描、扰动后的车道净距和故障停车拖尾待现场确认。

## 离线证据

- [八段整链与区间净距归档](p4-corners/interval-clearance/README.md) 包含空图、占用格绕行、单次轮位置漏回、穿障拒绝及历史影子回执。净距下界适用于理想轮端回放与设定误差。
- 2026-11-19 的 [校准入口 ARM 回执](../../../../tmp/zone2-calibration-recovered-entry-arm/result.json) 覆盖 36 个场景、255 项断言，[蜂鸣回执](../../../../tmp/zone2-calibration-recovered-entry-arm/buzzer_pulses/result.json) 验证了 200 ms 短响和 3 秒连续响。 [MATLAB 回收模型](../../../../tmp/zone2-calibration-recovery-matlab.json) 通过 3,086 项，[ARM 雷达观察器回放](../../../../tmp/zone2-radar-recovery-arm/result.json) 校准档与导航档合计 64 个场景、678 项断言。实物升降、扫描和声音待现场确认。
- 最终源码的 [八段绕障回放](../../../../tmp/zone2-field-final-nav-detour-20261119.json) 用时 179.100 秒，完成八站、六次 PATH 请求和两次复用，轮速指令校验值 3830796688 与归档一致。
- [MATLAB R2020a 前置](p4-corners/zone2_pose_capture_matlab.json) 覆盖 100 ms 周期、调度抖动、重复时间戳、回绕、定位失效和满容量终态。
- [记录开](p4-corners/zone2_pose_trace_arm_enabled.json)、[记录关](p4-corners/zone2_pose_trace_arm_disabled.json) 均通过 187599 项 ARM 断言。5 号格绕行用时 179.100 秒，两次速度指令校验值均为 3830796688，均完成八站与 32 帧 STOP，其余控制回执字段亦一致。开启得到 1792 条记录，1655 条有效。
- [穿障回复反例](p4-corners/zone2_pose_trace_arm_rejected_path.json) 通过 12185 项断言，首站后冻结首故障，保存 174 条记录。停稳后读回是现场操作，ARM 在仿真结束时读出同一结构。
- [开启构建](p4-corners/zone2_pose_trace_keil_enabled.json) 与 [默认关闭构建](p4-corners/zone2_pose_trace_keil_disabled.json) 的两个 Keil 目标均为 0 错误、0 警告。独审核对了周期、终态、溢出及读取协议，7 项独立主机内存模拟通过。
- [路线记录 MATLAB 前置](p4-corners/zone2_route_evidence_matlab.json) 覆盖合法零序号、计时回绕、缺图、缺路径、缓存来源、首故障和位姿缓存写满。[开启记录](p4-corners/zone2_route_trace_arm_enabled.json) 和 [关闭记录](p4-corners/zone2_route_trace_arm_disabled.json) 的绕障回放均为 179.100 秒，轮速指令校验 3830796688。六次实时 PATH 和两次缓存来源均已核对。 [穿障回复反例](p4-corners/zone2_route_trace_arm_rejected_path.json) 保存第二段 `[4,14]`，共一站停稳，故障码 3、路径错误码 8。[同版本双目标构建](p4-corners/zone2_route_trace_keil.json) 均为 0 错误、0 警告。
