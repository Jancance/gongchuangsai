# 区 2 雷达底盘现场验收记录

工程为 `3.stm32f407工程/gongxun-f407.uvprojx`。场上载物和全部伸出件计入实测车体。路线格号为 0→2→14→10→22→14→10→22→0。记录每站停稳及最终返回 300×300 mm 区 2。计时从 PC13 启动，包含雷达展开、升降、扫描和收拢。对照 RUN-07 记录是否在 180 秒内结束。

## 本地候选

用同一主工程配置构建 `Vehicle_Integrated`，显式启用 `ICM45686_SAMPLE_ONLY=0`、`APP_CONTINUOUS_MAP_ENABLE=1`、`APP_MAP_NAV_RESIDUAL_ENABLE=1`、`APP_ZONE2_NAV_ENABLE=1`、`APP_ZONE2_NAV_MOTION_ENABLE=1`、`APP_ZONE2_POSE_TRACE_ENABLE=1`。本机候选在 `工创赛/tmp/zone2-route-trace-build/Vehicle_Integrated/build/`，AXF SHA256 为 `b29394d0d3e80a104412b69b5688a213f6354370c58f3000423f69bb05034a47`，MAP 同目录。Offline 用于离线核对，现场驱动用 Integrated 配置。

记录缓存约覆盖前 204.7 秒，若后续仍在运行，满缓存只停止追加周期样本，末条保留终态。记录中的坐标可对比录像，样本周期无法直接证明采样间隔内的最小净距或物理停稳。32 KB RAM 的板端读回、CAN 任务实际栈余量和槽耗时仍待现场核对。

## 单趟记录

1. 登记实际使用的 Integrated AXF 哈希、同一构建的 MAP、雷达校准版本、HWT 安装方向、车体实测外廓、电池和载物情况。确认区 2 起始站位、机头朝向和车道尺寸，记录障碍物坐标和照片。
2. 开启固定机位录像，从按键前拍到车辆和机构都停止。可用已标定的四个场地角点对齐录像和场地坐标。逐站记下停车时刻、车体投影和是否进入下一段。如果提前结束，先记第一次异常、停轮和断电的现场观察。
3. 诊断构建启用 `APP_ZONE2_POSE_TRACE_ENABLE=1` 时，车上记录场地 X/Y、航向、采样时戳、阶段和完成站数。车辆实际停稳后、主控保留供电时，用与板上固件配对的 AXF 及 MAP 读出冻结轨迹。原始二进制一并保留。实际用时和停车位置以录像和人工标记交叉核对。

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

- [MATLAB R2020a 前置](p4-corners/zone2_pose_capture_matlab.json) 覆盖 100 ms 周期、调度抖动、重复时间戳、回绕、定位失效和满容量终态。
- [记录开](p4-corners/zone2_pose_trace_arm_enabled.json)、[记录关](p4-corners/zone2_pose_trace_arm_disabled.json) 均通过 187599 项 ARM 断言。5 号格绕行用时 179.100 秒，两次速度指令校验值均为 3830796688，均完成八站与 32 帧 STOP，其余控制回执字段亦一致。开启得到 1792 条记录，1655 条有效。
- [穿障回复反例](p4-corners/zone2_pose_trace_arm_rejected_path.json) 通过 12185 项断言，首站后冻结首故障，保存 174 条记录。停稳后读回是现场操作，ARM 在仿真结束时读出同一结构。
- [开启构建](p4-corners/zone2_pose_trace_keil_enabled.json) 与 [默认关闭构建](p4-corners/zone2_pose_trace_keil_disabled.json) 的两个 Keil 目标均为 0 错误、0 警告。独审核对了周期、终态、溢出及读取协议，7 项独立主机内存模拟通过。
- [路线记录 MATLAB 前置](p4-corners/zone2_route_evidence_matlab.json) 覆盖合法零序号、计时回绕、缺图、缺路径、缓存来源、首故障和位姿缓存写满。[开启记录](p4-corners/zone2_route_trace_arm_enabled.json) 和 [关闭记录](p4-corners/zone2_route_trace_arm_disabled.json) 的绕障回放均为 179.100 秒，轮速指令校验 3830796688。六次实时 PATH 和两次缓存来源均已核对。 [穿障回复反例](p4-corners/zone2_route_trace_arm_rejected_path.json) 保存第二段 `[4,14]`，共一站停稳，故障码 3、路径错误码 8。[同版本双目标构建](p4-corners/zone2_route_trace_keil.json) 均为 0 错误、0 警告。
