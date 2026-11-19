# 整合15接续：修复底盘STOP确认，保留视觉小步主线

## 2026-11-12最新停点：重烧成功，小步因USB超时中断，现场结果待答复

现场补充已到：用户确认有空间、完全停稳且无碰擦，方向/距离仍未知。只读复连完整Flash匹配，连续3组精简状态为inactive/ACK15、CAN与伸缩首故障0，后端首诊断reason0；见source-body-hold/usb-followup-20261112.json。未发动作/复位/清错。USB后续可读不证明故障已消除；此前待现场停稳答复已闭环，10mm位移和全程保持验收仍开放。

用户已确认撤出/供电/空间就绪并要求重烧，撤销下方历史待机条件；当前不因此授权自动重复测试。烧录前只读RUNNING、PE2=0，但首Flash块与9e6d8b39不匹配，未认定原因或沿用旧符号解释其RAM。原已验证9e6d8b39重烧Verify OK，read-only-20261112-005217.json完整168664字节匹配，VM/动作请求0。用户报告伸缩回零、4310略偏；本轮重新建实际参考后prepare-once-20261112-005252.json全部软件完成，实际回零、开210°、转180°/伸65mm，无软件故障。

仅一次step-once-20261112-005318.json左移请求，USB读取超时，主机随后发STOP。最终底盘inactive、owner9/reason9、底盘result2/cause9、STOP ACK15/fail0，PE2仍1，机构hold confirmed1、IT/安全/RTOS/CAN故障0，臂反馈179.99188°。缺少运动中连续轨迹，不签10mm完成、实物停稳或保持修复全程通过；cause9为取消后结果，不能据此判首故障。已问现场实际停稳/碰擦/方向距离，待答复，不重跑/复位/清错。证据仍归档source-body-hold，文件日期11月12日。交接已更新；需求沿下文R/E/Q，隔离fetch21522db、未改固件源码或上传。

## 最终停点：9e6d8b39已烧录，承托未撤，用户要求先待机

用户明确可靠承托并允许本次换版后，已烧录保持交接修订版9e6d8b39。Keil `source-body-hold/flash-20261111.log` Verify OK，20:05新鲜`read-only-20261111-200519.json`完整168664字节读回匹配；主控RUNNING，IT/底盘动作请求0、VM请求/PE2=0、软件故障0，旧机械参考失效。

用户随后明确“暂未撤出，先待机”。**当前承托未撤，不执行回顶、回缩、180°/65mm展开或左移，也不恢复旧参考。** 新版尚未做现场动作验证，当前试验次数0；已烧录不等于保持准入问题已实车修复。当前结果见`source-body-hold/field-result.json`。下一步等待用户明确人员/承托/工具撤出，再执行一次新真实参考和小步验证；此前空间条件未变不重复要求测量。

本轮仅本地交付，未提交/推送GitHub。R/E/Q核对仍见下节；视觉TCP/轴向、1号轮STOP E2的条件、毫米纠偏和完整自主流程仍未闭环。

## 最新现场：e2d0b367已烧录；一次小步失败，已找到保持准入冲突

本节覆盖下方准备期“未烧录/下载器仍报错”的历史快照。用户重插下载器并确认停稳后，连接恢复，读到F407 HALTED、Flash起始64字节全FF，两个候选在首个4KB比较均不匹配；只读证据`source-body-stop/board-identify-20261111-194810.json`。未据此认定全片擦除或原因，助手未在识别前复位、恢复运行或写板。

随后新STOP在线e2d0b367经Keil Verify OK，`read-only-20261111-195003.json`完整168584字节匹配、RUNNING、VM/PE2=0、动作请求0。用户确认人员、承托和工具撤出且无障碍后，一次回顶3.812s、伸缩回零1.469s、读臂0.266s、张爪0.828s、转180°0.953s、伸65mm1.031s均软件完成。不能把软件回顶/展开记录补写为新独立物理尺量。

随后仅一次左移10mm请求在owner elapsed1090ms失败，未完成10mm。主机STOP已发；用户明确现场已停稳、无碰擦。板端VM/PE2保持1，原故障未清、未重跑。新STOP实车观测state=PENDING、ACK14/fail1，其余三个轮已ACK，1号轮持续非成功回复；没有再误标ACKED。现场停稳确认与协议仍未收齐分别记录。

原件：`source-body-stop/step-once-20261111-195133.json`、`first-fault-followup-20261111-195527.json`、`field-result.json`。冻结首CAN控制请求为1号轮FE98006B，回复FEE26B、result8、ESR0、VM前后1，事务耗时1ms；不是超时或缺帧，未将E2接受成成功。

更早的真实首故障为伸缩保持`g_eh_first_reason=6`，tick113290、phase8、step9、上一条3A读取成功/flags3、service_gap0。CAN STOP请求tick113301，晚11ms。因此E2不能直接认定为此前伸缩失健康的原因。实际源码`Integration10_Service`在保持时调用`StartupAxes_Service(it_platform(),...)`，而`it_platform()`含`!g_chassis_auto_active`；显式底盘小步一活动，就把保持服务的platform_ready传为0，直接触发eh_fail(6)，后续MotionStop中止底盘（故障快照state1/DISABLE、cause9）。这是当前有源码和时间顺序支持的交接冲突。

助手曾提出g_safety_ready清0/置1窗口可能被CAN任务抢占，随后核对优先级发现Safety4、CAN3，该推断不受证据支持，已撤回，未按此猜测改Safety_Update。没有调整保护阈值、喂假健康或清除故障。

新独立`tmp/source_body_hold_20261111/project`基于e2d0b367，只把保持服务改用`it_hold_platform()`：原普通`it_platform()`及所有普通动作准入逐字节保留；仅已有真实Holding、显式SourceBodyStep所有权、平台及CAN测试健康、无真实故障时允许保持跨过body active。本机MATLAB前置8项/257组合、实际C514比较、原owner48/guard5/入口10断言及轨迹/STOP分派回归通过；原STOP修订和pzq50调度字节相同。在线与Offline均0错0警告；在线SHA256 `9e6d8b3972eda04937b054e128ae9f69b1ae4076de37b51e14ac24d2825ed5d4`，Offline `c25f02fe1d371420d1808b4bef2437fe75dae63c41ebf433f520d7b707d1b1e9`。2038文件审查仅integration10_complete.inc变化，证据`source-body-hold/`。本地manifest检查通过，尚未换版；已请用户可靠承托当前升降/伸缩再复位烧录，不能标根因已实车修复。

## 当前结论与版本

2026-11-11，本轮完成STOP修复的独立离线候选及主机故障收件，尚未烧录或发车。候选为 `tmp/source_body_stop_20261111/project/gongxun-f407.uvprojx`，基于最后已烧录的 `tmp/source_body_step_20261111/project`。不改实际外部工程或原65fccc70候选，避免旧板符号与新固件混用。

| 目标 | AXF SHA256 | Flash段字节数 | 状态 |
|---|---|---:|---|
| Vehicle_LiftRuler | `e2d0b3670f30a29c0199d9b145868af6f02473f742d07f1b6d2db64578403583` | 168584 | 构建通过，未烧录 |
| Vehicle_LiftRuler_Offline | `10cd2497211324f544f63b7e1ad6453031ec20bd9faae0483716d0898df45588` | 138532 | 构建通过，未烧录 |

最后一次成功读板仍为原记录19:05的65fccc70、168604字节，重刷后待机、VM请求/PE2=0、机械参考失效。本轮沙箱内外的只读连接均在CMSIS-DAP初始化时报 `ProbeError('expected DAP_INFO')`，没有完成新的板端读回。不能沿用19:05状态称当前仍停稳、已断电或仍在原姿态。

新只读失败原件：`evidence/integration/2026-11-11/source-body-step/read-only-20261111-194418.json`，complete=false，未进入动作写入路径。没有USB/core reset、主控复位、清故障、换版或重跑。

## 本轮修复

固件只改 `CHASSIS/SYSTEM/SAFETY/safety.c` 两处STOP分支，其余2037个源/工程文件逐字节相同；修改文件保留GBK兼容编码和CRLF。原pzq＋50ms抓放调度、速度、位置、DM启动阈值、wheel guard和看门狗均未改。

1. 原实现在20次重试后，即使ACK掩码为0也写MOTION_STOP_ACKED。新实现仅在四轮ACK掩码为15时进入ACKED；未齐保持PENDING。重试计数饱和20只用于诊断，不把耗尽次数当作成功。
2. 原失败回复会立即把待发队列重置为全部未ACK轮，低地址持续失败时可能反复占用队首。新实现先完成当前轮的其他轮STOP，再在原500ms轮期限重发仍未ACK的轮；不因一个轮子失败阻止其他轮尝试。
3. `fail_mask`保留本次停止事件中出现过的失败，后续重试/成功不擦除。其含义是失败历史，恢复后可以同时存在ACK=15与fail_mask非零，原始最后结果仍可读取。
4. 原 `APP_FAULT_CUT_VM_ENABLE=1` 编译分支保留：失败回复或超时仍触发原切VM出口。当前保持VM的profile仍不主动切VM；STOP持续收件不代表允许再次运动，也不代表物理停稳。

STOP重发只是故障停止服务，不是自动重试左移或机构动作。500ms是既有软件轮期限，不是实测四轮STOP完成时间保证；总线/RTOS现场时序仍需实测。

主机脚本 `scripts/source_body_step_live_20261111.py` 增加 `--stop-candidate` 以选择新候选，默认仍对应原已烧录版。默认无动作参数仅验证本地manifest和AXF，不连接设备。新增 `source_body_fault_snapshot_20261111.py` 读取固件原有的冻结CAN首故障请求、回复、ESR、收发耗时、VM命令上下文及轮询年龄，不再增加固件诊断状态。详细收件在发STOP之后执行，收件错误不会阻止此前的STOP请求。连接初始化失败也保存complete=false和错误原件。

主机只有在state=ACKED、ACK mask=15且底盘inactive时报告“协议ACK收齐”；physical_stop_verified仍为false，必须另取现场停稳结论。冻结标志要求收件前后CAN故障锁存均为1；未冻结快照不作为一致的首故障证据。

## 验证证据

证据目录：`evidence/integration/2026-11-11/source-body-stop/`。

| 层级 | 结果 | 原件与边界 |
|---|---|---|
| 本机MATLAB R2020a前置 | 18项、0失败；6组事件夹具 | `matlab-result.json/.mat`、`matlab-run.log`；覆盖超过20轮、轮间公平、500ms期限、故障历史、旧会话、错轮回复、VM撤销、uint32时钟回绕；只验证状态模型 |
| 实际C STOP函数 | 与MATLAB夹具3612个字段比较通过 | `stop-check.c/.log`；从候选safety.c抽取真实5个函数，硬件/时钟为模拟 |
| 原实现负例 | 如预期失败 | `stop-baseline-negative.log`；原实现首次失败就擦掉fail_mask，与模型不一致，不把此负例写成新候选失败 |
| 原切VM分支 | GCC独立编译运行通过 | `stop-cut-profile-check.c/.log`；失败ACK/期限超时仍切VM |
| 原底盘所有权/guard/入口 | 48/5/10断言及STOP分派、C/MATLAB轨迹对照通过 | `owner-regression/`；没有重做或放宽机械准入，没有实车跟随结论 |
| 主机收件 | 假目标回放通过 | `host-fault-capture-check.json`；保留FE E2 6B原回复；拒绝state2/ACK0及底盘仍active的“已停”推断 |
| Keil在线/Offline | 均0 Error、0 Warning | 两个`*-build.log`；不是烧录或实测 |
| 源码/符号审查 | 2038文件比较，仅safety.c变化；诊断块实际ELF大小匹配 | `review.json`、`candidate.json`、`source-diff.patch`；原已烧录源未动 |
| 新鲜硬件连接 | 失败 | 上述19:44只读失败JSON；当前硬件状态未知 |

首次C夹具生成器将printf换行写入了C字符串，导致测试程序编译失败；已修正生成器转义并运行成功，固件实现没有因此调整。

## 尚未闭环及下一步

1. 先恢复下载器只读连接：保持主控供电及机械姿态，退出可能占用的Keil调试，只重插下载器电脑端USB；若目前已经断电，如实记录，不把探针异常直接当断电证据。复连后先使用原板manifest读回，不直接烧录或重新建参考。
2. 明确当前机构是否可靠承托以及实际停稳/有无碰擦，再安排新候选换版。已有四周约10cm空间、线缆余量和用户在旁的记录保留，未变化不重复确认。换版会复位并撤保持，旧机械运行参考不跨复位使用。
3. 新版读回匹配、撤出人员工具后，重建真实顶部及伸缩参考、展开到180°/65mm。此前DM first_fault=11、事后速度−0.036630rad/s与0.035准入界限的关系仍只是假设；本轮没有改其阈值或伪造无故障。
4. 按已明确的单次左移10mm/15mm每秒目标推进。失败保留首故障及本次新增完整CAN收件，不自动清错/复位/重复发车；新STOP实现仍需实际应答与现场停稳证据。
5. 再取新图确认像素方向、尺度及真实位移，之后接三角形角度/平移纠偏、任务颜色/实例抓取和逐件180°/65mm再观测。TCP、车体系符号、遮挡底心归属、动态盘面时效及完整自主三件/第3段尚未通过；本轮没有宣称整合15全部实车闭环。

仅做本地manifest检查的命令：

```powershell
python scripts/source_body_step_live_20261111.py --stop-candidate
```

## 需求、规则分级及同步

- **R已检查**：SRC-01/02、RUN-01/02/04、FIELD-06、PWR-01、MECH-01/02。看护调试与正式自主入口分开；本轮不改变单件先放载台、车道包络、电源及初始尺寸/展开规则，也不宣称已验这些实物条目。
- **E已检查**：RUN-05/06、CAL-02/03/05、VIS-12/15、MECH-06；修订是停止确认与故障取证，500ms和10mm/15mm每秒是工程参数。STOP安全服务不将任务失败改写成成功，不发新的运动重试。
- **Q**：保持官方待技术答疑确认项，不以本轮模型或测试闭环。
- Git：根目录仍main尚无提交/remote、文件未跟踪；已有隔离仓库 `tmp/team_review_20261106.git` 本轮fetch成功，FETCH_HEAD=21522db，与此前相同，AGENTS无新的远程差异。没有pull覆盖根文件、修改AGENTS、提交或推送；不能称根工作区已与远程自动合并。
