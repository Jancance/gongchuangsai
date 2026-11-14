# 横向 DWT 计时插桩独立复核

2026-11-15。审阅最终冻结源 `chassis_auto_test.c` SHA256 `49a1c2bfcfb5f5b8fd7716444f395d6a88653a1309d985aab6c3b1334abacd94`。状态：**LOCAL_OBSERVATION_CANDIDATE；chip_cycles_measured=false**。

结论：未发现阻断本地交付的问题。计数/格式适用 MATLAB 验证、实际 ARM 固定输入边界与构建证据相符，可以保留为后续测芯片耗时的候选；不等于已测耗时、已提速、实际调度无变化或硬件验收。本次只读产品和既有证据，仅新增本文；没有重构建、Git 写入、联网、硬件读取、下载、复位或发车。

## 范围和数学隔离

本任务只改 `CHASSIS/USER/chassis_auto_test.c/.h`、`hwt101.c/.h`。独审核对四份 before/after SHA 均与 `source_manifest.json` 一致，逐项剥除具名插桩后四文件均与保存的 before **字节完全相同**，不只是数值相近：

- auto.c：增加头引用、RAM 统计/快照函数、计时局部变量、两个 begin/end、锁内记录；将原 Step 返回值暂存后仍进入同一失败分支。
- auto.h：只插入两个统计结构与声明，原有字节保持。
- hwt101.c/.h：只增加 `HWT101_TimingEnabled()` getter 及声明，原 DWT 初始化/使用方式没有修改。

S2.5 的两处 `/2.5f`、M1.5、Ky、FF、斜率/预算/量化、新鲜度、STOP/VM 等原控制表达式与状态分支保持；没有用统计值计算控制量，没有新增 UART 周期发送或改变 APP 配置。原 `HWT_TIMING_SCOPES=3`、`HWT_TIMING_FIELDS=28` 及原三 scope 编号/帧解释不变。

GBK roundtrip 均通过。auto.c 为 2088 CRLF，无裸 LF；hwt101.c/.h 为 752/76 CRLF；auto.h 原有的混合换行保持（229 LF 中仅原有 1 个 CRLF）。没有批量转码或换行清洗。

“控制源语义保持”不表示执行时间和调度完全不受影响：begin/end、统计更新和额外局部变量均有成本；尤其统计更新虽在测量范围外，仍增加原临界段工作。新候选必须与四趟已实测 S2.5 区分。

## 计时边界和样本解释

**scope0：**start 在 `Auto_LateralTarget` 之前，end 紧随可选 `Auto_LateralFeedforwardPerRpm`，且 **end 在进入原 `taskENTER_CRITICAL()` 之前**。读取 OD/heading/最初 tick 不含在该 cycle 范围内；记录动作在原锁内、原 active/横向 enabled/运行状态及 session/reference/OD 时效复核之后，仅 `lateral_valid` 时执行。idle、STOP、无效 OD 不污染此计数；但有效 OD 下 HWT 失鲜时 FF 可以提前返回，所以是 Target+可选 FF 路径，不是每次完整 sin/cos/tan 都执行。权重/1.5 合成在后面的原锁内，不包含于 scope0。

**scope1：**仅原横向运行分支中 `Auto_LateralStepLocked` 的调用前后，含 Step 内的预算、斜率与量化；记录在 end 之后、原失败出口之前。无效 OD 仍可按原逻辑用目标零进入 Step，此时 scope1 可增加而 scope0 不增。idle/STOP 不执行，两个 count 不要求相同。原合成权重、组四轮命令和阻塞 CAN 交易不在 scope1 内，不能将其称作全部浮点数学或整个 CanStep 耗时。

两 scope 都是经过调用边界的 CYCCNT elapsed，包含读计数/调用边界开销和可能的 IRQ 抢占；锁内也不能排除更高优先级 IRQ。未因计时改 IRQ 或额外关闭中断。最大值是已观察到的 elapsed 最大值，不是函数无中断纯成本或证明得到的最坏执行时间；`max` 更不能当成整体平均 CPU 占用。

ms 窗口使用 CanStep 原 tick 的记录时刻，`window_ms=last_ms-first_ms` 为无符号差，是首末被记录调用的时间跨度，不是 cycle 测量段长度。统计自 boot 累加、不会每次 ARM/STOP 清零；没有复位的多趟会混合，导出时须保留这个边界。

## 计数、原子性、回绕和导出

- 初始化 `valid=0/count=0` 代表没有样本。disabled 时不累加；合法的零 cycle 样本通过 valid/count 区分，不能把 disabled 的零数据报为零成本。
- `end-start` 使用 uint32 无符号差，单段少于 `2^32` cycles 时跨单次回绕正确；不支持把跨越多个 CYCCNT 周期的暂停恢复成真实耗时。`window_ms` 同样只有模 32 位含义，不把 boot-lifetime 无上限运行解释成无限精确时窗。
- count 达 `UINT32_MAX` 时冻结该 scope 整条记录，使 count 与 sum 保持一致；每样本 cycles≤UINT32_MAX，因此最大 sum≤`(2^32-1)^2`，不溢出 uint64。min/last/max/first/last/window 的更新与有效位顺序合理。
- 写入只在现有 CanStep 临界段内。getter 的整结构复制也加同一临界段，避免同一任务系统中的撕裂快照。**本次 release getter 没被使用，链接器已移除**，不能把未链接函数当现成远程调用接口。
- 直接调试器读取活跃 RAM 不能凭 volatile 获得 64 位 sum/多字段一致性。后续如获硬件读取授权，先后均核验同一 session 的 DONE/backend inactive，再完整读取104 B并稳定双读一致，绑定该次 ELF；这里依赖 DONE 后本统计不再写入，不能仅靠活跃运行时的两次相同读数推定一致。读取过程保持核在运行、**不 halt、不 reset、不改变 VM**，避免为快照暂停核引发看门狗/供电状态变化；用户 reset 前保留原始两份 dump 及前后状态。不能把不同时间单独读取的字段拼成一次样本。此前“halt after motion”的导出建议撤回，相关文档/元数据须同步更正，不涉及产品源码变化。

独立从最终 ELF 查得 `g_chassis_auto_lateral_timing` 确为 **104 B**：Integrated `0x200124c8`，Offline `0x200123f8`。header 8 B、两个 scope 各 48 B，scope1 偏移 56；sum64 在 scope 内偏移24，尾部/中间 padding 保留。实际 ARM 测试也核查 sizeof 与偏移，地址不能跨重构建照搬。

`parse_snapshot.py` 只解析 104 B 本地文件，时钟取 snapshot `SystemCoreClock`，不用固定168MHz；disabled/clock0/invalid 输出微秒 null，计频为 `(count-1)*1000/window_ms` 且有零窗保护。独审在内存独立检查未测零、disabled、clock0、跨回绕和大于32位 sum 的格式/均值，均通过。parser 保持 `chip_cycles_measured=false`，本地文件解析本身不构成硬件来源证明。

## 独立 ARM / ELF / 构建核验

本代理没有执行会覆盖证据的 runner 或重跑构建，直接核对 fixture C/AXF 哈希后初始化其全部分配节，在独立 Unicorn 实例中执行已有镜像：

|固定输入镜像|独立重放结果|边界|
|---|---|---|
|计时插桩当前源|return0；**1401 cases / 16649 assertions**|包含 disabled、有效零、cycle/ms回绕、sum64、饱和 count、getter 快照/布局、正常/无效OD/STOP/idle |
|before 的已实测 S2.5 控制源|return0；**1389 cases / 16629 assertions**|与当前源使用同一控制固定输入与外部 stub |

两者 entry/FF 观察值逐项相等；原精确输出/预算/状态边界断言保留。不能把模拟器中人为每次加32的 cycle stub 当 Cortex-M4F 真实耗时，也不能把 Cortex-M3/Unicorn 固定输入检查扩展成 RTOS、硬件 CAN/电源/HWT、板端调度或实车验证。

独立检查最终三目标日志均为 **0 Error(s), 0 Warning(s)**，对应 AXF SHA 吻合；没有复做无关回归。相对无计时的 S2.5：

|目标|新 Code / RO / RW / ZI bytes|分配节比较|
|---|---|---|
|baseline40_integrated|102096 / 6572 / 2952 / 119024|启用插桩，非相同|
|baseline40_offline|101448 / 6564 / 2760 / 119024|启用插桩，非相同|
|baseline_off_integrated|11974 / 470 / 960 / 25576|全部非空 SHF_ALLOC 节含 ZI 与无计时 S2.5 相同|

启用专项目标 Code 各 **+220 B**、ZI 各 **+112 B**，统计对象自身104 B，不能把增加的总静态RAM错报成104 B。没有新增动态分配或调大 heap/任务栈；未取得本候选真机 stack 高水位，不声称已实测栈余量不变。

候选 Integrated AXF SHA256：`b32ba55f389ed7738826c720ac3ee271faca2128869ef27fba87ff567d29af81`。四趟实测 S2.5 的 `41dd9265…58f7df` 不是本镜像；旧 `baa80295` 仍只是 S3 控制源单文件检查点，不是完整固件 checkout。本报告不声称板端已装本插桩。

实际 MATLAB R2020a 的适用验证为 `verify_timing_format.m`，先于产品插桩，覆盖 uint32/uint64、disabled/零样本、均值/时钟换算/快照；它不是动力学或芯片耗时模拟。审阅了 summary 与进程日志/exit0，保留 prefs/cache 启动警告；独审未再次启动完整 MATLAB。

## 交付边界和下一步

本地插桩可交付；真正的芯片耗时及热点判断仍未闭环，不能现在决定定点化、近似 tan 或承诺提速。用户不新增地面传感器、视频困难不影响完成本地静态/ARM检查；这些条件也不能被写成已经测到 chip cycles。后续现场若授权，先绑定版本，按同 session 完成态前后核验与104 B稳定双读获取记录，读取时不 halt/reset/改变 VM；未经授权本报告不触发任何硬件操作。

已核规则基线：R **RUN-01、PWR-01** 未变；E **LOC-02/03/05/07、MOT-02/04、RUN-06**，保留时效/故障出口和验证层级；Q **FIELD-05、OBS-03、QR-04** 仍“待技术答疑确认”。本次计时统计及其筛查标准属于 E。仅本地独审完成，未同步 GitHub；停车过渡后续独立观测设计不在本冻结版产品差异中。
