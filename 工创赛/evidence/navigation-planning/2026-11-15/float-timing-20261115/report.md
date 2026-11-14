# 浮点路径 DWT 仅观测候选（2026-11-15，本地未下载）

## 结论和边界

本次只布置计时，不修改浮点公式、不做定点优化、不改STOP、CAN轮序、串口帧、RTOS优先级或控制阈值。MATLAB数据格式验证与实际ARM C固定输入检查通过，三个原配置均构建0错误0警告。`chip_cycles_measured=false`：没有硬件读取、复位、烧录或发车，不能声称已测耗时或提速。

源只改CHASSIS/USER下chassis_auto_test.c/.h、hwt101.c/.h。HWT仅多一个enabled getter，旧3个scope/28字段原协议不变；没有新周期串口发送。其他工作区改动未处理，未执行Git或联网。交接由父任务汇总。

## 两个精确计时段

- scope0：原锁外`Auto_LateralTarget`调用前到条件调用`Auto_LateralFeedforwardPerRpm`完成之后。读OD、读heading和tick采样均不在cycle范围。记录在原临界区内、原active/横向enabled/状态门成立且session/reference与OD新鲜度复核后的`lateral_valid`为真时执行；idle/STOP/无效OD不计数。HWT失效时原FF函数仍可能走快退，所以这里是有效OD下的Target+条件FF路径，不等同于每次都执行三角函数。锁内增强权重/1.5倍混合不包含在scope0。
- scope1：原横向门成立时，`Auto_LateralStepLocked`调用前至返回后；包含其中斜率、budget、量化，记录后才沿原失败出口处理。无效OD时仍依原控制逻辑调用Step使横向撤回并记录；idle/STOP不执行这个门。`Chassis_Baseline40Commands`和其后CAN操作不包含在scope1。
- begin/end读数是CYCCNT无符号差，跨一次32位回绕正确；单段必须短于2^32个周期。elapsed包括IRQ抢占和调用/读计数器边界开销，不是无中断纯函数成本。未改变IRQ设置或扩大测量为整个CanStep，CAN等待不混入。
- 两段有独立count/min/last/max/sum64/first_ms/last_ms/window_ms/valid。ms取原CanStep锁内now对应时刻，标识被核验的调用；window_ms=last_ms-first_ms，不是cycle窗口。scope0与scope1计数不要求相等。count达到UINT32_MAX后整条记录冻结，sum64最大不超过(2^32-1)^2。统计更新在测量段之外、原锁之内，会增加少量临界段耗时；不声称物理调度完全不变。

统计从MCU启动累计，正常结束/故障/STOP均不清零；若一启动内运行多次则聚合多次。现每趟工具复位的流程可形成单趟窗口，但必须在用户结束后手动reset前读取。`enabled=0`或`valid/count=0`表示未测，不能报告0us；真零周期样本由valid=1,count>0区分。

## 只读导出方案（本轮不执行硬件）

独立RAM符号`g_chassis_auto_lateral_timing`，结构大小104字节。release未引用的`ChassisAuto_ReadLateralTiming`会被链接器剔除，不能假称镜像中有可调用getter。后续只在用户明确硬件授权后，以同一session的DONE、backend inactive且无fault作为前后状态核验条件，连续读取匹配ELF符号的完整104字节两次，并要求前后状态和两份内容一致。不得halt MCU、reset、改变VM、mask IRQ或调用getter；halt可能触发看门狗复位及VM掉电，与保VM目标冲突。若无法证明统计写入已停止、session/状态稳定或两份字节一致，采集标为unstable，不能采信计时结论。稳定双读只是软件快照一致性依据，不冒充现场物理停稳证明。父任务如后续叠加STOP源码，必须从新ELF重新解析符号，不能沿用此处地址或镜像SHA。

当前仅计时候选：Integrated地址0x200124c8，Offline地址0x200123f8。完整ELF SHA/地址在final_verification.json。字段布局：enabled0/core_clock_hz4；scope0偏移8，scope1偏移56，每段48B；段内valid0/count4/min8/last12/max16，padding20，sum64偏移24，first_ms32/last_ms36/window_ms40，尾padding44。该布局经实际ARM C sizeof/地址差断言核验，不能跨不匹配构建假定。

离线`parse_snapshot.py`只读bin文件不接硬件，无法独立核验前后状态或稳定双读；它的valid只表示计数格式有效，不代表采集一致性通过。缺少上述采集证据时即使解码出数值也须标unstable、不采信。匹配104字节输入后输出周期、均值及`cycles*1e6/SystemCoreClock`微秒；时钟取RAM实际SystemCoreClock，不固定168MHz。时钟0/disabled/invalid的微秒为null；窗口不足两点不计算调用间隔频率。频率定义为(count-1)*1000/window_ms。parser不具备硬件来源证明，始终chip_cycles_measured=false，真实现场证据由父任务另记录原始dump和匹配镜像。

## 适用验证和证据

1. `verify_timing_format.m`先于产品写入实际执行R2020a，进程exit0、TIMING_FORMAT_PASS，matlab_summary.json记录结果。覆盖uint32回绕/禁用/零周期有效样本/64位累加/饱和冻结/均值/84、168、180MHz换算/快照值隔离；不是动力学仿真。`matlab.log`保留启动prefs/cache访问警告，不能称MATLAB环境0警告。
2. `check_actual_entry.py`取当前真实函数，ARM编译并Unicorn执行：1401case/16649断言。计时专项覆盖回绕/零值/disabled/独立count/饱和/快照/结构布局/正常/ODinvalid/STOP/idle。原正常、失效、session更新和控制命令检查保留。
3. `check_baseline_control.py`使用保存的5742f22b…原源运行同一固定输入控制fixture，1389case/16629断言通过；entry与FF观测逐项相等。`final_verify.py`剥离确定的插桩后要求其余auto源字节与before完全相等，非专项ELF全部allocated内容逐字节相等。不将外部CAN/power/RTOS/HWT stub执行冒充板上时序、硬件故障或实车验证。
4. 三目标从唯一日常uvprojx复制的隔离工程构建，不改原uvprojx。专项Integrated Code102096/RO6572/RW2952/ZI119024，Offline Code101448/RO6564/RW2760/ZI119024；两者Code+220B、ZI+112B。非专项Code11974/RO470/RW960/ZI25576完全相同。见build_summary.json及三个log。
5. 链接map Integrated RW_IRAM1 size0x16e40/max0x20000，RW_IRAM2 size0x6e38/max0x10000；ucHeap仍20480B，未增加动态分配、静态任务栈或改heap配置。无运行栈高水位读取，不宣称栈余量实测通过。
6. 源GBK roundtrip保持；三个.c/.h原纯CRLF保持。chassis_auto_test.h原209 LF+1 CRLF混合，本次只按邻近LF新增，唯一原CRLF保留；未全文件转码/行尾清洗。源前后SHA见source_manifest.json。

实施过程首次检查发现头文件原混合换行，auto.c已写而其余尚未写，随后从保存原件在内存恢复并保留头文件行尾继续，没有盲重试覆盖。ARM初始新统计测试误把fixture setup内部调用也算入独立计数/误假定状态仍RUN，已只修测试隔离与明确RUN前置条件后通过；未为过测修改控制或降低断言。最终记录取最后通过结果，初步返回不是芯片测试故障。

## 规则分类与未闭环

已核R RUN-01/PWR-01；E LOC-05/MOT-02/RUN-06。插桩、快照和验证都是工程E，不是竞赛R。官方未明确的Q FIELD-05/OBS-03/QR-04继续待技术答疑确认，未修改规则基线。

未闭环：真实周期分布/IRQ占比/频率、板上SystemCoreClock值、观测开销的现场影响、动力学与地面效果。下一步先独立审查；本地commit由父任务处理。以后只在明确下载与现场安全授权后部署并在reset前保存原始104B及ELF SHA，再决定是否存在值得优化的热点。此候选没有优化数学，也不把浮点当作横偏病因。
