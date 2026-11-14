# half + 仅176 B停止轮程观察：最终独立复核

2026-11-15。**结论：PASS_LOCAL_HALF_OBSERVER；无本地发布阻断项。** 可按用户当前“恢复板端、停止试车”的授权交付一次下载；本报告不授权ARM/START、试车、故障SWD或额外复位。

本次只读产品/既有证据，仅新增本文，没有运行实现脚本、会覆盖原证据的验证入口、重新构建、Git写入或硬件操作。最终版不是停车控制优化：half横向控制保持、正常完成保VM保持，只增加176 B静态观察，不含104 B浮点统计、M1.5/S尺度或B1前视。

## 冻结源与控制等价

|文件|最终SHA256|
|---|---|
|chassis_auto_test.c|`ef5e6540df59b976b6200f7601443e7acdbc565ea3c8779f9f15343b64a59a24`|
|chassis_auto_test.h|`f1298246023c87b15388529ea5c26e5e23c40bb04ed9ac46e619e9e5bc881fa2`|

独审逐项检查诊断插入并反向剥除，得到的两文件与half原件 **4434979a…dac19 / e940c64b…b764逐字节完全相同**。half是 `P+H`，H已含50%几何系数；没有再乘0.5、没有1.5增强或S权重。恢复来源及历史四趟绑定见两趟/回退目标独审，旧half终点MAE5.375mm是选择依据，不是本新组合的精度承诺。

独立核1731项依赖相对纯half基线仅auto.c/h变化。hwt101.c/h仍为 **690b864c…3c90 / eb01b7fe…6429**，保留原HWT三scope/28字段；没有新增TimingEnabled getter或104 B浮点统计。app_motion_runtime **2cded70c…ef611**、app_serial_runtime **d99860e5…ca4** 未变，保VM和串口会话规则继续保留。它是“half控制+当前保VM+只读观察”的新镜像，不是旧2661174f整固件复刻。

原Ky、HWT控制、距离比例、floor1RPM、follow±2、参考/F/profile、预算/斜率、STOP和故障出口未改。检查实际 `Auto_DistanceCheckLocked` 无轮间spread判断，**没有复活旧轮间差50mm误停保护**；其中原有 `mean > target+50` 是目标越界余量，原TRACK50是参考跟踪界，两者不是轮间spread保护。编译没有添加自动运动路径：原request初值0和显式START许可保持，观察函数不写任何执行器请求。

GBK roundtrip通过，auto.c为2099 CRLF且无裸LF；header保留混合换行（241 LF中原有1个CRLF）。未批量转码、改配置或无关重构。

## 176 B三个事件和停止距离含义

三事件沿用已独审的观察方案，新增块与控制值不相互回写：

- entry在真实DistanceCheck校准共同mean、采样原reference之后；以elapsed≥boundary[4]且mean≥target−120记录第一次减速末段状态，不把匀速零加速度当入口。
- latest freeze仅在新四轮命令成功采用并进入START时覆写，明确是软件冻结，**不是完整四轮ACK或实测轮速**；未变批、正在发其余轮不增加计数。
- arrival在原reached/finish/stop赋值之前保存新reference及尚未推进的旧follow/follow_tick；不多调用一次控制更新。arrival后冻结所有槽，下次真正接受的匹配baseline40会话才reset；拒绝start、非匹配session和STOP/fault/idle保持原记录。

头32 B+三个48 B槽的真实C sizeof/偏移、编译断言与release ELF176 B对象相符。Reset直接清静态字节，无176 B栈临时对象；Record只做有界复制及观察换算。事件前reached/stop/finish flags与槽有效位分开，uint32时刻/回绕未转float保存。

`calculate_stop_wheel_distance.py` 只读本地snapshot、同次串口JSONL及XD派生记录。arrival给的是common_ms对应的轮端进度，PE3是更晚异步四轮末样本；脚本分别输出arrival控制时刻/共同时间年龄、首STOP ACK、各轮PE结束sample及其时差。**PE−arrival是轮端量在不同时刻的差，不能叫地面溜车或物理停止距离。** STOP ACK也不是机械停稳证明。

该脚本正确保留无STOP/缺失XD逐记录session时的未知标记并要求外部绑定同次raw/hash；不借缺失信息生成硬件来源。独审用合成ARM snapshot与构造的PE/STOP记录独立核签名计数、原77mm/65536/1.0218254换算和时差，结果一致且ground/provenance保持false。它不是本车新增一次实测。无新UART帧、RAM读取动作或传感器要求。

## 独立ARM、ELF与资源核验

不重构建，独审核对fixture C/AXF哈希后直接在独立Unicorn执行现成镜像：before **1375 cases/16679 assertions**、after **1393/16720** 均return0；before绑定half4434979a，after绑定ef5e6540。32×12个真实DistanceCheck控制字段/1536 B前后逐字节相同，SHA为 `908ed2248de6fde5e7ac90d93ee6b58310a65dbdedf32187cfe53ac8b0804188`，entry/FF观察一致。

这些是构造输入下的实际C执行，外部CAN/power/RTOS/source readiness等仍为stub；不是四趟内部历史重放、M4时序或地面验收。B0仍不可辨识，B1未运行。零记录、回绕、invalid及合成dump解析的前置边界保留，不能将valid_mask当硬件来源。

三目标实际日志均0 Error(s)/0 Warning(s)，AXF哈希吻合。独立解析所有非空SHF_ALLOC节，专项关闭目标与纯half全部相同、含ZI；最终ELF中不存在 `g_chassis_auto_lateral_timing / ChassisAuto_ReadLateralTiming`。

|目标|Code / RO / RW / ZI bytes|176 B tail符号|
|---|---|---|
|baseline40_integrated|102328 / 6572 / 2952 / 119096|`0x20012d84`|
|baseline40_offline|101680 / 6564 / 2760 / 119088|`0x20012c98`|
|baseline_off_integrated|11974 / 470 / 960 / 25576|无此符号、分配节同基线|

相对纯half，Code各+584 B；对象及auto.o ZI+176 B，整镜像Integrated +184 B（含+8对齐）、Offline +176 B。不能把对象大小与总ZI增长混称。独立callgraph核Reset stack0，CanStep local200/max584 B与half不变；其他原主要栈界在回执中一致，未取得本候选硬件stack高水位。

**最终Integrated AXF：** `工创赛/tmp/half-stop-observer-build-20261115/baseline40_integrated/Vehicle_Integrated.axf`。

**SHA256：** `3966ed37a6c2dd85319884fbe22a2c54a610a9bb9e50f46240e79c006bf1113d`。

旧5e803d的104/176 B双对象地址和reader均不适用于本镜像；本版没有104 B对象，tail新地址也必须绑定本ELF，不能沿用旧RAM脚本。本次仅下载、不读RAM、不ARM/START。未来若用户重新授权取数，应另核同session静止完成态、稳定双读，不halt/reset/改变VM；本文不生成这种现场授权。

## 两趟分析和交付限界

顺手核对 `diagnostic-two-analysis/input_manifest.json` **76项SHA全部匹配**，报告承认新两趟右20/30mm比half/S2.5更差，区分稀疏OD与地面尺量、真实RAM与离线parser固定来源标志、失败与精度样本，没有把恶化唯一归于FF/插桩/供电或复位。未重解所有历史raw或扩回归；启动异常原因仍未知，恢复half不等于已经修复它。

已核R **RUN-01/PWR-01**；E **LOC-02/03/05/07、MOT-02/04、RUN-06**；Q **FIELD-05/OBS-03/QR-04** 仍待技术答疑确认。新观察是E，不改竞赛R。当前仅本地离线候选通过；板端是否已恢复由父任务实际一次下载/核验结果单独证明，不能在下载前宣称完成。用户已取消试车，本报告不要求视频或新地面传感器，不再启动测试。
