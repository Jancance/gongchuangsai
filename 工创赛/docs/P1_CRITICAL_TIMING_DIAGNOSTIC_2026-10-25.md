# 启动临界区计时诊断（2026-10-25）

本轮只准备计时诊断，未烧录、未打开COM、未测到任何真实耗时，不得称启动临界区已超标或ORE根因已证实。板上仅沿最后已知的distance_handoff_fix_route_a失败状态，本轮未读取确认。

## 计时点与采集

使用F407 CMSIS DWT CYCCNT，以SystemCoreClock提供CPU周期频率。在HWT初始化时只设置TRCENA/CYCCNTENA并检查计数是否推进，不重置CYCCNT、CPU时钟、中断优先级或屏蔽阈值。计时器不支持/未推进时enabled0，主机不把零值解释成零耗时。

三个区间：hold_critical为App_ForwardHoldStep进入临界区后至退出前，包含其嵌套调用；backend_begin为其中BeginTranslationDistance/Timed调用区间，包含原重复自检与后端初始化；action_load为组合表装载调用，原来就在父临界区内，包含公共启动接口与布防。三者可能嵌套，不能相加当总耗时。

每个区间记录count/last_cycles/max_cycles/最大区间起止cycle/最大区间tick与arm/最后endcycle。每次HWT诊断布防时只重置诊断区间统计，不动接收状态、计数或运动故障。首个HWT接收错误另记event_cycle；这是ISR或校验诊断处理时刻，不是硬件字节到达时刻。

故障时随首trip复制冻结统计，CT1只在故障后约1Hz发送，和TM1错开至少70ms，忙时重试。28个uint32字段顺序见schema的critical_timing_diagnostic，CRC同HF2。HF2等既有格式和控制令牌规则不变。主机保存timing_frames/last_timing及JSONL CT1，按周期数/频率换算微秒；没有计时或无区间最大值时显示None，不编造0us。CT1不参与健康、令牌或KEEP。

## 解释边界

测量包括计时点之间的调用与允许的高优先级抢占，包含嵌套计算，但不覆盖临界区入口、计时终点后的记录开销及退出指令；它不是全系统最大中断屏蔽时间证明。插桩及统计重置本身也增加开销，尚未在MCU标定。

115200/8N1连续字节间隔约86.8us，可和区间耗时比较；超出说明存在接收不及时风险，仍须同时核对UART字节实际相位、ORE ISR时刻、arm和tick。小于该值也不能排除其它未插桩临界区。uint32周期会回绕（168MHz约25.6秒）；主机按模2^32处理短区间，跨长时段或调试暂停不能仅比较绝对cycle判断因果。DWT不是独立墙钟，测量时不能用断点/单步制造额外等待。

## 验证与后续

77项主机测试通过，含周期微秒换算/回绕、不可用标志、区间一致性、CRC/字段校验以及故障后HF+CT收包不再次ARM/START的模拟流程。默认/A组/B组/串口单段/手动/无线/位置诊断关7种完整Rebuild0错0警告；源与构建匹配、GBK/CRLF/diff通过。默认Code58200/RO788/RW2012/ZI33172。证据同日critical_timing_*_build.log及critical_timing_review.json。主机测试不证明DWT/MCU计时或串口实收正确。

此前旧到距串段修复保留，后端/HWT保护判定、查询策略和默认动作表不改。下一项需现场确认后重新烧录critical_timing_route_a_20261025，先只读健康，再按用户允许单次复现采CT1/HF2；本轮不发车、不自动重跑。若需要继续定位，可以先讨论VM关闭的纯计算基准，但本轮未实现或执行该基准。

构建策略后续精简：小修改先当前实测配置；涉及共享头/预处理模式等，再覆盖受影响配置。七种配置结果分别报告，不能当作七次同配置测试，更不代表七次硬件验证。

核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。仅本地未提交推送。
