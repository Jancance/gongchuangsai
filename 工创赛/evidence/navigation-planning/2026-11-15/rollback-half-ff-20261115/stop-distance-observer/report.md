# half原控制＋176B停止轮端距离观察

日期2026-11-15。用户先要求停止后续实车、回退较好数据版本，后又明确“在原来代码基础上增加那个停止的测距”。root已将纯half＋当前正常DONE保VM/握手基点单独提交为`2d01524d`，再greenlight本第二步。最终只在该half基础的auto.c/h加入原先实际MATLAB/ARM验证过的三个只读事件快照，没有恢复float统计、HWT新getter、M1.5调度或S尺度。没有修改控制公式、参数、curve/follow、距离比例、arrival、STOP、VM或既有串口协议。

## 产品和依赖冻结

- auto SHA256：`ef5e6540df59b976b6200f7601443e7acdbc565ea3c8779f9f15343b64a59a24`。
- header SHA256：`f1298246023c87b15388529ea5c26e5e23c40bb04ed9ac46e619e9e5bc881fa2`。
- 原half before分别为4434979a…dac19 / e940c64b…b764；依据`source_changes.json`逆向剥除仅新增诊断字节后，两文件严格恢复该before，而不是仅按输出近似相同判断。
- 1731份源依赖与纯half基点核对，只有上述auto.c/h改变；hwt101.c/h仍为690b864c…3c90 / eb01b7fe…6429。HWT原3scope/28fields不变，无新的float timing类型/符号。
- app_motion_runtime.c保持2cded70c…5611，app_serial_runtime.c保持d99860e5…bca4，因此正常DONE保VM、现client握手修复继续保留。GBK字节保持，auto全CRLF、header原混合行尾保持。

## 保存什么、怎样算轮端距离差

固定三槽，每槽48B，头32B，共176B；没有新串口帧、CAN轮询、动态内存或停后动作。

1. 首次进入尾段：原有效共同时间mean≥目标−120mm且原elapsed≥boundary_s[4]（减速相位）时保存。
2. 尾段内最近一次原CanStep**实际接受的整批命令冻结**。不是完整ACK批，可能被STOP打断。
3. 原arrival分支设置reached/finish/stop前保存。current reference已采样；follow仍是末次实际更新的值/rate/tick，保留了原先reached优先于follow的顺序。

字段为now/run_tick/common/follow_tick、共同mean、参考position/F、follow/rate、请求base F、接受applied base F与flags。F都是参考或命令量，不是实际轮速/车速。记录同session，拒绝start/非专项不清除旧session；正常结束/STOP/取消/故障后保持，下一次真实接受的匹配专项start才reset。reset为固定volatile byte clear，不使用176B临时栈结构。

最终轮端位置/时间复用现有PE3、PM1、OD1及XD STOP ACK保存路径，没有改变这些采样/导出。`calculate_stop_wheel_distance.py`只离线读取匹配snapshot、同趟serial JSONL及XD派生文件：

- 列出arrival处理时刻、其common-time和样本年龄、参考F/末follow状态年龄。
- 从PE起止符号/计数按原77mm轮径、65536counts/rev、前进scale1.02182540重算校准轮端mean。
- 列出四轮PE末采样时刻各自相对arrival/首STOP ACK的时间，再给`PE末端mean－arrival共同进度`。可选停稳后尺量作为独立输入并列，不能自动认为同点同刻。
- 无STOP数据或XD派生缺每条session时明确保留未知/出处限制；没有有效arrival或匹配完整同session PE3时不输出虚假测量。

**这些量不是实际溜车/地面停止距离。** arrival的common-time、STOP协议应答、四轮PE异步末采样和最终尺量时刻不同；编码器响应/采样年龄/轮地滑移/车头测点旋转不能在这里唯一分解。不把末PE减arrival的10～12mm统称溜动，也不据此提前改距离比例或停车阈值。

离线用法：`python calculate_stop_wheel_distance.py snapshot.bin serial.jsonl xd1_records.json --ruler-mm 2105`。这里只是参数格式示例，不是新发车指令或实际尺量声明。

## 验证与资源

- 复用先前本机MATLAB R2020a通过的176B布局、uint32回绕、reset/保持/validmask/相位及arrival旧follow前置；本次没有新数学控制。父目录纯half恢复也已引用原half MATLAB及正常保VM249case前置。原S2.5 B0仍不可唯一复原，B1速度前视仍未运行/启用。
- 原half入口1374case/16614断言已在纯half基点重新通过。本次同一half fixture增加1组32×12字段控制trace：before1375case/16679断言；加18个observer事件case后1393case/16720断言通过。所有原half精确值断言保留，没有使用增强版M1.5/S断言。
- 候选/before的384个真实DistanceCheck控制字段逐字节一致，entry F/L/C和FF观察相同；真实Prepare/Begin/DistanceCheck/CanStep及事件reset/冻结/arrival/拒绝start/STOP保持/跨session/回绕等通过。外部CAN/供电/RTOS/source就绪等有桩，不能当板载/物理验证。
- parser 5组合成检查通过；新增离线轮程差脚本5组合成检查通过（计数标定、异步时间、回绕、未知字段、无效/错session拒绝）。`synthetic_*.bin/json`明确来自ARM fixture/构造记录，不是硬件采集。
- 三目标均0 Error/0 Warning；非专项所有allocated ELF节（含ZI）与纯half基点地址/flags/size/内容hash完全一致。
- sizeof/实际ARM偏移/ELF global均为176B；auto.o ZI增加176B。Integrated整ZI增加184B＝176B对象＋8B链接padding；Offline整ZI增加176B。
- reset静态栈0B，record12B；BeginPrepared48/max376、DistanceCheck96/max192、CanStep200/max584均与纯half相同。静态调用图不等于实车任务栈水位/IRQ叠加或CPU计时通过。

## 最终隔离镜像和读取边界

|目标|Code / RO / RW / ZI bytes|AXF SHA256|
|---|---|---|
|baseline40 Integrated|102328 / 6572 / 2952 / 119096|3966ed37a6c2dd85319884fbe22a2c54a610a9bb9e50f46240e79c006bf1113d|
|baseline40 Offline|101680 / 6564 / 2760 / 119088|6d1a006d7a02c8e92361bc5913a57f9a5c4c8a28d6c5a2e9b6dd8869d9b8405a|

镜像位于`工创赛/tmp/half-stop-observer-build-20261115`，不覆盖纯half或历史2661镜像。Integrated snapshot global为`0x20012d84`，Offline为`0x20012c98`；均176B、getter被linker移除。

未来读取仅能依据匹配ELF的global symbol。需同session、DONE、backend inactive、fault0前后核对，完整176B稳定双读相同；不halt、不reset、不改VM、不屏蔽IRQ、不调用getter。不能证明停止更新/一致性则标unstable不采信；全零/invalid不代表实测零距离。

本child没有硬件、调试器、串口、下载、复位、ARM/START或新发车，也没有生成任何新测试授权。root负责最终high独审/本地commit以及用户范围内的一次恢复下载；用户本轮已取消试车，不再自动执行。原half的较好历史结果不保证在当前场地/版本复现相同数值。后续仍待实际数据才能评价地面停车效果。

R已核RUN-01/PWR-01/MECH-01/02；E LOC-01/02/03/05/07、MOT-02/04/RUN-06；Q FIELD-05/OBS-03/QR-04保留待技术答疑确认。该观察和时间比较是E，不是比赛规则或新增停车保护。
