# half几何前馈控制回退基点（保留当前正常DONE保VM）

日期2026-11-15。本目录是用户要求停止后续实车测试、回退到既有较好数据版本时形成的**第一个本地可恢复基点**。用户随后又要求“在原来代码基础上增加停止的测距”，该第二步须在root提交此base并greenlight后单独交付；本报告不把后续observer混入纯half基点。

## 恢复内容及来源

仅精确恢复四个当前统一入口源文件，没有复制整工程：

|文件|恢复来源|SHA256|
|---|---|---|
|chassis_auto_test.c|bounded-lateral-enhancement/chassis_auto_test.before.c|4434979a40f940d8ee582378e5b57b45da8caeefb29bdde1558d1b17e85dac19|
|chassis_auto_test.h|float-timing-20261115/chassis_auto_test.h.before|e940c64b4a5126f63a6cc6e4d814be2ef3096bd0d59f2e049d6a53257033b764|
|hwt101.c|float-timing-20261115/hwt101.c.before|690b864c1144baf247a93cffb8be585675a0612002333edaf006026b5dbb3c90|
|hwt101.h|float-timing-20261115/hwt101.h.before|eb01b7feb7e18c73495c472b09497514ab14de6941013683315b61f929b56429|

恢复前四源逐字节归档于`recover_previous_diag`，可以恢复此前ada294…三快照诊断/计时源状态；本轮没有自动重应用。备份与恢复前后hash见`source_restore_receipt.json`。auto为half几何前馈原控制，撤回本轮M1.5/S增强和float/tail插桩；APP参数、130RPM、PID、距离、HWT/ICM基准及其它未提交模块未改。

**正常DONE保VM和现client握手修复继续保留**：app_motion_runtime.c仍`2cded70cccf56c9464a0787be6656e81aad5e9a9d9334620d3c5a958b5eef611`，app_serial_runtime.c仍`d99860e5266f1e56156f41f88452895aff55d20a35459649ca97f72facdfbca4`。因此版本标为“half控制＋当前正常DONE保VM/串口握手”，不能声称与历史`2661174f…9c1e7`整AXF逐字相同。

恢复auto确实由2026-11-14的half build与四份conservative-ff-run1..4原auth绑定，原镜像SHA2661174f…9c1e7。已保存尺量右向带符号为3.5/−.5/10/7.5mm，描述性MAE5.375mm；多走2.5/2.5/2.5/1mm。这是已有四趟事实，不是本轮新实测，也不等于不同批次之间已排除混杂的因果结论。

## 本地验证

- 1731份源依赖前后核对，仅上述四源改变；当前所有after hash再次核对一致。四源GBK字节保持；auto/hwt为原CRLF，header恢复原混合行尾。
- 使用原half的`2026-11-14/.../field-candidate-conservative-ff/check_actual_entry.py`入口，在独立新临时目录重新编译真实ARM C并执行，1374case/16614断言通过。没有把增强版M1.5/S精确输出断言硬套half。entry与FF观察输出和历史half回执一致。
- 没有新数学控制，引用已完成的half实际MATLAB选择验证及正常保VM249case/2248断言前置。VM/serial源hash与其既有actual-C通过证据严格一致；不重复无关大矩阵。引用路径及hash列在最终回执。
- 从统一uvprojx生成隔离项目，未覆盖旧镜像；baseline40 Integrated、baseline40 Offline、专项关闭Integrated均0 Error/0 Warning。
- 专项关闭镜像的全部allocated节（含ZI）与回退前浮点基点保持地址/flags/size/内容hash一致。恢复后不存在新增lateral timing或tail snapshot global；旧HWT计时协议仍3scope/28fields。
- ARM/编译证据使用外部CAN、供电、RTOS等桩，不能代替真实板载动作、启动成功、车体距离或保护验收。本轮未修启动故障。

## 隔离镜像

|目标|Code/RO/RW/ZI bytes|新AXF SHA256|
|---|---|---|
|baseline40 Integrated|101744 / 6572 / 2952 / 118912|2e4cd7946dcfa537fdf29eea1b0f6514916b5913b00f66020dec064883ae5d6b|
|baseline40 Offline|101096 / 6564 / 2760 / 118912|9ea061191a743e4820615a6ebe5ee6997d004fc5aeb2b1dee51539148317d21a|
|专项关闭Integrated|11974 / 470 / 960 / 25576|66935b32e2507bca7c9ee03ac027d925512d0b35e5c963a05290b033b2c61cd7|

路径在`工创赛/tmp/rollback-half-ff-build-20261115`，完整记录为`build_summary.json`和`final_verification.json`。此base只在本地完成，child没有Git操作、下载、SWD、串口、复位、ARM/START或新发车，也没有生成任何新硬件测试授权。root负责独审/本地提交，以及后续明确的half＋仅停车轮端距离观察第二步；本轮不再实车。

需求核对：R RUN-01/PWR-01/MECH-01/02；E LOC-01/02/03/05/07、MOT-02/04/RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。既有控制恢复与正常保VM保留均属工程选择，不修改比赛规则。
