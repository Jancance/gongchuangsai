# 整合13最终本地冻结交付（2026-11-09）

本轮三件现场确认通过：第三次松爪命令17.987秒，回0°并软件完成19.185秒；三槽各一件放稳、无碰擦、已停稳。计时不含建参考；第三段未启动。

- 完整源码：project/gongxun-f407.uvprojx，目标Vehicle_LiftRuler及Vehicle_LiftRuler_Offline。
- 固件：images/下冻结AXF，哈希见candidate-manifest.json；实际板为在线8da8171c…，Flash166752字节全匹配。
- 参数：下180°/0mm，左195°/156mm，右165°/160mm，共用放料37°，松爪后至少200ms再转；两处提前5mm、带料回升5mm开始回收，末段转臂与回收重叠。
- project内保留原文件编码/换行；不含Objects/Listings和用户调试布局。外部实车正本、根旧STM32工程未覆盖，未提交推送GitHub。
- 验证及现场原件副本：verification/；完整历史证据仍在仓库evidence/integration/2026-11-07/stage2-mainline/。
- 现场入口：仓库scripts/integration13_empty_overlap_live_20261109.py --three-immediate-once。需要匹配实际Flash、真实参考、摆料和一次授权；默认/--check不接探针。故障保留首故障并发IT9停止，不自动重试。
- 收尾：4310约0.0219°、伸缩−0.3°、升降809.7电机度，夹爪开、三槽有料、动力PE2=1。断电后不继承参考，不盲目再运行。
- 尚未闭环：视觉毫米纠偏、颜色/实例驱动、动态停盘、正式整车使命/真实库存证据提供者、第三段路线与行驶中回0°。本包是已验机械基线，不是完整比赛程序验收。

已核R SRC-01/02、RUN-01/02/04/07、FIELD-06、PWR-01、MECH-01/02；E SRC-04、CAL-02/05、VIS-15、RUN-06、MECH-06；官方Q仍待技术答疑确认。详见根交接和docs/INTEGRATION13_SUB20_CONTINUATION_2026-11-09.md。
