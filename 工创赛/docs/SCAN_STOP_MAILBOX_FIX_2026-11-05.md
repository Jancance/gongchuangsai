# 扫码区停车：航向指令邮箱年龄误判

本次E工程修复。实车首因来自`evidence/integration/2026-11-04/integration4-fix/dm-transition-run/uart1.raw`的HF3：tick24100，后端cause3/reason1、故障state5，command_age101ms，status/speed mask均15、feedback_age492ms；航向源有效、角度/速度年龄15/16ms，全局CAN/RTOS/安全故障0。距离估计1069.659mm对目标1050mm、distance_fault_reason0，未到位；车辆随后VM2第1段FAIL/cause8。该轮源区未开始，不归因扫码触发停止或机构故障。

实际后端Auto_CheckLocked使用CHASSIS_HEADING_MAILBOX_MAX_AGE_MS=100，检查最后提交指令年龄；CAN任务在总线事务间也提交航向，因此一次服务延迟即可触发该门。之前只修了HeadingController_HoldStep中的100ms限制，遗漏底盘邮箱独立门限。这次以共享HEADING_HOLD_SOURCE_TIMEOUT_MS=500统一邮箱/会话/控制器截止，数值控制步长仍最多100ms；持续500ms未提交、真实源失效、驱动状态/速度/里程/停止确认仍会停车。不修改CAN事务等待期限或屏蔽底盘故障。

新增实际后端边界场景：在RUN状态分别设置指令年龄101、499、500ms。旧代码在前两项复现失败，新版101/499仍active，500精确cause3/reason1停车；完整整车94场景2193569检查0失败。机构33场景7459检查、FD实际编码回归4检查通过，设备/部分调度模型。历史失败日志保存在revision6/mailbox-before.log。之前手册Scale/驱动FD差异与单位修复保持；DM启动组cause7仍待DF1实物首因，不以本修复代替。

候选正式/离线构建0错0警告，正式Code139228、RO7052、RW3568、ZI126116。实际安装、下载结果见installed.json、actual日志、flash-result.json、flash.log。相对最初基线12个文件修改、1854其他源码/工程SHA保护。软件通过不等于扫码区同版实物复测成功。

已核R：RUN-01/02/04/07、PWR-01、MECH-01/02、QR-01、DSP-01，规则不变；各阈值和修改为E。官方未明确Q仍待技术答疑确认。未闭环：同版扫码区不断行、源区FD目标实读/70mm两段和三机构完成、DM交接cause7、机械与竞赛验收。
