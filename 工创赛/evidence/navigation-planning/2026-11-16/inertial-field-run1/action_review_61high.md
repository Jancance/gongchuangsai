# 单次实车操作独立审查

2026-11-16；GPT-6.1-sol high。当前结论：**DOWNLOAD_ONLY_PASS**。用户已明确“先下载”，协调 root 可签发本次 download GO；此结论不包含独立 reset 或 START GO。下载工具自身会初始化 MCU，该影响由 root 向用户说明。本审查代理未打开串口、连接目标、下载、复位或发送命令。

完整候选 AXF SHA256：`d7d66e1dd78fcfc70dceb7d4dc7569c987a87be5687abcf69f42f9b8d9a5d609`。本次独立重算与既有 0 错 0 警告构建回执一致；隔离工程 SHA 亦一致，沿用已独审相同运动 PT_LOAD 的结论，不重复构建或展开旧 RAW。检查及文件 SHA 见 `action_review_checks_61high.json`。

下载阶段已修正并复核：`run_binding.validate('download')` 核候选、1934 输入、实际设备 binding 与 root 签发的独立 GO；下载独占创建 `download_once_started.json`。真正执行 Keil 前重新只读枚举全部探针，必须恰好一个，且 UID 与本次 binding 精确匹配。Keil 的 Any 模板因该实时唯一设备检查得到限制，不继承历史 UID。一次 `UV4 -f` 仅编程/验证，30 秒有界，保存返回值、完整日志、Erase/Programming/Verify/Application-running 标记及下载前后完整候选 SHA；任何异常保留首次回执，不重试，不串联旧 campaign、reset 或串口动作。当前实际现场枚举 UID 为 `6afaae24`；必须由 root 写入本次 binding/GO 后才可执行。

其余阶段静态检查：R/S 路径正确，六份 Python 语法通过。monitor 固定 3 秒纯接收，实际客户端 monitor 分支不产生 TX、不设置 DTR/RTS；root 授权的实际 monitor 由执行代理记录，审查者未重跑。reset 入口会在 Flash 核验后立即执行 reset，不能误当纯只读 attach。run 固定 mode3/PLAN8、forward210_40、powered-DONE、30 秒活动窗与约 20 秒接收尾窗，沿用 PLAN/ARM/START/KEEP/STOP；START 不重发，各阶段独占 started，无横移/转向或自动循环。正常软件 DONE/零速/VM 保持与现场停稳必须分开记录。

采后读取仍须按当次情况确认：现场停稳、留电、不移车、不复位，expected_session 来自本次非零 session；现有 reader 核完整 Flash、RUNNING/inactive/session、冻结 bank 稳定双读及独立尾对象，不 flash/reset/halt/resume/写内存或重试。已补核 readback 子进程显式传入 turn_probe_deps 的 PYTHONPATH，并要求 session 等于本次 result_summary.last_position.session 且非零；静态修正通过，真实冻结读取仍未执行。

已读取本次 monitor 回执：COM17，37 合法/0 坏帧、0 TX、idle/VM0/故障0，最后 HWT angle/rate age 为 18/19ms；但该板最后 plan0/limit170，不是候选 plan8，不据此确认新版本。现有 run_once 对该旧 preflight 的 plan8 健康断言会拒绝执行，需要独立保存下载后的新只读监测回执再评估 run 准备；该项不阻塞本次仅下载。

已核 R：RUN-01/PWR-01/MECH-01/02；调试无线与正式自主模式分开。E：LOC-01/02/03/05/07、MOT-02/04、RUN-06。本次沿用已有运动配置，无新控制实现/MATLAB 前置仿真项。Q FIELD-05/OBS-03/QR-04 仍待技术答疑确认。

下载回执补审：已核 `download_go.json` 为 root 对用户“先下载”的本次独立授权；11:04:04 开始的一次下载，实际临下载仅 UID `6afaae24`，返回0，日志于 11:04:20 结束，Erase/Programming/Verify/Application-running 四标记均真，AXF 前后完整 SHA 保持。**DOWNLOAD_RECEIPT_PASS**；可报告一次 Keil 下载/验证成功。该证据是 Keil 验证回执，尚无独立全 Flash 读回；目录仅 monitor/download 的 started，未见独立 reset/run started，回执亦记录无串口打开/ARM/START。审查者仅读取这些新回执，没有触硬件。

后续只读与运行准备补审：**RUN_ONLY_PASS**。独立核 `readonly_ready_receipt.json` 的 Flash/新 monitor 来源 SHA 全部一致；真实 PT_LOAD 115064B 全匹配、前后 RUNNING，idle/VM0/PE2=0/故障0、backend session0；capture state0/session0/PRE100/post0/quality0，未触发采集。源码 state0 持续滚动 PRE，只有 START 才转 state1，因此下载初始化足够，本次不需要额外 reset。新 monitor 独立 stage/marker/输出，36合法/1坏/0TX；最后 plan8/RPM100/limit130/目标40000/序列1/HWT17与18ms，独立调用实际客户端纯健康判定为真。

当前 `run_once.py` 已改绑新 readonly_ready 初始化策略及 postdownload_preflight，不读旧 monitor，也不强求 reset 回执。运行仍必须 root 独立 run GO、starts_authorized=1/site_ready=true，独占 run started 后只调用一次既有客户端；fresh stream 再用当前帧检查健康、PLAN8/ARM 后生成一次 START，KEEP/STOP 和 powered-DONE/20秒尾窗保持。不包含横移/转向/重试。此签署仅表示可由 root 放行既定一次前进，不表示审查者发出命令或已经发车。

运行回执补审：**SOFTWARE_COMPLETION_RECEIPT_PASS**。独立数本次新 JSONL 中 TX：PLAN8=1、ARM=1、START=1、KEEP=48、STOP=0，共51条；未独立 reset，未重试或追加动作。客户端退出0、software_complete、321合法/0坏帧、末 active0/四轮命令零/故障0/VM1/PE2=1、session1；powered 尾199帧、20.031秒验证通过。该证据区分软件完成与物理停稳，不据命令零或轮计进度2100.030mm声称地面2100mm准确。

冻结读取补审：**READBACK_RECEIPT_PASS**。已核 root 本次 readback GO 绑定 result_summary 实际 SHA/session1；实际 reader 回执 accepted，Flash115064B匹配。独立复算 A34020B/B33600B/尾176B 长度、哈希及各自两读逐字节相同，冻结 state2/reason1/session1，非截断，PRE100＋POST1710＝1810条 RAW，wheel298/HWT171/OD69，quality0、源drop0、保留样本 gap 列表为空。触发366846ms、首停止374897ms、冻结375397ms；首停止后500ms冻结，不保证覆盖物理停止。尾对象 schema/session1/mask7完整且稳定。读取无 reset/halt/resume/目标写入/重试；审查者只读新文件，没有硬件动作。

未闭环：用户现场视频/同点地面尺量及其时间不确定范围、B0/BM 本机 MATLAB 同窗重放。此次只签软件完成与记录完整性，不做 ground 判优，不签惯导闭环或比赛性能。仅本地审查文件，无 Git 同步/提交/推送；根交接由 root 更新。
