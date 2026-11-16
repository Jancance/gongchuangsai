# 第二趟实车操作独立审查

2026-11-16；GPT-6.1-sol high，独立于 high 规划与 medium 执行。结论：**RESET_ONLY_PASS**，root 可单独签发一次 reset GO；本签署不包含下载或 START。用户已确认回到同起点、同条件继续，不新增现场确认或测量设备条件。

第二趟独立目录 `inertial-field-run2` 的六份 wrapper 语法通过，候选完整 SHA 独立重算仍为 `d7d66e1dd78fcfc70dceb7d4dc7569c987a87be5687abcf69f42f9b8d9a5d609`。首次审查时没有继承 GO/started/receipt，expected_session 为空；新目录所有阶段 marker 均独占创建，旧第一趟文件保持。细项和哈希见 `action_review_checks_61high.json`。

第一趟 capture 已冻结 state2，重采需要重新初始化，因此此次一次明确 reset 有依据。`reset_once.py` 先核 root 本次 reset GO、完整 AXF/1934输入/客户端身份、实际 UID，attach 后核 RUNNING、全 Flash 逐字节匹配、plan8、active0及故障0；允许前次正常 DONE state9/result1且 VM1/PE2=1 的保持状态。新 exclusive started 后仅一处 `t.reset()`，不下载、不打开串口、不写执行器请求、不自动重试。复位本身会初始化电源/VM，不把该阶段描述为连续保持 VM。

复位后必须使用本第二趟新 monitor/Flash/ready，重新建立 PRE/session0/航向及客户端 token，不复用第一趟 session1、yaw参考、ARM/START token或 readonly_ready。后续仅一次原 PLAN8 forward210_40（2100mm、400mm/s上限、a150、jerk300、RPM100），powered-DONE/20秒尾窗及停后无复位冻结读取；各阶段由 root 分别 GO，runtime 回执另审。

用户第一趟手量“210cm、右0.1cm，手量不准”只记为近似现场观察，不能宣称1mm精度；第二趟也不改变控制或按该误差调参。R RUN-01/PWR-01/MECH-01/02；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。无新控制实现/MATLAB前置项。

运行准备补审：**RUN_ONLY_PASS_RUN2**。独立重算本次 ready 绑定的 reset/Flash/新 monitor SHA 全部一致，路径均属于 run2；COM17/UID6afaae24 等值。一次 agent reset 成功；复位前实际快照为 idle/VM0/PE2=0，按原始证据保留，不推测现场断电或额外操作。复位后全Flash115064B匹配、core前后RUNNING、session0/active0/VM0/PE2=0/故障0，capture state0/PRE100/post0/quality0；新monitor33合法/1坏/0TX，独立调用实际客户端纯健康判定为真。

run wrapper 已核要求 `this_run_reset_then_readonly_ready`、本run2 reset成功回执与新monitor，实际运行再由当前无线帧建立新token；不继承run1 yaw/READY/ARM令牌。root可单独GO原条件一次START，维持powered-DONE/约20秒尾窗，失败不自动重发。复位后session可能再次为1，必须按本run2结果/触发时间/GO来源区分，不用数字唯一性混淆两趟。

实际运行补审：**SOFTWARE_COMPLETION_RECEIPT_PASS_RUN2**。本次新JSONL独立计数 PLAN8=1、ARM=1、START=1、KEEP=48、STOP=0，共51条；agent reset1次、无下载/自动重试。客户端退出0/software_complete，319合法/1坏帧，末active0/轮命令零/VM1/PE2=1/故障0、序列1完成；powered尾199帧/19.969秒有效。轮计进度2100.588mm属于软件量，不当作第二趟尺量。

冻结读取补审：**READBACK_RECEIPT_PASS_RUN2**。独立核本次 readback GO 绑定 run2 summary SHA/PM1 session1；Flash115064B仍匹配。A34020B/B33600B/尾176B实际双读逐字节一致，独立复算哈希匹配，11:18:28读取结束。capture state2/reason1非截断，PRE100＋POST1710＝1810条，wheel293/HWT171/OD69，quality0/源drop0/保留样本gap空；触发107823、首停止115874、冻结116374ms，停止后500ms。尾schema/session1/mask7完整且稳定。run2与run1虽均session1，触发时刻与bank哈希不同，且本次GO/summary来源独立，没有拿第一趟数据代用。reader无reset/halt/resume/目标写入/重试。

本审查代理只读软件/新证据并写审查文件，未触硬件、未外部搜索或Git操作。未闭环：第二趟用户尺量、视频/时标及地面精度；不由软件完成、轮计进度或第一趟近似1mm报告推定。根交接由 root 统一更新。
