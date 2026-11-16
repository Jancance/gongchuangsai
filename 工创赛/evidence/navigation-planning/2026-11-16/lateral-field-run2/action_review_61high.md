# 第二趟同版左移窄独审

2026-11-16，gpt-6.1-sol high。最终：**SOFTWARE_COMPLETION_RECEIPT_PASS / READBACK_RECEIPT_PASS**。用户已授权同起点/朝向/载物再一趟，本审查只核新会话与动作边界，不重复已通过MATLAB/构建/软件回归，不改参数、不分析融合性能。本代理未连接硬件、串口、复位、发车或Git同步。

沿用第一趟已审AXF `0d5e2b752c92db9094393723b42411cbf02dbdc2f3950ac4c4ca6847fe73242f`、client `c470b3c00d69c4ef0f6b53c0da37a47f6d526eb1fb464e9bd356ec93c6b9da12`、decoder及200Hz采集；PLAN9/kind5/dir1/102预算/2100mm/400参考/150/300/反馈130保持。用户第一趟“向前8.5cm，左移差2cm”保留原话；+85mm/2080mm为当前解释，不当严格同窗真值、不据此调参。

已审repeat_plan_61xhigh.md、reset/fresh preflight/postreset monitor/run/readback路径。当前reset_once SHA `c491dde1d604233bfdd8c170da5fc298df59c05a7c7d138101a9b2693702804b`、run_binding SHA `1e087548235c82ebe4ecc5940999c20a004edf81dae417fbeab487eade7f7ad2`、candidate_binding SHA `a19d3fd291b0dc9b81ebc813c8b1b2672dd46f0dcfb29f620a3e12c31b1dbe2f`独立核一致。

reset前只读核plan9/session1旧冻结capture、inactive/故障0/VM边界，显式引用run1真实下载成功及12:40稳定完整读回，candidate校验其SHA与同AXF/源码；独占started后最多一次t.reset，无下载/串口/START，不自动重试。复位后新PRE/健康状态和运行会话必须来自run2目录，不复用run1 token/ready/session；数值session可能再次为1，身份键必须含新目录及本次reset/START证据。

本次不重复完整Flash读取：fresh preflight只读RAM/ICM配置，采后reader只读bank/tail，映像依据明确继承第一趟最新完整Flash匹配和冻结输入SHA，不能声称run2又作Flash比较。未采前进补偿tail仍如实标记；新expected_session取run2真实PM1。独立读回保持RUNNING/inactive/session一致、稳定双读，无reset/halt/resume/目标写/自动重试。

当前仅签一次reset路径，实际复位/新ready、START和读回待回执。动作预算download0/reset1/START1，无第三趟或回起点。R RUN-01/PWR-01/MECH-01/02，E LOC-01/02/03/05/07、MOT-02/04、RUN-06已核；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。仅本地证据。

## 实际复位与新就绪补审

用户明确“直接复位开始测试呀后面在分析数据”，并进一步要求后续每趟统一直接复位，不因fresh状态跳过。按此最新指令，本趟统一reset1后连续运行、尾收及读回；不逐阶段等待内部签发，不作性能分析。

12:46:52原reset脚本仅读到core RUNNING、plan8/session0/state0、VM0/PE2=0、inactive/故障0，旧plan9/session1前置在t.reset前拒绝；原失败回执保留reset_attempted=false/reset=false，实际reset0/START0。初始化来源未知，不认定用户手动复位或断电。

root已令去除旧会话假设，uniform_reset_once于12:49:56在inactive/无故障下实际执行唯一一次t.reset，回执reset_attempted/reset均true，无Flash读取、下载或串口命令。因此当前实际助手reset总数1。其后重采的新monitor32合法0坏/0TX、HWT4/5ms，新RAM capture state0/session0/PRE100/POST0/quality0、VM0/PE2=0/故障0、core RUNNING通过；readonly_ready为authorized_reset_once。之前省reset提议已撤回，复位前旧ready均归档，不用作本次START就绪证据。12:50:30已接续唯一run，不额外等待本审查包装。

## 本趟实际完成及读回补审

独立读本次原JSONL计数PLAN9=1、ARM=1、START=1、KEEP=48、STOP=0；连同实际reset证据，最终download0/reset1/START1，无自动重试/第三趟。result_summary return0/software_complete、331合法0坏，末plan9/kind5、active0/四轮零/故障0/VM1/PE2=1；powered尾199帧19.969s有效。只签软件健康，不推断地面测距表现。

独立核capture/read_receipt、decoded和6个原bin：A34020B/B33600B/tail176B各两读逐字节相同，长度及重算SHA与回执一致；expected_session/header/本次PM1均为run2真实session1，不能与run1数值1混配。readback明确flash_read_performed=false、flash_checks空，身份依据继承run1完整Flash匹配与冻结输入SHA，未声称本趟又作Flash比较。

RAW2032=PRE100+POST1932，wheel338/HWT194/OD78；quality0、各源drop0、retained_gap_indices空。reason1、terminal_tail_completed真且未截断；trigger38105、首STOP46189、terminal47267、freeze47767ms，终态后500ms，RAW支持−499..9657ms。前进补偿tail176全零按PLAN9未采如实保存，非独立RAW失败。

- bank A SHA：`8e0176c3ec1539f56110b000a9506e900fb4bb1f70121ffee20101ff720ecbd6`
- bank B SHA：`a2e653ecd857867a89a7b08b7a3179152f8dbbe9088672690343ac2b6a48c37d`
- tail SHA：`86d2cf5b090f43ee54d8f7c1dcf746a853951191457ff6dac96269a9d24860b9`

数据已安全保存，root已掌握可断开/复位通知；尺量前不移车。第二趟地面尺量、物理静止同步与融合性能尚未评价。本轮按用户“后面再分析数据”只完成操作次数、故障和完整性审查，没有新MATLAB重放、性能对比、调参或追加硬件动作。仅本地审查完成，无Git上传。
