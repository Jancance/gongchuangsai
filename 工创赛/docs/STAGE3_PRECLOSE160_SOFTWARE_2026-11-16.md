# 第三段160°预收候选（2026-11-16，仅本地软件）

## 2026-11-16 06:53后接续补记（以本段判定当前进度，旧段保留历史）

用户已明确接受Z8000适合槽2红料抓取。06:36:27 IT3/sequence6完成时actual_depth=8003.46728515625p、state7/fault0，160°预收已确认；本轮尚未140°夹紧或抬升。下文“用户尚未明确接受”“当前仍fa7/未下降”等为旧时刻，不能继续代表本次进度。

06:40:45 teach-kind1请求completed=false，pick_depth仍−1/teach_result0。DM保持吞read_request2，等待新DM cycle未完成；Zdiag done0/result全9，raw0无效；Zdiagraw还存在uint8声明与真实uint32/192B不符。不能声称MCU示教保存成功，固件问题未修复。

06:52:32最终只读证据`evidence/integration/2026-11-16/stage3-preclose160/teach-readonly-20261116-065232.json`核对：VSP current903.8099975585938°与用户接受的063627记录一致，IT sequence6/完成状态一致，真实LRC top_valid1/raw342/scale100；hardware_writes为空、physical_motion_requested=false、fresh_poll=false，失败Zdiag不参与。导出配置`evidence/integration/2026-11-16/stage3-preclose160/teach-readonly-20261116-065232-pick-config.json`只含g_stage3_pick_depth=8003.466644965279p。历史actual8003.467285与double派生8003.466645/float存储8003.466797存在远不足1p的计算/float量化差，并非完全相等或机械校准变化；均约8003.47p。前文“float32精确复现历史”属于导出前的方案描述，以本段实得数值为准。

06:53:14 `stage3-live-20261116-065314.json`记录--restore-calibration completed=true/first_error=null，完整Flash193556字节匹配a54eb0ab，writes仅g_stage3_pick_depth，恢复后读回8003.466796875p。没有运动请求或healthy/holding、参考/状态改写，teach_result仍0；这是配置恢复成功，不是06:40固件示教成功。没有夹紧请求；随后对话中断，140°夹紧及抬升均未执行。

主代理后续只读连接失败Expected 1 probe got 0，没有新快照；最后已知槽2/160预收/接受取高/state7不能继承为当前live。当前连接、供电及位姿未知，已请用户接好并说明是否断电/移动，等待回复后先只读核对。本补记未连接探针、未改代码或触发硬件。R已核RUN-01/02、PWR-01、MECH-01/02，E CAL-02/03/05、RUN-06；Q仍待技术答疑确认。固件示教问题、夹牢/抬升、放环高度/左右几何、B3/TCP及中心单件/三件仍未闭环。仅本地文档，未提交推送。


2026-11-16现场更新（主代理证据）：06:16:45已下载a54候选，06:17:18完整Flash快照匹配。用户完成两次PC13并确认180观察，06:22:41进入teach；槽2/DM36.5后06:25:58执行IT17预收，用户确认。随后Z目标3200→4800→8000，06:36:27 IT完成/fault0，用户确认8000适合红料抓取。06:40:45 teach-kind1超时，pick_depth仍-1，不能记为保存成功；不再下降。

只读源码与日志诊断：Z/X诊断cycle从0到1，DM cycle始终5。DM位置保持时，ModuleService主动吞read_request2，因此S3_Teach一直等待新DM cycle，尚未进入fresh判据。另发现S3_Teach声明Zdiagraw为uint8数组，但实际adapter定义为uint32数组（ELF192B），现固件示教解析也需另行修正；本轮不借此伪造成功或反复换版。只读host工具按真实宽度读取已完成Z诊断、DWARF解析真实顶部参考，归档后可生成仅含实测pick_depth的配置，使用现restore入口仅回填配置，不改teach_result/健康/holding。挂起teach在下个真实IT忙态自然取消，不能单独DM新读触发旧pending。此host改动15测试通过，尚须主代理执行实读并核对结果；没有新增固件阈值或修改保持状态。

06:47只读补充：Zdiag实际done0/result全9，准入未通过，因此没有可用Zdiag测量，不能用其raw0生成高度；Xdiag成功。主机首次保存因此没有导出配置，已将缺失值异常改为明确unavailable说明。后续采用用户已确认的06:36:27完成IT3/sequence6的真实VSP历史反馈记录：current903.8099975585938°、actual_depth8003.46728515625；由live只读VSP.current及LRC顶部引用交叉核对。导出须保持ITseq/完成状态一致，current与历史值逐float一致、top_valid1、float32计算精确复现历史actual_depth。明确标记fresh_poll=false、失败Zdiag不参与，不把目标8000当实际反馈，不触发新诊断请求。高审接受此记录方法，主代理执行后再记配置保存成功。

## 最新实际部署补记（主代理现场证据，2026-11-16）

主代理已完成本候选a54eb0ab下载及完整读回匹配，初始state0/fault0；此前USB超时和“未部署/板仍fa7”描述为历史时刻，不能继续代表当前状态。下载、读回及后续stage3-live JSON保存在本专题证据目录。用户之后确认160预收正常、180观察无碰擦；槽2红料对准，按用户指令Z3200→4800→8000分步完成、fault0、未夹紧。8000是已执行的示教深度，用户尚未明确接受其为最终取料高度，不能提前保存为已验取高。

统一桌面55项包已由主代理安装并启动PID11517；真实最新observations为stage3/run2/state7/pose5/item0/colors215、align_done0/config_ready0、preview_only=true、calibration_verified=false、observation_wire=null、motor_commands_sent=false。属于第三段预览与真实状态接收，不是视觉标定或抓放通过。新59项日志/截图可观测性候选仍在本地，等待主代理增量审查与部署安排。详见STAGE23_DESKTOP_2026-11-16.md。


最新部署状态：软件验证与high最终审查已通过。用户已明确重新承托并授权下载；主代理下载前读取旧板时遇USB连接超时10060，未开始烧录、未写动作请求。已请用户只重插探针USB，保持主板供电、承托和物料槽位。最后成功只读快照为06:00:39：旧teach `fa7b7e6f`、state7/fault0、DM36.5°/X0/顶部；此后连接失败期间的新鲜硬件状态未知，不能把候选记为已部署。本子任务不并发连接探针，后续下载由主代理执行并核验。

本轮任务聚焦第三段：固定槽取料角到位后，夹爪由210°预收到160°，再下降取料、140°夹紧、升顶，按该颜色对应的任务目标环转运。颜色→任务目标环和载盘颜色→槽角是独立映射；当前215/123仅为测试配置，不能把槽号固定等同于环号。原料业务与速度、加速度不改。

## 本地实现与验证

- 独立工程 `tmp/stage3_preclose160_20261116/project`，基线为teach候选；实际仅两个include变化。未安装到外部工程、未烧录、未复位、未发硬件指令、未推送。
- IT17仅Stage3拥有时准入，固定160°；复用IT5/6的PWM写入错误检查和既有600ms完成窗口。这个窗口不是舵机实际角度反馈；拒绝写入仍拒绝7，不记录DONE。原IT16自动流程特判保留，源区IT17仍拒绝。
- 第三段每件的固定槽DM到位后、下降前插入IT17；其余阶段顺延，真实顶部判据相应为1/7/12，保留回顶与STOP。普通速度仍Z1846.4RPM、X106RPM、DM300°/s，原分支加速度不变，原料170°等保持。
- 主机工具 `scripts/stage3_live_20261116.py --grip preclose` 映射17；须明确指定声明 `preclose_request:17` 的新manifest。旧manifest在导入/连接硬件前拒绝，不会向旧固件误发新请求。
- MATLAB原前置结果为05:57:37（原会话工具读取JSON，早于05:58:37实现）；本轮真实MATLAB复核06:02:45通过4组并保存console，原JSON时间已被复核覆盖，不伪造旧console。
- 实际C：IT准入/5/6/17共46检查；第三段owner3907检查，覆盖每件预收位于顶部/DM36.5后且先于夹紧、36任务映射和STOP；实际LRC/top56、deep143、home560、原料并行group10均通过。硬件/时钟在这些测试中模拟，不等同实车。
- 主机14项测试通过；在线/Offline Keil重建均0错0警告。新在线AXF SHA256：`a54eb0abecca02949ee8e68a0c133b0e893cb77ef2766d9cd32d7864e7d5ea74`。证据位于 `evidence/integration/2026-11-16/stage3-preclose160/` 的candidate、checks、各C及build日志。实际PWM、舵机位置、运动高度和抓放尚未验收。

## 当前实物接续与未闭环

- 当前板仍teach `fa7b7e6f`，state7；槽2红料用户已确认对准，DM36.5°、X0、顶部，尚未下降。本轮没有改变该状态。
- 2号环为DM180°/X0。205mm为顶部指尖距地实量；物料底低于指尖约50mm是用户估计，因此155mm相对下行仅估算。当前真实顶部depth600加62000得到62600也只是候选，不能直接作为精确放料指令；须实际放料示教。
- 1/3环采用用户手摆DM、软件读取新鲜角，X由用户尺量，不用几何推算代替实量。手摆前复用已有jog_request3发送真实0xFD，确认motor0/holding0/state8/result2/stop_result1；手摆后read_request2读取新鲜角，不写0xFE零位。本轮只记录后续方法，不新增DM动作API。
- 未闭环：160预收实物、取料/放料Z、1/3角与X、带料扫掠/净距、B3/TCP与视觉纠偏、中心单件和三件完整循环。

## R/E/Q与需求核对

R已检查：QR-01/05/06、RUN-01/02/03/04、FIELD-06、PWR-01、MECH-01/02；工程改变未覆盖这些规则事实。E已检查：VIS-04/07/12/15、CAL-02/03/05、RUN-06；160预收、600ms软件窗口、155mm估算和示教方法均为工程选择/候选。官方未明确内容Q继续保留“待技术答疑确认”，不将本轮工程选择写成正式规则。
