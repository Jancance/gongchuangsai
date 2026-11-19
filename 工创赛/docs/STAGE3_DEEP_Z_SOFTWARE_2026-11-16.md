# 第三段 deep-Z 本地软件交付（2026-11-16 额度恢复后）

实现代理仅完成隔离候选及离线验证；主代理随后已实际换版，事实见部署追加。没有提交或推送。准备入口 `scripts/prepare_stage3_deep_z_20261116.py`，构建入口 `scripts/build_stage3_deep_z_20261116.py`；部署清单 `evidence/integration/2026-11-16/stage3-deep-z/candidate.json`。旧815候选清单未覆盖。

E：SPF不可变命令锁存第三段范围82000，0保持原料29700；第三段X106、Z1846.4、DM300，原料X150不改。IT3实际分支加速600和IT4返回45055保留，低速回零不改。LRC仅显式StartReturnWithLimit锁存本次返回范围，旧StartReturn默认29700、旧StartDown/SetDownTarget未放宽。前9次复用真实顶部回到top+1800，第10次重新回零并退出600。

第三段准备phase0和state5 phase1/6/11采用现有真实it_at_top判定，已经满足时仅推进编排，不发重复IT4、不写LRC DONE；真实深位置仍发IT4。LRC零delta故障与STOP应答保留。只改8文件，main、原料流程、参数表及motion_control不变；原文件字节定点编辑，生成候选与src一致，准备脚本独立重建8文件逐字节一致。

前置MATLAB9组已通过（matlab-result.json，含重复IT4）；本轮C检查：deep143，适配后的owner3422，实际owner+实际it_at_top谓词+真实LRC56（含600/1800参考、两scale、深返回、零delta拒绝、无反馈不跳过、第9/10次分支），当前80p home-exit560，group10。旧owner测试夹具顶部假定和旧home-exit32p只在生成夹具中修正，不更改实参。双Keil在线/Offline均0错0警告；证据checks.json及两build.log。在线AXF SHA fdbb4d2f80a2ba0d39032fb51fd55fe85344e6506f6a69ccf427f9462f3ed918；Offline d6d3eff1c30bc76f610497f599155c85d44ed9b7f9aefa66932e2918ab53fab1。高审代理确认无本轮阻断。

用户实量205mm为顶部夹爪最低端到地面，82000是第三段业务范围，不是已验机械极限或触地目标。放料高度仍未知，只能按授权分段示教，不自动下降至地面。B3/TCP、真实毫米/偏角精度、取槽/放环高度、中心单件/左右/三件仍待实物闭环。本记录不代表已烧录或实测。

已核R：QR-01/05/06、RUN-01/02/03/04、FIELD-06、PWR-01、MECH-01/02；E：VIS-04/07/12/15、CAL-02/03/05、RUN-06。官方Q继续待技术答疑确认。本轮分段示教/人工载物是工程调试流程，不是正式比赛规则。

## 2026-11-16 05:38 · deep-Z已换版，待PC13重建观察

- 当前车上为deep-Z `fdbb4d2f80a2ba0d39032fb51fd55fe85344e6506f6a69ccf427f9462f3ed918`。主代理05:35:49 Keil Verify OK / Application running；05:36:18完整193324字节读回匹配，stage3 state0/fault0、IT request0。用户换版前已重新确认承托、停稳空爪及载盘215不变。旧815和05:26 checkpoint保留历史，不再代表当前版本。证据 `evidence/integration/2026-11-16/stage3-deep-z/flash.log`、`stage3-live-20261116-053618.json`。
- MATLAB前置9组、实际C143/3422/56/560/10、双Keil0错0警告及高审通过，prepare独立重建8文件逐字节一致。第三段82000/X106/Z1846.4/DM300；原料29700/X150、原分支加速度、低速回零不变。准备与ensure-top采用真实it_at_top跳过重复IT4，不伪造LRC DONE；零delta拒绝、故障、STOP保持。新manifest/checks位于stage3-deep-z，旧815manifest未覆盖。
- 05:37:23仅写center-only first_item1/count1，无动作请求，证据 `stage3-live-20261116-053723.json`。已通知用户两次PC13：首次仅已有215载荷转位，无需重装；盘停后撤承托，第二次重建参考到180观察。05:38:24结束30秒只读watch仍state0/fault0、IT request0；尚无按键/观察物理确认，不能记为已回零/已到观察姿态。证据 `stage3-live-20261116-053824.json`。
- 新deep尚未实际下降、抓放或验证深度回顶。205mm是用户实量几何，82000是业务范围，不是直接触地目标；后续分段示教。B3/TCP、真实毫米/偏角精度、取槽/放环高度、中心单件/左右/三件仍开放。工具已支持显式新manifest及同连接轮距离只读证据，10测试通过；轮距离命令积分/反馈不认作尺量实走。
- 核R QR-01/05/06、RUN-01/02/03/04、FIELD-06、PWR-01、MECH-01/02；E VIS-04/07/12/15、CAL-02/03/05、RUN-06；官方Q继续待技术答疑确认。根已fetch仍21522db；成果仅本地，未提交推送。专题 `docs/STAGE3_DEEP_Z_SOFTWARE_2026-11-16.md`。

