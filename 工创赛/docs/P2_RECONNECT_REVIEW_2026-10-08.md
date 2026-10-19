# FOLLOW断连恢复证据核对

## 日志补齐后的当前结论

用户已补齐camera-reconnect-console-20261007.log（34655字节，SHA256 64acb7d5447bedd159eac38464050b49154d07bcdf8ac101ca26e33df7d83181）。完整日志包含三轮，已仅提取071416-797897对应第166～282行至output/v13-reconnect-review-20261008/console-selected-run.log。该轮2次Camera connected（初次及重连）、10次Camera unavailable、10次Mode=FOLLOW Camera=WAITING，随后FOLLOW/targets=1恢复；没有Mode changed或TURNTABLE状态。VIDIOC_REQBUFS errno=19为拔设备期间No such device，与本次注入相符，不能当正常持续运行告警。

结论：本轮单次USB断连被检测并进入FOLLOW等待，重新连接后恢复正确green输出，历史重新建立且首条无旧速度/预测，恢复后持续约53.415秒。本项有限范围的断连自动恢复行为通过。等待画面由既有代码黑底重绘提供，用户本轮未另附等待截图，不据终端日志单独认定每个GUI刷新细节。整机相机verified不变，旧帧/曝光到结果实际时延、5分钟连续稳定、其他故障仍分别待验。

日志没有逐行时间戳或物理插回时刻，因此精确插回到首个green耗时仍未测；21.276秒只是前后有效观测间隙。早期T001→T002也没有相机或模式变化日志来解释；同模式按F会清历史但不打印Mode changed只是代码可能路径，不能据此认定用户按键。该未解释重置保留，下一项无操作连续运行检查是否重现；不改算法，也不要求再次拔插。

下一项为[5分钟连续运行](P2_FIVE_MINUTE_STABILITY_2026-10-08.md)。以下缺日志/下载命令段落保留为补齐前历史，已由本段和更新后的report.json替代。

## 已确认的结果

证据为20261007-071416-797897.zip、用户“插回去有恢复”及FOLLOW/T003 green/21.7FPS截图。ZIP通过CRC，SHA256为4256b95c5d125ba894ac2437b990e223f4a8d3d6751176631b5e2c0c919ee6df。检测器/颜色配置匹配V13，检测、跟随、跟踪、相机及诊断入口哈希均与上轮黄黑V13记录相同。归档脚本scripts/archive_v13_reconnect_review.py，报告output/v13-reconnect-review-20261008/report.json及同目录配置/模块快照。

1542条有效检测记录跨度91.356秒，全部FOLLOWING_PIXEL、恰好一个green，没有其他色目标或时效拒绝。主机read完成到检测结果年龄中位数57.852ms、95分位73.910ms、最大108.274ms；检测调用95分位40.733ms。不是曝光到动作端的延迟。全记录ready=False/motion_allowed=False，run中Serial=DISABLED、正常退出。

第365→366条之间约21.276秒没有有效检测行，T002→T003。恢复首条为CANDIDATE、consecutive_observations=1、velocity_px_s=None/predicted_px=None，支持新建历史而没有沿用旧速度/预测。其后1177条约53.415秒一直恰好green/T003，支持恢复后持续检测。

此前第79→80条也有T001→T002，间隔仅0.062636秒、位置接近，新条亦为CANDIDATE。原因尚未确定，不能将两次换号都当USB重连，也不能直接推测用户按F；需终端模式/相机事件记录核对。物理背景由用户明确确认为地面A4白纸，没有实物转盘；run的Desktop scope为旧诊断措辞，不覆盖用户实物说明。

## 证据限制与当前下一项

本次自动恢复正确绿色输出有用户实物反馈、截图及检测历史证据。现有诊断只在有效FOLLOW帧写observations，断流等待阶段没有行。因此run的longest_watched_loss_s=0仅表示已处理帧中没有green缺失，不等于无断流。21.276秒是前后有效观测间隔，包含拔掉等待、重新插入、设备初始化等，不能称插回后的恢复耗时，也不能推算实际拔掉了多少秒。

预期配套evidence/camera-reconnect-console-20261007.log没有下载到本地；已查项目文件和用户Downloads中的reconnect名称，ZIP内部也无该日志。同轮模式变化、Camera unavailable/connected、旧框撤销过程及恢复时间未完整核对。无需重复拔插、无需修改算法，先仅下载原日志：

```powershell
scp "orangepi@10.31.154.211:/home/orangepi/p2-integrated-20261003/camera-reconnect-console-20261007.log" "D:\Codex\工创赛\evidence"
```

以上在电脑Windows PowerShell运行。若提示远端文件不存在或连接失败，保留错误原文再处理，不假定日志存在或自动恢复失败。当前使用本会话已知IPv4，未再次联网测通。该日志使用tee -a，可能混有多轮；后续必须根据Diagnostic folder=20261007-071416-797897定位对应轮，而不是合并全部模式变化作为本轮。

完成本轮日志核对后再选择计划C剩余采集门：JPEG坏帧/旧帧与真实时延边界、长期稳定性，以及另行登记的前视/扫码输入；不能因单次重连成功将相机verified改true。最终安装、毫米/TCP/运动补偿及实际跟随抓取仍独立验收，进入安装或机构/真实场地前先提醒。

## 需求检查与验证范围

R：FIELD-07垂直向下补光，本次未改灯具，不新增正式规则。

E：VIS-03、VIS-12、VIS-15、CAL-02/03、RUN-06。对应证据为版本/配置核对、1542条新鲜度与动作禁用记录、重捕获首帧清速度/预测、用户恢复反馈及截图。配套日志、等待阶段输出和恢复耗时仍未闭环；触摸/强反光/边缘、手部误报、多实例、曝光时间/运动补偿、实际毫米及机构继续未闭环。依用户前移决定保持V13，不开启新一轮颜色补丁。

Q：QR-04仍待技术答疑确认；其他官方未闭环Q不因本次测试改变。

本次仅归档及计划更新；未改算法、配置或串口，未发送动作。归档脚本执行成功，ZIP CRC和版本/动作状态断言通过，知识审查无冲突；这些软件核对不替代物理验收，不重复历史127测试。
