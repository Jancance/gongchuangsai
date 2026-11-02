# 距离动作保电换段修复与复测（2026-10-25）

用户要求用HF2复现并修复，明确重新下载。首次下载失败，电脑识别DAP/COM17但无遥测；用户检查恢复供电/连接后，14:04:39 HF2诊断版重新烧录Verify OK/Application running，新会话健康后按用户允许单次测试。

## 原故障及修复

左移30cm完成后进入前进准备，再次中止。HF2 backend_fault_reason1、原state4 VERIFY_ON、命令年龄107ms，状态/速度mask均15、最旧反馈106ms，HWT源有效/帧龄9/10ms、错误全0。明确是命令邮箱100ms过期分支，不能继续泛称查询失败。

代码链：MoveStart清前端但ReleaseCompleted保留后端上一段distance_mode/reached用于历史和交接。App_ForwardHoldSafety在HR_WAIT与HR_RUN均运行，原先不验证阶段/会话就接受distance_reached，导致下一段stopping提前置1；新后端启动后stopping不自动恢复，App_ForwardHoldStep因stopping1不提交ChassisAuto_UpdateForwardHold，最后邮箱过期。

仅修改main.c该到距锁存条件，增加phase==HR_RUN和App_ForwardHoldOwns(status)。真正当前段到距仍能立即进入原停止流程；WAIT期间不消费旧结果，异会话结果不参与控制。不清历史、不靠重新置stopping0掩盖来源，也不改100ms时效。其余后端/HWT/config字节与复现版一致。

## 构建及修复版烧录

70项既有主机测试通过；这是协议/主机回归，不替代C控制逻辑实测。七种完整隔离构建默认/A组三段/B组转向右移/串口单段/手动/无线/诊断关均0错0警告，源字节匹配及GBK/CRLF通过。默认Code57284/RO788/RW2004/ZI32460。14:09:15修复版A组重新烧录Verify OK/Application running，用户已回起点并允许单次复测；健康检查区分旧积压HF2和新会话后才发START。

## 复测未完成：捕获HWT硬件ORE

第一段左移完成，PE2投影均值301.53302mm。第二段准备时首个HF2 fault_tick37868、trip2/source_valid0、backend原状态VERIFY_ON/cause0；event_tick37857/reason1/SR232(0xE8)，UART错误1且ORE1，FE/NE/PE/环形溢出/校验错0。因此本次是USART2接收硬件溢出引起源失效中止，首因不是命令年龄107ms。后续组合包装显示trip5/backend9取消不能覆盖HF2原首因。

最终VM请求/输出0、捕获四轮0，用户确认已停下灯灭；未重试。HWT ORE的具体成因尚未测定，启动临界区屏蔽USART2过久是重点候选，不唯一归于电气噪声或线路。没有测量临界区最长时间，也未修改其结构。原换段逻辑修复已本地实施并上板，但整个三段回归被ORE阻断，不能写组合通过或HWT已修。

证据同日backend_fault_repro_*、distance_handoff_fix_*及distance_handoff_fix_review.json；下一项先定位/量测启动临界区与ORE，再在现场确认后单次回归两个换段。保留当前失败态，不自动发车。当前板为修复版A组失败态，B组未执行、完整回位未验。

核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，Q不变。仅本地未提交推送。
