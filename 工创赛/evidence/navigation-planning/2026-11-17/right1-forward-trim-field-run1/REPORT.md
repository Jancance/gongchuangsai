## 2026-11-17 右1%右210阶段已完成，下一前进210三次（最新）

- 用户明确恢复、各次摆好并逐次授权。原无线COM17/UID6afaae24已核；右1%单独冻结镜像3043...8ad3下载1次，Keil Programming/Verify/Application running齐。板最后已装更新为3043，不再写79e74；左nav50%合并c51d仍未装。
- 右210共5次动作，显式reset/PLAN10 ARM START各5，KEEP220、STOP0，无自动重试。均software_complete/result1/stop_mask15/终态四轮零反馈，系统fault/linklost/rejected0。每趟名义20s保电尾；第3末帧19.937s是采样边界，199帧均有效，无硬件故障。末VMrequest/PE2均1，不当已断电。
- 实尺右距/前向旁偏mm：1=2100/-2.5，2=2106/+7.5，3=2097/+12.5，4=2097/+8.75，5=2105/+6.25。第1摆车断电由用户确认，RAW丢失；第2RAW也已回session0未触发，原因未获明确回答，用户指令重新测。全部原话/串口/失败保留，不以新RAW替代旧趟。
- 完整RAW为第3/4/5：2030/2027/2027，双bank稳定双读，quality0、源drop0、无保留记录断点、terminal tail完成。三趟平均右2099.6667mm/前9.1667mm，只是实测描述，不宣布因果改善或拟合部署。第1/2不计完整RAW，未剔除其尺量。没有产品/控制/保护改动或新仿真。
- 用户后续明确“后面运动完直接保存raw就行”，以后运动完成且软件inactive/零反馈后先直接只读保存RAW，不再等单独停稳回复；失败保首证据，不盲目复位丢数据。用户又说“前进已经放好”，已授权下一前210 PLAN8第1次，当前尚未执行；同当前3043镜像/原前PD/100RPM预算/400mm每秒，不重复下载。每次存RAW后再问尺量与下一摆位，不自动连跑；前组3次后再后退组3次。
- 证据：工创赛/evidence/navigation-planning/2026-11-17/right1-forward-trim-field-run1/stage_checkpoint.json、stage_sha256_manifest.json及run1..5。五次执行门全false、无后台硬件命令。原6dirty哈希未变，GBK/原CRLF保持；本阶段本地commit，不fetch/push。
- R已核RUN-01/PWR-01/MECH-01/02；E LOC-02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。完整轮/HWT模型残差、ICM有效性、前/后分组及后续导航验收尚未闭环，不用控制前馈代实际位移。
