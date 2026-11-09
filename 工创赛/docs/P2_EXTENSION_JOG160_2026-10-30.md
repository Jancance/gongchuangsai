# 28伸缩轴160脉冲低速方向试验

> 本日伸出及双向单次功能确认（E，最新）：用户明确“对的伸出并停稳”，Watch运动7/1/move1、stop0、flags3、delta0xfffffffffffff2da（有符号−3366）、speed0、请求0。结合此前缩回口述及截图，确认请求1/协议0=缩回，请求2/协议1=伸出，双向各一次最终停稳功能已确认，不重复小步或重编译下载。原始计数不当准确脉冲位移、毫米/比例线性或重复精度。证据extension-jog160-board/extend-functional-acceptance.json及原图。下一项确定伸缩原点及有效行程，已询问缩回端原点开关是否安装；历史PE10规划不当实装，35无开关方案不自动套28；当前先不发回零/撞端。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05；原点/行程/毫米/带载抓放未闭环，官方Q保持待技术答疑确认。无固件或助手硬件命令变化。

> 本日缩回单次功能确认（E，最新）：用户明确“是缩回停稳了最后”，补图motion_state7/result1/move1、stop0、flags3、delta0x43（67）、speed0，motion_request0、VM1/lift_state4/DM许可0。接受本次请求1（协议方向0）对应缩回及最终停稳，软件反馈完成；不以67原始计数证明精确脉冲行程、比例线性或毫米精度，不据旧16脉冲单次230计数直接定标。下一单项保持同固件160脉冲/5RPM/3秒超时及其他轴请求0，request2一次验证反向伸出/停稳并回同组运动Watch，不重复缩回、不重编译下载。证据evidence/p2-arm/2026-10-30/extension-jog160-board/retract-functional-acceptance.json及原图；请求2伸出、原点/有效行程/毫米/带载抓放仍待实测。核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05，官方Q保持待技术答疑确认，本轮无固件变化/助手硬件命令。

日期2026-10-30，E类工程变更。用户反馈“好像是缩回，步长太小了给我拉大点”，按同速度下逐步扩大观察步长处理。

## 原16脉冲反馈

用户补充Watch显示motion_state7/result1/move_result1、stop_result0、flags3、delta_raw=0xE6（230）、speed_rpm0，VM1、lift_state4、DM点位许可0。接受本次16脉冲试走的软件反馈完成；用户只说“好像缩回”，不将伸缩方向登记为实物确认，也不虚构明确的停稳观察、毫米位移或精度。stop_result0是正常完成未发停止，不是停止失败；旧只读state/result各0表示本次启动没有单独做三项读取，不当通信回退失败。原图及字段见`evidence/p2-arm/2026-10-30/extension-jog16-board/board-observation.json`。

## 本次改动及验证

实际工程`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`仅改`HARDWARE/EXTENSION/extension_config.h`一行：`EXTENSION_JOG_PULSES 16U → 160U`。保持5RPM、加速20、3秒整体超时、默认请求0、节点6及所有互锁/反馈/停止逻辑。共享配置同时约束报文步长及传输上限，不出现上层加大而下层仍拒绝的配置分离。

原16脉冲代码、配置和证据保留。Keil当前项目文件相比上一候选只移除了配置头的可选列表项；编译包含路径和C文件引用仍在，按当前文件保护，不回写旧项目。差异见`extension-jog160/project-source-context.json`。

64项合成检查通过（25状态机、7调度互锁、32传输），明确验证160脉冲报文分帧及161脉冲/高位越界拒绝，速度/模式/地址/校验/超时/忙时/失败等检查继续有效。副本与实际Build均0错误0警告，Code55212/RO608/RW2128/ZI25888；其余810个源/项目文件SHA保持，实际配置与验证副本一致。原配置备份、manifest/installed/review、主机日志及两次构建日志见`evidence/p2-arm/2026-10-30/extension-jog160/`。交付配置副本`output/extension_jog160_20261030/extension_config.h`。

本轮已安装源码，未下载固件、未由助手发送电机命令。合成/构建不代表160脉冲实物验收。若小步因3秒超时提前停止，应按真实状态记录，不当160脉冲已走完，也不自动延长时限。

## 下一单项

1. 保留当前两侧余量，其他轴请求0，DM点位许可0。
2. 退出旧调试会话，下载实际工程本次新版本，F5，VM请求1等待约3秒及lift_state4。
3. `g_extension_motion_request=1`仅一次，保持程序运行观察约3秒，确认究竟伸出或缩回、实际位移及停稳。需要停止写3并保持运行以执行停止；停止未确认则按既有外部停机流程处理。
4. 回以下Watch；只发旧只读变量不能判断本次运动结果。不连续触发、不自动往返。

```text
g_extension_motion_state
g_extension_motion_result
g_extension_move_result
g_extension_stop_result
g_extension_flags
g_extension_delta_raw
g_extension_speed_rpm
```

正常完成候选state7/result1、move_result1、速度0；stop_result0为未发停止。result3表示3秒超时，失败锁存后先明确停止情况。请求1仍是协议方向0，伸/缩映射以本次实物确认，不因此前“好像”而反转或改名。

R核RUN-01/PWR-01/MECH-01/02；E核RUN-06/LOC-03/CAL-05。本次数字为工程调试候选，不是官方规则或毫米行程。方向/物理停止、原点/有效行程、毫米映射、带载和固定抓放仍未闭环；官方Q继续待技术答疑确认。
