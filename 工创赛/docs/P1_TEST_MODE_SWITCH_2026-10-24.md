# 测试模式与当前验证基线（2026-10-24）

实际工程：`3.stm32f407工程/gongxun-f407.uvprojx`。无线双向收发、15RPM左右横移和四段组合已各完成本轮实测，用户确认方向、平稳程度、停稳和VM灯现象符合。当前源码与板上固件均为 `APP_TEST_MODE=3`、组合转向30秒上限；板上已完成一轮，不会自动再运行。

| APP_TEST_MODE | 功能 | 启动入口 |
| --- | --- | --- |
| 0 | 原手动单段平移 | Watch `g_forward_hold.request=1`；2取消 |
| 1 | 独立无线收发，强制VM关闭 | 无运动入口；PING/PONG、ECHO、SET/GET及LINK心跳/YAW |
| 2 | 串口单次平移 | 主机明确 `run-once --mode 0/1/2`，方向须匹配编译配置 |
| 3（当前） | 串口四段组合，段间保持供电 | 主机明确 `run-once --mode 3`，整组一次 |

只改 `APP_TEST_MODE` 选择入口；APP_UART_LINK_TEST/APP_SERIAL_MOTION_CONTROL由其派生，不要另改。修改模式后须重新编译下载；Watch或主机--mode不能切换板上编译方向。单段方向APP_TRANSLATION_HOLD_MODE：0前进、1左移、2右移（当前保留2，对组合私有路线不起作用）。平移基础15RPM、8秒，航向修正±2RPM、反馈限30RPM。

## 编译、下载和运行

Keil命令行 `UV4.exe -r <工程> -o <构建日志> -j0` 编译，`UV4.exe -f <工程> -o <下载日志> -j0` 下载。实际已通过CMSIS-DAP验证自动下载后复位运行，无需手动进入Debug或按F5。隔离构建必须复制当前uvoptx中的原CMSIS-DAP配置，防止下载器设置丢失。构建0错0警告、下载Verify OK/Application running后，还必须通过串口确认新会话实时健康状态，不能仅凭下载日志发车。偶发下载失败后须先排查，不声称已写入。

当前COM17，115200/8N1。主机默认只监听：

```powershell
D:/python/python.exe 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action monitor --mode 3 --seconds 3
```

现场准备好并提前提醒观察后，才能执行一次组合：

```powershell
D:/python/python.exe 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action run-once --mode 3 --seconds 120
```

单段模式2对应左移--mode1或右移--mode2，观察时长可设25秒。--mode只核对固件身份，绝不远程改方向。STOP可用--action stop或现场PC13取消；端口不能同时由多个软件占用。每次上电只启动一轮，不自动重试，成功/故障后须开始新会话并确认摆位。DAP可能补传旧串口缓存，脚本保留清RX、健康帧到达节奏和主控tick新鲜度门禁，不用旧帧判定准备完成。

## 模块与组合

简单模块优先用现有任务调用测试函数，需要独立周期/等待时才建测试任务；测完关闭入口，不反复改业务主体。模式1的UartTest替代业务TestA，复用UartRx，保留真实任务进展、安全监督和看门狗。

组合顺序：前进8秒→停稳→左移8秒→停稳→逆时针+90度→停稳→沿新车头前进8秒→停稳关VM。整组只准备一次；段间至少1秒、保持VM及使能，下一段只重新核对状态/零速。任一故障/取消/失联仍停止并断电。模式3转向30秒、每段35秒、整组120秒上限；其他模式转向20秒。详见P1_COMBINED_MOTION_TEST_2026-10-24.md。

## 验证证据与界限

原无线7条双向事务全部匹配；左右横移各8008ms自动完成；四段组合56304ms，完成4段、停止mask15/四轮0/VM0，用户确认现场均符合。25项主机协议/状态机测试通过。证据在同日evidence目录，保留早期失败和修复过程，文件时间/标题用于区分版本。数据raw、JSONL和汇总能复核，不将历史失败版的参数当当前配置。

精确XY轨迹和距离、高速/负载、完整故障注入和长期无线可靠性仍未验收。轮径77mm、用户确认直驱1:1，但用户已决定距离控制后续再做，当前仍按时间运行。软件错误计数包含结束边界KEEP拒绝，sequence_gaps可能包含旧缓存跳变；不能混称运动故障或真实运行丢帧。

本次上传范围以根交接和Git提交为准；tmp、本机布局、Objects/Listings不作为当前源码交付。正式比赛完全自主，串口启动仅用于开发测试（R RUN-01）；R PWR-01、E LOC-03/RUN-06/MOT-01/02已核，官方Q不变。
