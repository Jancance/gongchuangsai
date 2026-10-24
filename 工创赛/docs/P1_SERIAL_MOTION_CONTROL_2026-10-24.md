# 串口控制与遥测接口（2026-10-24当前版）

当前APP_TEST_MODE=3为组合测试，模式2为单次平移，模式1为无线收发小模块。无线、左右移及整组动作已实测；具体结果与限制见P1_TEST_MODE_SWITCH_2026-10-24.md和根交接。打开COM默认仅监听，不自动发车；方向/动作模式编译锁定。

## 协议和调度

USART1 PA9/TX→DAP RX、PA10/RX←DAP TX，GND共地；115200/8N1，无流控。RC1只允许ARM/START/KEEP/STOP，不提供任意内存写、故障清除或PID修改。ASCII帧 `RC1,<seq>,<tick>,<verb>,<CRC>\n`，CRC16-CCITT-FALSE覆盖末尾逗号，4位大写十六进制。seq/tick引用主控近期遥测，令牌用于会话和重复包约束，不是加密认证。

主机3帧健康且至少150ms真实到达观察窗口后ARM；设备tick与主机节奏差过大重建窗口，防止DAP旧帧突发。主控核500ms新鲜度/近期序号和闲置门禁，再给予2秒启动窗口；主机核匹配令牌后仅START一次。重复START不重开或延长动作。模式2进入原平移前端；模式3进入整组调度。私有启动许可防止Watch绕过串口门禁。STM32负责PD、状态机和保护，电脑不逐帧算轮速。

主机200ms KEEP，主控1秒无有效KEEP取消，主机600ms无新遥测尝试STOP；原HWT/提交新鲜度、停止/后端/安全/看门狗保护均保留。PC13及STOP可取消整个当前组，包含等待/段间；已失败时STOP不覆盖首因。准备、失败、成功都有边界，不自动重试或清故障。

既有任务分工：TaskA更新姿态并100ms发遥测，UartRx解析请求，唯一CAN任务执行运动/换段，Safety独立抢占监督。CAN等待在组合临界区外；快照复制使用短临界区，格式化/CRC在外。串口TX中断非阻塞，忙时丢帧计数；串口模式TX768字节、TaskA512字栈、RX384字栈。实测RX余栈303字、最小堆9784字节只是本轮观测，不作为所有模式的资源保证。

## 遥测

TM1每100ms发送带CRC的整数CSV。普通平移48字段，早期组合51字段，当前组合55字段；主机兼容三种格式，schema见P1_SERIAL_MOTION_SCHEMA_2026-10-24.json。角度/角速度乘1000，非法浮点哨兵INT32_MIN；序号/tick以uint32回绕处理。组合追加sequence_state/index/completed、can_guard_cause/bad_nodes、power_wait_state/post_settle_mask。

组合phase描述整个组，只有completed4、正常停止mask15和VM0/PE20才完成，不能首段结束就停止保活。组合转向时correction_rpm承载转向轮速命令，非平移PD修正；peak_error和纠偏汇总来自当前平移前端，须按阶段解释。四轮RPM为缓存反馈，PE2是输出命令，误差和最新yaw可能来自不同更新时刻，不作同步位置测量或电压实测。

## 主机与记录

`scripts/p1_serial_motion.py`仅用Python标准库及Windows COM，默认monitor；run-once显式选择才启动，stop用于取消。默认预期左移--mode1；当前组合须明确--mode3，右移--mode2。例：

```powershell
D:/python/python.exe 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action monitor --mode 3 --seconds 3
D:/python/python.exe 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action run-once --mode 3 --seconds 120
```

动作命令仅在已确认现场准备且发车前提醒后执行。脚本关闭DTR/RTS，不靠开串口复位；清理RX并用时序门禁处理仍可能从DAP送来的积压，单靠PurgeComm不足。raw/逐帧RX/TX JSONL/summary默认存tmp/serial_motion_captures，关键实测已复制进evidence。每秒输出阶段/角度/轮速/VM状态。

25项主机测试通过；完整组合实测56.304秒、段间保持VM、90度到位误差约0.922度，最终停稳关VM，用户现场确认。结束边界一个KEEP被终态拒绝，不等于重复启动；旧缓存跳变不等于真实运行丢包。精确距离/XY、高速载荷、全部故障注入与无线长期可靠性仍未闭环。R RUN-01/PWR-01及E LOC-03/RUN-06/MOT-01/02已核，官方Q不变。
