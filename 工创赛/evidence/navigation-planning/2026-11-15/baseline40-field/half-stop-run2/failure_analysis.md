# half-stop-run2：START后ACK超时的只读诊断

2026-11-15。绑定half+176 B观察镜像 `3966ed37a6c2dd85319884fbe22a2c54a610a9bb9e50f46240e79c006bf1113d`。本代理只读既有raw/jsonl、failure_read_receipt及源码，仅新增本文；没有连接硬件、复位、发命令、修改产品或Git。当前这一趟授权已消费，不新增重试。

## 当前可下的结论

**本次不是“START没有被固件接受”，现有证据也不支持主控卡死、看门狗重启或仅客户端ACK窗口误判。** START已经被固件接受，之后KEEP没有被UART接收任务消费；主控任务与发送路径仍推进，最终由原1秒keepalive超时保护置link_lost/result8并关闭VM。故障边界已缩小到通信链路或接收路径，但还不能唯一锁定USB、串口转接器、线缆/共地、引脚/USART配置或接收队列中的哪一处。

这次失败不是完成的精度样本。先前half-stop-run1成功后用户手动复位、使tail读取门禁拒绝，是已确认的另一现场事件，不能拿来解释本次START后链路中断。更早出现的开机tick回退也不能只凭相似外观被一律重标成本次原因。

## 67字节与START接受的证据闭环

`main.c` 的 `App_UartRxTask` 每当 `USART1_WaitByte` 从队列取出 **一个字节** 就执行 `g_serial_rx_count++`，然后调用 `App_SerialMotionByte`；因此67是任务消费字节数，不是帧数或USART硬件总接收字节数。

本次保存的实际命令长度（含客户端发出的单个LF）为：

|命令|字节数|
|---|---:|
|`RC1,73,7121,PLAN8,38BA\n`|23|
|`RC1,77,7521,ARM,3B77\n`|21|
|`RC1,77,7521,START,20F0\n`|23|
|前三条合计|**67**|

RAM双读 `g_serial_rx_count=67`、`g_serial_rx_last=10`（LF），与前三条总长度完全吻合；`g_serial_motion_accepted=3`、rejected=0进一步支持PLAN8、ARM、START均被接受。主机最后只保存到ARM accepted2那帧，是**最后已收到的旧状态**，不是START后板端状态的证明。

主机随后记录8条KEEP及1条STOP，每条22字节、共198字节，均通过独立CRC复算。WindowsSerial只在WriteFile成功且返回完整写入长度后才记TX，因此这198字节是主机已交给驱动的完整写入，不代表它们已到MCU引脚或被固件消费。RAM仍只有67；也没有额外rejected计数。不能解释为“固件收到了全部KEEP但CRC都拒绝”。

保存raw仅2350字节、8条TMQ，所有已保存完整行CRC合法，最后TMQ为seq78/tick7621/accepted2；raw随后没有新数据，而不只是parser丢弃了ACK。客户端2秒 `ack_timeout` 是后续无确认帧的结果，不是根因。

## 主控及USART发送侧仍活动

既有只读回执在复位后约137秒、约80ms采样窗内保存：

|量|前→后|含义|
|---|---|---|
|g_test_tick|137401→137481|任务时基继续推进|
|g_test_count_a|13741→13749|APP任务继续运行|
|g_can_task_count|13710→13719|CAN任务继续运行|
|g_watchdog_feed_count|6866→6870|监督喂狗路径继续运行|
|s_serial_tx_seq|1377→1378|状态遥测仍成功进入发送路径|
|s_serial_tx_tick|137331→137431|约100ms状态发送时窗继续推进|
|g_tx_length/position|148/15→101/20|发送缓冲及USART TX中断路径没有固定卡在同一位置|
|g_serial_rx_count/last|67/10→67/10|任务未消费新输入字节|

前后core为RUNNING，g_rtos_fault/assert_line、g_safety_fault及CFSR/HFSR均0。现存证据不支持持续的全局关中断/整个调度器挂住或RTOS栈溢出hook已触发。TX缓冲进度不能单独证明PA9线上波形或USB实际送达，只能证明到USART发送软件/中断这一侧仍在推进。

`g_reset_iwdg=0`、保存的 `g_reset_flags=0x14000000` 是PIN+软件复位标志，与本次工具复位相容；watchdog.c启动时保存后清RCC标志，所以当前CSR=3不能代替保存的上次复位原因。结合137s时基持续推进，本次没有看门狗重启证据。CFSR有效位为0时，非零MMFAR/BFAR只是无效残值，不能作为出错地址或越界证据。

当前serial state3/result8/link_lost1，正对应 `App_SerialMotionSafety` 中START后超过1000ms没有合法KEEP时的分支：撤销许可、置link_lost/result8、停止组合动作并关闭VM。当前VM/PE2为0是与该失联保护相符的结果，不能据此倒推START没执行或电源从未短暂开启。

## 源码及客户端检查

- START在原临界段中设置start permit、组合request，再写serial state/result2和accepted++。没有START时禁用USART1 RX、重配PA9/10或阻塞等待TX完成的语句。176 B观察只在后续已接受动作/末段事件执行，本版也没有新增104 B浮点统计；没有证据把本次失联直接归因于前馈、PID或观察计算。
- USART1 RX中断把字节放进64 B队列，RX任务消费；TX独立通过同一IRQ发送。若全部后续198字节都持续进入队列而任务完全不消费，通常会出现队列满错误计数；本次UART errors=0不支持这个简单版本，但不能单凭0计数排除RX中断未使能、引脚/接收器状态、队列/任务异常或输入根本没到达。
- 可见源码只在USART1初始化时配置PA9/10为AF7；开始阶段没有关闭RXNEIE。ICM使用PA5/6/7，区开关使用PA1，其它带9/10的设备使用不同GPIO端口，未发现START明确抢占PA9/10的代码。**静态搜索不能替代失效时CR1/GPIO寄存器实读**，因此不把重配置可能性彻底排除。
- WindowsSerial设置115200/8N1、DCB flags=0x81：无CTS/DSR及软件XON/XOFF流控，DTR/RTS禁用；PurgeComm仅在打开时清RX，没有START后再次清接收缓冲的代码。WriteFile/ReadFile为同步调用，发送完整才记日志，异常和短写会报错；本次没有此类error，但驱动接受写入不证明转接器/线端真正送达。

## 最小下一步：先分层查链路，不再发车验证

以下仅列可验证路径，本文不执行，不产生新的复位/START或故障读取授权。

1. 保留当前raw/jsonl、只读回执、镜像绑定与时间线，不再先reset掩盖现状。先核主机在 **2026-11-15 07:17:36～07:17:40（Asia/Shanghai）** 是否有COM17对应设备/USB转接器掉线、重枚举或驱动错误；确认是同一设备实例及端口，不靠重新发车碰运气。观察USB线/接头、转接器供电与共地是否会受VM切换影响，只列假设，不先认定硬件损坏。
2. 若后续获准做静止只读链路检查，最小补取USART1 CR1（UE/RE/RXNEIE）、CR2/CR3/BRR及SR、GPIOA MODER/AFRH、RCC相关使能；必要时结合当前ELF核RX队列待处理数及RX任务活性。**不要读USART DR**以免消费/清状态，不写寄存器，不halt，不为读取核心寄存器而停核。当前回执没有这些配置，不能伪造为已验证正常。
3. 若有现成被动测量条件，分清PA9有TX而主机无RX，还是PA10确有输入但任务不消费；前者偏向MCU引脚之后至USB/主机接收链路，后者再核接收器/中断/队列。没有线端证据前，USB/COM驱动、转接器/线缆、USART接收路径均保留，不做唯一归因。
4. 连接处理应在证据保留、板端静止及明确操作范围下单独进行。不要把拔插/重开COM/复位后一次恢复误写成修复根因，不把后续静止通信检查变成又一趟运动，也不为通过而放宽keepalive或故障保护。

已核R RUN-01/PWR-01；E LOC-03/05、MOT-04、RUN-06，保留失联停止；Q FIELD-05/OBS-03/QR-04不变。当前唯一收敛结论是：**此趟START已接受，随后双向通信/接收消费链路失联，主控仍活跃且保护生效；具体故障点尚未定位。**

证据哈希：raw `c795b46a6f51c714b12e64d7fad111364fc63178296e498d6187d825e98f470b`；journal `56e66e08e029a31ade0c2822d0020eb8d6ee7fa1dcd42095da28398d0bf4fe3d`；failure_read_receipt `1e667246bd074e88602a6f4d249b0aff313b9bf148580c4962298051627d1281`。
