# half-stop-run4：再次PLAN8超时，未发ARM/START

2026-11-15 07:43～07:44（Asia/Shanghai）。用户“再试一次”，沿用刚确认的摆位/载荷/净空及可断电条件，只授权一趟；无自动重试。

- 产品10项SHA、既有下载回执/AXF 3966ed37…1113d均匹配。未改固件/参数、未下载、未fetch/push。
- 统一工具复位一次成功，fresh HWT −0.016°、健康通过。实际只发PLAN8和2秒超时后的STOP，ARM/START为0。
- 超时期间收到23条合法TMQ，tick 3721→5921，accepted0、VM0、四轮指令0。MCU到主机遥测仍通，不称本次双向断链。
- 故障静止只读：core前后RUNNING，RX消费0→0、accepted/rejected0；tick17561→17641、APP/CAN1757→1765、feed874→878、TX seq178→179。g_reset_iwdg/RTOS/safety/CFSR/HFSR均0；VM/PE2为0，backend inactive/session0。
- USART1 CR1 0x202c、BRR 0x2d9、CR2/3 0，SR0xc0；PA9/10 AF模式/AF7。读时接收和RXNE中断使能，未读DR、未halt/reset/resume或写VM。不能排除瞬态/接收队列路径，也不能证明信号到达PA10。
- 与run3连续复现命令不被MCU消费、遥测仍返回。原因未唯一定位；不继续靠复位发车排障。下一步先检查主机TX→转接器/线缆/共地→PA10，再结合被动测量或队列证据细分。未授权下一次复位/下载/运动。

本次没有成功尾窗、176B快照或有效地面精度样本；软件VM关闭不冒充现场静止观察。R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。控制未变，本次无新增仿真需求。
