# half-stop-run3：PLAN8确认超时，未发ARM/START

2026-11-15 07:40（Asia/Shanghai）。用户确认已摆好，授权一趟40cm/s、210cm；实际未进入运动，不计精度样本。

- HEAD 51a7f15c，控制0f0ffad2；10项产品源SHA及既有下载回执/AXF 3966ed37…1113d匹配。没有修改固件、下载、fetch或push。
- 工具仅复位一次，fresh HWT +0.010°且健康预检通过。客户端发PLAN8一条，2秒确认超时后发STOP一条；ARM/START均0，没有重试。失败后没有收成功尾窗或读取176B。
- 本次与run2不同：PLAN超时期间仍收到23条合法TMQ，tick3521→5721，accepted始终0、VM0/PE2低、四轮指令0。主机到MCU接收方向无消费证据，而MCU到主机遥测仍继续；不能写成与run2完全相同的双向断链。
- 静止只读SWD：core RUNNING；tick33171→33241、APP3317→3325、CAN3318→3326、feed1654→1658、TX seq334→335。RX消费0→0、accepted/rejected0、UART errors0；CFSR/HFSR/RTOS/safety0，g_reset_iwdg0。VM0且backend inactive/session0；没有本次看门狗重启或整机卡死证据。
- USART1 CR1=0x202c（UE/RE/TE/RXNEIE均1），BRR=0x2d9，CR2/3=0；GPIOA PA9/10均AF模式、AF7。读取时配置未见关闭接收；SR=0xc0未见ORE/FE/NE/PE/RXNE。未读取DR、未halt/reset/resume/写VM或控制。单个时点不能排除瞬态或队列/接收任务路径问题，不能证明输入到达PA10。
- 主机Win32_SerialPort CIM查询被拒绝访问，未取得设备实例/驱动事件证据；不要把监听恢复或本次配置正常写成链路已修复。

下一步先核PC TX→转接器→线缆/共地→PA10的接收路径；优先现场检查连接或被动波形，再按证据查接收中断/队列。未授权新的复位、下载、ARM/START或自动重试。车辆物理静止由现场确认；软件状态不冒充地面观测。

规则核对：R RUN-01/PWR-01（此次是调试入口，非正式赛制验证）；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。本次未变控制，无需新增动力学仿真；不证明控制精度、停距或比赛验收。
