# P1无线串口双向收发专用测试（2026-10-24）

本次只测试无线串口收发，不读取运动变量、不调PD、不发车。固件和电脑端测试脚本已经本地完成；尚未烧录，实际无线链路还未验证。

## 接线和模式

先关闭外部VM、断开主控及车端DAP电源，再接：车端DAP RX/RXD→STM32 PA9/USART1_TX；车端DAP TX/TXD→STM32 PA10/USART1_RX；GND→GND。原SWD线保留，若已有可靠共地不必重复加地线。按器件丝印确认RX/TX，不按插座照片方向猜。电脑端接无线主机USB，车端为配对的无线从机并由原板上电源供电；不要将电源脚当串口信号脚。

实际工程`3.stm32f407工程/gongxun-f407.uvprojx`，当前新增`APP_UART_LINK_TEST=1`。该模式强制PE2关闭：Power_Enable编译分支拒绝开启；Safety每20ms撤销VM请求，CAN任务跳过外部交易，PC13只记录事件不发布运动请求。现有任务正常推进，保留Safety/看门狗和故障锁存，不无条件喂狗。最初CAN内部回环初始化保留，其结果仍交给原监督。模式1期间不要写任何运动请求。

前进/横移/原转角源码和参数保留。测试结束将APP_UART_LINK_TEST改0，重新编译下载，才恢复此前运动调试配置。不要只改Watch变量来切换模式。

## 测试协议

USART1：115200、8位数据、无校验、1停止位、无硬件流控。ASCII行协议，以LF结束，兼容CRLF。固件每秒发`LINK,HB,计数,系统毫秒,VM0`；VM0是软件模式声明，不是实测VM电压。

| 电脑发送 | 固件回复 | 用途 |
| --- | --- | --- |
| PING,随机串 | PONG,同一随机串 | 验证PC下发、固件接收并生成新回包 |
| ECHO,文本 | ECHO,同一文本 | 核对数据内容 |
| SET,321 | VALUE,321 | 仅写入独立测试数字g_uart_link_test_value |
| GET | VALUE,当前测试数字 | 确认测试数字能读回 |
| INFO | INFO,UART1,115200,8N1,VM0 | 确认专用固件和设置 |
| SET,1001 | ERR,RANGE | 验证范围拒绝，原测试数字不变 |

SET只支持0至1000的十进制整数，不能修改任意变量或电机参数。未知命令回ERR,COMMAND。输入最多47个可打印字符，PING负载1至32字符；非法/过长输入丢弃到换行，不执行截断尾部。中间停顿500ms重置未完成输入。发送复用64字节中断缓冲，回复忙时待下一轮重试，不阻塞运动任务。只有一个待回包槽，突发多个命令可能丢弃并计数；电脑脚本每次只发一条等匹配回复。

诊断可选观察g_uart_link_test_value、g_uart_link_rx_lines、g_uart_link_tx_replies、g_uart_link_errors、g_uart_link_dropped、g_uart_link_hb_count及原g_uart1_rx_errors。前一个errors包含故意非法命令，后一个才是UART接收硬件/队列错误，不混为一谈。

## 电脑端测试

脚本`工创赛/scripts/test_wireless_uart.ps1`默认COM17。默认先等待最多5秒的专用VM0心跳，没收到则不发送任何字节；收到后依次发送7条无运动指令，随机串核对PONG/ECHO，随机测试数字核对SET/GET，再检查超范围写入被拒绝且原值未变。每条只发一次，匹配超时2秒就停止，关闭串口，不自动重发或重启设备。DTR/RTS关闭，避免调试控制信号。

可执行方式：

```powershell
& 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/test_wireless_uart.ps1' -Port COM17
```

只接收验证可加`-ListenOnly`；不接硬件的19项主机协议自检使用`-SelfTest`。每次原始方向/时间/文本写JSONL，汇总写summary.json，默认位于`工创赛/tmp/uart_link_captures/`；通过标志为bidirectional_pass且sent=matched=7。receive_only_ok只代表收到心跳，不能当双向通过。端口不存在、占用、无专用心跳、回包不匹配均明确失败。串口助手等其他程序应先释放同一个COM端口。

## 当前验证与下一步

隔离完整ARMCC5构建：测试模式Code31608/RO608/RW1724/ZI27516；关闭测试恢复前进模式39296/608/1680/27512，均0错误0警告。主机19项协议/命令数组检查通过，含错误随机串/本地回显不当PONG、错误心跳拒绝及命令形状。固件解析只做编译和源码审查，未声称运行时或硬件已通过。

本机预检端口列表只有COM3/COM6/COM12，没有之前的COM17。脚本尝试打开COM17返回不存在，opened=false、sent=0、matched=0，没有发送任何字节；不将端口未连接当无线产品故障。证据evidence/p1-foundation/2026-10-24/uart_link_test_review.json及两个uart_link_*_rebuild.log。

下一步先在外部VM关闭下，对实际工程完整Rebuild（预期Code31608），下载专用测试固件并连续Run；接好线、插入电脑端DAP后告知端口号，我再开COM口完成收发验证。未完成实际回包前，不宣称无线串口可用。原工程AXF未重建，板上仍是此前固件，当前不能直接下载旧AXF冒充测试固件。

R核RUN-01/PWR-01，E核LOC-03/RUN-06；串口测试入口是开发选择，不作为正式比赛遥控方式，官方Q不变。仅本地完成，未提交推送GitHub。
