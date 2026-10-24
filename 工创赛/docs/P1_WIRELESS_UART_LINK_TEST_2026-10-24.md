# P1无线串口双向收发专用测试（2026-10-24）

## 2026-10-24最新实测：COM17专用无线串口双向收发通过

用户先说明未下载最新程序，首次COM17打开成功但8秒无数据，sent0；随后按指导下载APP_TEST_MODE=1并回复“已运行”。本机实际执行test_wireless_uart.ps1 -Port COM17 -WaitSeconds 8，115200/8N1，收到专用LINK心跳，7条指令全部正确匹配，status=bidirectional_pass、opened=true、heartbeat=true、sent=matched=7，无I/O错误。随机串PING/PONG及ECHO匹配，SET569/GET569正确，INFO确认UART1/115200/8N1/VM0，SET1001被ERR,RANGE拒绝后GET仍569。证明本次PC下发→板端处理→回传双向链路可用，不是仅识别端口或本地回显。

全程只发无运动测试指令，无发车、横移或VM开启命令；VM0为专用测试模式声明，不是电压测量。串口已关闭释放。证据evidence/p1-foundation/2026-10-24/wireless_uart_bidirectional_pass.summary.json及.jsonl。无线物理路径/车端接线依用户现场准备，此回包试验未另行隔离有线/无线对照，不宣称长距离、移动中抗干扰、长期可靠性或串口控制运动已验收。下一步可切APP_TEST_MODE=2重建下载串口单次横移固件，再先监听健康数据、具备原运动条件后明确发起一次横移；本轮未切模式、未自动接续发车。组合动作/90度仍未实测。仅本地更新证据/交接，未提交推送。核R RUN-01/PWR-01、E LOC-03/RUN-06，Q不变。


## 2026-10-24最新：串口自动横移接入完成，单一开关切换，当前仍无线收发测试

用户要求无线收发通过后方便切换自动横移。复用已有CRC串口控制草稿与BeginTranslationHold，不另写运动闭环；新增统一APP_TEST_MODE：1为专用无线收发（当前默认），2为串口单次平移自动测试，0为原手动平移；APP_UART_LINK_TEST与APP_SERIAL_MOTION_CONTROL均由此派生，不再分别修改。APP_TRANSLATION_HOLD_MODE当前1左移，改2右移；每轮基础15RPM、8秒、修正±2、反馈保护30RPM及原停止/看门狗保持。

主机p1_serial_motion.py默认仅监听，预期方向改为1左移；明确run-once后等3帧健康数据、核对编译方向/速度、自检/姿态/故障/VM状态，ARM确认后仅START一次。主控继续执行原VM上电至少1秒及四轮状态就绪检查，完成准备后自动横移；STM32本地PD闭环，不由电脑逐帧算轮速。遥测100ms，主机KEEP200ms，主控1秒失联取消、主机600ms无新遥测尝试STOP；到时自动停车、停帧核对并关闭VM。不重试启动、不清故障。原无线模块测试task/function切换保留，不需要增加另一套任务。

主机自动保存raw、RX/TX JSONL和summary，每秒输出关键状态；默认监听或收到LINK测试协议均不发车。run-once的含义是本次明确请求自动执行一轮，不是固件上电自动发车。脚本只核模式，不能远程把左移改右移。四轮转速是缓存反馈，PE2是控制命令，软件完成不替代实车轨迹和停车观察。正式比赛入口与该开发测试接口须分离（R RUN-01）；R PWR-01、E LOC-03/RUN-06已核，官方Q不变。

最终四个完整隔离构建0错0警告：link25822/462/1816/27392；serial left/right42700/664/1836/29156；manual left39296/608/1768/27424。18项主机测试通过，含左右移等待准备/运行/完成、方向不符拒绝、LINK协议不触发启动、丢包/失联/取消及不重发；原19项无线主机协议检查通过。这些是主机软件模拟与编译证据，未执行MCU运行模拟、未烧录、未打开COM或发车；无线与横移仍待实测。原工程AXF保留。证据lateral_serial_*_rebuild.log和lateral_serial_review.json；操作见docs/P1_TEST_MODE_SWITCH_2026-10-24.md。当前本地完成未提交推送，已上传回退点仍5f132ad。


## 2026-10-24最新：无线收发独立测试入口，暂停串口发车

当前按用户要求先验证小模块，采取最小测试入口：简单模块优先由现有任务调用测试函数，需要独立周期/等待时才创建任务；测完关闭调用或编译开关，不固定要求每模块建任务。本轮已实现UartTest替代TestA（不重复创建业务任务），复用原UartRx进行PING/PONG、ECHO、SET/GET；UartTest每10ms解析HWT并上报真实APP任务进展，不运行扫码显示/运动门控。LED、按键、安全与CAN内部回环监督保留；CAN测试分支不进行外部电机交易，VM由已有Power/Safety测试分支强制关闭，看门狗不绕过。原自动发车草稿保留但APP_SERIAL_MOTION_CONTROL=0，不参与当前测试。

当前APP_UART_LINK_TEST=1，APP_SERIAL_MOTION_CONTROL=0；未启用的手动运动选择APP_TRANSLATION_HOLD_MODE=1（左横移），每轮基础15RPM、8秒及原保护不变。串口115200/8N1，每秒LINK,HB后追加LINK,YAW,<毫度>,<valid_mask>,<age_ms>；无效角度数值哨兵-2147483648，valid_mask及age必须一起检查，不以数据存在代表姿态健康。消息共用原RX任务发送入口，回复优先、忙时重试；原主机测试脚本忽略额外YAW行，仍可使用。专用测试不接受发车指令。测试结束仅改APP_UART_LINK_TEST=0，保持SERIAL_MOTION_CONTROL=0，重新编译下载即可恢复手动左横移流程；不要手改Watch切换测试模式。

隔离测试完整构建Code25822/RO462/RW1816/ZI27392，0错误0警告，日志uart_isolated_rebuild.log；恢复手动左移构建结果见uart_isolated_manual_rebuild.log。主机原19项协议自检通过，未打开COM、未发字节。固件运行/无线双向链路、横移方向和姿态纠偏仍未实际验证，未烧录，未覆盖原工程AXF。R核RUN-01/PWR-01、E核LOC-03/RUN-06，Q不变。本轮仅本地完成，未提交或推送；GitHub回退点仍5f132ad。后续先独立确认无线收发，再继续手动横移，不把两项同时改成自动发车。


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
