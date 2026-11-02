# 左移30cm准备阶段失败排查（2026-10-24）

本轮只读诊断，未改固件、下载、复位、清故障或再次START。板仍为lateral_distance_left_20261024的单左移300mm固件，保留失败态，VM关闭；未上传GitHub。

## 已确认的时序

- 设备27400ms健康，发送ARM；27500ms收到ARM接受后发送START。
- 27600至28500ms上电准备：VM请求/实际输出均1，HWT源有效、角度/速度龄期约13ms，后端idle，无非零速度记录。
- 28600ms首次捕获失败：流程1067ms，后端会话已进入distance_mode1/目标300mm，forward trip5、backend FAULT/result2/cause9，HWT source_valid0、两龄期-1；进度0、无终点。主机此后才发STOP，故主机STOP不是最初中止触发。
- 事后源恢复，STOP已确认，未恢复失败会话。额外8秒监听中剔除旧积压，227100至235000ms的80条当前TM1全部source_valid1，两龄期最大7/8ms，VM0、捕获轮速0。
- 此前distance_87cm_repeat2_run也在elapsed1067ms、trip5、backend cause9、两龄期-1、progress0中止。距离方向不同而故障指纹相同，优先检查共用启动路径；不据此排除电气干扰。

## 可确认的代码机制

1. hwt101_parser.c:50校验失败会清seen；hwt101.c:563..572的UART错误或环形队列溢出会增加error_epoch；Update:605..618见epoch变化也会清seen。623..624在seen缺失时把龄期设UINT32_MAX。因而-1不是单纯100ms超时的直接证据，也不唯一指向UART硬件错误。解析器初始化后此前已经健康，须追查哪种清seen路径发生。
2. 遥测trip来自g_forward_hold.trip（main.c:2163），trip5表示底层接受/所有权/状态等执行器监督失败，不能当作HWT专用码。backend cause9是Abort，不能覆盖为最初电气故障原因。
3. UART遥测uart_rx_errors来自无线/调试USART1接收统计，不是g_hwt_uart_errors；其为0不能排除USART2的HWT错误。
4. main.c:1397..1541的App_ForwardHoldStep有外层临界区，1432调用BeginTranslationDistance；chassis_auto_test.c:1026会再次运行距离数学自检，嵌套Begin系列还会生成命令矩阵。FreeRTOS临界区屏蔽优先级>=5的中断，而HWT USART2优先级6，因此这些运算期间其接收ISR不能执行。115200/8N1单字节约86.8微秒，若连续到字节期间屏蔽过久可能ORE。这是已确认的结构性候选，未测临界区耗时或读到本次ORE，不能称根因证实。

## 当前证据的限制及最小下一步

现有TM1没有g_hwt_uart_errors、g_hwt_overflows、g_hwt_checksum_errors，更没有首次失效时USART2 SR错误分项和临界区最长周期；也未通过调试器读到本次计数。不能从已恢复的遥测反推出此前错误是哪一种，不能凭本轮现有日志分清噪声、接收过载与屏蔽过久。

下一步应先补只读诊断：在首次源作废/首个trip锁存tick、UART错误分项SR、溢出数、校验错数、seen/valid/两龄期、后端state/cause/session；测启动临界区耗时，并将耗时数学预计算移出锁或复用启动自检结果，提交时仍在短临界区复核会话健康及故障。上述是待实施建议，不是本轮已完成修改或验证。保留100ms姿态保护、400ms位置保护、故障锁存、停车及看门狗，不能靠扩大门限或自动重试求通过。

核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04；Q不变。当前仅确认失败发生在共用启动交界，横移方向/距离/停稳实物验收仍未通过。
