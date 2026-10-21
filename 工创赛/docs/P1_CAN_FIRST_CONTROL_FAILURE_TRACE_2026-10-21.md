# 首次控制CAN失败取证：用户仅手动修改can.c

最新+5度复测未通过，用户报告VM仍没亮。Watch2为exec phase4/trip3/state4/cause2、target-110.724487度/error5度，执行器result2/cause1/io_result5/last_address1、停止mask0。Watch1源有效、帧龄期15/6ms、yaw-115.724487度/rate0，VM及两fault0、接收错误0。HWT源已不是本次记录的退出原因，尚未进入旋转。

实际代码对应：第一阶段向1号电机发送失能准备命令；CAN1_ExchangeSingle返回5表示当前回复窗口内未取得匹配地址/类型/命令的有效回复。它不等于总线上完全无帧，也不能证明电机损坏或VM从未上电；准备失败后保护会撤VM。先前缩短等待只解决与100ms提交监督的时限冲突，不是厂商保证30ms内应答，后续若需更长合法应答须调整交易架构，不能直接放宽安全监督。

本步只记录首次失败交易：发送内容、阶段/时长、接收数量和最后原始帧、PE2输出允许位和CAN ESR。不改任何帧/等待/滤波/地址/控制顺序或电源输出。PE2 ODR是软件输出位，不是VM电压实测。只保存最后RX，不是完整总线抓包；失败后锁存，不被后台轮询覆盖。首次失败前每次控制交易会更新观察值，首次失败后新交易仍按原代码执行，只不再改这份诊断。

实际工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。保持VM外部开关关闭，退出Debug。

## 1. 只在实际SYSTEM/CAN/can.c增加变量

找到：

```c
static uint32_t s_can1_normal_ready = 0U;
```

在这一行之前粘贴[完整诊断变量](code_samples/can_control_trace_globals.c.txt)，原行保留。放在文件包含区之后，不放到函数体内，不改can.h。

## 2. 整段替换私有收发函数

找到`static uint32_t CAN1_ExchangeSingle(...)`的函数定义，连同原参数列表和完整函数体，替换为[完整带记录的收发函数](code_samples/can_control_trace_exchange.c.txt)。只替换这个函数，后面的CAN1_ReadParameter和其他函数保留。

这里使用现有CAN1_SINGLE_TX_WAIT_MS和CAN1_SINGLE_REPLY_WAIT_MS，没有改数值。前段FIFO清理只增加flush计数；真正回复等待中的任何帧增加rx_count并保留last_rx。TX成功和回复等待阶段分别记录，不把发送成功当回复成功。

## 3. 整段替换私有控制包装函数

找到`static uint32_t CAN1_ControlSingle(...)`的函数定义，连同原参数列表和完整函数体，替换为[完整带记录的控制包装](code_samples/can_control_trace_wrapper.c.txt)。

不要替换公共CAN1_SetMotorEnable，也不要修改原F3/FE/F6命令数组。新包装只围绕原调用做记录，原非02返回8、失败重置CAN会话仍保留。非法指针/长度不做诊断拷贝，不改变原输入拒绝行为。

## 4. 本轮只保存、完整Rebuild

把完整日志发来，先不下载或再发request1。只改can.c，main、app_config、power、safety、hwt代码全部保持。不得手动调用Power_Enable或强写PE2/VM，也不改20/30ms或100ms保护。

## 后续观察（构建核对后再安排取证）

```text
g_can_ctrl_fault_latched
g_can_ctrl_trace_result
g_can_ctrl_trace_phase
g_can_ctrl_trace_address
g_can_ctrl_trace_length
g_can_ctrl_trace_request
g_can_ctrl_trace_pe2_start
g_can_ctrl_trace_pe2_end
g_can_ctrl_trace_tx_ms
g_can_ctrl_trace_total_ms
g_can_ctrl_trace_rx_count
g_can_ctrl_trace_last_rx
g_can_ctrl_trace_esr
g_can_ctrl_trace_flush_count
```

phase1为初始化/发起前，2为等待发送完成，3为等待回复。第一条准备控制应为地址1、请求5字节F3 AB 00 00 6B；实际ExtId按原逻辑0x0100。rx_count0表示回复窗口内没取到FIFO帧；rx_count>0而result5时检查last_rx的IDE/RTR/ExtId（或StdId）/DLC/Data，可能是非匹配帧。result8表示已收格式匹配回复但状态非02。pe2_start/end只表明GPIO输出允许位，不能代替实际电源测量；若start1/end0，供电可能在交易中因独立监督撤销，需结合首次故障再判断。

不要清零g_can_ctrl_fault_latched以制造新证据；复位才开始新记录。诊断版不新增硬件动作，但原会话仍具备受控运动功能，构建通过不等于可以随意重发运动。

## 验证和需求边界

实际Exchange/Control样本以GCC C99 Wall/Wextra/Werror和mock CAN/GPIO/时钟运行，覆盖无回复、错地址原帧、非02、发送超时、PE2高低记录、首次失败后继续交易/轮询不覆盖、非法参数及原20/30ms期限和帧保持。完整实际can.c的隔离候选ARMCC Cortex-M4.fp无诊断编译；初次生成脚本的缩进子串匹配不唯一已修为逐行锚点，只影响工具生成，实际文件未改。未整套链接/下载/操作硬件。证据can_control_trace_preparation.json、tmp/can_control_trace_20261021/，失败图与读数heading_after50Hz_can1_prepare_timeout.json。

R已核对RUN-01/PWR-01；E已核对LOC-03/05/07、RUN-06、MOT-02/04。官方Q继续待技术答疑确认。真实首次CAN/PE2状态、实际VM电压、合法应答时延、转角/物理停车和X/Y仍待闭环，不能因HWT50Hz通过而放行整车。
