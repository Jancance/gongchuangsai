# HWT首次故障诊断（2026-10-24）

本轮仅本地新增诊断代码、主机解码及隔离构建，未下载、开COM或发车。板仍运行之前单左移30cm失败固件，当前不能从板上取得新增HF1，也不宣称HWT故障已修复。

## 记录的内容

每次平移请求被接受时重新布防诊断，保存HWT UART错误、环形缓冲溢出、校验错累计值作为基线。只清诊断锁存，不清错误累计数、接收状态、有效性或任何运动故障。

HWT接收路径锁存布防后的首个软件错误事件：reason1为USART2 UART错误，2为环形缓冲溢出，4为帧校验错误；保存事件时刻、USART2 SR、当时错误累计值和parser seen。UART另累计ORE/FE/NE/PE分项。reason0表示尚未锁存接收错误，不能当成硬件无故障的证明。ISR事件时间使用ISR tick，校验事件时间为错误帧末字节的接收tick；ISR/解析并发时它是首次成功锁存的软件事件，不能严格推定物理事件先后或排除后续多种错误。

首次App_ForwardHoldTrip在原始cause及后端状态尚未被Abort改变前，复制上述信息、后端state/cause/session、原phase、源有效性及帧龄。后续STOP不覆盖，供Watch查看的g_motion_fault_diag[29]及g_motion_fault_diag_valid保留到下一次接受平移或复位。手动模式可用Watch，无线模式不增加HF1发送；串口运动模式2/3支持HF1。

## HF1帧与主机

HF1为独立29字段uint32诊断帧，完整顺序见P1_SERIAL_MOTION_SCHEMA_2026-10-24.json的fault_diagnostic。CRC16与TM1一致。HF1不参与控制令牌、健康判断或KEEP续期，TM1/PM1/PE1/PE2格式均不变；只在故障后约每秒重发，和TM1至少错开60ms，忙时稍后重试。运行中不新增HF1串口流量。

主机解析并保存fault_frames/last_fault及JSONL frame_type=HF1。检测motion_fault_or_cancelled后先按原机制发送一次STOP，再最多额外接收1.5秒捕获HF1，不再次ARM/START/KEEP。该接收窗口不改变固件运动截止或保护，也不保证丢包/串口拥塞时必收HF1，必要时另开monitor读取重发；Watch可独立读取。

UART错误原因需结合基线差值和SR分项解释：ORE证明接收发生过溢出，不单独证明是临界区、通信负载或外部电气因素；FE/NE/PE也不等于接线根因已证实。校验错误同样可能由丢字节、噪声或错位引起。first trip和软件事件相互印证，不把取消码9当原始故障码。

## 验证及边界

64项主机测试通过，包括HF1 CRC/uint32/非法标志/格式隔离及真实主机Controller的模拟串口故障流程：ARM、START、STOP各一次，故障后收到HF1仍保留失败结果，绝不重发运动。

默认组合、单左移300mm、串口单段、手动、无线、位置诊断关闭六种完整隔离Rebuild均0错0警告。源逐字节与构建副本一致、GBK/CRLF及diff检查通过。证据为hwt_fault_diag_*_build.log和hwt_fault_diag_review.json；默认Code56776/RO788/RW2000/ZI32224。新C锁存未在MCU执行，ISR/任务开销、最坏栈和UART运行影响仍待硬件验证。

未调整启动临界区、没有测量其耗时，原先屏蔽USART2过久的猜测仍待证据；没有改变100ms姿态时效、400ms位置保护、速度/方向限制、VM与停稳保护或看门狗。下一步若安排复现，应下载已核对的隔离单段诊断版，先只读健康检查，再按现场确认执行一次；发生故障先保存HF1，不自动连续重跑。如HF1指向ORE再测启动临界区最长时间及字节到达相位；不能只见ORE便直接放宽门限。

核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04；官方Q不变。根因、横移实际位移/停稳、距离精度及临界区耗时仍未闭环。
