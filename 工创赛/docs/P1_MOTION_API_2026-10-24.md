# 单动作接口与速度显示（2026-10-24）

## 2026-10-24最新本地：运动接口整理完成，增加上层只读快照

用户要求先整理现有运动接口供后续调用。本轮motion_control.h按方向、范围、状态、读取、完成、取消和显示组织，补充非阻塞语义/任务上下文/模式3开发限制；MoveStart/TurnStart校验改引用头文件常量，原参数范围不变。新增MotionStatus及MotionReadStatus一次短临界区复制动作状态、会话是否打开、VM请求与PE2输出、后端活动/结果/原因/停止掩码、CAN/RTOS/Safety故障，空指针拒绝，不发送CAN、不修改运动。读取成功不等于健康，backend结果可暂留上一段，DONE不等于已断VM。

新增docs/P1_MOTION_CALLING_GUIDE_2026-10-24.md给出单一上层的周期调用示例、等待DONE/下一段/Finish/输出关闭、拒绝与超时处理。示例未接入任何任务，未增加自动启动或绕过串口保护；动作表继续复用现有接口，不重复新建调度器，不新增正式任务流程或后退。四种模式隔离完整构建均0错0警告，日志motion_interface_*_build.log；快照当前无人调用可被链接器裁剪，编译不算实机快照验证。证据motion_interface_review.json。原文件GBK/CRLF保持，差异检查通过。

本轮没有下载、开COM或发车；板上仍上轮两段短前进实测版本，源码默认四段表未变。忙时拒绝、独立调用方、新TurnStart及本轮快照运行未单独实测。保留已有源码和产物修改，仅本地完成未提交推送；fetch确认与origin/main均54bb617、无新提交。下一步可按此头文件接入确定的上层动作，无需重新全测已通过路线；正式自主入口、距离/XY与异常验证后续。核R RUN-01/PWR-01与E LOC-03/RUN-06/MOT-01/02，官方Q不变。

## 2026-10-24最新实车：新单动作接口两段前进衔接通过

用户确认摆好并可观察，发车前明确提醒；隔离构建motion_api_smoke_20261024仅将动作表设为两段前进15RPM各2000ms，不修改实际工程默认四段表。完整编译0错0警告，17:10命令行烧录Verify OK/Application running，先只读确认新58字段/sequence_count2/健康空闲/VM0，再一次ARM/START执行。当前板上已变为该两段短前进测试版，原54bb617四段程序已被替换；程序已完成，不自动再次发车。

软件software_complete，完成2段、总流程8895ms（含准备/停车/间隔）；段间四轮0且VM请求/PE2均1，最终四轮0、stopmask15、VM请求/PE2均0。CAN/RTOS/Safety/backend cause均0，遥测最大角度误差0.598度，当前段固件峰值0.422度；两者范围不同。用户确认两次前进、无明显摆动、段间灯亮、最终停稳熄灯。新MoveStart经动作表调用的衔接及MotionFinish关电通过本轮低速验证，不扩大为直接独立调用、忙时拒绝或新TurnStart已实测。

证据同日motion_api_smoke_build.log/download.log、api_smoke_prestart和api_smoke_run原始/JSONL/summary、motion_api_smoke_review.json。DAP开口仍见旧缓存，启动跳变及1无效帧保留，不把summary累计sequence_gaps73认定运行期丢帧。主机summary的baseline字段仍旧5f132ad，实际源基线54bb617加本地修改，已在review纠正说明。29项主机检查此前通过；本轮未上传GitHub。下一步按需要接入上层路线调用，未单测忙时拒绝/独立调用/新转向路径仍明确保留；距离/XY按用户决定后续再做，不测后退。

核R RUN-01/PWR-01与E LOC-03/RUN-06/MOT-01/02，官方Q不变；串口仅开发验证。

本轮仅本地完成。已上传回退点54bb617的四段组合实车通过；当前新增接口和动作表没有下载或实车验证。后退按用户决定排除。

## 调用接口

实际工程USER/motion_control.h声明接口，main.c复用原CAN任务和姿态闭环，不新建任务。APP_TEST_MODE=3启用；其余模式启动返回0、状态返回MOTION_UNAVAILABLE。

- MoveStart(direction, rpm, run_ms)：MOVE_FORWARD、MOVE_LEFT、MOVE_RIGHT；基础3至15 RPM，时长1至8000ms。
- TurnStart(direction, angle_deg, max_rpm)：TURN_CW顺时针、TURN_CCW逆时针；角度为正5至90度，轮速上限1至5 RPM。接近目标仍按原分段减速，上限只限制原5/3/1 RPM，不强制匀速。
- MotionGetState()：IDLE、BUSY、DONE、CANCELLED、FAULT；启动返回1仅表示接受，需等DONE才能提交下一动作，返回0表示拒绝。
- MotionFinish()：最后一个动作正常停稳后请求关闭VM，忙时拒绝。返回1表示请求接受，实际关闭仍由电源任务执行。
- MotionStop()：取消并走原停止/断VM路径，不能清除故障重新启动。
- MotionRpmToCmPerSec(rpm)：按当前轮径显示理论轮缘速度。

例如先提交MoveStart(MOVE_FORWARD, 15U, 8000U)，周期检查DONE后才提交TurnStart(TURN_CCW, 90.0f, 5U)；再次DONE后调用MotionFinish()。不要在同一次调用中连续提交动作或忙等阻塞原任务。这些是固件C接口，不是新增串口命令。

## 调度与电源

动作接口只提交请求，原CAN任务周期执行；忙时拒绝覆盖。成功单动作停稳后保持VM及使能，下一动作复核准备状态，不反复上电。原动作表现在调用同一接口；表项顺序、默认15 RPM/8秒及90度保持原配置，整组末尾显式Finish，等待关闭才成功。

直接调用与动作表互斥，不能抢占表正在执行的会话。保留后端会话归属、四轮停止确认、姿态新鲜度、独立截止、失联/取消、看门狗与故障锁存。故障不作为正常DONE续跑。当前仍沿用每次复位的新会话边界，Finish或取消后不通过清标志再次发车。

## 显示单位与协议

控制和接口参数继续用RPM，显示按rpm × π × wheel_diameter_mm / 600换算cm/s。直连77mm车轮，15 RPM约6.05 cm/s；这是目标理论轮缘速度，横移打滑、载荷和实际车速需要另行测量。

组合TM1在56字段基础上追加action_kind和target_theoretical_milli_cm_s成为58字段；后者单位0.001 cm/s，转向填INT32_MIN表示不适用。主机兼容48/51/55/56/58字段，并输出target_theoretical_cm_s。旧组合记录没有速度显示值；旧单平移记录按已知77mm轮径显示。原ARM/START/KEEP/STOP白名单不扩展为任意参数写入。

## 验证与下一步

29项主机协议/状态机测试通过；最终源文件与组合、单段、无线、手动四份隔离构建逐字节核对一致，各构建0错误0警告，GBK或UTF-8原编码及CRLF已检查。证据motion_api_*_build.log、motion_api_review.json。主机测试不代表MCU调度、动作接口或电源路径实测。

未烧录、未打开COM、未发车、未提交推送。下一步只需安排与改动相关的小范围实机回归：一个已通过的平移动作、忙时拒绝、DONE保持VM、Finish关闭VM，不重新全测组合，也不测后退。发车前告知用户观察；距离/XY仍留后续。

核对R RUN-01/PWR-01与E LOC-03/RUN-06/MOT-01/02；串口控制仅开发验证，正式比赛自主入口要求不变，官方Q不变。
