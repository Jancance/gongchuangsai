# 真实扫码接入当前原料区定位与三件抓取

## 范围与当前状态

用户授权：按一次PC13后接收真实扫码，合法码锁存后继续现有视觉定位、双鸣和三件抓取放车归位。仅接此入口，不接第1段路线或第3段，不改机械参数、Pi视觉算法、HSV阈值或保护条件。实现基线为已实车通过的073dfa1e（`tmp/source_loaded_retract30_20261116/project`），候选为`tmp/source_real_qr_20261116/project`。

本地MATLAB前置282项、实际C2312项和双目标完整构建均通过，在线cd9fa966/Offline945ff9fd。独立审查发现的邮箱发布竞态已修复并有确定性交错回归，最终复审无剩余阻断发现；Pi实际包process/SC2/OC2独立163项通过，详见证据目录`review.md`。这些属于软件证据，不代表本候选实物扫码、显示或三件通过。

主代理收到用户停稳空爪承托确认后于03:06:04执行烧录，Keil `Verify OK / Application running`；03:07:35主代理只读完整186976字节匹配cd9fa966，state0/scanowner0/lock0/colors000/故障0，证据`read-only-20261116-030735.json`。Pi同包pid2236持续frame_sequence5912，已收到新固件SC2 idle000，证据`pi-postflash-readonly.log`。随后用户已按一次PC13并扫码，但主控零字节接收、实屏无显示；现场扫码诊断见下一节。三件尚未开始，本实现代理没有操作硬件。

## 03:13—03:14现场扫码未收字节诊断

用户已按一次PC13并扫码，反馈实屏没有显示。`read-only-20261116-031347.json`再次完整186976字节匹配cd9fa966：state1/prep0/run1/scanowner1/lock0/colors000，无故障、无运动，扫码接收数/错误数/接受数/拒绝数均0，displayed_run0。这表明已进入等待真实码状态，但当前证据里主控未收到扫码字节；尚未锁定任务，屏幕显示函数尚未被触发。

`scanner-registers-0314.json`只读核对：UART5_CR1=8228（UE/RE/RXNEIE均开），BRR=4375对应42MHz外设时钟下9600，SR=192未见接收错误；PD2配置为UART5复用，UART5时钟与NVIC使能已核对。上述寄存器仅说明读取时的软件配置，不能据此认定外部接线、电平、扫码头输出或屏幕已正常，也不能把零接收直接归因为屏幕、代码或电气故障。

旧只读监测已Ctrl+C保存`pc13-watch-20261116-031049.json`，属于人工截断而非完成结果；主代理新watch session57402继续只读等待。当前只要求用户确认扫码头是否亮、扫到码是否有成功提示音，等待回复后再据证据缩小范围。没有复位、清错或运动写入，没有改代码/阈值/参数，不重复按键。R/E/Q分类和未闭环项保持上述边界，扫码/实屏/本候选三件实物均未通过。

## 实现

隔离工程仅改三个文件，保持原编码/CRLF：`CHASSIS/USER/source_mcu_cycle.inc`、`main.c`、`vehicle_arm.h`。机械owner、参数、驱动、准备请求表、原定位与三件路径、第一/第三段路线源码逐字节保留。

TaskA独占UART5扫码队列、解析器及屏幕。新run通过既有`SAOMA_DisableReceive→EnableReceive`清空旧完整码、排队字节及解析中半帧，记录accept基线。按键和CAN任务不直接操作扫码内部状态。短临界区快照完整16字节码、accept/result及帧起止时刻到单槽邮箱；邮箱pending期间不覆盖。CAN在原state1/prep0等待并消费，复用原`Vision_SetTask(code,1,run)`执行完整四段业务校验，合法后复制第一段三色并最后发布`locked_run`。最终发布临界区重查state/run/owner/lock/pending，避免CAN抢占接受首码后TaskA用后一码覆盖显示内容。

无合法码时颜色全零，不发原准备/底盘/抓取动作，没有固定215回退。错误业务码只计数并等重扫，没有新增停车条件、故障阈值或等待超时；等待时间仍受正式比赛预算约束，不能说已完成RUN-04现场验收。再次按键沿用原STOP。

锁存后TaskA关闭接收并仅调用一次原`Screen_ShowTaskCode`显示完整四段码，保留原显示结果/成功计数。屏幕I/O不在CAN或临界区。低层解析器同次drain可能处理多个帧，因此语义是“锁存本轮一条合法完整快照”，不宣称锁存同次排队的第一条UART帧。后续完整码/重复码均不能修改已锁任务。`g_vis_loaded_mask`、`batch_complete`等仅原任务初始化，不伪造抓取/库存成功。

Pi包保持`output/source_observe_ready_each120_pi_20261116`，原26文件SHA及ZIP10d0ec98不变。等码保留run1/pose1/state1/prep0/item0供原快重启重绑，state1不识别物料/发观测；原动态SC2 colors在合法码锁定后传给现有process，无需修改Pi。

## 验证与证据

证据目录：`evidence/integration/2026-11-16/source-real-qr`。

- `matlab-result.json`：本机`D:/MATLAB1/bin/matlab.exe`前置模型282项。模型为扫码门控/顺序/锁存合成状态验证，不是动力学或实物证明。
- `actual-real-qr.log`：实际UART5 IRQ/队列/解析器、原四段任务函数与MCU门控C共2312项；全部120种三色有序组合，旧完整/半帧清除，错误码重扫，pending后新码，锁定重复码，确定性CAN抢占，屏幕完整文本且一次，间隔分帧及STOP。仅寄存器/RTOS队列/时钟/屏幕/机械总线为夹具。
- `pi-dynamic-task-review.json`：独立实际Pi process163项，包含动态643目标、120种任务、等码不出观测及快重绑。
- `Vehicle_LiftRuler[-_Offline]-build.log`：在线和Offline完整构建均0错误0警告。`candidate.json`保存源差异、机械/驱动/参数未变哈希和AXF哈希，`checks.json`记录软件结论。
- 在线AXF SHA256：`cd9fa9663eb46f4b67d0128bcac58ecc871606bd148083b6662be6cfb97cda3c`。Offline：`945ff9fdcde71f42fc997280971fc93e18086c8c0e8fc75d861cb78ab0bf69d4`。
- `scripts/source_real_qr_live_20261116.py`默认仅本地manifest/ELF符号检查；`--snapshot`/`--watch`显式只读连接，包含完整码、扫码计数/帧时刻、任务颜色和库存字段。没有请求写入、复位、烧录或主机动作命令。修复了通用ELF加载器只提取`g_`导致遗漏`SAOMA_Data`的监测问题，准确补地址并断言16字节，本地检查通过。

## R / E / Q 与未闭环

已检查规则事实R：QR-01（四段码/原始完整保留）、QR-05（颜色1至6）、QR-06（三色不同且两批同集合）、RUN-01（一键自主）、RUN-02（原抓取放车再运输次序保留）、RUN-04（15秒停滞/等待转盘加8秒，未以软件模型声明现场通过）。相关QR-07/DSP-01/SRC-01/02由独立审查一并核对。工程选择E：本轮只执行第一批三件，真实UART5接收、原业务校验、邮箱锁存和原屏幕函数复用。官方Q：QR-04等未明确事项仍待技术答疑确认，未把工程建议写成规则。

仍须实物验证：真实扫码电气链路与非法码重扫、无扫码无动作、锁存码与屏幕一致、实际三色顺序及三件归位、STOP；后续第1/3段整合、3秒定位/25秒抓取、运动前窗停盘例外及正式比赛等待预算继续开放。当前仅本地交付，未提交或上传GitHub。
