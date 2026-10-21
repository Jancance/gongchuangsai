# 目标航向会话管理模块：本轮用户手动加入并编译

实际工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。助手不自动修改实际固件。当前仍APP_HWT_STATIC_TEST=1、APP_HWT_TURN_TEST=0，本轮不改main或配置、不下载。

## 1. 新建头文件

退出Debug。在实际工程USER目录新建`heading_session.h`，将[完整头文件代码](code_samples/heading_session.h.txt)全部复制进去并保存。注意最终扩展名是.h，不是.h.txt。

## 2. 新建实现文件

在同一个USER目录新建`heading_session.c`，将[完整实现代码](code_samples/heading_session.c.txt)全部复制进去并保存。最终扩展名是.c，不是.c.txt。中文编码沿当前USER文件使用方式保存。

## 3. 加入Keil USER组

在Keil左侧工程树的USER组上右键，选Add Existing Files to Group 'USER'，选实际USER目录刚保存的heading_session.c，点击Add，再Close。头文件位于已有USER包含目录，无需新增IncludePath。

不要把docs/code_samples或tmp中的.txt/隔离文件加入Keil。现有heading_controller.c/h及其他模块保留。

## 4. 当前唯一验收

保存，完整Rebuild，把完整日志发来。本轮应看到compiling heading_session.c且0错误0警告；没有main调用时链接器可能移除未使用函数，不能用程序大小不变判断没加入。

本轮没有新增运行入口，VM保持关闭，无需下载。真实任务/CAN绑定是下一项，不直接从Watch调用此模块或开启VM。

## 模块职责和行为

本模块只根据输入快照计算动作，不发送CAN、不修改VM，也不创建任务。未来由单一任务持有HeadingSession对象，其他任务只能交接完整输入/请求，不得并发调用同一对象。Init用于上电初始化；终态后不自动重启，不得在故障中循环Init来解除使用次数限制。

| 会话状态 | 含义 |
|---|---|
| HS_IDLE=0 | 未开始 |
| HS_RUNNING=1 | 航向运行/临时容差带稳定检查 |
| HS_STOPPING=2 | 控制器已稳定到位，等待执行器最终停车 |
| HS_DONE=3 | 停车请求已接受，执行器结果成功、非活动且四轮停止mask0F |
| HS_FAULT=4 | 故障锁存，要求ABORT |
| HS_CANCELLED=5 | 取消锁存，要求ABORT |

| 输出动作 | 未来绑定职责（本轮不接入） |
|---|---|
| NONE=0 | 不重发缓存，不刷新指令提交时间 |
| BEGIN=1 | 根据允许条件调用BeginHeading并用ReportBegin反馈是否接受 |
| UPDATE=2 | 提交本轮轮速，调用UpdateHeading并用ReportUpdate反馈接受结果 |
| STOP=3 | 调用RequestStop，并用ReportStop反馈接受结果 |
| ABORT=4 | 调用Abort并撤销VM请求，不能把取消/故障伪装成功 |
| COMPLETE=5 | 正常完成后撤销VM请求并保留会话结果 |

暂时SETTLING只输出UPDATE且轮速0，仍等待连续稳定；这时不能调用RequestStop结束会话。连续稳定后才输出STOP，STOP请求接受也不等于停车完成。backend_stop_mask必须来自旋转段stop_mask[2]，backend_result/active必须来自实际执行器的一致快照，不能填默认成功。

Start只接受相对±5～±45度工程候选目标，目标在接受时固定；同一对象只接受一次成功会话。当前首轮硬件候选应从±5度开始，不能因软件允许45度就宣布45度实转已经放行。输入不合法、后台已有活动/残留结果或平台不允许时拒绝开始。

临时零速后若控制器要求反向，本模块与执行器一样故障拒绝，尚不支持超调反向纠偏。后续必须完成真实零速反向互锁与实测后再扩展，不能删保护制造成功。

## 时序及源数据契约

输入now_ms/sample_ms及控制器时间均为32位毫秒，不直接把任意RTOS tick当毫秒。当前接口命名中controller.start_tick依旧保留原名，但本模块传入的是毫秒。实现覆盖32位时间和序号回绕。

source的sequence表示完整计算快照的更新序号，不是轮询CAN次数，也不是每次读取随意加1。sample_ms是该计算快照生成时刻。healthy必须由生产方检查实际HWT帧校验、真实帧年龄、采集任务状态和校准/配置互斥等条件，不能靠刷新sample_ms掩盖HWT没有新数据。HWT角度计数、计算快照序号及真实帧时间应在后续绑定中分开；此前静态预览的500ms窗口不自动批准为行车时效。

计算快照到龄100ms、同序号持续100ms或序号回退均失败。重复读同一序号不产生UPDATE，不能续执行器提交期限。平台故障、执行器异常/提前结束、BEGIN/UPDATE/STOP请求拒绝、9秒仍未到位、停车阶段2.5秒未确认均走ABORT。终态不会因数据恢复而自动重启。STOPPING不需要继续调速，正常反馈完成后允许平台因正常切VM而不再ready。

HS_RUN_MS=9000是首轮工程候选，从会话接受开始，包含之后执行器准备耗时，早于现有执行器10秒上界；不修改原默认heading_controller的20秒配置，本模块只复制配置并局部用9秒。大目标/慢速或准备较长时可能按设计超时失败，不能宣称必能完成±45度。后续根据实测协调准备/运行时限，不直接放宽保护。

platform_ok由后续绑定汇总新入口互斥、VM及CAN/Safety许可等，不是单一Watch变量；模块自身不操作供电或判断真实车体包络。接受BEGIN前后的状态交接及具体新模式开关仍待接入，不把本模块编译通过当整合完成。

## 已做检查和范围

代码样本逐字复制到tmp/heading_session_20261021/，与实际heading_controller.c一起通过GCC C99 Wall/Wextra/Werror编译和主机运行；检查程序tmp/check_heading_session_20261021.c。用例包含固定目标、正反分段轮速、临时稳定与最终停车区别、停车请求拒绝/部分mask不算完成、取消/故障锁存、过期/重复/回退、平台/HWT失败、9秒未到位、2.5秒停车超时、提前执行器结束、BEGIN/UPDATE拒绝、时间和序号回绕、一次使用及NaN/非法角度。样本heading_session.c通过ARMCC V5 Cortex-M4独立编译，无诊断。

这些是软件虚拟输入用例，不代表真实HWT、RTOS并发、CAN或物理停车通过。当前实际工程只读确认上一轮38380/576/1424/25408、0错0警告；新增文件的用户完整构建待日志。后续主流程绑定、独立Safety联动及运行入口互斥、端到端时效、反向纠偏、物理误差和X/Y仍待完成。

R检查RUN-01/PWR-01；E检查LOC-03/05、RUN-06、MOT-02/04。官方Q继续待技术答疑确认，未修改R规则。
