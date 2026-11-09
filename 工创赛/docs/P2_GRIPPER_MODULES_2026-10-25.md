# 夹爪代码模块化已接入实际工程

> 2026-10-25当前用户明确设定：未抓取时180度张开，抓取时140度闭合。实际配置已加入JIAZHUA_OPEN_ANGLE_DEG=180及JIAZHUA_GRAB_ANGLE_DEG=140，调试角度默认180；夹爪任务首次调度后自动发送180度准备动作，之后等待显式抓取请求。取代此前默认不发位置命令/默认140及150度试用建议。实际Build0错误0警告，Code39248；未下载。新固件下载/F5后会张开夹爪；140抓取位置不等于已确认夹持物料成功。证据evidence/p2-arm/2026-10-25/open180-grab140/。

> 2026-10-25实物反馈更新：用户实操确认angle=140刚好闭合、角度增大则打开。实际gripper_config.h已记录JIAZHUA_CLOSED_ANGLE_DEG=140并以此作为默认调试角度，增量Build0错误0警告，未下载/未触发运动。下文90/100为此前示例，当前夹爪实操改用140闭合、150小幅打开试用；150是否满足物料开口和最大张开端点仍未确认。小于140的行为未知，不把140外推成所有安装通用角度。

2026-10-25用户授权“把代码优化好分模块，别都往main里面写，以后你自己来把代码加入进去”。该决定取代此前仅提供手动片段、不改源的默认方式。后续相关工作由助手直接更新用户指定工程、加入Keil项目并编译，不默认让用户手工拼接模块；本次不执行固件下载或硬件动作。

## 实际变更

实际工程：D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx。

- HARDWARE/GRIPPER/gripper.c/h：PA0/TIM2_CH1驱动，保持jiazhua(float)和Gripper_Stop接口；初始化默认不发位置脉冲。
- HARDWARE/GRIPPER/gripper_config.h：270度/500～2500us标称映射、周期20ms、任务栈256字/优先级2/周期10ms等集中参数。标称映射尚不是本台实物机械角度标定。
- USER/gripper_task.c/h：独立原生FreeRTOS夹爪任务，调试变量定义及请求处理全部放在本模块；未知请求返回3，角度非法返回0，1表示PWM指令已写入，2表示已撤销信号，均不冒充机械到位。
- USER/main.c：夹爪相关仅包含头文件、Gripper_Init()和Gripper_TaskInit()入口；移除App_TestTaskA中的夹爪逻辑、旧g_gripper_task_create定义及已不存在的Gripper_TestTask创建。
- Keil项目：加入gripper_task.c/h、gripper_config.h和遗漏的标准库stm32f4xx_tim.c，不重复加入已有源。

本次只改上述6文件，其他底盘/航向/CAN/安全/电源及应用/RTOS配置字节保持。没有对整个main的历史业务进行重写，后续各机构新增功能按模块继续接入。用户保留的jiazhua(90);为非阻塞目标角度接口，位置PWM持续保持到下一目标或Gripper_Stop，不是相对再转90度。

## 编译与备份

先在工作区完整副本Rebuild，再将通过的文件同步实际工程并对实际工程Rebuild；两次0错误0警告，实际Code39232/RO588/RW1628/ZI25492。实际5个源码/头文件SHA与验证副本一致；Keil在实际构建时整理工程文件空白，工程XML内容逐项比较等价，最终文件SHA另存installed.json。完整日志evidence/p2-arm/2026-10-25/gripper-modular/actual-build.log，文件列表与SHA在manifest.json/installed.json，改前文件存同目录original/，没有删除用户原功能。

用户提供的5条错误是未定义的调试变量与遗留旧任务名称；模块已正确定义调试变量并移除旧任务入口，另补TIM标准库，最终链接通过。不需要再手工添加上一版调试片段。

## 下一项直接实操

退出旧Debug会话，重新打开实际gongxun-f407.uvprojx（已有窗口提示外部修改时重新加载），下载本次固件并F5。首次仍将舵机与夹爪机构脱开空载，其余轴/底盘动力关闭；不按旧PC13，不改旧底盘请求。

Watch：g_jiazhua_debug_angle=90后把g_jiazhua_debug_request=1一次；需要另一位置时改angle=100再触发1。观察转动，request=2撤销PWM。g_jiazhua_debug_result=1仅表示命令已写入，不能证明到位；g_gripper_task_create=pdPASS为任务创建成功。不要把表达式jiazhua(90)作为Watch显示项反复求值；该接口在C任务/动作组中按需调用。

按用户要求不增加额外电平/波形门，先实际动作，有异常再依据现象/日志排查。

## 规则与未闭环

R已核PWR-01/RUN-01/FIELD-07/MECH-01/02；E已核RUN-06/LOC-03。模块化、直接源码编辑与调试方式是用户E类决定，不新增或替代官方规则；Q保持待技术答疑确认。证据为用户原编译日志、实际源码、变更备份/哈希、两次Rebuild。下载/新任务板端运行、实物角度、行程端点、到位/停止、供电带载和固定动作组均未验，视觉后置不变。
