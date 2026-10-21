# 执行器状态快照：用户手动增加只读接口

> 最新：模式取值修正已由用户完整Rebuild通过，38908/576/1456/25432、0错0警告、2秒，实际源已确认s_mode。下方更正要求已完成。下一项见[真实任务接入指南](P1_HEADING_RUNTIME_BINDING_MANUAL_2026-10-21.md)，不再重复本接口构建。

> 本轮构建已完成：用户日志38908/576/1456/25432、0错0警告、3秒，程序大小不变符合接口未调用的范围，不重复要求原接口构建。只读检查发现上一版样本dry_run字段误取g_chassis_auto_dry_run请求参数；该参数不等于Begin时实际锁存的s_mode。当前唯一修正：在实际chassis_auto_test.c的ChassisAuto_ReadStatus中，把`status->dry_run = g_chassis_auto_dry_run;`改为`status->dry_run = s_mode;`，保存并完整Rebuild后回传日志。其他文件、main和静态配置保持，本轮不下载。以下完整样本已更正；先前mock未覆盖请求与实际模式不一致，不能沿用其通过结论证明此问题正确。

接续：真实航向源快照完整构建38908/576/1456/25432、0错0警告；用户Watch截图显示valid1、sequence0x2B8=696、发布/角度帧/角速度帧龄期1/4/5ms、yaw约0.038452度、rate0、VM请求/RTOS fault/Safety fault全0。接受本次观察时刻的静态数据交接通过，不推断连续时效、断流保护或真实闭环通过。

本步解决后续HeadingSession同时读取执行器active/result/stop_mask时，可能跨状态更新混读的问题。接口在同一个临界区复制当前状态，不读取电机、不发CAN、不请求VM、不开始或重置会话。助手不自动修改实际固件。实际工程仍为`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。

## 1. 修改实际USER/chassis_auto_test.h

退出Debug。在文件最后的`#endif`之前加入[完整类型及声明](code_samples/chassis_auto_status_header.c.txt)。保留现有所有声明和结构体，不把新代码放到已有typedef内。

## 2. 修改实际USER/chassis_auto_test.c

在文件末尾、所有现有函数之外加入[完整只读函数](code_samples/chassis_auto_status_body.c.txt)。现有文件已经包含chassis_auto_test.h和FreeRTOS任务头，不新增包含目录或Keil文件。

该函数访问本文件已有的s_heading_session私有标志；因此必须放在chassis_auto_test.c中，不能贴到main.c或其他文件。

## 3. 保存、完整Rebuild

回传完整构建日志。本轮main和配置不变，APP_HWT_STATIC_TEST=1、APP_HWT_TURN_TEST=0，不下载、不开始动作。接口未被main引用时可能被链接器移除，程序大小不变也不代表没有加入，不因此重复构建。

## 字段含义及后续接入约束

返回1只说明复制成功；active/result/cause为原执行器状态，不能把返回1当完成。rotation_stop_mask固定读取stop_mask[2]，不能用其他段的旧应答代替。

session用于后续核对BeginHeading实际接受的执行器会话号，heading_mode区分原三段执行与单次航向，dry_run读取s_mode，只在已接受的对应会话中解释为实际锁存的演练/执行模式；Init后的闲置字段不构成运行模式证据。g_chassis_auto_dry_run只是请求参数，不能用于判断当前会话模式。接入时必须校验会话归属，不能把别的动作、演练或旧会话结果算成本次目标航向成功。cut_vm是执行器请求切VM的标志，不是物理断电反馈。

STOPPING完成仍必须结合本次STOP请求被接受、正确会话、result1/非活动/旋转stop_mask0F等条件；SETTLING临时零速不能结束会话。上述软件结果不等于车体已物理停稳。

接口只复制现有状态，不主动执行Safety检查。它不能代替实时源健康、platform_ok、互斥入口、独立Safety监督或故障锁存，后续整合仍须逐项完成。

样本已通过GCC C99 Wall/Wextra/Werror编译及mock运行：空指针返回0、闲置/活动/终态字段复制、始终取stop_mask[2]、状态不被修改、临界区成对。实际私有标志和字段存在已只读核对。未做ARMCC/完整构建或板上并发测试；证据evidence/p1-foundation/2026-10-21/heading_backend_snapshot_preparation.json。

规则检查：R RUN-01/PWR-01；E LOC-03/05、RUN-06、MOT-02/04。官方Q保持待技术答疑确认。未闭环：本接口用户整工程构建、真实会话/动作/Safety绑定、连续时效及失效响应、物理停止/到位误差、X/Y定位。助手未修改实际工程或操纵硬件。
