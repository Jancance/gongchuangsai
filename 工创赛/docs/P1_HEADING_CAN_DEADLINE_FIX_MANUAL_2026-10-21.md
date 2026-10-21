# 新会话CAN等待期限与100ms提交监督协调

> 最新：用户已纠正两处发送等待位置，整工程35744/564/1444/25452、0错0警告/3秒；实际can.c完全匹配已测候选，main/模式和100ms监督未变。下方修改/构建要求已完成，不再重复。当前下一项：外部VM关闭时下载并复位新固件，按既定完整装机载物条件做一次+5度复测；Run≥3秒后仍在Run中写exec_request1一次，不手改VM或重复请求，观察约18秒后保留现有Watch及last_address，反馈实际VM/转动/自动停稳。仍未宣称实机故障解决。证据heading_can_deadline_actual_rebuild_pass.json。

> 最新位置核对：用户完整构建0错0警告，但TX宏替换到了LoopbackTest，ExchangeSingle发送等待仍100ms。当前只纠正实际can.c两处：第152行回环发送恢复`pdMS_TO_TICKS(100U)`；第269行ExchangeSingle发送改为`pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)`。回复等待已正确，不重复修改。纠正候选与已测版本token一致，实际配置ARMCC无诊断；用户纠正后完整Rebuild，先不下载。证据heading_can_deadline_placement_review.json。

本次失败证据：用户+5度试验报告VM指示未亮，第二张Watch显示backend_session1、会话state4/cause2、执行器state10/result2/cause3/io_result0/start_mask[2]0、运行桥phase4/trip3。说明执行器接受过会话，但没有记录成功旋转启动；这些终态不能单独证明具体失败分支或VM物理电压。

已核对的设计缺陷：Runtime由唯一CAN任务计算并提交；该任务调用CAN1_ExchangeSingle时，原发送等待100ms、回复等待200ms，可能超过独立提交监督的100ms期限，使执行器先cause3退出，原IO结果因会话已失活而没有写入。源码可确认这一时限冲突；它是本次现象的可复现机制，实机首次触发原因仍需修正后结果进一步确认，不认定电源或CAN硬件损坏。

本轮不放宽100ms监督，只使新SESSION模式单次发送/回复等待各20/30ms，标称两阶段合计50ms，给10ms任务间隔与真实调度留候选余量；不保证实际整机响应一定小于100ms。旧模式仍100/200ms。正常模式初始化的独立等待未纳入此两阶段预算；活动会话应复用等待平台阶段已经建立的正常CAN会话，不宣称所有初始化都在50ms内。

## 1. 修改实际SYSTEM/CAN/can.c

退出Debug，关闭VM外部开关。保留原#include，在文件开头的包含区增加：

```c
#include "app_config.h"
```

在全部#include之后、首个static变量之前加入：

```c
/* 新目标航向模式不允许单次交易长于指令提交监督期限。 */
#if APP_HEADING_SESSION_TEST
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 30U
#else
#define CAN1_SINGLE_TX_WAIT_MS 100U
#define CAN1_SINGLE_REPLY_WAIT_MS 200U
#endif
```

20/30ms为待真实CAN应答验证的工程候选，不是厂商响应保证；如果现有合法应答确实更慢，后续应改为异步交易/独立计算提交，不能自动扩大安全监督期限。

## 2. 只替换CAN1_ExchangeSingle内部的两个等待参数

找到该函数发送等待循环中的：

```c
             pdMS_TO_TICKS(100U)))
```

仅把100U替换为CAN1_SINGLE_TX_WAIT_MS：

```c
             pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)))
```

同一函数后面的回复等待循环原来是：

```c
    while ((TickType_t)(xTaskGetTickCount() - start_tick) <
           pdMS_TO_TICKS(200U))
```

改为：

```c
    while ((TickType_t)(xTaskGetTickCount() - start_tick) <
           pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
```

不要全文件替换100U/200U，不改CAN1_InitMode、回环、帧格式/滤波/地址/速率/校验或任何发送参数。不改chassis_auto_test的100ms监督，也不改100ms源数据期限，不重写main或电源控制。

## 3. 本轮只完整Rebuild

保存、完整Rebuild，回传日志。本轮先不下载、不重新发request1。原失败截图已归档，不需要保持它来继续修改。下一轮核对实际修改与构建后，再决定受控复测；若CAN本身没有及时回复，预期能先得到真实IO失败码，不能把这种新错误当作运动通过。

## 范围、证据和未闭环项

检查已完成：同一实际CAN1_ExchangeSingle函数经GCC C99 Wall/Wextra/Werror模拟新旧模式，覆盖未改帧/地址/长度、发送超时、缺回复、截止前最后有效回复、截止时拒绝、tick回绕；旧等待可跨过100ms监督期限，新模式两阶段先返回。实际完整can.c候选经ARMCC Cortex-M4.fp分别以SESSION1/0编译无诊断，原实际can.c哈希前后不变。证据heading_can_deadline_preparation.json，复现tmp/check_heading_can_deadline_20261021.py。这些为软件mock/隔离编译，不是板上超时或合法应答保证；用户真实整工程构建及实测未做。

原图/读数见heading_positive5_backend_cause3.png/json。100ms提交监督与两种cause3路径（提交中断/运行阶段反馈失效）仍保留；未在失败当刻记录分支，故不伪称已实测锁定唯一根因。用户没有把实物VM电压或整车动作报告为通过。

R检查RUN-01/PWR-01；E检查LOC-03/05/07、RUN-06、MOT-02/04。官方Q继续待技术答疑确认。本步只修新模式交易等待预算，不改变竞赛规则或解除正式运动许可。真实合法应答时延、调度/失效响应、物理VM、目标角与停车、X/Y仍待闭环。
