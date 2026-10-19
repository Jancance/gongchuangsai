# 张大头CAN正常模式只读版本（2026-10-10）

> 最新接续2026-10-11：四台单节点/共CAN1基本读取及复位默认请求0/VM实测0V闭环。下一项主控暂停且VM请求1时，用现有硬件电源开关切断VM并测压；测后撤销软件请求，源/菜单保持。完整故障安全、持续网络/运动尚未验收。

用户明确希望按已给厂家手册直接联调，减少前置测量。已重新阅读/完整查看归档ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527的第28页CAN接线图、第42～43页帧格式、第67页版本命令，并核第18页配置。厂家接线图要求CAN硬件和CAN1_MAP，未在该图标出两端120Ω；终端要求是总线可靠性工程建议，不能把额外测量擅自当所有首条只读命令的硬门槛。当前用户已接短线/模块/菜单且F407内部回环通过，先保持现有PCB/短线做实际只读交易；如无ACK/错误再针对性查终端/接线。64.5kΩ的隔离读数保留，未新增外接电阻，不声称完整四轮分支网络/运动可靠性已验。

本指南不改用户源，由用户手动在原工程补丁接入。标准库＋FreeRTOS原生API、中文注释保持。只发版本读取1F，不发运动、校准、回零、位置清零或使能指令；VM仍由已有临时请求/硬件开关控制，启动时请求均为0，CAN查询须用户手动触发。

## 1. 厂家命令依据

| 项 | 当前首台设置 |
|---|---|
| 节点 | 左后ID1 |
| 速率 | 500000 bit/s，按已记录实际菜单 |
| 类型 | CAN扩展数据帧，DLC2 |
| 扩展ID | `(1 << 8) | 0 = 0x0100` |
| 数据 | `1F 6B`，地址不放进CAN数据区 |
| 正常回复 | 扩展ID0x0100，DLC6，`1F FW高 FW低 HW高 HW低 6B` |

原始FW/HW字段先保存，不能仅凭版本数字混用X/Emm运动格式。第67页硬件类型码数量/示例文本存在歧义，不据此猜完整板卡类型。CAN层发送OK只证明有节点ACK；必须实际收到匹配的版本数据，才能认为本次读取通过。

## 2. SYSTEM/CAN/can.c改三处

原CAN模块已经存在，保持GPIO、500k参数及过滤器。将静态初始化函数声明改成：

```c
static uint32_t CAN1_InitMode(uint8_t mode)
```

原来的静默模式注释和`can.CAN_Mode = CAN_Mode_Silent_LoopBack;`两行一起改成：

```c
/* 由调用者选择静默回环或正常通信模式 */
can.CAN_Mode = mode;
```

原CAN1_LoopbackTest()内初始化调用改成：

```c
result = CAN1_InitMode(CAN_Mode_Silent_LoopBack);
```

文件头补：

```c
#include <string.h>
```

在文件末尾添加：

```c
/* 只读版本：1成功，2/3初始化问题，4发送失败，5无回复，6格式异常 */
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t start_tick;
    uint32_t result;
    uint8_t mailbox;
    uint8_t status;

    if ((address == 0U) || (reply == NULL))
    {
        return 7U;
    }

    memset(reply, 0, sizeof(*reply));
    result = CAN1_InitMode(CAN_Mode_Normal);
    if (result != 1U)
    {
        return result;
    }

    memset(&tx, 0, sizeof(tx));
    tx.ExtId = (uint32_t)address << 8;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = 2U;
    tx.Data[0] = 0x1FU;
    tx.Data[1] = 0x6BU;

    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        return 4U;
    }

    start_tick = xTaskGetTickCount();
    while (1)
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
        if (status == CAN_TxStatus_Ok)
        {
            break;
        }
        if ((status == CAN_TxStatus_Failed) ||
            ((TickType_t)(xTaskGetTickCount() - start_tick) >=
             pdMS_TO_TICKS(100U)))
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return 4U;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    start_tick = xTaskGetTickCount();
    while ((TickType_t)(xTaskGetTickCount() - start_tick) <
           pdMS_TO_TICKS(200U))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0) != 0U)
        {
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
            if ((rx.IDE == CAN_Id_Extended) &&
                (rx.RTR == CAN_RTR_Data) && (rx.ExtId == tx.ExtId))
            {
                /* 保存该节点的原始帧，异常时也能查看 */
                *reply = rx;
                if ((rx.DLC != 0U) && (rx.Data[0] == 0x1FU))
                {
                    if ((rx.DLC == 6U) && (rx.Data[5] == 0x6BU))
                    {
                        return 1U;
                    }
                    return 6U;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    return 5U;
}
```

发送等待最多100ms、回复等待最多200ms，原生vTaskDelay让其他任务继续；本轮只有静态只读测试，不将这些等待当正式运行安全状态机。初始化正常模式会清原CAN1状态/过滤器，当前没有CAN2业务；后续完整共享CAN服务另设计。不无限重试，不广播，不在每次循环自动读取。

## 3. SYSTEM/CAN/can.h补声明

在已有STM32头后补CAN库头、在endif前添加函数声明：

```c
#include "stm32f4xx_can.h"
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply);
```

## 4. USER/main.c全局与TaskA

全局区添加：

```c
/* 版本读取仅由Watch手动触发一次 */
volatile uint32_t g_can_version_request = 0U;
volatile uint32_t g_can_version_result = 0U;
volatile uint32_t g_can_version_esr = 0U;
CanRxMsg g_can_version_reply;
```

在TaskA循环内Power_Enable/Power_Disable的if/else完整结束后、g_test_tick更新前加入：

```c
/* 已请求VM供电后，手动读取左后ID1的版本 */
if ((g_can_version_request == 1U) && (g_vm_test_request == 1U))
{
    g_can_version_request = 0U;
    g_can_version_result = 0U;
    g_can_version_result = CAN1_ReadVersion(1U, &g_can_version_reply);
    g_can_version_esr = CAN1->ESR;
}
```

保留原一次回环调用、Power_Init及所有现有任务。版本函数内部切正常模式，不能只把过去回环发送当电机通信。暂无新任务/ISR/队列；错误码7为调用参数异常，主控正常调用1U不会广播。

## 5. 用户执行与回报

按厂家已有CAN接线，断电恢复左后CAN模块/接口及电机VM供电连接；若此前为隔离测量拔下了它，先按原接法恢复。其他节点参数保持，首条查询目标只为ID1。若35/28另接当前CAN1须保证没有同址1，实际回复出现冲突时再定位，不凭旧图推定这些节点已接或已设地址。

1. 保存Rebuild，0错误0警告后下载、F5运行；所有请求默认0。
2. 当前仅静态读版本，机构保持静止；先设g_vm_test_request=1并F5，待电机屏幕完成正常启动后再触发查询。实际硬件开关须处于允许VM的状态，g_vm_test_request=1只是软件请求，不等于实测VM必然有电。
3. 再Stop设g_can_version_request=1，F5运行约1秒，Stop看Watch。
4. 回报g_can_version_result、g_can_version_reply.ExtId、.DLC、.Data及g_can_version_esr；测完request0/F5撤销VM请求。

预期result1、ID0x100、DLC6、数据`1F xx xx xx xx 6B`。result4先查是否真实上电/收发器/ACK与速率，result5是发送后未等到版本，result6保留原帧看格式。不要从CAN层ACK签动作完成或完整四轮通过。

## 6. 需求与范围

R检查PWR-01，E检查LOC-03、RUN-06、MOT-04，官方Q不变。改为先现有短线只读是用户明确的工程排查顺序，不否定规范终端和分支可靠性目标，不放行运动或完整安全；手册图未画终端不代表任何总线都不需终端。实际版本/帧/错误/故障仍需实物证据。用户手动代码，中文注释，原PCB和源文件不被助手修改。
