# 电机使能、立即停止与Emm速度接口：手动修改指南

2026-10-15。本轮只手动修改can.c/can.h并Rebuild，不添加main调用、不触发动作。用户已确认整车机械臂、夹爪、托盘完整装好，可以按实际载物状态测试；这属于装配状态报告，不等于低速运动、载荷、轮向或安全限值已经验收。

## 1. 接口与已核对的协议

当前四轮地址1左后/2右后/3左前/4右前，FW_Emm、CAN1_MAP、500k、固定6B保持。厂家手册V1.0.5第42/43/50/53/60/81页已提取并查看渲染页，三个接口均为单扩展帧，地址放ExtId=address<<8，数据里不再重复地址。

| 接口 | CAN数据 | 应答 |
|---|---|---|
| 使能/失能 | F3 AB enabled 00 6B，DLC5 | F3 02 6B，DLC3 |
| 立即停止 | FE 98 00 6B，DLC4 | FE 02 6B，DLC3 |
| Emm速度 | F6 direction speedH speedL acceleration 00 6B，DLC7 | F6 02 6B，DLC3 |

本阶段同步位固定00（立即执行），不发送广播、不缓存同步命令。使能1是锁轴，0会松轴；停止命令与失能不是同一动作。接口返回1只表示收到该地址/功能码的02应答，必须另用3A/35等实际反馈确认状态和速度，不等于动作完成或安全停车。

速度用speed_raw命名：手册第81页说明缩小10倍输入设置会改变命令值对应的速度。当前四台该设置尚未报告，不擅自当作RPM换算，不发送4F配置命令改它。3000是本阶段遵守的协议值上限，不是本车安全转速；加速度0意味着直接启动，实际运动参数需后续限幅/试验。F6表还列出12/22而第4章未完整解释，因此本阶段非02均返回8并保留原始回复，不能默认为成功。后续遇到12/22需按真实驱动版本确认，不能盲目放行。

## 2. can.h新增三个声明

文件：SYSTEM/CAN/can.h。放在原位置读取声明之后、#endif之前，原有所有声明保留：

```c
/* 仅由独立CAN任务串行调用；不得在中断或其他任务中调用 */
/* 控制返回1仅表示收到02应答，不能代替实际状态/转速确认 */
/* 返回8表示非02应答，原始应答仍在reply中；地址限定1到4 */
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                              CanRxMsg *reply);
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply);
/* speed_raw是原始指令值；缩小10倍输入设置未核实前不换算为RPM */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                              uint16_t speed_raw, uint8_t acceleration,
                              CanRxMsg *reply);
```

## 3. can.c将原单帧查询改成通用单帧收发

文件：SYSTEM/CAN/can.c。找到原来的 `static uint32_t CAN1_ReadParameter(...)`，只将这个函数从签名到匹配结束大括号替换为下面两个函数。前面的初始化/回环和后面的CAN1_ReadVersion/ReadStatus/ReadSpeed/ReadPosition全部保留；不能删掉已有读取接口。

```c
static uint32_t CAN1_ExchangeSingle(uint8_t address,
                                    const uint8_t *request,
                                    uint8_t request_length,
                                    uint8_t reply_length,
                                    CanRxMsg *reply)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t start_tick;
    uint32_t result;
    uint8_t mailbox;
    uint8_t status;
    
    uint8_t flush_count;
    if ((address == 0U) || (reply == NULL) || (request == NULL) ||
        (request_length < 2U) || (request_length > 8U) ||
        (reply_length < 2U) || (reply_length > 8U))
    {
        return 7U;
    }

    memset(reply, 0, sizeof(*reply));

    /* 同一连续会话只初始化一次，避免每条请求反复重置CAN */
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U)
        {
            return result;
        }
        s_can1_normal_ready = 1U;
    }

    /* FIFO0最多容纳3帧，有限清理前一请求遗留的数据 */
    for (flush_count = 0U; flush_count < 3U; flush_count++)
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0U)
        {
            break;
        }
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
    }

    memset(&tx, 0, sizeof(tx));
    tx.ExtId = (uint32_t)address << 8;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = request_length;
    memcpy(tx.Data, request, request_length);

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
                (rx.RTR == CAN_RTR_Data) &&
                (rx.ExtId == tx.ExtId))
            {
                /* 保存原始回复，异常时也能查看 */
                *reply = rx;

                if ((rx.DLC != 0U) && (rx.Data[0] == request[0]))
                {
                    if ((rx.DLC == reply_length) &&
                        (rx.Data[reply_length - 1U] == 0x6BU))
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

/* 原查询接口保留：两字节请求仍为功能码和固定6B */
static uint32_t CAN1_ReadParameter(uint8_t address,
                                  uint8_t command,
                                  uint8_t reply_length,
                                  CanRxMsg *reply)
{
    uint8_t request[2] = {command, 0x6BU};
    return CAN1_ExchangeSingle(address, request, 2U, reply_length, reply);
}
```

新底层只把发送区从固定2字节改成传入的2～8字节，其余初始化/清FIFO/100ms发送等待/200ms接收等待/同地址同功能码和DLC/6B检查保持。原ReadParameter包装仍构造command/6B两个字节，现有四种读取无需改main调用。

can.c开头“所有CAN调用来自任务A”的旧注释可改成“所有CAN调用仅来自独立CAN任务，不允许其他任务并发使用”；不增加第二个CAN调用者，不改50ms任务延时、监督或喂狗条件。

## 4. can.c末尾追加控制函数

在现有CAN1_ReadPosition函数结束后追加：

```c
/* 单帧控制应答：本阶段只接受02，其他应答保留但返回8 */
static uint32_t CAN1_ControlSingle(uint8_t address,
                                   const uint8_t *request,
                                   uint8_t request_length,
                                   CanRxMsg *reply)
{
    uint32_t result;
    if ((address < 1U) || (address > 4U) || (reply == NULL))
    {
        return 7U;
    }
    result = CAN1_ExchangeSingle(address, request, request_length, 3U, reply);
    if ((result == 1U) && (reply->Data[1] != 0x02U))
    {
        /* E2/EE或未解释的12/22都不能当成已成功执行 */
        result = 8U;
    }
    if (result != 1U) CAN1_ResetSession();
    return result;
}

/* 使能状态0为松轴、1为锁轴；本接口同步位固定0，立即执行 */
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                              CanRxMsg *reply)
{
    uint8_t request[5] = {0xF3U, 0xABU, 0U, 0U, 0x6BU};
    if (enabled > 1U) return 7U;
    request[2] = enabled;
    return CAN1_ControlSingle(address, request, 5U, reply);
}

/* 立即停止：同步位必须为0，不能把停止命令只缓存起来 */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    uint8_t request[4] = {0xFEU, 0x98U, 0U, 0x6BU};
    return CAN1_ControlSingle(address, request, 4U, reply);
}

/* Emm速度控制：方向0为CW、1为CCW；速度高字节在前 */
/* 3000仅是本阶段协议输入上限，不是小车允许的安全转速 */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                              uint16_t speed_raw, uint8_t acceleration,
                              CanRxMsg *reply)
{
    uint8_t request[7] = {0xF6U, 0U, 0U, 0U, 0U, 0U, 0x6BU};
    if ((direction > 1U) || (speed_raw > 3000U)) return 7U;
    request[1] = direction;
    request[2] = (uint8_t)(speed_raw >> 8);
    request[3] = (uint8_t)speed_raw;
    request[4] = acceleration;
    /* 加速度0代表直接启动；实际测试参数须另行给定 */
    return CAN1_ControlSingle(address, request, 7U, reply);
}
```

控制接口地址限定1～4，拒绝0广播和其他地址；方向/使能值/速度原始值先检查，非法参数返回7且不发帧。返回值：1收到02，2时钟不符，3初始化失败，4发送失败，5未等到匹配回复，6回帧格式不符，7参数无效，8非02应答。8时reply用于诊断保留电机应答，不能作为有效动作确认。

这些函数有FreeRTOS等待，只能从App_CanTask串行调用。不要从按键中断、TaskA、Safety任务或Keil命令窗口直接调用。本轮main.c完全不改，因此没有新的动作触发入口。故障后关VM仍由原监督控制；这些底层接口本身不代替READY、限速、反馈失效停止和重新允许门禁。

## 5. 保存并Rebuild

只改can.c/can.h，保存后Rebuild并发送完整日志。本轮先到构建，不下载执行新的动作、不设任何新动作请求。新增接口未被main引用时，链接器可能移除未用函数，Code大小变化不能证明动作功能已经执行。

隔离副本已用本工程实际包含路径和ARMCC5.06分别编译can.c及当前main，无错误警告输出。还把同一份通用收发/原四个读取/新控制接口函数放入主机模拟CAN收发环境，检查帧内容、非法参数不发帧、非02应答、匹配过滤、超时/tick回绕和旧读取解码；结果见证据。测试是主机模拟，不访问板卡、不代替实际电机通信；助手没有修改实际工程、链接固件、烧录或发送CAN。

## 需求与未闭环

PWR-01（R，单内置电源保持）；LOC-03/RUN-06/MOT-01～04（E）。按用户手动修改、标准库＋原生FreeRTOS、原模块布局及中文注释，不新增空底盘阶段。原扫码/显示/独立采集、VM默认0及故障关断不删。Q项仍待技术答疑确认，无官方规则更改。

当前未闭环：用户正式构建；修改后实际读取回归/控制应答；四台速度输入缩放设置；READY/停止/重新允许及速度/位置失效门禁；运动所需更快时效和载物限值、轮向/几何/HWT/实时X/Y。后续先接立即停止与零速度验证，再给带界限和超时的低速动作入口；本步不签运动功能完成。
