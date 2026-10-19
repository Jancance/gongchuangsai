#include "can.h"
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* 当前所有CAN调用都来自任务A，不允许其他任务并发使用 */
static uint32_t s_can1_normal_ready = 0U;
/* 标记下一次查询需重新初始化，不在这里发送或切换硬件模式 */
void CAN1_ResetSession(void)
{
    s_can1_normal_ready = 0U;
}
/* 成功返回1，时钟不匹配返回2，初始化失败返回3 */
static uint32_t CAN1_InitMode(uint8_t mode)
{
    TickType_t init_start_tick;
    RCC_ClocksTypeDef clocks;
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    RCC_GetClocksFreq(&clocks);
    if (clocks.PCLK1_Frequency != 42000000U)
    {
        return 2U;
    }

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    CAN_DeInit(CAN1);

    /* 成品PCB：PD0接收，PD1发送 */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &gpio);

    CAN_StructInit(&can);

    /* 由调用者选择回环或正常通信模式 */
    can.CAN_Mode = mode;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_11tq;
    can.CAN_BS2 = CAN_BS2_2tq;

    /* 42MHz / 6 / (1 + 11 + 2) = 500kbit/s */
    can.CAN_Prescaler = 6U;
    can.CAN_NART = ENABLE;

    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success)
    {
        /* 进入初始化未成功，保留失败 */
        if ((CAN1->MCR & CAN_MCR_INRQ) != 0U)
        {
            return 3U;
        }

        /* 已请求退出，等待硬件同步完成，最多100毫秒 */
        init_start_tick = xTaskGetTickCount();

        while ((CAN1->MSR & CAN_MSR_INAK) != 0U)
        {
            if ((TickType_t)(xTaskGetTickCount() - init_start_tick) >=
                pdMS_TO_TICKS(100U))
            {
                return 3U;
            }

            vTaskDelay(pdMS_TO_TICKS(1U));
        }
    }
    /* 过滤器0接收全部帧，送入FIFO0 */
    filter.CAN_FilterNumber = 0U;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0U;
    filter.CAN_FilterIdLow = 0U;
    filter.CAN_FilterMaskIdHigh = 0U;
    filter.CAN_FilterMaskIdLow = 0U;
    filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    return 1U;
}

/* 一次完整回环测试，不开启VM，不使用电机协议 */
uint32_t CAN1_LoopbackTest(void)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t start_tick;
    uint32_t result;
    uint8_t mailbox;
    uint8_t status;
    uint8_t i;
    
    CAN1_ResetSession();
    result = CAN1_InitMode(CAN_Mode_Silent_LoopBack);
    if (result != 1U)
    {
        return result;
    }

    tx.StdId = 0U;
    tx.ExtId = 0x12345U;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = 8U;

    for (i = 0U; i < 8U; i++)
    {
        tx.Data[i] = (uint8_t)(0x10U + i);
    }

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

    while (CAN_MessagePending(CAN1, CAN_FIFO0) == 0U)
    {
        if ((TickType_t)(xTaskGetTickCount() - start_tick) >=
            pdMS_TO_TICKS(100U))
        {
            return 5U;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    CAN_Receive(CAN1, CAN_FIFO0, &rx);

    if ((rx.IDE != CAN_Id_Extended) ||
        (rx.RTR != CAN_RTR_Data) ||
        (rx.ExtId != tx.ExtId) ||
        (rx.DLC != tx.DLC))
    {
        return 6U;
    }

    for (i = 0U; i < 8U; i++)
    {
        if (rx.Data[i] != tx.Data[i])
        {
            return 6U;
        }
    }

    return 1U;
}

/* 只读版本：1成功，4发送失败，5无回复，6格式异常 */
/* 共用短帧查询，供版本和状态读取使用 */
/* 通用单帧收发：只允许独立CAN任务串行调用 */
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
    /* 请求和回复均限制在单个CAN帧内 */
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
    /* 按实际请求长度复制，包括功能码和末尾校验字节 */
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

                /* 回复功能码必须与本次请求一致 */
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
/* 保留原查询入口，四种读取函数继续使用原来的调用方式 */
static uint32_t CAN1_ReadParameter(uint8_t address,
                                  uint8_t command,
                                  uint8_t reply_length,
                                  CanRxMsg *reply)
{
    uint8_t request[2] = {command, 0x6BU};

    return CAN1_ExchangeSingle(address, request, 2U,
                               reply_length, reply);
}

/* 读取版本，失败后下一次查询重新建立正常模式会话 */
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x1FU, 6U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}

/* 读取状态，不改变驱动器的使能或运动状态 */
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x3AU, 3U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}
/* Emm实时转速只读：CAN回帧35、符号、速度高低字节、6B */
uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, int32_t *rpm)
{
    uint32_t result;
    uint32_t magnitude;
    if (rpm == NULL) return 7U;
    *rpm = 0;
    result = CAN1_ReadParameter(address, 0x35U, 5U, reply);
    if (result == 1U)
    {
        if (reply->Data[1] > 1U)
        {
            result = 6U;
        }
        else
        {
            magnitude = ((uint32_t)reply->Data[2] << 8) |
                        reply->Data[3];
            *rpm = (reply->Data[1] == 0U) ?
                   (int32_t)magnitude : -(int32_t)magnitude;
        }
    }
    if (result != 1U) CAN1_ResetSession();
    return result;
}
/* Emm当前位置只读；符号与32位幅值分开，避免int32溢出 */
uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply,
                           uint8_t *sign, uint32_t *raw)
{
    uint32_t result;
    if (sign == NULL || raw == NULL) return 7U;
    *sign = 0U;
    *raw = 0U;
    result = CAN1_ReadParameter(address, 0x36U, 7U, reply);
    if (result == 1U)
    {
        if (reply->Data[1] > 1U)
        {
            result = 6U;
        }
        else
        {
            *sign = reply->Data[1];
            *raw = ((uint32_t)reply->Data[2] << 24) |
                   ((uint32_t)reply->Data[3] << 16) |
                   ((uint32_t)reply->Data[4] << 8) |
                   (uint32_t)reply->Data[5];
        }
    }
    if (result != 1U) CAN1_ResetSession();
    return result;
}
/* 控制应答统一处理：只接受02，其他应答保留并返回8 */
static uint32_t CAN1_ControlSingle(uint8_t address,
                                   const uint8_t *request,
                                   uint8_t request_length,
                                   CanRxMsg *reply)
{
    uint32_t result;

    /* 只允许四轮地址，不允许广播 */
    if ((address < 1U) || (address > 4U) || (reply == NULL))
    {
        return 7U;
    }

    result = CAN1_ExchangeSingle(address, request,
                                 request_length, 3U, reply);

    if ((result == 1U) && (reply->Data[1] != 0x02U))
    {
        /* 非02不能作为成功确认，原始回复仍保留 */
        result = 8U;
    }

    if (result != 1U)
    {
        CAN1_ResetSession();
    }

    return result;
}

/* 使能控制：0松轴，1锁轴；同步位固定0，立即执行 */
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                             CanRxMsg *reply)
{
    uint8_t request[5] = {0xF3U, 0xABU, 0U, 0U, 0x6BU};

    if (enabled > 1U)
    {
        return 7U;
    }

    request[2] = enabled;

    return CAN1_ControlSingle(address, request, 5U, reply);
}

/* 立即停止：同步位固定0，不缓存停止命令 */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    uint8_t request[4] = {0xFEU, 0x98U, 0U, 0x6BU};

    return CAN1_ControlSingle(address, request, 4U, reply);
}

/* Emm原始速度控制：方向0为CW，1为CCW */
/* CW/CCW与整车前后方向的对应关系仍需实测 */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                             uint16_t speed_raw, uint8_t acceleration,
                             CanRxMsg *reply)
{
    uint8_t request[7] = {
        0xF6U, 0U, 0U, 0U, 0U, 0U, 0x6BU
    };

    /* 3000是协议输入上限，不是整车允许的安全速度 */
    if ((direction > 1U) || (speed_raw > 3000U))
    {
        return 7U;
    }

    request[1] = direction;
    request[2] = (uint8_t)(speed_raw >> 8);
    request[3] = (uint8_t)speed_raw;
    request[4] = acceleration;

    /* 加速度0有直接启动含义，实际运动参数后续单独确定 */
    return CAN1_ControlSingle(address, request, 7U, reply);
}










