#include "can.h"
#include "app_config.h"
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "app_config.h"

/* –¬ƒø±Í∫ΩœÚƒ£ Ω≤ª‘ –Ìµ•¥ŒΩª“◊≥§”⁄÷∏¡ÓÃ·Ωªº‡∂Ω∆⁄œﬁ°£ */
#if APP_HEADING_SESSION_TEST
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 30U
#else
#define CAN1_SINGLE_TX_WAIT_MS 100U
#define CAN1_SINGLE_REPLY_WAIT_MS 200U
#endif
/*  ◊¥Œøÿ÷∆Ωª“◊ ß∞‹øÏ’’£ªΩˆ∂¡»°”≤º˛◊¥Ã¨£¨≤ª∏ƒ÷°ªÚµ»¥˝∆⁄œﬁ°£ */
volatile uint32_t g_can_ctrl_fault_latched = 0U;
volatile uint32_t g_can_ctrl_trace_result = 0U, g_can_ctrl_trace_phase = 0U;
volatile uint32_t g_can_ctrl_trace_address = 0U, g_can_ctrl_trace_length = 0U;
volatile uint8_t g_can_ctrl_trace_request[8] = {0U};
volatile uint32_t g_can_ctrl_trace_pe2_start = 0U, g_can_ctrl_trace_pe2_end = 0U;
volatile uint32_t g_can_ctrl_trace_esr = 0U;
volatile uint32_t g_can_ctrl_trace_rx_count = 0U, g_can_ctrl_trace_flush_count = 0U;
volatile uint32_t g_can_ctrl_trace_tx_ms = 0U, g_can_ctrl_trace_total_ms = 0U;
volatile CanRxMsg g_can_ctrl_trace_last_rx = {0};
static uint32_t s_can_ctrl_trace_active = 0U;
static TickType_t s_can_ctrl_trace_tick = 0U;

/* µ±«∞À˘”–CANµ˜”√∂º¿¥◊‘»ŒŒÒA£¨≤ª‘ –Ì∆‰À˚»ŒŒÒ≤¢∑¢ π”√ */
static uint32_t s_can1_normal_ready = 0U;
volatile uint32_t g_can_lift_home_done_notice = 0U;
static uint32_t s_lift_home_capture = 0U;
static void CAN1_CaptureLiftHomeNotice(const CanRxMsg *rx)
{
    if (s_lift_home_capture && rx->IDE == CAN_Id_Extended && rx->RTR == CAN_RTR_Data &&
        rx->ExtId == 0x0500U && rx->DLC == 3U && rx->Data[0] == 0x9AU &&
        rx->Data[1] == 0x9FU && rx->Data[2] == 0x6BU)
        g_can_lift_home_done_notice = 1U;
}

/* ±Íº«œ¬“ª¥Œ≤È—Ø–Ë÷ÿ–¬≥ı ºªØ£¨≤ª‘⁄’‚¿Ô∑¢ÀÕªÚ«–ªª”≤º˛ƒ£ Ω */
void CAN1_ResetSession(void)
{
    s_can1_normal_ready = 0U;
}
/* ≥…π¶∑µªÿ1£¨ ±÷”≤ª∆•≈‰∑µªÿ2£¨≥ı ºªØ ß∞‹∑µªÿ3 */
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

    /* ≥…∆∑PCB£∫PD0Ω” ’£¨PD1∑¢ÀÕ */
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

    /* ”…µ˜”√’ﬂ—°‘Òªÿª∑ªÚ’˝≥£Õ®–≈ƒ£ Ω */
    can.CAN_Mode = mode;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_11tq;
    can.CAN_BS2 = CAN_BS2_2tq;

    /* 42MHz / 6 / (1 + 11 + 2) = 500kbit/s */
    can.CAN_Prescaler = 6U;
    can.CAN_NART = ENABLE;

    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success)
    {
        /* Ω¯»Î≥ı ºªØŒ¥≥…π¶£¨±£¡Ù ß∞‹ */
        if ((CAN1->MCR & CAN_MCR_INRQ) != 0U)
        {
            return 3U;
        }

        /* “—«Î«ÛÕÀ≥ˆ£¨µ»¥˝”≤º˛Õ¨≤ΩÕÍ≥…£¨◊Ó∂‡100∫¡√Î */
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
    /* π˝¬À∆˜0Ω” ’»´≤ø÷°£¨ÀÕ»ÎFIFO0 */
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

/* “ª¥ŒÕÍ’˚ªÿª∑≤‚ ‘£¨≤ªø™∆ÙVM£¨≤ª π”√µÁª˙–≠“È */
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

/* ÷ª∂¡∞Ê±æ£∫1≥…π¶£¨4∑¢ÀÕ ß∞‹£¨5Œﬁªÿ∏¥£¨6∏Ò Ω“Ï≥£ */
/* π≤”√∂Ã÷°≤È—Ø£¨π©∞Ê±æ∫Õ◊¥Ã¨∂¡»° π”√ */
/* Õ®”√µ•÷° ’∑¢£∫÷ª‘ –Ì∂¿¡¢CAN»ŒŒÒ¥Æ––µ˜”√ */
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
    /* «Î«Û∫Õªÿ∏¥æ˘œﬁ÷∆‘⁄µ•∏ˆCAN÷°ƒ⁄ */
    if ((address == 0U) || (reply == NULL) || (request == NULL) ||
        (request_length < 2U) || (request_length > 8U) ||
        (reply_length < 2U) || (reply_length > 8U))
    {
        return 7U;
    }

    memset(reply, 0, sizeof(*reply));

    /* Õ¨“ª¡¨–¯ª·ª∞÷ª≥ı ºªØ“ª¥Œ£¨±‹√‚√øÃı«Î«Û∑¥∏¥÷ÿ÷√CAN */
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U)
        {
            return result;
        }
        s_can1_normal_ready = 1U;
    }

    /* FIFO0◊Ó∂‡»›ƒ…3÷°£¨”–œﬁ«Â¿Ì«∞“ª«Î«Û“≈¡Ùµƒ ˝æ› */
    for (flush_count = 0U; flush_count < 3U; flush_count++)
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0U)
        {
            break;
        }
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
            CAN1_CaptureLiftHomeNotice(&rx);
        if (s_can_ctrl_trace_active) g_can_ctrl_trace_flush_count++;
    }

    if (address == 5U && request[0] == 0x9AU)
    {
        /* Clear old completion only after draining pre-command packets. */
        g_can_lift_home_done_notice = 0U;
        s_lift_home_capture = 1U;
    }
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = (uint32_t)address << 8;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    /* ∞¥ µº «Î«Û≥§∂»∏¥÷∆£¨∞¸¿®π¶ƒ‹¬Î∫Õƒ©Œ≤–£—È◊÷Ω⁄ */
    tx.DLC = request_length;
    memcpy(tx.Data, request, request_length);

    if (s_can_ctrl_trace_active) g_can_ctrl_trace_phase = 2U;
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
            if (s_can_ctrl_trace_active) g_can_ctrl_trace_tx_ms =
                (uint32_t)(xTaskGetTickCount() - s_can_ctrl_trace_tick);
            break;
        }

        if ((status == CAN_TxStatus_Failed) ||
            ((TickType_t)(xTaskGetTickCount() - start_tick) >=
             pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)))
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return 4U;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    start_tick = xTaskGetTickCount();

    if (s_can_ctrl_trace_active) g_can_ctrl_trace_phase = 3U;
    while ((TickType_t)(xTaskGetTickCount() - start_tick) <
           pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0) != 0U)
        {
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
            CAN1_CaptureLiftHomeNotice(&rx);
            if (s_can_ctrl_trace_active)
            {
                g_can_ctrl_trace_rx_count++;
                g_can_ctrl_trace_last_rx = rx;
            }

            if ((rx.IDE == CAN_Id_Extended) &&
                (rx.RTR == CAN_RTR_Data) &&
                (rx.ExtId == tx.ExtId))
            {
                /* ±£¥Ê‘≠ ºªÿ∏¥£¨“Ï≥£ ±“≤ƒ‹≤Èø¥ */
                *reply = rx;

                /* ªÿ∏¥π¶ƒ‹¬Î±ÿ–Î”Î±æ¥Œ«Î«Û“ª÷¬ */
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

/* ±£¡Ù‘≠≤È—Ø»Îø⁄£¨Àƒ÷÷∂¡»°∫Ø ˝ºÃ–¯ π”√‘≠¿¥µƒµ˜”√∑Ω Ω */
static uint32_t CAN1_ReadParameter(uint8_t address,
                                  uint8_t command,
                                  uint8_t reply_length,
                                  CanRxMsg *reply)
{
    uint8_t request[2] = {command, 0x6BU};

    return CAN1_ExchangeSingle(address, request, 2U,
                               reply_length, reply);
}

/* ∂¡»°∞Ê±æ£¨ ß∞‹∫Ûœ¬“ª¥Œ≤È—Ø÷ÿ–¬Ω®¡¢’˝≥£ƒ£ Ωª·ª∞ */
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x1FU, 6U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}

/* ∂¡»°◊¥Ã¨£¨≤ª∏ƒ±‰«˝∂Ø∆˜µƒ πƒ‹ªÚ‘À∂Ø◊¥Ã¨ */
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x3AU, 3U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}
/* Emm µ ±◊™ÀŸ÷ª∂¡£∫CANªÿ÷°35°¢∑˚∫≈°¢ÀŸ∂»∏ﬂµÕ◊÷Ω⁄°¢6B */
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
/* Emmµ±«∞Œª÷√÷ª∂¡£ª∑˚∫≈”Î32Œª∑˘÷µ∑÷ø™£¨±‹√‚int32“Á≥ˆ */
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
/* øÿ÷∆”¶¥Õ≥“ª¥¶¿Ì£∫÷ªΩ” ‹02£¨∆‰À˚”¶¥±£¡Ù≤¢∑µªÿ8 */
static uint32_t CAN1_ControlSingle(uint8_t address,
                                   const uint8_t *request,
                                   uint8_t request_length,
                                   CanRxMsg *reply)
{
    uint32_t result, trace, i;
    CanRxMsg empty = {0};

    /* ÷ª‘ –ÌÀƒ¬÷µÿ÷∑£¨≤ª‘ –Ìπ„≤• */
    if ((address < 1U) || (address > 4U) || (reply == NULL))
    {
        return 7U;
    }

    trace = ((g_can_ctrl_fault_latched == 0U) && (request != NULL) &&
        (request_length >= 2U) && (request_length <= 8U)) ? 1U : 0U;
    if (trace)
    {
        g_can_ctrl_trace_address = address;
        g_can_ctrl_trace_length = request_length;
        for (i = 0U; i < 8U; i++) g_can_ctrl_trace_request[i] =
            (i < request_length) ? request[i] : 0U;
        g_can_ctrl_trace_result = 0U; g_can_ctrl_trace_phase = 1U;
        g_can_ctrl_trace_rx_count = g_can_ctrl_trace_flush_count = 0U;
        g_can_ctrl_trace_tx_ms = g_can_ctrl_trace_total_ms = 0U;
        g_can_ctrl_trace_last_rx = empty;
        g_can_ctrl_trace_pe2_start = (GPIOE->ODR & GPIO_Pin_2) ? 1U : 0U;
        s_can_ctrl_trace_tick = xTaskGetTickCount();
        s_can_ctrl_trace_active = 1U;
    }
    result = CAN1_ExchangeSingle(address, request,
                                 request_length, 3U, reply);

    if ((result == 1U) && (reply->Data[1] != 0x02U))
    {
        /* ∑«02≤ªƒ‹◊˜Œ™≥…π¶»∑»œ£¨‘≠ ºªÿ∏¥»‘±£¡Ù */
        result = 8U;
    }

    if (trace)
    {
        g_can_ctrl_trace_total_ms = (uint32_t)(xTaskGetTickCount() - s_can_ctrl_trace_tick);
        g_can_ctrl_trace_pe2_end = (GPIOE->ODR & GPIO_Pin_2) ? 1U : 0U;
        g_can_ctrl_trace_esr = CAN1->ESR;
        g_can_ctrl_trace_result = result;
        s_can_ctrl_trace_active = 0U;
        if (result != 1U) g_can_ctrl_fault_latched = 1U;
    }
    if (result != 1U)
    {
        CAN1_ResetSession();
    }

    return result;
}

/*  πƒ‹øÿ÷∆£∫0À…÷·£¨1À¯÷·£ªÕ¨≤ΩŒªπÃ∂®0£¨¡¢º¥÷¥–– */
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

/* ¡¢º¥Õ£÷π£∫Õ¨≤ΩŒªπÃ∂®0£¨≤ªª∫¥ÊÕ£÷π√¸¡Ó */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    uint8_t request[4] = {0xFEU, 0x98U, 0U, 0x6BU};

    return CAN1_ControlSingle(address, request, 4U, reply);
}

/* Emm‘≠ ºÀŸ∂»øÿ÷∆£∫∑ΩœÚ0Œ™CW£¨1Œ™CCW */
/* CW/CCW”Î’˚≥µ«∞∫Û∑ΩœÚµƒ∂‘”¶πÿœµ»‘–Ë µ≤‚ */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                             uint16_t speed_raw, uint8_t acceleration,
                             CanRxMsg *reply)
{
    uint8_t request[7] = {
        0xF6U, 0U, 0U, 0U, 0U, 0U, 0x6BU
    };

    /* 3000 «–≠“È ‰»Î…œœﬁ£¨≤ª «’˚≥µ‘ –Ìµƒ∞≤»´ÀŸ∂» */
    if ((direction > 1U) || (speed_raw > 3000U))
    {
        return 7U;
    }

    request[1] = direction;
    request[2] = (uint8_t)(speed_raw >> 8);
    request[3] = (uint8_t)speed_raw;
    request[4] = acceleration;

    /* º”ÀŸ∂»0”–÷±Ω”∆Ù∂Ø∫¨“Â£¨ µº ‘À∂Ø≤Œ ˝∫Û–¯µ•∂¿»∑∂® */
    return CAN1_ControlSingle(address, request, 7U, reply);
}











/* ‰∏ì‰æõÈöîÁ¶ªÂçáÈôçÊ®°ÂºèÔºåÂõ∫ÂÆöËäÇÁÇπ5Ôºõ‰∏çÊîæÂºÄÂéüÂ∫ïÁõòÊéßÂà∂Êé•Âè£ÁöÑÂú∞ÂùÄ‰øùÊä§„ÄÇ */
uint32_t CAN1_LiftExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result, pulses, speed_rpm;
    uint8_t offset, packet, chunk, mailbox, status, flush;

    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    if (request == NULL || reply == NULL) return 7U;
    /* Âè™ÂÖÅËÆ∏ÈîÅËΩ¥„ÄÅÂÅúÊ≠¢„ÄÅÊúâÈôêÁõ∏ÂØπ‰ΩçÁΩÆÔºõÁ¶ÅÊ≠¢ÊùæËΩ¥„ÄÅÂõûÈõ∂ÂíåÂèÇÊï∞‰øÆÊîπ„ÄÇ */
    if (length == 5U && request[0] == 0xF3U && request[1] == 0xABU &&
        request[2] == 1U && request[3] == 0U && request[4] == 0x6BU)
    {
        result = CAN1_ExchangeSingle(5U, request, length, 3U, reply);
        return (result == 1U && reply->Data[1] != 0x02U) ? 8U : result;
    }
    if (length == 4U && request[0] == 0xFEU && request[1] == 0x98U &&
        request[2] == 0U && request[3] == 0x6BU)
    {
        result = CAN1_ExchangeSingle(5U, request, length, 3U, reply);
        return (result == 1U && reply->Data[1] != 0x02U) ? 8U : result;
    }
    /* RPM is a two-byte field; validate both bytes before sending. */
    speed_rpm = (length == 12U) ? (((uint32_t)request[2] << 8) | request[3]) : 0U;
    if (length != 12U || request[0] != 0xFDU || request[1] > 1U ||
        speed_rpm == 0U || speed_rpm > 3000U ||
        request[4] == 0U || request[9] != 2U || request[10] != 0U ||
        request[11] != 0x6BU) return 7U;
    pulses = ((uint32_t)request[5] << 24) | ((uint32_t)request[6] << 16) |
             ((uint32_t)request[7] << 8) | request[8];
    if (pulses == 0U || pulses > 5000U) return 7U;
    memset(reply, 0, sizeof(*reply));
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U) return result;
        s_can1_normal_ready = 1U;
    }
    for (flush = 0U; flush < 3U && CAN_MessagePending(CAN1, CAN_FIFO0); flush++)
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
    offset = 1U; packet = 0U;
    while (offset < length)
    {
        memset(&tx, 0, sizeof(tx));
        chunk = (uint8_t)(length - offset);
        if (chunk > 7U) chunk = 7U;
        tx.ExtId = 0x0500U | packet;
        tx.IDE = CAN_Id_Extended; tx.RTR = CAN_RTR_Data;
        tx.DLC = (uint8_t)(chunk + 1U); tx.Data[0] = request[0];
        memcpy(&tx.Data[1], &request[offset], chunk);
        mailbox = CAN_Transmit(CAN1, &tx);
        if (mailbox == CAN_TxStatus_NoMailBox) return 4U;
        started = xTaskGetTickCount();
        for (;;)
        {
            status = CAN_TransmitStatus(CAN1, mailbox);
            if (status == CAN_TxStatus_Ok) break;
            if (status == CAN_TxStatus_Failed ||
                (TickType_t)(xTaskGetTickCount() - started) >=
                pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS))
            {
                CAN_CancelTransmit(CAN1, mailbox);
                return 4U;
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
        offset = (uint8_t)(offset + chunk); packet++;
    }
    started = xTaskGetTickCount();
    while ((TickType_t)(xTaskGetTickCount() - started) <
           pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
            if (rx.IDE == CAN_Id_Extended && rx.RTR == CAN_RTR_Data &&
                rx.ExtId == 0x0500U && rx.DLC != 0U && rx.Data[0] == 0xFDU)
            {
                *reply = rx;
                if (rx.DLC != 3U || rx.Data[2] != 0x6BU) return 6U;
                /* Êé•Êî∂Á°ÆËÆ§ÊàñÂà∞‰ΩçÈÄöÁü•ÈÉΩ‰∏çËÉΩÊõø‰ª£ÂêéÁª≠ÁúüÂÆû‰ΩçÁΩÆ/ÈùôÊ≠¢ÂèçÈ¶à„ÄÇ */
                return (rx.Data[1] == 0x02U || rx.Data[1] == 0x9FU) ? 1U : 8U;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return 5U;
}

uint32_t CAN1_ReadMotorOptions(uint8_t address, CanRxMsg *reply)
{
    return CAN1_ReadParameter(address, 0x1AU, 4U, reply);
}

/* ÷ª∂¡»°5∫≈ªÿ¡„≤Œ ˝22£ª»˝÷°—œ∏Ò∞¥–Ú◊È∞¸£¨≤ª∑¢ÀÕ…Ë÷√ªÚªÿ¡„√¸¡Ó°£ */
uint32_t CAN1_ReadLiftHomeParameters(uint8_t out[17], CanRxMsg replies[3])
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result;
    uint8_t mailbox, status, flush, packet = 0U, offset = 1U, chunk;
    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    if (out == NULL || replies == NULL) return 7U;
    memset(out, 0, 17U);
    memset(replies, 0, 3U * sizeof(CanRxMsg));
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U) return result;
        s_can1_normal_ready = 1U;
    }
    for (flush = 0U; flush < 3U && CAN_MessagePending(CAN1, CAN_FIFO0); flush++)
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = 0x0500U;
    tx.IDE = CAN_Id_Extended; tx.RTR = CAN_RTR_Data;
    tx.DLC = 2U; tx.Data[0] = 0x22U; tx.Data[1] = 0x6BU;
    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox) return 4U;
    started = xTaskGetTickCount();
    for (;;)
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
        if (status == CAN_TxStatus_Ok) break;
        if (status == CAN_TxStatus_Failed ||
            (TickType_t)(xTaskGetTickCount() - started) >= pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS))
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return 4U;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    started = xTaskGetTickCount();
    out[0] = 0x22U;
    while ((TickType_t)(xTaskGetTickCount() - started) < pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
            if (rx.IDE == CAN_Id_Extended && rx.RTR == CAN_RTR_Data &&
                (rx.ExtId >> 8) == 5U && rx.DLC != 0U && rx.Data[0] == 0x22U)
            {
                replies[packet] = rx;
                chunk = (uint8_t)(17U - offset);
                if (chunk > 7U) chunk = 7U;
                if (rx.ExtId != (0x0500U | packet) || rx.DLC != (uint8_t)(chunk + 1U))
                    return 6U;
                memcpy(&out[offset], &rx.Data[1], chunk);
                offset = (uint8_t)(offset + chunk);
                if (offset == 17U) return out[16] == 0x6BU ? 1U : 6U;
                packet++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return 5U;
}

/* Ωˆ5∫≈ ÷∂Ø≈ˆ◊≤ªÿ¡„°¢ÕÀ≥ˆªÿ¡„∫Õ÷ª∂¡ªÿ¡„◊¥Ã¨£ª≤ª∏ƒ≤Œ ˝£¨≤ªÀ…÷·°£ */
uint32_t CAN1_StartLiftHome(CanRxMsg *reply)
{
    const uint8_t q[4] = {0x9AU, 2U, 0U, 0x6BU};
    uint32_t r;
    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    r = CAN1_ExchangeSingle(5U, q, 4U, 3U, reply);
    return (r == 1U && reply->Data[1] != 2U && reply->Data[1] != 0x9FU) ? 8U : r;
}
uint32_t CAN1_AbortLiftHome(CanRxMsg *reply)
{
    const uint8_t q[3] = {0x9CU, 0x48U, 0x6BU};
    uint32_t r;
    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    r = CAN1_ExchangeSingle(5U, q, 3U, 3U, reply);
    s_lift_home_capture = 0U;
    return (r == 1U && reply->Data[1] != 2U) ? 8U : r;
}
uint32_t CAN1_ReadLiftHomeStatus(CanRxMsg *reply)
{
    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    return CAN1_ReadParameter(5U, 0x3BU, 3U, reply);
}
