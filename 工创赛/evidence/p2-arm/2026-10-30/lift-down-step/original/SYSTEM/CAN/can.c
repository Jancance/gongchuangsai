#include "can.h"
#include "extension_config.h"
#include "app_config.h"
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "app_config.h"

/* ĞÂÄ¿±êº½ÏòÄ£Ê½²»ÔÊĞíµ¥´Î½»Ò×³¤ÓÚÖ¸ÁîÌá½»¼à¶½ÆÚÏŞ¡£ */
#if APP_HEADING_SESSION_TEST
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 30U
#else
#define CAN1_SINGLE_TX_WAIT_MS 100U
#define CAN1_SINGLE_REPLY_WAIT_MS 200U
#endif
/* Ê×´Î¿ØÖÆ½»Ò×Ê§°Ü¿ìÕÕ£»½ö¶ÁÈ¡Ó²¼ş×´Ì¬£¬²»¸ÄÖ¡»òµÈ´ıÆÚÏŞ¡£ */
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

/* µ±Ç°ËùÓĞCANµ÷ÓÃ¶¼À´×ÔÈÎÎñA£¬²»ÔÊĞíÆäËûÈÎÎñ²¢·¢Ê¹ÓÃ */
static uint32_t s_can1_normal_ready = 0U;
volatile uint32_t g_can_lift_home_done_notice = 0U;
static uint32_t s_lift_home_capture = 0U;
volatile uint32_t g_can_extension_home_done_notice = 0U;
static uint32_t s_extension_home_capture = 0U;
static void CAN1_CaptureLiftHomeNotice(const CanRxMsg *rx)
{
    if (s_lift_home_capture && rx->IDE == CAN_Id_Extended && rx->RTR == CAN_RTR_Data &&
        rx->ExtId == 0x0500U && rx->DLC == 3U && rx->Data[0] == 0x9AU &&
        rx->Data[1] == 0x9FU && rx->Data[2] == 0x6BU)
        g_can_lift_home_done_notice = 1U;
    if (s_extension_home_capture && rx->IDE == CAN_Id_Extended && rx->RTR == CAN_RTR_Data &&
        rx->ExtId == 0x0600U && rx->DLC == 3U && rx->Data[0] == 0x9AU &&
        rx->Data[1] == 0x9FU && rx->Data[2] == 0x6BU)
        g_can_extension_home_done_notice = 1U;
}

/* ±ê¼ÇÏÂÒ»´Î²éÑ¯ĞèÖØĞÂ³õÊ¼»¯£¬²»ÔÚÕâÀï·¢ËÍ»òÇĞ»»Ó²¼şÄ£Ê½ */
void CAN1_ResetSession(void)
{
    s_can1_normal_ready = 0U;
}
/* ³É¹¦·µ»Ø1£¬Ê±ÖÓ²»Æ¥Åä·µ»Ø2£¬³õÊ¼»¯Ê§°Ü·µ»Ø3 */
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

    /* ³ÉÆ·PCB£ºPD0½ÓÊÕ£¬PD1·¢ËÍ */
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

    /* ÓÉµ÷ÓÃÕßÑ¡Ôñ»Ø»·»òÕı³£Í¨ĞÅÄ£Ê½ */
    can.CAN_Mode = mode;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_11tq;
    can.CAN_BS2 = CAN_BS2_2tq;

    /* 42MHz / 6 / (1 + 11 + 2) = 500kbit/s */
    can.CAN_Prescaler = 6U;
    can.CAN_NART = ENABLE;

    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success)
    {
        /* ½øÈë³õÊ¼»¯Î´³É¹¦£¬±£ÁôÊ§°Ü */
        if ((CAN1->MCR & CAN_MCR_INRQ) != 0U)
        {
            return 3U;
        }

        /* ÒÑÇëÇóÍË³ö£¬µÈ´ıÓ²¼şÍ¬²½Íê³É£¬×î¶à100ºÁÃë */
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
    /* ¹ıÂËÆ÷0½ÓÊÕÈ«²¿Ö¡£¬ËÍÈëFIFO0 */
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

/* Ò»´ÎÍêÕû»Ø»·²âÊÔ£¬²»¿ªÆôVM£¬²»Ê¹ÓÃµç»úĞ­Òé */
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

/* Ö»¶Á°æ±¾£º1³É¹¦£¬4·¢ËÍÊ§°Ü£¬5ÎŞ»Ø¸´£¬6¸ñÊ½Òì³£ */
/* ¹²ÓÃ¶ÌÖ¡²éÑ¯£¬¹©°æ±¾ºÍ×´Ì¬¶ÁÈ¡Ê¹ÓÃ */
/* Í¨ÓÃµ¥Ö¡ÊÕ·¢£ºÖ»ÔÊĞí¶ÀÁ¢CANÈÎÎñ´®ĞĞµ÷ÓÃ */
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
    /* ÇëÇóºÍ»Ø¸´¾ùÏŞÖÆÔÚµ¥¸öCANÖ¡ÄÚ */
    if ((address == 0U) || (reply == NULL) || (request == NULL) ||
        (request_length < 2U) || (request_length > 8U) ||
        (reply_length < 2U) || (reply_length > 8U))
    {
        return 7U;
    }

    memset(reply, 0, sizeof(*reply));

    /* Í¬Ò»Á¬Ğø»á»°Ö»³õÊ¼»¯Ò»´Î£¬±ÜÃâÃ¿ÌõÇëÇó·´¸´ÖØÖÃCAN */
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U)
        {
            return result;
        }
        s_can1_normal_ready = 1U;
    }

    /* FIFO0×î¶àÈİÄÉ3Ö¡£¬ÓĞÏŞÇåÀíÇ°Ò»ÇëÇóÒÅÁôµÄÊı¾İ */
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
    /* °´Êµ¼ÊÇëÇó³¤¶È¸´ÖÆ£¬°üÀ¨¹¦ÄÜÂëºÍÄ©Î²Ğ£Ñé×Ö½Ú */
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
                /* ±£´æÔ­Ê¼»Ø¸´£¬Òì³£Ê±Ò²ÄÜ²é¿´ */
                *reply = rx;

                /* »Ø¸´¹¦ÄÜÂë±ØĞëÓë±¾´ÎÇëÇóÒ»ÖÂ */
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

/* ±£ÁôÔ­²éÑ¯Èë¿Ú£¬ËÄÖÖ¶ÁÈ¡º¯Êı¼ÌĞøÊ¹ÓÃÔ­À´µÄµ÷ÓÃ·½Ê½ */
static uint32_t CAN1_ReadParameter(uint8_t address,
                                  uint8_t command,
                                  uint8_t reply_length,
                                  CanRxMsg *reply)
{
    uint8_t request[2] = {command, 0x6BU};

    return CAN1_ExchangeSingle(address, request, 2U,
                               reply_length, reply);
}

/* ¶ÁÈ¡°æ±¾£¬Ê§°ÜºóÏÂÒ»´Î²éÑ¯ÖØĞÂ½¨Á¢Õı³£Ä£Ê½»á»° */
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x1FU, 6U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}

/* ¶ÁÈ¡×´Ì¬£¬²»¸Ä±äÇı¶¯Æ÷µÄÊ¹ÄÜ»òÔË¶¯×´Ì¬ */
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x3AU, 3U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}
/* EmmÊµÊ±×ªËÙÖ»¶Á£ºCAN»ØÖ¡35¡¢·ûºÅ¡¢ËÙ¶È¸ßµÍ×Ö½Ú¡¢6B */
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
/* Emmµ±Ç°Î»ÖÃÖ»¶Á£»·ûºÅÓë32Î»·ùÖµ·Ö¿ª£¬±ÜÃâint32Òç³ö */
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
/* ¿ØÖÆÓ¦´ğÍ³Ò»´¦Àí£ºÖ»½ÓÊÜ02£¬ÆäËûÓ¦´ğ±£Áô²¢·µ»Ø8 */
static uint32_t CAN1_ControlSingle(uint8_t address,
                                   const uint8_t *request,
                                   uint8_t request_length,
                                   CanRxMsg *reply)
{
    uint32_t result, trace, i;
    CanRxMsg empty = {0};

    /* Ö»ÔÊĞíËÄÂÖµØÖ·£¬²»ÔÊĞí¹ã²¥ */
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
        /* ·Ç02²»ÄÜ×÷Îª³É¹¦È·ÈÏ£¬Ô­Ê¼»Ø¸´ÈÔ±£Áô */
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

/* Ê¹ÄÜ¿ØÖÆ£º0ËÉÖá£¬1ËøÖá£»Í¬²½Î»¹Ì¶¨0£¬Á¢¼´Ö´ĞĞ */
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

/* Á¢¼´Í£Ö¹£ºÍ¬²½Î»¹Ì¶¨0£¬²»»º´æÍ£Ö¹ÃüÁî */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply)
{
    uint8_t request[4] = {0xFEU, 0x98U, 0U, 0x6BU};

    return CAN1_ControlSingle(address, request, 4U, reply);
}

/* EmmÔ­Ê¼ËÙ¶È¿ØÖÆ£º·½Ïò0ÎªCW£¬1ÎªCCW */
/* CW/CCWÓëÕû³µÇ°ºó·½ÏòµÄ¶ÔÓ¦¹ØÏµÈÔĞèÊµ²â */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                             uint16_t speed_raw, uint8_t acceleration,
                             CanRxMsg *reply)
{
    uint8_t request[7] = {
        0xF6U, 0U, 0U, 0U, 0U, 0U, 0x6BU
    };

    /* 3000ÊÇĞ­ÒéÊäÈëÉÏÏŞ£¬²»ÊÇÕû³µÔÊĞíµÄ°²È«ËÙ¶È */
    if ((direction > 1U) || (speed_raw > 3000U))
    {
        return 7U;
    }

    request[1] = direction;
    request[2] = (uint8_t)(speed_raw >> 8);
    request[3] = (uint8_t)speed_raw;
    request[4] = acceleration;

    /* ¼ÓËÙ¶È0ÓĞÖ±½ÓÆô¶¯º¬Òå£¬Êµ¼ÊÔË¶¯²ÎÊıºóĞøµ¥¶ÀÈ·¶¨ */
    return CAN1_ControlSingle(address, request, 7U, reply);
}











/* ä¸“ä¾›éš”ç¦»å‡é™æ¨¡å¼ï¼Œå›ºå®šèŠ‚ç‚¹5ï¼›ä¸æ”¾å¼€åŸåº•ç›˜æ§åˆ¶æ¥å£çš„åœ°å€ä¿æŠ¤ã€‚ */
uint32_t CAN1_LiftExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result, pulses, speed_rpm;
    uint8_t offset, packet, chunk, mailbox, status, flush;

    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    if (request == NULL || reply == NULL) return 7U;
    /* åªå…è®¸é”è½´ã€åœæ­¢ã€æœ‰é™ç›¸å¯¹ä½ç½®ï¼›ç¦æ­¢æ¾è½´ã€å›é›¶å’Œå‚æ•°ä¿®æ”¹ã€‚ */
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
    if (pulses == 0U || pulses > (request[1] == 1U ? 233333U : 5000U)) return 7U;
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
                /* æ¥æ”¶ç¡®è®¤æˆ–åˆ°ä½é€šçŸ¥éƒ½ä¸èƒ½æ›¿ä»£åç»­çœŸå®ä½ç½®/é™æ­¢åé¦ˆã€‚ */
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

/* Ö»¶ÁÈ¡5ºÅ»ØÁã²ÎÊı22£»ÈıÖ¡ÑÏ¸ñ°´Ğò×é°ü£¬²»·¢ËÍÉèÖÃ»ò»ØÁãÃüÁî¡£ */
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

/* ½ö5ºÅÊÖ¶¯Åö×²»ØÁã¡¢ÍË³ö»ØÁãºÍÖ»¶Á»ØÁã×´Ì¬£»²»¸Ä²ÎÊı£¬²»ËÉÖá¡£ */
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

/* èŠ‚ç‚¹6ç‹¬ç«‹å°æ­¥ä¼ è¾“ï¼›ä¸æ‰©å¤§åŸåº•ç›˜/å‡é™è¿åŠ¨æ¥å£çš„åœ°å€æˆ–é€Ÿåº¦èŒƒå›´ã€‚ */
uint32_t CAN1_ExtensionExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result, pulses, speed_rpm;
    uint8_t offset, packet, chunk, mailbox, status, flush;

    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    if (request == NULL || reply == NULL) return 7U;
    /* åªå…è®¸é”è½´ã€åœæ­¢ã€æœ‰é™ç›¸å¯¹ä½ç½®ï¼›ç¦æ­¢æ¾è½´ã€å›é›¶å’Œå‚æ•°ä¿®æ”¹ã€‚ */
    if (length == 5U && request[0] == 0xF3U && request[1] == 0xABU &&
        request[2] == 1U && request[3] == 0U && request[4] == 0x6BU)
    {
        result = CAN1_ExchangeSingle(6U, request, length, 3U, reply);
        return (result == 1U && reply->Data[1] != 0x02U) ? 8U : result;
    }
    if (length == 4U && request[0] == 0xFEU && request[1] == 0x98U &&
        request[2] == 0U && request[3] == 0x6BU)
    {
        result = CAN1_ExchangeSingle(6U, request, length, 3U, reply);
        return (result == 1U && reply->Data[1] != 0x02U) ? 8U : result;
    }
    /* RPM is a two-byte field; validate both bytes before sending. */
    speed_rpm = (length == 12U) ? (((uint32_t)request[2] << 8) | request[3]) : 0U;
    if (length != 12U || request[0] != 0xFDU || request[1] > 1U ||
        speed_rpm == 0U || speed_rpm > EXTENSION_JOG_RPM ||
        (request[4] == 0U || request[4] > EXTENSION_JOG_ACCELERATION) || request[9] != 2U || request[10] != 0U ||
        request[11] != 0x6BU) return 7U;
    pulses = ((uint32_t)request[5] << 24) | ((uint32_t)request[6] << 16) |
             ((uint32_t)request[7] << 8) | request[8];
    if (pulses == 0U || pulses > EXTENSION_JOG_PULSES) return 7U;
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
        tx.ExtId = 0x0600U | packet;
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
                rx.ExtId == 0x0600U && rx.DLC != 0U && rx.Data[0] == 0xFDU)
            {
                *reply = rx;
                if (rx.DLC != 3U || rx.Data[2] != 0x6BU) return 6U;
                /* æ¥æ”¶ç¡®è®¤æˆ–åˆ°ä½é€šçŸ¥éƒ½ä¸èƒ½æ›¿ä»£åç»­çœŸå®ä½ç½®/é™æ­¢åé¦ˆã€‚ */
                return (rx.Data[1] == 0x02U || rx.Data[1] == 0x9FU) ? 1U : 8U;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return 5U;
}

/* åªè¯»6å·å›é›¶å‚æ•°22ï¼Œä¸¥æ ¼æ‹¼æ¥0x600/601/602ï¼Œä¸æä¾›å›é›¶æˆ–å†™å‚æ•°ã€‚ */
uint32_t CAN1_ReadExtensionHomeParameters(uint8_t out[17], CanRxMsg replies[3])
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
    tx.ExtId = 0x0600U;
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
                (rx.ExtId >> 8) == 6U && rx.DLC != 0U && rx.Data[0] == 0x22U)
            {
                replies[packet] = rx;
                chunk = (uint8_t)(17U - offset);
                if (chunk > 7U) chunk = 7U;
                if (rx.ExtId != (0x0600U | packet) || rx.DLC != (uint8_t)(chunk + 1U))
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

/* èŠ‚ç‚¹6å•æ¬¡02æ— é™ä½ç¢°æ’å›é›¶ï¼›ä¸æ›´æ”¹èœå•ã€é˜ˆå€¼æˆ–ä¸Šç”µè‡ªåŠ¨å›é›¶ã€‚ */
uint32_t CAN1_StartExtensionHome(CanRxMsg *reply)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t started;
    uint32_t result;
    uint8_t mailbox, status, flush;
    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    if (reply == NULL) return 7U;
    s_extension_home_capture = 0U;
    g_can_extension_home_done_notice = 0U;
    memset(reply, 0, sizeof(*reply));
    if (s_can1_normal_ready == 0U)
    {
        result = CAN1_InitMode(CAN_Mode_Normal);
        if (result != 1U) return result;
        s_can1_normal_ready = 1U;
    }
    /* å¯åŠ¨å‰æ’ç©ºçš„æ—§é€šçŸ¥ä¸ç®—æœ¬æ¬¡å®Œæˆï¼›åŸ5å·é€šçŸ¥æ•è·ä»ä¿ç•™ã€‚ */
    for (flush = 0U; flush < 3U && CAN_MessagePending(CAN1, CAN_FIFO0); flush++)
    {
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
        CAN1_CaptureLiftHomeNotice(&rx);
    }
    memset(&tx, 0, sizeof(tx));
    tx.ExtId = 0x0600U; tx.IDE = CAN_Id_Extended; tx.RTR = CAN_RTR_Data;
    tx.DLC = 4U; tx.Data[0] = 0x9AU; tx.Data[1] = 2U;
    tx.Data[2] = 0U; tx.Data[3] = 0x6BU;
    s_extension_home_capture = 1U;
    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox) return 4U;
    started = xTaskGetTickCount();
    for (;;)
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
        if (status == CAN_TxStatus_Ok) break;
        if (status == CAN_TxStatus_Failed ||
            (TickType_t)(xTaskGetTickCount() - started) >= pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS))
        { CAN_CancelTransmit(CAN1, mailbox); return 4U; }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    started = xTaskGetTickCount();
    while ((TickType_t)(xTaskGetTickCount() - started) < pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS))
    {
        if (CAN_MessagePending(CAN1, CAN_FIFO0))
        {
            CAN_Receive(CAN1, CAN_FIFO0, &rx);
            CAN1_CaptureLiftHomeNotice(&rx);
            if (rx.IDE == CAN_Id_Extended && rx.RTR == CAN_RTR_Data &&
                rx.ExtId == 0x0600U && rx.DLC != 0U && rx.Data[0] == 0x9AU)
            {
                *reply = rx;
                if (rx.DLC != 3U || rx.Data[2] != 0x6BU) return 6U;
                return (rx.Data[1] == 2U || rx.Data[1] == 0x9FU) ? 1U : 8U;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return 5U;
}

uint32_t CAN1_AbortExtensionHome(CanRxMsg *reply)
{
    const uint8_t q[3] = {0x9CU, 0x48U, 0x6BU};
    uint32_t result;
    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    result = CAN1_ExchangeSingle(6U, q, 3U, 3U, reply);
    return (result == 1U && reply->Data[1] != 2U) ? 8U : result;
}

uint32_t CAN1_ReadExtensionHomeStatus(CanRxMsg *reply)
{
    if (APP_ARM_LIFT_TEST != 1U) return 9U;
    return CAN1_ReadParameter(6U, 0x3BU, 3U, reply);
}

void CAN1_ExtensionHomeSessionEnd(void)
{
    s_extension_home_capture = 0U;
}
