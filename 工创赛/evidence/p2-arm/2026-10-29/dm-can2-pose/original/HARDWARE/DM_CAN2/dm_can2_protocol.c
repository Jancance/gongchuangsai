#include "dm_can2.h"
#include <string.h>

uint32_t DM_CAN2_BuildRead(uint8_t rid, CanTxMsg *tx)
{
    if (tx == 0 || (rid != 0x08U && rid != 0x07U && rid != 0x0AU))
        return 0U;
    memset(tx, 0, sizeof(*tx));
    tx->StdId = DM_CAN2_PARAMETER_ID;
    tx->IDE = CAN_Id_Standard;
    tx->RTR = CAN_RTR_Data;
    tx->DLC = 8U;
    tx->Data[0] = DM_CAN2_NODE_ID;
    tx->Data[1] = 0U;
    tx->Data[2] = 0x33U;
    tx->Data[3] = rid;
    return 1U;
}

uint32_t DM_CAN2_ParseRead(const CanRxMsg *rx, uint8_t rid, uint32_t *value)
{
    if (rx == 0 || value == 0 ||
        (rid != 0x08U && rid != 0x07U && rid != 0x0AU) ||
        rx->IDE != CAN_Id_Standard || rx->RTR != CAN_RTR_Data ||
        rx->StdId != DM_CAN2_MASTER_ID || rx->DLC != 8U ||
        rx->Data[0] != DM_CAN2_NODE_ID || rx->Data[1] != 0U ||
        rx->Data[2] != 0x33U || rx->Data[3] != rid)
        return 0U;
    /* 参数是小端uint32；反馈ID0有效，不能用“ID非0”判断成功。 */
    *value = (uint32_t)rx->Data[4] | ((uint32_t)rx->Data[5] << 8) |
             ((uint32_t)rx->Data[6] << 16) | ((uint32_t)rx->Data[7] << 24);
    return 1U;
}
