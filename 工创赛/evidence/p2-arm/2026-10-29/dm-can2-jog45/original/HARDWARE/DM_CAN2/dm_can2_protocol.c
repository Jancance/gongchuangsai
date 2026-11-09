#include "dm_can2.h"
#include <string.h>

static uint32_t DM_CAN2_ReadRid(uint8_t rid)
{
    return (rid == 0x08U || rid == 0x07U || rid == 0x0AU ||
            rid == 0x15U || rid == 0x16U || rid == 0x17U) ? 1U : 0U;
}

uint32_t DM_CAN2_BuildRead(uint8_t rid, CanTxMsg *tx)
{
    if (tx == 0 || !DM_CAN2_ReadRid(rid))
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
        !DM_CAN2_ReadRid(rid) ||
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

uint32_t DM_CAN2_BuildStatus(CanTxMsg *tx)
{
    if (tx == 0) return 0U;
    memset(tx, 0, sizeof(*tx));
    tx->StdId = DM_CAN2_PARAMETER_ID;
    tx->IDE = CAN_Id_Standard;
    tx->RTR = CAN_RTR_Data;
    tx->DLC = 8U;
    tx->Data[0] = DM_CAN2_NODE_ID;
    /* 厂家refresh_motor_status使用7FF/ID低/ID高/CC，不是使能指令。 */
    tx->Data[2] = 0xCCU;
    return 1U;
}

uint32_t DM_CAN2_DecodeRange(uint32_t bits, float *value)
{
    float decoded;
    if (value == 0 || (bits & 0x7F800000UL) == 0x7F800000UL) return 0U;
    memcpy(&decoded, &bits, sizeof(decoded));
    /* 仅是工程数据合理性门限，不当成厂家额定范围或运动限位。 */
    if (!(decoded > 0.0F && decoded <= 1000000.0F)) return 0U;
    *value = decoded;
    return 1U;
}

uint32_t DM_CAN2_ParseStatus(const CanRxMsg *rx, float pmax, float vmax,
                            float tmax, DM_CAN2_Feedback *feedback)
{
    if (rx == 0 || feedback == 0 || rx->IDE != CAN_Id_Standard ||
        rx->RTR != CAN_RTR_Data || rx->StdId != DM_CAN2_MASTER_ID ||
        rx->DLC != 8U || (rx->Data[0] & 0x0FU) != DM_CAN2_NODE_ID ||
        !(pmax > 0.0F && pmax <= 1000000.0F) ||
        !(vmax > 0.0F && vmax <= 1000000.0F) ||
        !(tmax > 0.0F && tmax <= 1000000.0F)) return 0U;
    /* 迟到参数帧不能被解释成状态/角度。信息有歧义时丢弃，等待新状态。 */
    if (rx->Data[1] == 0U &&
        (rx->Data[2] == 0x33U || rx->Data[2] == 0x55U) &&
        DM_CAN2_ReadRid(rx->Data[3])) return 0U;
    feedback->motor_state = (rx->Data[0] >> 4) & 0x0FU;
    feedback->position_raw = ((uint32_t)rx->Data[1] << 8) | rx->Data[2];
    feedback->velocity_raw = ((uint32_t)rx->Data[3] << 4) | (rx->Data[4] >> 4);
    feedback->torque_raw = ((uint32_t)(rx->Data[4] & 0x0FU) << 8) | rx->Data[5];
    feedback->position_rad = ((float)feedback->position_raw * 2.0F / 65535.0F - 1.0F) * pmax;
    feedback->velocity_rad_s = ((float)feedback->velocity_raw * 2.0F / 4095.0F - 1.0F) * vmax;
    feedback->torque_nm = ((float)feedback->torque_raw * 2.0F / 4095.0F - 1.0F) * tmax;
    return 1U;
}
