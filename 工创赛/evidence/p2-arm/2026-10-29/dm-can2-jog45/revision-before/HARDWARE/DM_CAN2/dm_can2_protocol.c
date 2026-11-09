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

uint32_t DM_CAN2_BuildSwitch(uint8_t command, CanTxMsg *tx)
{
    if (tx == 0 || (command != 0xFCU && command != 0xFDU)) return 0U;
    memset(tx, 0, sizeof(*tx));
    tx->StdId = DM_CAN2_NODE_ID;
    tx->IDE = CAN_Id_Standard;
    tx->RTR = CAN_RTR_Data;
    tx->DLC = 8U;
    memset(tx->Data, 0xFF, 7U);
    tx->Data[7] = command;
    return 1U;
}

uint32_t DM_CAN2_BuildMitHold(float position, float pmax, float vmax,
                             float tmax, CanTxMsg *tx)
{
    uint32_t p, v, kp, kd, torque;
    if (tx == 0 || !(pmax > 0.0F && pmax <= 1000000.0F) ||
        !(vmax > 0.0F && vmax <= 1000000.0F) ||
        !(tmax > 0.0F && tmax <= 1000000.0F) ||
        !(position >= -pmax && position <= pmax)) return 0U;
    p = (uint32_t)((position + pmax) * 65535.0F / (2.0F * pmax));
    v = 2047U;
    torque = 2047U;
    kp = (uint32_t)(DM_CAN2_JOG_KP * 4095.0F / 500.0F);
    kd = (uint32_t)(DM_CAN2_JOG_KD * 4095.0F / 5.0F);
    memset(tx, 0, sizeof(*tx));
    tx->StdId = DM_CAN2_NODE_ID;
    tx->IDE = CAN_Id_Standard;
    tx->RTR = CAN_RTR_Data;
    tx->DLC = 8U;
    tx->Data[0] = (uint8_t)(p >> 8);
    tx->Data[1] = (uint8_t)p;
    tx->Data[2] = (uint8_t)(v >> 4);
    tx->Data[3] = (uint8_t)(((v & 0xFU) << 4) | (kp >> 8));
    tx->Data[4] = (uint8_t)kp;
    tx->Data[5] = (uint8_t)(kd >> 4);
    tx->Data[6] = (uint8_t)(((kd & 0xFU) << 4) | (torque >> 8));
    tx->Data[7] = (uint8_t)torque;
    /* v_des、t_ff均为0的量化近似，避免失联后维持非零速度参考。 */
    return 1U;
}

float DM_CAN2_SmoothStep(float ratio)
{
    if (ratio <= 0.0F) return 0.0F;
    if (ratio >= 1.0F) return 1.0F;
    return ratio * ratio * ratio * (10.0F + ratio * (-15.0F + 6.0F * ratio));
}

uint32_t DM_CAN2_BuildMitPreload(float position, float pmax, float vmax,
                                float tmax, CanTxMsg *tx)
{
    if (!DM_CAN2_BuildMitHold(position, pmax, vmax, tmax, tx)) return 0U;
    /* 首次使能前位置增益为0；使能反馈确认坐标未跳变后才使用Kp5。 */
    tx->Data[3] &= 0xF0U;
    tx->Data[4] = 0U;
    return 1U;
}
