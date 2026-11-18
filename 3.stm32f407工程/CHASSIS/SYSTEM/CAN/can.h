#ifndef CAN_H
#define CAN_H
#include "zdt_emm_protocol.h"
#include "chassis_load_diag.h"

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
/* 首次控制故障锁存，只供遥测读取，不从串口清除。 */
extern volatile uint32_t g_can_ctrl_fault_latched;
/* 在FreeRTOS任务中调用一次，返回测试结果 */
uint32_t CAN1_LoopbackTest(void);
/* 四台Emm菜单核对后的源码放行状态，离线工程始终为0。 */
uint32_t CAN1_EmmSettingsConfirmed(void);
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply);
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply);
void CAN1_ResetSession(void);
uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t address, CanRxMsg *reply, float *rpm);
#define CAN1_NAV_ADMISSION_MS 2U
#define CAN1_NAV_COMPLETION_MS 6U
#define CAN1_TIMING_WINDOW 10U
typedef struct
{
    uint32_t tx_ms, reply_ms;
    uint8_t mailbox_accepted, tx_complete, ack_valid;
} CAN1_EmmSpeedReceipt;
/* Single CAN1 owner only. tx_ms timestamps software mailbox submission.
 * Same-address delayed F6 ACKs have no sequence. The owner must start from
 * a drained session and must not retry an ambiguous transaction in that run. */
uint32_t CAN1_SetEmmSpeedRawBounded(uint8_t address, uint8_t direction, uint16_t speed_raw,
                                   uint16_t acceleration, uint32_t slot_ms, CanRxMsg *reply,
                                   CAN1_EmmSpeedReceipt *receipt);
typedef struct
{
    uint32_t request_ms, reply_ms;
    uint32_t position_counts;
    int32_t speed_decirpm;
    uint8_t position_sign, motor_flags, home_flags, address, valid;
} CAN1_EmmSystemFeedback;
uint32_t CAN1_ReadEmmSystemFeedback(uint8_t address, CanRxMsg *reply,
                                    CAN1_EmmSystemFeedback *feedback);
/* 唯一CAN任务调用，使用已建立的会话。提交邮箱不超过槽起点+2ms，
 * 完成校验和发布不超过+6ms。0x43使用8/8/8/8/2字节的五帧。
 * 时间为主控观察时间。仅返回1时使用读数和接收时间。
 * 协议无请求序号，同轮同功能的迟到旧帧无法按内容区分，运行中不得重试歧义事务。 */
uint32_t CAN1_ReadEmmSystemFeedbackBounded(uint8_t address, uint32_t slot_ms,
                                          CanRxMsg *reply, CAN1_EmmSystemFeedback *feedback);
uint32_t CAN1_ReadStatusBounded(uint8_t address, uint32_t slot_ms,
                                CanRxMsg *reply, uint32_t *reply_ms);
/* 返回物理RPM的float；Emm反馈为整数RPM，不除10。 */
uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, float *rpm);
/* 返回符号及Emm原生65536计数/圈，不做二次缩放。 */
uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply,
                           uint8_t *sign, uint32_t *raw);

/* 使能控制：enabled为0松轴，为1锁轴；地址限定1～4 */
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                             CanRxMsg *reply);

/* 立即停止；地址限定1～4 */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply);

/* Emm速度控制：输入缩小10倍，加速度为0..255档。 */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                             uint16_t speed_raw, uint16_t acceleration,
                             CanRxMsg *reply);
uint32_t CAN1_StageEmmSpeedRaw(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,CanRxMsg *reply);
#endif
