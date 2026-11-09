#ifndef EXTENSION_H
#define EXTENSION_H
#include "stm32f4xx_can.h"

/* 28夹爪伸缩轴，地址6；本模块仅查询，不含任何运动或参数写入。 */
extern volatile uint32_t g_extension_read_request;
/* 0空闲，1读版本，2读状态，3读选项，4读取完成，5失败。 */
extern volatile uint32_t g_extension_state;
extern volatile uint32_t g_extension_version_result;
extern volatile uint32_t g_extension_status_result;
extern volatile uint32_t g_extension_options_result;
/* 0无错误，7供电未就绪/升降忙，其他值同CAN查询结果。 */
extern volatile uint32_t g_extension_error;
extern CanRxMsg g_extension_version_reply;
extern CanRxMsg g_extension_status_reply;
extern CanRxMsg g_extension_options_reply;
/* 只由现有CAN任务串行调用；升降运动或回零时拒绝开始。 */
void Extension_Service(uint32_t ready, uint32_t lift_busy);
#endif
