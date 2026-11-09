#ifndef CARRIER_PWM_H
#define CARRIER_PWM_H
#include <stdint.h>
/* 初始化只保持PB1低，不输出位置脉冲。 */
uint32_t CarrierPwm_Init(void);
uint32_t CarrierPwm_Write(uint32_t pulse_us);
/* 撤销信号不是已确认机械停止。 */
void CarrierPwm_Off(void);
#endif
