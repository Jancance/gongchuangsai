#ifndef ARM_POWER_MODE_H
#define ARM_POWER_MODE_H
#include <stdint.h>

/* 0未请求，1菜单供电请求有效，2本次菜单供电时间已到。不是VM电压反馈。 */
extern volatile uint32_t g_arm_menu_power_state;
uint32_t ArmPower_IsMenuMode(void);
uint32_t ArmPower_FilterRequest(uint32_t request);
#endif
