#ifndef __STM32F4XX_CONF_H
#define __STM32F4XX_CONF_H

/* 基础阶段使用的标准库外设头文件 */
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_can.h"

/* 基础阶段暂不启用完整参数断言 */
#define assert_param(expr) ((void)0U)

#endif
