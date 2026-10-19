#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 初始化提示次数：0静音，1默认，可改为3 */
#define APP_INIT_BEEP_COUNT    2U

/* 单次鸣叫和两次之间的间隔，单位：毫秒 */
#define APP_BEEP_ON_MS         200U
#define APP_BEEP_GAP_MS        200U
/* 启动按键连续稳定时间，单位：毫秒 */
#define APP_START_KEY_DEBOUNCE_MS    20U

#endif
