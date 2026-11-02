#ifndef ZDT_X_CONFIG_H
#define ZDT_X_CONFIG_H
/* 离线候选默认禁止VM、驱动使能及非零速度，不是实车发布固件。 */
#ifndef APP_X_OFFLINE_ONLY
#define APP_X_OFFLINE_ONLY 1U
#endif
#define ZDT_X_APP_MAX_SPEED_RAW 3000U
#define ZDT_X_POSITION_UNITS_PER_REV 3600U
#endif
