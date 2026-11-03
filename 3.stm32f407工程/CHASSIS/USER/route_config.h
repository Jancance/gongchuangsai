#ifndef ROUTE_CONFIG_H
#define ROUTE_CONFIG_H
#include "../../USER/vehicle_parameters.h"
/* 整车只读取vehicle_parameters.h的八段表，删除旧周圈试验路线预设。
 * 以下是底层曲线选择，日常不改；启用距离闭环，不要求Watch/无线ARM。
 */
#define APP_ROUTE_USE_CUSTOM 1
#define APP_ROUTE_SPEED_TRIAL_ENABLE 1
#define APP_ROUTE_FORWARD_MAX_CM_S VEHICLE_MAX_TRANSLATION_CM_S
#define APP_ROUTE_LEFT_MAX_CM_S VEHICLE_MAX_TRANSLATION_CM_S
#define APP_ROUTE_RIGHT_MAX_CM_S VEHICLE_MAX_TRANSLATION_CM_S
#define APP_ROUTE_ROWS(X)
#endif
