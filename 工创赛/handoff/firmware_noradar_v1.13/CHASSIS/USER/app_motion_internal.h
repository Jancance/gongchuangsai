#ifndef APP_MOTION_INTERNAL_H
#define APP_MOTION_INTERNAL_H
#include <stdint.h>
/* 底盘子系统内部故障取证桥；不改变原临界区和首次故障语义。 */
void AppMotion_InternalFaultCapture(uint32_t cause);
#endif
