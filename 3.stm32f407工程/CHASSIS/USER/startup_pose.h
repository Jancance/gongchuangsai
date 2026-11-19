#ifndef STARTUP_POSE_H
#define STARTUP_POSE_H
#include <stdint.h>
void StartupPose_Init(void);
/* 联合启动前纯参数核验；执行函数只写本次舵机目标。 */
uint32_t StartupPose_GroupValid(void);
uint32_t StartupPose_GroupCommand(void);
#endif
