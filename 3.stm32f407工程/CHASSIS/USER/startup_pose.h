#ifndef STARTUP_POSE_H
#define STARTUP_POSE_H
#include <stdint.h>
void StartupPose_Init(void);
uint32_t StartupPose_GroupValid(void);
uint32_t StartupPose_GroupCommand(void);
#endif
