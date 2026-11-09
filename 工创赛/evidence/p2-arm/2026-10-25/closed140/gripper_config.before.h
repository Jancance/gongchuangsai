#ifndef GRIPPER_CONFIG_H
#define GRIPPER_CONFIG_H

/* 标称角度映射起点，实际机械开合范围后续按实物调整。 */
#define JIAZHUA_ANGLE_MAX_DEG  270.0f
#define JIAZHUA_PULSE_MIN_US   500U
#define JIAZHUA_PULSE_MAX_US   2500U
#define JIAZHUA_PERIOD_US      20000U

/* 调试任务：原生FreeRTOS，默认不发送位置命令。 */
#define GRIPPER_TASK_STACK_WORDS  256U
#define GRIPPER_TASK_PRIORITY     2U
#define GRIPPER_TASK_PERIOD_MS    10U
#define GRIPPER_DEBUG_ANGLE_DEG   90.0f

#endif
