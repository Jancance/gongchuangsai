#ifndef EXTENSION_CONFIG_H
#define EXTENSION_CONFIG_H

/* 28伸缩轴首次方向试验；脉冲数不表示已标定的毫米行程。 */
#define EXTENSION_CAN_ADDRESS       6U
#define EXTENSION_JOG_PULSES        1600U
#define EXTENSION_JOG_RPM           5U
#define EXTENSION_JOG_ACCELERATION  20U
#define EXTENSION_MOVE_TIMEOUT_MS   10000U
#define EXTENSION_POLL_MS           100U
#define EXTENSION_SERVICE_GAP_MS    500U
/* 本机Clog_Ms口述2000ms；只约束普通运行的瞬时堵转，不写驱动参数。 */
#define EXTENSION_STALL_CONFIRM_MS  2000U

#endif
