#ifndef EXTENSION_CONFIG_H
#define EXTENSION_CONFIG_H

/* 普通运动按参考X报文等效106RPM，适配本车Emm；回零参数不改。 */
#define EXTENSION_CAN_ADDRESS       6U
#define EXTENSION_JOG_PULSES        1600U
#define EXTENSION_JOG_RPM           106U
#define EXTENSION_JOG_ACCELERATION  255U
#define EXTENSION_MOVE_TIMEOUT_MS   10000U
#define EXTENSION_POLL_MS           100U
#define EXTENSION_SERVICE_GAP_MS    500U
/* 本机Clog_Ms口述2000ms；只约束普通运行的瞬时堵转，不写驱动参数。 */
#define EXTENSION_STALL_CONFIRM_MS  2000U
/* 1600脉冲约50mm粗标：仅看护调试，不代表精密标定或绝对行程。 */
#define EXTENSION_PULSES_PER_MM     32.0f
#define EXTENSION_DEFAULT_MM        10.0f
#define EXTENSION_DEFAULT_USE_MM    1U

#endif
