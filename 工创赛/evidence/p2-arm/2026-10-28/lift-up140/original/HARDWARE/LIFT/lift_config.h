#ifndef LIFT_CONFIG_H
#define LIFT_CONFIG_H

/* 用户当前升降实参：张大头35、地址5、CAN1_MAP、500kbit/s。 */
#define LIFT_CAN_ADDRESS       5U
#define LIFT_CAN_BAUD          500000U
#define LIFT_POWER_SETTLE_MS   1000U
#define LIFT_READ_RETRY_MS     300U
#define LIFT_READ_MAX_ATTEMPTS 3U

/* 单次观察步长5000个脉冲；不是已经标定的毫米行程。 */
#define LIFT_JOG_PULSES       5000U
/* 向上单次步长上限；默认仍为5000，需Watch显式调整。 */
#define LIFT_UP_MAX_PULSES    50000U
#define LIFT_JOG_RPM          3000U
#define LIFT_JOG_ACCELERATION 250U
#define LIFT_MOVE_TIMEOUT_MS  12000U
#define LIFT_POLL_MS          100U

#endif
