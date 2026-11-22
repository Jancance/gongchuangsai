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
#define LIFT_UP_MAX_PULSES    233333U
/* 用户2026-11-02要求：放行已示教105000等效脉冲的单条X下降；默认步长仍5000。 */
#define LIFT_DOWN_MAX_PULSES  105000U
/* 参考X报文常用速度1846.4RPM，Emm以整数RPM明确设1846，不复制uint16溢出。 */
#define LIFT_DOWN_JOG_RPM     1846U
#define LIFT_JOG_RPM          1846U
#define LIFT_JOG_ACCELERATION 255U
#define LIFT_MOVE_TIMEOUT_MS  12000U
#define LIFT_POLL_MS          100U

/* X模式单位：速度0.1RPM，加/减速度RPM/s。旧细分须实读后填入。 */
#define LIFT_X_SPEED_DECI_RPM 18464U
#define LIFT_X_ACCEL_RPM_S 45055U
#define LIFT_X_DECEL_RPM_S 45055U
#define LIFT_X_LEGACY_MSTEP 16U

#endif
