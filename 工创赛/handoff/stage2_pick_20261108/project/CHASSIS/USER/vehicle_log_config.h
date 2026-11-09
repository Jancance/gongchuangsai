#ifndef VEHICLE_LOG_CONFIG_H
#define VEHICLE_LOG_CONFIG_H
/* USART1只读实验日志。修改后重新编译下载，不通过Watch修改。
 * 每200ms回传四轮最近一次真实反馈及其采样时刻；不是200ms同步采样。
 * 记录CAN已应答目标与实测轮速，不能把应答当作实际到速。
 * 后续统一整车调参文件可包含本文件；串口忙时丢观察帧并累计计数。
 * 不新增CAN查询，不改变控制周期、保护阈值、VM和看门狗。 */
#define VEHICLE_WHEEL_LOG_ENABLE 1U
#define VEHICLE_WHEEL_LOG_PERIOD_MS 200U
#if VEHICLE_WHEEL_LOG_PERIOD_MS < 100U || VEHICLE_WHEEL_LOG_PERIOD_MS > 1000U
#error Vehicle_wheel_log_period_out_of_range
#endif
#endif
