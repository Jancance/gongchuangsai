#ifndef ICM45686_RAW_H
#define ICM45686_RAW_H
#include <stdint.h>
#ifndef ICM45686_RAW_ENABLE
#define ICM45686_RAW_ENABLE 0
#endif
/* 传感器原生XYZ；时间为主机DRDY到达代理，不是ADC采样时刻。 */
typedef struct {
    uint32_t state, last_error, failed_reg, whoami;
    uint32_t accel_config, gyro_config, big_endian;
    int16_t raw_accel[3], raw_gyro[3], raw_temperature;
    uint32_t sample_sequence, edge_sequence, edge_cycle;
    uint32_t read_start_cycle, read_end_cycle, sample_tick_ms;
    uint32_t age_ms, fresh, transfer_errors, skipped_edges, read_races;
} IcmRawStatus;
enum { ICM_RAW_OFF, ICM_RAW_STARTING, ICM_RAW_READY, ICM_RAW_FAILED };
enum { ICM_RAW_OK, ICM_RAW_TRANSFER, ICM_RAW_ID, ICM_RAW_RESET,
       ICM_RAW_CONFIG, ICM_RAW_TIMEBASE, ICM_RAW_ASSOCIATION };
/* Published by main RAW task; equal even publication guards the entire snapshot. */
uint32_t AppIcm_ReadPublished(IcmRawStatus *status, uint32_t *publication);
int IcmRaw_Init(void);
void IcmRaw_Service(void);
void IcmRaw_GetStatus(IcmRawStatus *status);
#endif
