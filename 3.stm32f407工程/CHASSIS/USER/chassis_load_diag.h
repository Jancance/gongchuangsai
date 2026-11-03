#ifndef CHASSIS_LOAD_DIAG_H
#define CHASSIS_LOAD_DIAG_H
#include <stdint.h>

#define CHASSIS_LOAD_CAPACITY 3072U
#define CHASSIS_LOAD_TX 1U
#define CHASSIS_LOAD_ACK 2U
#define CHASSIS_LOAD_SAMPLE 3U
#define CHASSIS_LOAD_STOP_ACK 4U
#define CHASSIS_LOAD_STAGED_ACK 5U
#define CHASSIS_LOAD_POSITION_NEGATIVE 1U
#define CHASSIS_LOAD_TARGET_VALID 2U
#define CHASSIS_LOAD_SPEED_CLIPPED 4U

/* 固定24字节小端记录；flags的3至6位保存后端状态。 */
typedef struct {
    uint32_t tick_ms, target_ack_ms, position_deci_deg;
    int16_t target_decirpm, actual_decirpm;
    uint16_t phase_ma, bus_mv;
    uint8_t address_kind, result, flags, faults;
} ChassisLoadRecord;
typedef char ChassisLoadRecordSizeCheck[(sizeof(ChassisLoadRecord)==24U)?1:-1];
typedef struct {
    uint32_t session, count, dropped;
} ChassisLoadStatistics;
typedef struct {
    uint32_t position_deci_deg;
    int32_t speed_decirpm;
    uint16_t phase_ma, bus_mv;
    uint8_t position_sign, home_flags, motor_flags;
} ChassisSystemSnapshot;

void ChassisLoad_RecordCommand(uint8_t address, int32_t target_decirpm,
    uint8_t kind, uint32_t result, uint32_t tick_ms);
void ChassisLoad_RecordSnapshot(uint8_t address, uint32_t result,
    const ChassisSystemSnapshot *sample, uint32_t tick_ms);
uint32_t ChassisLoad_Read(uint32_t index, ChassisLoadRecord *record, ChassisLoadStatistics *statistics);
#endif
