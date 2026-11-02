#ifndef X_LOAD_DIAG_H
#define X_LOAD_DIAG_H
#include <stdint.h>

#define XLOAD_CAPACITY 3072U
#define XLOAD_TX 1U
#define XLOAD_ACK 2U
#define XLOAD_SAMPLE 3U
#define XLOAD_STOP_ACK 4U
#define XLOAD_STAGED_ACK 5U
#define XLOAD_POSITION_NEGATIVE 1U
#define XLOAD_TARGET_VALID 2U
#define XLOAD_SPEED_CLIPPED 4U

/* 固定24字节小端记录；flags的3至6位保存后端状态。 */
typedef struct {
    uint32_t tick_ms, target_ack_ms, position_deci_deg;
    int16_t target_decirpm, actual_decirpm;
    uint16_t phase_ma, bus_mv;
    uint8_t address_kind, result, flags, faults;
} XLoadRecord;
typedef char XLoadRecordSizeCheck[(sizeof(XLoadRecord)==24U)?1:-1];
typedef struct {
    uint32_t session, count, dropped;
} XLoadStatistics;
typedef struct {
    uint32_t position_deci_deg;
    int32_t speed_decirpm;
    uint16_t phase_ma, bus_mv;
    uint8_t position_sign, home_flags, motor_flags;
} XSystemSnapshot;

void XLoad_RecordCommand(uint8_t address, int32_t target_decirpm,
    uint8_t kind, uint32_t result, uint32_t tick_ms);
void XLoad_RecordSnapshot(uint8_t address, uint32_t result,
    const XSystemSnapshot *sample, uint32_t tick_ms);
uint32_t XLoad_Read(uint32_t index, XLoadRecord *record, XLoadStatistics *statistics);
#endif
