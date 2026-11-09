#ifndef FAKE_CAN_H
#define FAKE_CAN_H
#include <stdint.h>
typedef struct { uint32_t StdId,ExtId; uint8_t IDE,RTR,DLC,Data[8],FMI; } CanRxMsg;
typedef CanRxMsg CanTxMsg;
#endif
