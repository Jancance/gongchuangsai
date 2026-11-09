#ifndef FAKE_RTOS_H
#define FAKE_RTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(n) ((TickType_t)(n))
#define portTICK_PERIOD_MS 1U
#endif
