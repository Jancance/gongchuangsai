#ifndef TEST_TASK_H
#define TEST_TASK_H
#include <stdint.h>
typedef uint32_t TickType_t;
extern uint32_t test_tick,test_depth;
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() (++test_depth)
#define taskEXIT_CRITICAL() (--test_depth)
static __inline uint32_t xTaskGetTickCount(void){return test_tick++;}
static __inline void vTaskDelay(TickType_t ticks){test_tick+=ticks;}
#endif
