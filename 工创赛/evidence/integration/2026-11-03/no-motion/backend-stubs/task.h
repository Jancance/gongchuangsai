#ifndef TASK_H
#define TASK_H
#include <stdint.h>
typedef uint32_t TickType_t;
extern uint32_t test_tick,test_depth,test_primask;
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() (++test_depth)
#define taskEXIT_CRITICAL() (--test_depth)
static __inline uint32_t xTaskGetTickCount(void){return test_tick;}
static __inline uint32_t __get_BASEPRI(void){return test_depth ? 80U : 0U;}
static __inline uint32_t __get_PRIMASK(void){return test_primask;}
#endif
