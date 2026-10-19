#ifndef HWT101_PARSER_H
#define HWT101_PARSER_H
#include <stdint.h>
#define HWT101_FRESH_MS 500U
#define HWT101_GAP_MS 30U
/* valid位0角度，位1角速度；只读原始模块符号，未校准车体方向 */
typedef struct {
    uint8_t frame[11], last_frame[11], length, seen;
    int16_t yaw_raw, wz_raw;
    uint32_t byte_tick, angle_tick, rate_tick;
    uint32_t angle_count, rate_count, checksum_errors, other_count;
    uint8_t register_data[8];
    uint32_t register_count, register_tick;
} Hwt101Parser;
void Hwt101Parser_Init(Hwt101Parser *p);
void Hwt101Parser_Invalidate(Hwt101Parser *p);
void Hwt101Parser_Feed(Hwt101Parser *p, uint8_t data, uint32_t tick);
uint32_t Hwt101Parser_Valid(const Hwt101Parser *p, uint32_t now);
#endif
