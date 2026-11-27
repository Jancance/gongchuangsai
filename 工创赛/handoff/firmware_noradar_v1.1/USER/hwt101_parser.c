#include "hwt101_parser.h"
#include <string.h>
#include <stddef.h>
void Hwt101Parser_Init(Hwt101Parser *p)
{ if (p != NULL) memset(p, 0, sizeof(*p)); }
void Hwt101Parser_Invalidate(Hwt101Parser *p)
{ if (p != NULL) { p->length = 0U; p->seen = 0U; } }
/* 显式符号扩展，不依赖负数左移或超范围有符号转换 */
static int16_t Hwt_Signed(uint8_t lo, uint8_t hi)
{
    uint32_t u = (uint32_t)lo | ((uint32_t)hi << 8);
    int32_t s = (u >= 32768U) ? (int32_t)u - 65536 : (int32_t)u;
    return (int16_t)s;
}
void Hwt101Parser_Feed(Hwt101Parser *p, uint8_t data, uint32_t tick)
{
    uint32_t i;
    uint8_t sum = 0U;
    if (p == NULL) return;
    if ((p->length != 0U) && ((uint32_t)(tick - p->byte_tick) > HWT101_GAP_MS))
        p->length = 0U;
    p->byte_tick = tick;
    if ((p->length == 0U) && (data != 0x55U)) return;
    p->frame[p->length++] = data;
    if (p->length < 11U) return;
    for (i = 0U; i < 10U; i++) sum = (uint8_t)(sum + p->frame[i]);
    if (sum == p->frame[10])
    {
        memcpy(p->last_frame, p->frame, sizeof(p->last_frame));
        if (p->frame[1] == 0x53U)
        {
            p->yaw_raw = Hwt_Signed(p->frame[6], p->frame[7]);
            p->angle_tick = tick; p->angle_count++; p->seen |= 1U;
        }
        else if (p->frame[1] == 0x52U)
        {
            p->wz_raw = Hwt_Signed(p->frame[6], p->frame[7]);
            p->rate_tick = tick; p->rate_count++; p->seen |= 2U;
        }
        else if (p->frame[1] == 0x5FU)
        {
            memcpy(p->register_data, p->frame + 2, 8U);
            p->register_tick = tick; p->register_count++;
        }
        else p->other_count++;
        p->length = 0U;
    }
    else
    {
        p->checksum_errors++; p->seen = 0U;
        /* 错位/丢字节后逐字滑动找帧头，不丢弃后续完整有效帧 */
        for (i = 1U; i < 11U; i++) if (p->frame[i] == 0x55U) break;
        p->length = (uint8_t)(11U - i);
        if (p->length != 0U) memmove(p->frame, p->frame + i, p->length);
    }
}
uint32_t Hwt101Parser_Valid(const Hwt101Parser *p, uint32_t now)
{
    uint32_t mask = 0U;
    if (p == NULL) return 0U;
    if ((p->seen & 1U) && ((uint32_t)(now - p->angle_tick) <= HWT101_FRESH_MS)) mask |= 1U;
    if ((p->seen & 2U) && ((uint32_t)(now - p->rate_tick) <= HWT101_FRESH_MS)) mask |= 2U;
    return mask;
}
