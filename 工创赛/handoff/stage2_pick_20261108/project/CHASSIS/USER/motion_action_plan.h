#ifndef MOTION_ACTION_PLAN_H
#define MOTION_ACTION_PLAN_H
#include <stdint.h>
/* 编译期动作表：前/左/右平移、相对转向；不可通过串口修改。 */
#define MOTION_FORWARD 0U
#define MOTION_LEFT    1U
#define MOTION_RIGHT   2U
#define MOTION_TURN    3U
#define MOTION_FORWARD_DISTANCE 4U
#define MOTION_LEFT_DISTANCE    5U
#define MOTION_RIGHT_DISTANCE   6U
#define MOTION_BACKWARD_DISTANCE 7U
#define MOTION_ACTION_CAPACITY 8U
typedef struct
{
    uint32_t kind;
    uint32_t base_rpm;       /* 平移3..15；转向1..5为上限，0兼容原5RPM。 */
    uint32_t run_ms;         /* 定时平移时长；旧距离/转向填0，新候选距离存规划截止ms。 */
    float relative_deg;     /* 平移填0；转向±5..±90度。 */
    float distance_mm;     /* 仅距离平移100..1000mm，其余0。 */
    float reference_mm_s; /* 仅新路线候选：明确S曲线参考速度，0保留旧动作语义。 */
} MotionAction;
/* 只改这里安排路线；默认仍是已验证的四段顺序和参数。 */
#define APP_MOTION_ACTIONS { \
    {MOTION_FORWARD, 15U, 8000U, 0.0f, 0.0f}, \
    {MOTION_LEFT,    15U, 8000U, 0.0f, 0.0f}, \
    {MOTION_TURN,     0U,    0U, 90.0f, 0.0f}, \
    {MOTION_FORWARD, 15U, 8000U, 0.0f, 0.0f} \
}
#endif
