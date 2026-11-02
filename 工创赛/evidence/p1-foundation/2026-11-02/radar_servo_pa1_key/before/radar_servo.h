#ifndef RADAR_SERVO_H
#define RADAR_SERVO_H
#include <stdint.h>
/* 状态0未初始化、1PWM就绪、2强制离线不输出、3时钟配置不适用。 */
extern volatile uint32_t g_radar_servo_state;
extern volatile float g_radar_servo_angle_deg[2];
extern volatile uint32_t g_radar_servo_pulse_us[2];
uint32_t RadarServo_Init(void);
/* id为1或2；只更新对应通道，返回1仅表示PWM命令写入，不是到位反馈。 */
uint32_t RadarServo_SetAngle(uint32_t id, float angle_deg);
/* 首次空转调试邮箱：仅0～180度，request=1只消费一次；最后写request。
 * result：1写入PWM，2未就绪，3参数拒绝；sequence每次处理后递增。
 * 这些值只是命令处理结果，不是舵机实物到位反馈。 */
extern volatile uint32_t g_radar_servo_test_request,g_radar_servo_test_id;
extern volatile uint32_t g_radar_servo_test_result,g_radar_servo_test_sequence;
extern volatile float g_radar_servo_test_angle_deg;
void RadarServo_DebugService(uint32_t safe_idle);
/* 初始位置1号0度/2号180度；出发目标1号180度/2号0度。 */
extern volatile uint32_t g_radar_servo_departure_done,g_radar_servo_departure_id;
uint32_t RadarServo_OnDeparture(uint32_t zone);
#endif
