#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H
#include <stdint.h>

/* 上层只包含本头文件；在调度器启动后的任务上下文调用，不在中断中调用。
 * 调试入口复用单段/组合接口；比赛入口未接任务调度前拒绝Start。
 * 单一上层负责提交；与动作表互斥。调用不等待CAN或运动结束。 */
#define MOVE_FORWARD 0U
#define MOVE_LEFT    1U
#define MOVE_RIGHT   2U
#define MOVE_BACKWARD 3U /* candidate: route-distance only, not timed public Move */
#define TURN_CW      0U
#define TURN_CCW     1U

/* 保持现有数值范围，RPM是电机/直连轮转速，不是车体角速度。 */
#define MOTION_MOVE_MIN_RPM   3U
#define MOTION_MOVE_MAX_RPM  15U
#define MOTION_MOVE_MAX_MS 8000U
#define MOTION_DISTANCE_MIN_CM  10.0f
#define MOTION_DISTANCE_MAX_CM 100.0f
#define MOTION_TURN_MIN_DEG   5.0f
#define MOTION_TURN_MAX_DEG  90.0f
#define MOTION_TURN_MAX_RPM   5U

#define MOTION_IDLE        0U
#define MOTION_BUSY        1U
#define MOTION_DONE        2U
#define MOTION_CANCELLED   3U
#define MOTION_FAULT       4U
#define MOTION_UNAVAILABLE 5U

/* Start返回1只代表请求接受；0代表参数/忙/准备状态等拒绝，不自动重试。
 * Move：方向为上面三个MOVE常量，时长1..8000毫秒；无后退入口。
 * Turn：正角度5..90度，方向由TURN常量指定，最大轮速1..5RPM；
 *       仍保持接近目标分段减速和原到位容差。 */
uint32_t MoveStart(uint32_t direction, uint32_t rpm, uint32_t run_ms);
uint32_t TurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm);

/* 距离前进：第1参数最大轮速3..15RPM，第2参数10..100厘米。
 * 例：MoveForwardDistanceStart(15U, 87.0f)。仅模式3且位置诊断开启。
 * 返回1是接受请求；仍须查询BUSY/DONE，拒绝不自动重发。
 * HWT保持方向、接近目标减速；30秒截止或反馈异常按原路径停止。
 * 单位换算用名义轮径，未加行程补偿，不保证物理厘米精度。
 * 低速/长距离可能先触发超时，不能将超时当作到距成功。 */
/* 通用距离平移：MOVE_FORWARD/LEFT/RIGHT，范围与上面相同，无后退。
 * MoveDistanceStart(MOVE_LEFT,15U,50.0f)请求左移50cm。
 * 横移理论距离基于45度X形轮，实际侧向距离须独立验证。 */
uint32_t MoveDistanceStart(uint32_t direction, uint32_t max_rpm, float distance_cm);
/* 旧前进接口保留包装，等价于MoveDistanceStart(MOVE_FORWARD,...)。 */
uint32_t MoveForwardDistanceStart(uint32_t max_rpm, float distance_cm);


/* DONE表示本动作正常停稳，通常仍保电，可提交下一动作。
 * CANCELLED/FAULT不可当作DONE，也不能清故障续跑。 */
uint32_t MotionGetState(void);

/* 状态为同一临界区内快照；读取不发送CAN、不改变运动、不清故障。
 * vm_output_active只是PE2输出命令/寄存器状态，不是VM电压实测。
 * backend_*来自底层执行器，初始/新动作排队时可能仍保留上段结果；
 * 必须先看state，不能单凭backend_result=1认定当前动作完成。 */
typedef struct
{
    uint32_t state;
    uint32_t session_open;
    uint32_t vm_requested, vm_output_active;
    uint32_t backend_active, backend_result, backend_cause, stop_mask;
    uint32_t can_fault, rtos_fault, safety_fault;
    /* distance_mode=1表示当前请求是距离前进；非距离动作下以下字段为0。
     * 排队/准备阶段目标已可读，进度/到距/基础RPM等后端匹配本会话才更新。
     * 到距与DONE都须成立才是正常完成；distance_reached单独不表示停稳。
     * progress在完成/故障后是最后估算，不是PE1终点行程或实时定位。
     * distance_fault_reason=0也不能替代state和系统/后端故障检查。 */
    uint32_t distance_mode, distance_reached, distance_base_rpm, distance_fault_reason;
    uint32_t distance_direction; /* distance_mode=1时为MOVE_FORWARD/LEFT/RIGHT */
    float distance_target_cm, distance_progress_cm;
} MotionStatus;

/* 1=已复制，0=空指针；复制成功不代表机器人健康。 */
uint32_t MotionReadStatus(MotionStatus *status);

/* 整组最后一个动作DONE后调用：1=已接受关VM请求，0=忙/非本调用方等拒绝。
 * 等快照vm_requested=0且vm_output_active=0再认为输出命令已关闭。
 * 动作表自行Finish，上层不要抢先Finish。当前整组结束后需复位新会话。 */
uint32_t MotionFinish(void);

/* 取消整组并走原停车/断VM路径；不是正常动作间的停顿函数。
 * 异常后不自动重发Start，不清系统或驱动故障。 */
void MotionStop(void);

/* 按当前轮径换算理论轮缘cm/s，不能代替实际车速/横移距离测量。 */
float MotionRpmToCmPerSec(float rpm);
#endif
