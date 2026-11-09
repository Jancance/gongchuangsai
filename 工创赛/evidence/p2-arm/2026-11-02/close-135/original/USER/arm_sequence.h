#ifndef ARM_SEQUENCE_H
#define ARM_SEQUENCE_H
#include <stdint.h>
/* 调试组：在已人工调好夹取高度、夹爪张开时才允许启动。默认均0。 */
extern volatile uint32_t g_arm_sequence_enable;
extern volatile uint32_t g_arm_grip_height_ready;
/* 1单次夹紧/抬起/放回/松开；2中止保持夹爪；3停稳后清锁存。 */
extern volatile uint32_t g_arm_sequence_request;
/* 夹紧后state2观察确实夹住，再写1；不能提前预填。 */
extern volatile uint32_t g_arm_grip_confirm;
extern volatile uint32_t g_arm_test_lift_pulses;
/* 模式0保留小幅往返；模式1夹紧后顶部回零，再按显式记录的脉冲下降放回。 */
extern volatile uint32_t g_arm_sequence_mode;
/* 顶部到夹取处的已记录脉冲，默认0拒绝；不是毫米或自动测量值。 */
extern volatile uint32_t g_arm_return_down_pulses;
/* 模式2：从人工确认的顶部开始，下降夹取、有限抬升、低速回顶、下放。 */
/* 顶部准入接单即消耗；高度/配置改变后必须重新人工确认。默认0。 */
extern volatile uint32_t g_arm_top_start_ready;
extern volatile uint32_t g_arm_pick_down_pulses;
extern volatile uint32_t g_arm_top_approach_pulses;
/* 到state12且实际已落稳才写1；进入该状态会清除旧值。 */
extern volatile uint32_t g_arm_place_confirm;
uint32_t ArmSequence_AllowHomeRequest(uint32_t request);
uint32_t ArmSequence_OwnsHome(void);
/* 0空闲，1闭合，2夹住确认，3抬升，4顶部短停，5放回，6张开，7结束，8中止，9失败；模式2另有10首次下降、11低速回顶、12落稳确认、13准备张开。 */
extern volatile uint32_t g_arm_sequence_state;
/* 1仅软件流程结束，2中止，3准入，4配置，5夹爪，6升降，7超时，8服务间隔。 */
extern volatile uint32_t g_arm_sequence_result;
extern volatile uint32_t g_arm_sequence_elapsed_ms;
/* 当前阶段计时；故障/中止时冻结，清锁存或新接单清零。 */
extern volatile uint32_t g_arm_sequence_phase_elapsed_ms;
/* 保存发生故障/中止前的阶段，不以故障态9覆盖原阶段。 */
extern volatile uint32_t g_arm_sequence_fault_state;
/* 用户指定：state2/state12及动作层等待无期限；显式停止仍有效。 */
uint32_t ArmSequence_IsBusy(void);
uint32_t ArmSequence_AllowLiftRequest(uint32_t request);
void ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now_ms);
/* 模式3固定动作：源下降g_arm_pick_down_pulses须现场示教；0拒绝。
 * 顶部/源料/空槽及路径人工确认后，top_start_ready=1，enable=1，request=1一次。
 * 不等待中途人工确认，不重复循环；state7/1仅软件结束，不自动登记库存。
 * 20张开、21转源、22源下降、23夹紧、24源回升、25转车载、26车载下放、27松开、28回升。
 * result9为旋转/保持失败；其他结果沿用旧定义。 */
extern volatile float g_arm_replay_pick_deg,g_arm_replay_place_deg;
uint32_t ArmSequence_OwnsDM(void);
uint32_t ArmSequence_AllowDMRequest(uint32_t request);
/* 模式4：只按本次示教105000脉冲单条X位置命令下降后闭合145度。
 * 不转DM/转盘、不伸缩、不回零、不抬升；默认关闭，顶部且已对中才置top_start_ready。
 * mode4/enable1/top_start_ready1后request1一次；request2中止，停稳后request3清锁存。
 * state30下降，31闭合；7/1只是软件完成，真实夹住需现场确认。 */
extern volatile uint32_t g_arm_source_down_done_pulses;
/* 源105000夹取点返回：先有限上升104000，再走原60RPM/40秒回零。
 * mode3源上升保留1000等效脉冲余量，state29回零成功后才转车载。
 * mode5只执行返回，不动夹爪/DM；从源夹取点置grip_height_ready1才可启动。
 * mode5 state32上升、33回零，7/1仅软件完成；不得在已经顶部时启动。 */
#define ARM_SOURCE_RETURN_UP_PULSES 104000U
#define ARM_SOURCE_TOP_MARGIN_PULSES 1000U
/* 完整模式3：源下降105000/夹紧/预升104000/回零/转车载；
 * 车载单次下降28000/松爪/预升27000/回零，state34为最终回零。
 * 最终7/1回到车载侧顶部，保持张开及DM模式2，不自动重跑。 */
#endif
