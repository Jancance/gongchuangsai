# 上层调用运动接口（2026-10-24）

最新通用距离接口（前/左/右）和PE2方向记录见[P1_LATERAL_DISTANCE_2026-10-24.md](P1_LATERAL_DISTANCE_2026-10-24.md)。默认路线未改，左右距离本轮只编译未实测。

当前距离调用及新增厘米状态字段见[P1_DISTANCE_API_2026-10-24.md](P1_DISTANCE_API_2026-10-24.md)。Start参数为最高RPM、目标厘米；本轮只整理接口，不启用校准。

最新补充：新增前进距离接口、动作表第5字段distance_mm、kind4及64字段TM1，参数/失败修复/87cm实测边界见[P1_FORWARD_DISTANCE_2026-10-24.md](P1_FORWARD_DISTANCE_2026-10-24.md)。下方保留原定时调用说明。

上层仅包含实际工程USER/motion_control.h，不直接写Watch变量、VM请求、四轮指令或底层会话。当前仍是模式3开发接口，正式自主任务入口尚未接入；不要把示例编译进去就自动启动。执行器继续在原CAN任务运行，不新增运动任务、不阻塞等待。

## 参数与返回值

| 调用 | 参数 | 含义 |
|---|---|---|
| MoveStart | MOVE_FORWARD / MOVE_LEFT / MOVE_RIGHT，3..15 RPM，1..8000ms | 请求一次平移，没有后退 |
| TurnStart | TURN_CW / TURN_CCW，正角度5..90度，最大轮RPM 1..5 | 请求相对转向，保持原接近目标减速 |
| MotionGetState | 无 | 读取IDLE / BUSY / DONE / CANCELLED / FAULT / UNAVAILABLE |
| MotionReadStatus | MotionStatus指针 | 同一临界区读取动作、会话、电源输出、底层结果和故障快照 |
| MotionFinish | 无 | 正常最后动作DONE后请求关VM |
| MotionStop | 无 | 异常或取消，走原停止和断VM路径 |
| MotionRpmToCmPerSec | RPM | 理论轮缘速度显示，不参与控制 |

Start/Finish返回1表示接受，0表示拒绝，均不表示已运动或已断电。ReadStatus返回1只表示复制成功，空指针返回0；读取成功不等于健康。范围常量统一在motion_control.h，边界检查引用相同常量，数值不变。DONE是当前动作已正常停稳，通常仍保电。底层backend_result可能暂留上一动作结果，必须先看state；CAN/RTOS/Safety故障也须核对。Finish后state可保持DONE，需要另外等vm_requested和vm_output_active均0。后者只是PE2命令/输出寄存器，不是电压检测。

## 周期调用顺序示例

以下是上层接入示例，不加入当前任务、不自动运行；调用方必须在受控入口显式启动，独占本会话且没有动作表运行。当前模式3仅作开发验证，不能据此宣称正式比赛入口完成。不要在中断内调用，也不要用while等待DONE阻塞调度器。

```c
#include "motion_control.h"

/* 0=未启动，1=等动作，2=等断VM，3=完成，4=失败。
 * upper_stage和upper_begin_ms由一个上层任务持有。
 * 仅在明确启动请求时执行一次： */
if (upper_stage == 0U && start_requested) {
    upper_begin_ms = now_ms;
    upper_stage = MoveStart(MOVE_FORWARD, 15U, 2000U) ? 1U : 4U;
}

/* 每轮任务周期调用，不在这里创建循环等待。 */
if (upper_stage == 1U || upper_stage == 2U) {
    MotionStatus status;
    if (!MotionReadStatus(&status) ||
        (uint32_t)(now_ms - upper_begin_ms) >= 15000U ||
        status.can_fault || status.rtos_fault || status.safety_fault ||
        status.state == MOTION_CANCELLED || status.state == MOTION_FAULT ||
        status.state == MOTION_UNAVAILABLE) {
        MotionStop();
        upper_stage = 4U;
    } else if (upper_stage == 1U && status.state == MOTION_DONE) {
        /* 如果还有下一段，这里改为MoveStart或TurnStart；
         * 只提交一次，接受后继续等新一段DONE，不要先Finish。 */
        if (MotionFinish()) upper_stage = 2U;
        else { MotionStop(); upper_stage = 4U; }
    } else if (upper_stage == 2U && !status.session_open &&
               !status.vm_requested && !status.vm_output_active &&
               !status.backend_active && status.stop_mask == 15U) {
        upper_stage = 3U;
    }
}
```

15秒只是此2秒平移示例的上层总超时，不能照搬给90度转向；上层为实际路线另定有限截止，底层原保护仍独立生效。动作被拒绝应记录并结束，不在周期里持续重发。取消时可以调用MotionStop；故障后不得清标志续跑。当前Finish/取消/故障后仍沿用复位新会话边界，不添加自动复位。

## 与动作表的关系

APP_MOTION_ACTIONS和直接上层调用两选一，不能同时发指令。现有表已经调用MoveStart/TurnStart并在最后调用MotionFinish，使用表时上层只观察，不抢先提交或Finish。四段默认路线未改；不新增抓取/扫码等未确定任务，不改HWT纠偏、轮速、到位容差、电源与故障处理。RPM继续控制，77mm轮15RPM理论约6.05cm/s。

## 验证边界

本轮只整理头文件、统一既有边界常量、增加只读MotionReadStatus，不改运动执行参数。组合/单段/无线/手动四份隔离完整构建均0错0警告；证据motion_interface_*_build.log和motion_interface_review.json。接口未被当前运行路线调用时可能由链接器移除，编译通过只证明语法/类型及模式适配，不代表该快照已实机验证。

上轮经动作表调用的两段15RPM/2秒平移、段间保电、最终Finish关电已实车通过；新状态快照、直接调用方、忙时拒绝、新TurnStart运行仍未单独实测。板上仍上轮两段测试版，本轮不下载、不接串口、不发车；默认源码四段表与板上版本不同。所有修改仅本地，未推送。

核对R RUN-01/PWR-01，E LOC-03/RUN-06/MOT-01/02；正式自主启动、距离/XY及全部异常验证未闭环，官方Q不变。
