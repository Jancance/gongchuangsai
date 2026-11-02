# 前进厘米接口调用说明（2026-10-24）

最新通用距离接口（前/左/右）和PE2方向记录见[P1_LATERAL_DISTANCE_2026-10-24.md](P1_LATERAL_DISTANCE_2026-10-24.md)。默认路线未改，左右距离本轮只编译未实测。

本次整理已有前进距离算法的公开接口和只读状态，不新建任务，不新增自动发车入口，不启用补偿。当前仍仅APP_TEST_MODE=3开发模式可启动距离，必须开启APP_POSITION_DIAGNOSTIC；正式比赛自主入口尚未接入。其余模式Start返回0、MotionGetState为UNAVAILABLE。

## 提交一次请求

```c
#include "motion_control.h"

/* 最大基础轮速15RPM，目标87厘米；必须检查返回值，只提交一次。 */
uint32_t accepted = MoveForwardDistanceStart(15U, 87.0f);
```

第一个参数是最大基础轮RPM（3..15），第二个参数是厘米（10..100，常量MOTION_DISTANCE_MIN_CM/MAX_CM）。backend内部仍用mm，调用者不用换算。返回1仅表示接受请求，0表示范围、忙、模式、准备、自检等拒绝，不自动重发。不要把MoveStart的第三个毫秒参数当距离；定时平移和距离前进是不同调用。

30秒距离截止及现有准备/单段/停止/反馈保护保留，低速长距离可能先超时，允许参数不代表保证所有组合都成功；HWT和位置失效应按故障处理。当前只前进，不扩展后退、左右距离、外部坐标或X/Y定位。

## 用同一状态快照查询

MotionReadStatus返回1表示快照复制成功，0为空指针；读取不等待CAN、不驱动电机、不清故障。新增字段均为公开MotionStatus成员：

| 字段 | 含义 |
|---|---|
| distance_mode | 当前已接受请求是距离前进时为1，定时/转向/未提交时为0 |
| distance_target_cm | 已接受的目标厘米数，排队和准备阶段即可读取 |
| distance_progress_cm | 当前会话后端的最近估算厘米数；排队未绑定后端时为0 |
| distance_reached | 后端已触发到距停止；不是四轮已经停稳 |
| distance_base_rpm | 后端最近选定的基础轮速档位，停止后可能仍为3；不是实测轮速 |
| distance_fault_reason | 原距离原因0..6，只读暴露，0不能单独证明健康 |

当非距离动作时新增字段全0；只有后端distance_mode且session确属本次前端会话，才复制进度、到距、档位和专用原因，防止下一请求排队时混入上一个动作结果。目标由本次已接受的前端请求锁存。完成/失败后的progress是最后估算，不是当前位置或PE1最终轮子行程，不能当作新鲜定位数据。

原state、session_open、vm_requested、vm_output_active、backend_active/result/cause/stop_mask及CAN/RTOS/Safety故障字段继续保留。vm_output_active只代表PE2控制命令/寄存器，不是VM电压测量。

## 完成与关电顺序

上层一个调用者负责请求，使用动作表时不要另外提交抢占。每轮任务周期读取快照，不用while忙等，不在中断中调用。

1. 明确的上层启动事件中Start一次；拒绝则记录失败，不能反复Start。
2. 接受后周期等待BUSY变DONE；任何FAULT/CANCELLED/UNAVAILABLE或系统故障均失败，不能因progress看起来够长而报成功。上层仍要有有限总截止，例如本开发单段35000ms，不能无期限等待。
3. 正常完成同时核distance_mode、distance_reached、stateDONE，底层active0/result1/cause0/stopmask15且系统故障0。reached1而BUSY仍要等停稳，超时不算到距。
4. 如有下一动作，此时提交下一动作，继续保电；若整组结束，调用MotionFinish一次并检查是否接受。
5. 再等session_open0、VM请求0、PE2输出0、后端active0，作为输出命令关闭；动作表自己完成Finish，上层不抢先关电。

异常/取消调用MotionStop走原停车及断VM路径，不清故障或自动恢复。当前整组结束/失败后仍按新复位会话启动，不添加自动复位任务。

以下函数片段已用ARMCC针对公开头文件编译，仅读取示例，不接入实际任务：

```c
uint32_t ExampleDistanceDone(void) {
    MotionStatus s;
    if (!MotionReadStatus(&s)) return 0U;
    return s.distance_mode && s.distance_reached && s.state == MOTION_DONE &&
        !s.backend_active && s.backend_result == 1U && !s.backend_cause &&
        s.stop_mask == 15U && !s.can_fault && !s.rtos_fault && !s.safety_fault;
}
```

## 编译与实测边界

本轮新增公开状态字段与厘米单位说明、用现有同值公共范围常量替代距离入口的10/100字面值，运动算法、减速档位、保护、TM1/PM1/PE1和串口命令均不变。五配置完整构建（默认组合/单段/无线/手动/诊断关闭）0错0警告，公开头文件示例独立编译通过，原50项主机协议测试通过，源比对/GBK/CRLF与差异检查通过。证据distance_api_*_build.log和distance_api_review.json。主机测试不覆盖新增C状态快照运行，尚未上板验证；调用者尚未引用该函数时链接器可以裁剪它。

几何继续使用用户确认轮径77mm、左右轮心距212mm、前后206mm。用户手量约89cm明确不准确，不将其作为标定真值，不应用约1.022候选系数或改轮径；历史理论与手量差值只是观察，不能断言真实稳定偏长1.9cm。偶发HWT源失效和减速速度/方向cause8仍待查，不因本轮整理或编译消除。

本轮未下载、未打开COM、未发车、未上传。板上仍此前distance_sched_87cm已完成程序（旧几何210/210及旧MotionStatus），源码默认四段表不变，新212/206和新增状态接口都只在本地构建。后续接入确定的上层调用时按本说明使用，不为本次整理重复跑87cm。核R RUN-01，E LOC-03/RUN-06/MOT-02，官方Q不变。
