# 2026-11-20 三个区域任务交接

本版把原料三抓、粗加工三放三收、码垛三放分别交给上层路线调用。由上层确认车辆到达指定区域并停稳，准备好对应视觉入口和本轮任务，再调用模块；本版没有串接整张赛场路线。正式工程安装目标为 `D:\新建文件夹4\工创赛\代码\stm32\3.stm32f407工程`，安装与构建状态以同目录当前进度说明为准。以本轮候选为工程基线，旧版副本只用于对照证据。

## 调用约定

声明在 [region_dispatch.h](../../3.stm32f407工程/CHASSIS/USER/region_dispatch.h)，派发实现见 [region_dispatch.inc](../../3.stm32f407工程/CHASSIS/USER/region_dispatch.inc)。在任务上下文调用；CAN 服务异步接单并沿用各模块已有运动所有权。

| 输入 | 实际含义 |
| --- | --- |
| `run_id` | 非零。由整车上层统一分配，每次调用 `Begin` 使用新的递增编号，跨区域、跨批次都不重用；`Poll` 和 `Stop` 使用本次已接受的编号。 |
| `batch_id` | `1` 或 `2`，是任务批次，不是模块序号。 |
| `module` | `REGION_MODULE_SOURCE`、`REGION_MODULE_ROUGH` 或 `REGION_MODULE_STACK`。 |
| `colors[3]`、`rings[3]` | `colors[i]` 是第 `i` 件任务颜色，编码 1～6；`rings[i]` 是同一件的物理目标环，三个值须为 1、2、3 的排列。Source 接口也验证 `rings`，但抓取定位由原料区视觉和原有逻辑处理，不按这个字段抓原料。 |
| `slots[3]` | 按物理槽 1、2、3 排列的颜色，不是执行顺序。Source 将这一映射作为装槽输出目标；例如颜色 `5` 在 `slots[2]`，动作目标就是槽 3。Source 的 `DONE` 表示对应装槽动作已完成，接口不返回独立槽内在位传感器确认。交给后续模块的是上层维护的物理槽颜色表。 |

`RegionDispatch_Begin(&request)` 返回 `1` 表示请求已复制并排入服务，返回 `0` 表示拒绝；它不等于动作已完成。平台、现有所有权或参数检查不通过时不能开始；忙时不替换当前请求。Source 使用上层锁定颜色，不用固定 `215` 测试颜色，也不等待重新扫码；原料区视觉仍需提供真实的本轮定位。

| `RegionDispatch_Poll(run_id)` | 上层处理 |
| --- | --- |
| `UNKNOWN` | 没有匹配的当前轮次，不能推进路线。 |
| `BUSY` | 请求已接受或正在执行，继续异步轮询。 |
| `DONE` | 仅匹配轮次满足本模块完成判据时返回；可以交给上层决定下一步。只在本轮处理一次。 |
| `STOPPING` | 停止请求已发出，禁止继续路线或启动下一模块。 |
| `STOPPED` | 未接管即取消，或已接管且现有停止/释放条件满足。若此前 DM 在运动或保持，还要等其保持退出和 STOP 确认。此状态不是成功，也不是实体停稳验收。 |
| `FAULT` | 记录首故障并交上层处置，不重发 `Begin` 清故障。已接管后停止同样不能用下一次 `Begin` 重新开跑。 |

`RegionDispatch_Stop(run_id)` 返回 `1` 只表示该轮次接受停止请求，仍需 `Poll`；旧编号不会影响当前模块。下例仅展示上层已有“到位且停稳”事件后的调用，不实现任何路线：

```c
#include "region_dispatch.h"

/* 上层调度器持有统一的递增编号，跨区域与批次共享。 */
static uint32_t next_run_id;
static uint32_t active_run;

uint32_t StartRoughAtVerifiedArrival(void)
{
    RegionModuleRequest req = {
        .run_id = ++next_run_id, .batch_id = 1U,
        .module = REGION_MODULE_ROUGH,
        .colors = {2U, 1U, 5U}, .rings = {2U, 1U, 3U},
        .slots = {2U, 1U, 5U}
    };
    if (!RegionDispatch_Begin(&req)) return 0U;
    active_run = req.run_id;
    return 1U;
}
/* 后续由上层服务：RegionDispatch_Poll(active_run)。
 * 需要停机时：RegionDispatch_Stop(active_run)，继续轮询。 */
```

## 动作和完成判据

| 模块 | 动作与本轮完成信号 |
| --- | --- |
| Source | 按锁定颜色逐件抓取并放入指定物理槽。每件只消费当前低层动作序列的完成，上层匹配 `run_id`、Source `state 8` 且已完成三件才得到 `DONE`。见 [source_mcu_cycle.inc](../../3.stm32f407工程/CHASSIS/USER/source_mcu_cycle.inc)。 |
| Rough | 三放三收。末放松爪 PWM 成功满 100 ms 后，Z 从末件锁定放置深度上抬 40 mm，反馈到位后才转 DM 至首回收环，DM 到位后才走 X 至首回收环绝对目标，X 确认后才下降；无需先收 X 到零。回槽松爪仍走原 100 ms，最后回槽后 DM 实际到 0°，才匹配 `state 6 / phase 32`、三放三收并结束计时。 |
| Stack | 按显式的颜色与环位对应，从车载槽取三件放至三环，三放零收。每轮从原始粗加工放置深度减一次 `24000` 虚拟脉冲（60 mm），环 1/2/3 为约 **94.506 / 91.988 / 97.506 mm**。第三件松爪后，依次确认安全顶、伸缩 X0、DM0，才进入 `state 6 / phase 36` 并发布 `DONE`。粗加工中已经收回的物料不算作码垛底层；实际底层与环位仍需现场确认。 |

粗加工和码垛的动作实现见 [stage3_cycle.inc](../../3.stm32f407工程/CHASSIS/USER/stage3_cycle.inc)。现场全局参数集中在 [USER/vehicle_parameters.h](../../3.stm32f407工程/USER/vehicle_parameters.h) 的中文注释区；几何数组按物理环 1、2、3 排列。`VEHICLE_STAGE3_PROFILE_PRESET_ENABLE=1` 保留几何，独立台架入口的任务默认值不覆盖上层 `Begin` 传入的颜色、环位和槽位。

## 构建、视觉和现场边界

1. 正式工程安装后，在 Keil 打开 `gongxun-f407.uvprojx`，主目标选 `Vehicle_LiftRuler`。本地候选同时构建 `Vehicle_LiftRuler_Offline`。用户按现场流程自行下载；当前证据不声明这版已下载或实车执行。
2. PC13 归整车入口，区域台架键和旧第三区独立入口默认关闭。`VEHICLE_FULL_RUN_ENTRY_ENABLE=0`，`VEHICLE_RUN_ALL=0`；整图路线、到位停稳交接和双批调度尚未串通，不能单独打开 `RUN_ALL` 当作具备全程运行条件。
3. Pi 现有自启只服务 Stage3 单区。跨区需接通 Source/Stage3 观察选择桥，使 Pi 按 MCU 当前任务切换并提供本轮数据；当前粗加工配置为 2 mm / 1°、前 13 mm 右 5 mm 补偿。码垛视觉候选及实际底层高度、干涉净空需要另验。

软件证据：[模块 MATLAB 72 场景](../handoff/2026-11-20_stage23/stage3_modules_validation.json)、[模块主机夹具](../handoff/2026-11-20_stage23/current_validation/modules-fixture.log)、[主目标构建](../handoff/2026-11-20_stage23/current_validation/Vehicle_LiftRuler.log)、[Offline 构建](../handoff/2026-11-20_stage23/current_validation/Vehicle_LiftRuler_Offline.log)，两个目标各 0 错 0 警告。[归档说明](../handoff/2026-11-20_stage23/README.md)按版本区分历史现场读回与模型；旧镜像的读回不能作本次验收。相关规则复核 `RUN-01/02/03/04`、`SRC-01/02`，工程项 `RUN-06`、`SRC-04`、`MECH-06`；本页的深度、流程和接口是工程选择，现场用时、扫掠净空、码垛和完整上层运行仍待验收。
