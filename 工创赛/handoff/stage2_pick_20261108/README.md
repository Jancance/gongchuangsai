# stage2_pick_20261108 · 固件快照

## 这是什么

2026-11-08 19:24 实车跑通「左部抓取六步」时的完整可编译工程副本，从
`tmp/integration12_stage2_vision_20261107/project/` 原样固化，去掉了 Objects/Listings
等构建产物和两个 `main.c.bak*` 调试备份，共 2033 个源文件。

`manifest.json` 记录每个文件的 SHA256 和字节数，`diff-vs-vehicle.json` 是相对
`D:\新建文件夹4\工创赛\代码\stm32\3.stm32f407工程` 的文件级差异清单。

## 已验证（实车，fault=0）

| 步骤 | 耗时 | 实测 | 误差 |
|---|---|---|---|
| IT1 顶部参考 | 41.23s | after=26970 | — |
| IT2 回零建 hold | 1.28s | hold=1，ext=0.00 | — |
| 转臂 → 左部 | 0.46s | real=166.383°（目标 166.03） | 0.35° |
| 伸出 156mm | 1.50s | 155.58mm | 0.42mm |
| 下降 | 2.04s | after=1336463（目标 1336500） | 37 计数 |
| 夹爪 140° | 0.68s | 通过 | — |

编译：Keil UV4，Target `Vehicle_LiftRuler`，Code=152724 / 0 Error / 0 Warning。

> **2026-11-08 21:37 更新**：已改为 **Code=152888**（`lift_ruler_calibration.c`：读 0x22 之前先发 `F3 AB 01 00 6B` 使能；`LiftRuler_Start()` 复位使能标志；`LiftRuler_CommandAllowed()` 放行 phase 2/20 的使能帧），0 Error 0 Warning，已烧录 Verify OK。复位后冷启动 IT1 实车通过：41.36s、`after=27009`、`top_valid=1`、fault=0。驱动器回零参数同时修回 speed=60（此前被 `tmp/write_home_params.py` 写成 0x60=96）。详见根目录 `P1电控组交接_2026-10-15.md` 的 21:45 节。

## 关键参数（改动务必三处一致）

| 项 | 值 | 依据 |
|---|---|---|
| `VEHICLE_EXTENSION_PULSES_PER_MM` | **315.5**（35.55°/mm） | 实车钢尺：3306.4°=93mm，3552.5°=100mm |
| 0xFD `units` | `deg × 10`（0.1°/unit） | X42S 手册 V1.0.5 P56 5.3.10 |
| `VSP_Position` 回读 | `raw × 0.1` | 手册 P74 5.5.13 同标度 |
| IT3 delta 上限 | `170mm × PPM × 360/3200` ≈ 6044° | 由常量推导，不再硬编码 |
| VSP 超差 / 到位窗口 | ±60° / ±45°（1.7 / 1.3 mm） | 实测 100mm 过冲 33° |
| `SPF_STEPPER_ACCEL` | 600 | 与指令值一致，否则 GroupBegin 拒收 |
| `extension_rpm` | 106（被 `SPF_EXT_RPM` 锁死） | 改速必须同改该宏 |

## 编译与烧录

```powershell
$proj='<本目录>\project\gongxun-f407.uvprojx'
& 'C:\Keil_v5\UV4\UV4.exe' -b $proj -j0 -o build.log
& 'C:\Keil_v5\UV4\UV4.exe' -f $proj -j0 -o flash.log
```

Keil 须独占。烧录日志出现 `Verify OK. Application running ...` 才算完成。

## 已知坑（复现条件与对策）

1. **复位后第一次 IT1 必失败**（`it_cause=3`、`lift_ruler_cause=15`）：驱动器上电后首个 `0x22`
   回零参数应答为全 0（只有首尾 `22`/`6B`），`home_parameters_valid` 拒绝。清
   `g_lift_ruler_controller` 的 state/cause/phase 后重试即过。**尚未做成自动重试**。
2. **`eh_phase` 残留**：撞限位或 hold 超差 fault 后再请求 IT2，会在 0.07s 内 `eh_fail(5)`，
   清 `g_startup_axes_state` 无效。必须整机复位：写 `0xE000ED0C = 0x05FA0004`。
3. **四轮守卫窗口**：`safety.c` 的豁免条件已加入 `g_it_lift_permit` / `g_it_state`；
   若改回只按 `lift_ruler_state>=1 || axes_state>=1` 判断，IT 启动前会锁 `0x20`。
4. **`source_prepare.inc` 的 PPM 门禁**已是范围检查（0<PPM<100000），不要再写回 `==32.0f`。
5. **`SPF_GroupBegin` 硬性要求** `extension_rpm == SPF_EXT_RPM`、`stepper_accel == SPF_STEPPER_ACCEL`、
   `dm_deg_s == SPF_DM_DEG_S`，任何不一致直接 `it_fail(7)`。
6. 调试变量 `g_it_ext_rpm_test`（0=106rpm）受第 5 条限制，实际不可在线调。

## 未闭环

- 夹爪只有 140/210 两档，无 160° 预收时序
- 抓取后回升 + 放料到三槽载盘未接
- 右部（227.13°）、下部（180°/0mm）未实车验证
- 视觉仍是脚本写死

## 证据

`evidence/integration/2026-11-07/stage2-mainline/` 下：
`units-speed-sweep`、`units-fix-100mm`、`ppm294-100mm`、`ext-cal-point2`、
`left-grab-full`、`left-grab-descend-grip` 等 JSON。
