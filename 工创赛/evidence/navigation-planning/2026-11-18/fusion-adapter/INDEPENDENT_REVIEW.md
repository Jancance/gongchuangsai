# 数据适配层最终独立审查

日期：2026-11-18。基线 HEAD `dedd8d5661c98aff8c4f2f97fb7799e263f34067`；参考层功能基线 `d023c37a6c51e5a454b828cf1e557643ee31c4dd`。结论：**PASS，仅限本阶段 pure copied-value adapter 的源码、合同与实际 ARM 离线验证**。没有发现最终版本仍存在的确定阻塞缺陷，可继续 root 的唯一工程双目标构建。

## 最终被审源码与独立执行

- `fusion_adapter.c` SHA256：`33dcb51411f0fd9cc76ba977187a5d8f9f6aa19a20ad55a7fcf5e6733cc371b9`。
- `fusion_adapter.h` SHA256：`9e344b6d37c10f8b0c69b09d0b552cae0ad677c2a45234fdd7f05c3ca9d8849d`。
- 独立运行回执：`independent-review/actual-arm-final-33dcb514/actual_arm_results.json`；复核回执：同目录 `review_receipt.json`；可复放入口：`independent-review/run_final_33dcb514.py`。该入口故意拒绝覆盖已有 run，重跑应选新目录。
- 实际 ARMCC 编译 adapter、现有 Reference、kinematics 及独立夹具，在 Unicorn 执行 Cortex-M3 Thumb soft-float 机器码。2026-11-18 01:50:56～01:51:05 +08:00，退出 0，**2114 条断言、120 个 MATLAB v10 向量全部通过**。2114=原独立语义 978（其中 81 个整数 dt × 正负边界 × 4 检查=648）+最终数值补测 56+向量断言 1080。此计数不同于实现侧 run04 的 2529，不互相冒用。
- 真实 `app_odometry.h` / `chassis_odometry.h` / `fusion_reference.h` 参与编译，未以自造布局替代。实际 sizeof：state 172B、input 168B、report 92B、AppMapPose 128B、Observation 76B。无平台函数桩；编译/链接日志为空，符号中未见 CAN/getter/RTOS/GPIO/HAL 调用。8 个源/头在执行前后及报告落盘前 SHA 一致。
- 本次写入仅为新的独审脚本、新 final run 目录及本报告；未改产品、冻结前置审查、旧首错、工程文件或根交接，未执行 Git 写命令、fetch/push 或任何硬件操作。

## 已闭环的确定缺陷与过程记录

修前 `bd94ea3e` 在有限 raw `3e38 -> -3e38` 时浮点差分溢出，实际 ARM 未按 NUMERIC 拒绝。首独审失败保存在 `independent-review/actual_arm_failure_overflow.json`（line154/assert259）；增加合法边界扫描后，原版先通过648条边界检查，仍在同一巨大 raw 反例失败，见 `actual_arm_boundary_scan_before_fix.json`（line160/assert907）。首轮 symbols 夹具问题保留于 `actual-arm-attempt1-harness-symbols/`，另两轮失败目录也保留。旧 `independent-review/actual_arm_results.json` 仍是修前 FAIL，**不能当最终结果**；本报告只引用上述新 final run。

最终 C 对冻结修前 C 的唯一差异，是在差分前按 `anchor ± 10*dt` 检查两个 raw 分量；20..100ms 下这对应已有 ±10000mm/s 分量域，未新增更严合速度门禁。正常速度乘除顺序和后置有限/范围检查未改。独立 ARM 已证实：所有整数 dt 的正负精确分量边界继续通过，巨大 raw 的两个轴/两个方向/20及100ms均拒绝、零输出且状态字节不变；±FLT_MAX 不变 raw 仍正常输出估计零速度，不能把该浮点现象当物理停稳。

过程上曾有提前写入数值预检的临时 run02，随后恢复到冻结 `bd94ea3e`，执行 MATLAB v9/v10 后再重施最终预检。早期临时编码不能冒称 MATLAB 前置已满足；最终修复的补验顺序由 v10 receipt 的产品前后哈希证明。独审核对的是已恢复后的实际补验及最终 C，不删除或洗白早期记录。

## MATLAB 前置证据复核

初始前置 gate 仍以冻结 `independent-review/preimplementation_review.md` 为准：产品首次写入前 v8 740检查/120向量通过。数值修复的最终补验为 **v10 946检查/120向量**，M1=37、M2=12、M3=71、M4=27、M5=241、M6=22、M7=536。本次核对了 v8→v10 的实际脚本差异、数值预检/单精度端点/各整数 dt 边界语义，以及实际日志、summary、向量、量化数据与 receipt 的 SHA 一致性。

v10 本机 MATLAB 9.8.0.1323502 (R2020a) receipt：2026-11-18 01:42:31.929467～01:42:40.161506 +08:00，returncode0；脚本 SHA `31a5fed336e691182e0fc8ca50acbec98fc34269d13e73e56cdf0fd5577def8f`。数值补验前后产品 C 均为修前 `bd94ea3e23abe8d54168b998174620b41e38f9ae44e9eed623e4a4a878fee708`，H 均为上述最终 H。v8/v10的120向量内容完全相同，但本次独立执行明确绑定 v10文件与哈希。独审没有重新运行 MATLAB；这是对实现侧真实本机执行证据的独立复核，不称独立 MATLAB 重跑。

## 合同审查和独立测试覆盖

- prepared-plan 最小外壳、caller六身份/map三身份、NULL输出与可选report、合法绑定和begin重建；坏计划/错身份/旧sample/同戳核心冲突/数值失败保持状态，要求的输出均清零。
- uint32自然回绕、timestamp0、未来/半周期/旧sample拒绝，common与逐源age边界，metadata自然变化；失效清窗，恢复两fresh重建。
- 短窗保anchor但记录latest，重复WAIT/READY和完整Observation缓存，旧sample优先于updates回退、同戳核心冲突优先于updates回退；长gap/计数回退不跨窗估速。
- corrected位置保真一次，速度只用raw，地图heading差分wrap；±10000分量、合法合速度大于10000、±3600角速、NaN/Inf和有限raw溢出；估计均速不能冒称瞬时反馈。
- stop证据run/begin/未来/age校核，cache false新证据仅pending，cache true撤销/旧证据过期/新时间戳不得续命。同sample的stopped不翻转。
- 与真实 `FusionReference_Step` 联验：首窗零Observation不可推进；无stop的估计零速不冒进phase；重复Observation不二次推进；valid stop但估计速度大仍报SPEED_STATE并保持Reference状态；下一fresh真实合同值可恢复，末端只有全部条件符合才DONE。输出仍model_only=1、deployment_allowed=0、calibration_verified=0。

## 规则、限制和下一步

已核 R：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02。相关 E：LOC-02/03/05/06/07、MOT-01/02/03/04、CAL-02，并核 RUN-05/06 的有限恢复/安全出口。20..100ms估速、缓存及copied值接口是工程合同，不是官方规则。Q：FIELD-05、OBS-03、QR-04继续**待技术答疑确认**。

本PASS没有覆盖运行owner、CAN/RTOS动态接入、停机可用真实轮速/HWT反馈、完整雷达bundle、source skew/时延、mixed校准、真实停车点/运输包络/G-F布局、A/J执行制动、真实栈余量、非零过角及双起点整场；没有访问/枚举/轮询硬件、下载、reset或发车。root尚需完成Integrated/Offline的实际M4F双目标工程构建与交接；本独审的portable soft-float ARM执行不替代该构建，更不替代实车验收。
