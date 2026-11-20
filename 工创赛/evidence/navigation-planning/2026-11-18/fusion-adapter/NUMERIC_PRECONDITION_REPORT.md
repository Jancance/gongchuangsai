# 数值修复追加MATLAB前置与采用范围

日期：2026-11-18。本说明补充最初v8产品编写放行；不改写其历史回执，也不把MATLAB等同实际ARM或物理验收。

## 1. 确定缺陷与最小修改

初C `bd94ea3e23abe8d54168b998174620b41e38f9ae44e9eed623e4a4a878fee708` 在raw anchor为+3e38、20ms后raw为-3e38时，没有按合同返回零Observation/状态不变的NUMERIC拒绝。主实际ARM run01在第97行/第252个断言失败；独立实际ARM在对应反例第154行/259断言失败。初源存于 `actual-c-run01/fusion_adapter.c` 与 `before_numeric/fusion_adapter.c`，机器码及回执均保留。

本阶段既有速度域是每个XY分量±10000mm/s；20..100ms窗内等价位移域为每分量±10*dt mm。最终C只在减法前比较当前raw是否越过anchor±10*dt，避免已知巨大反向差分先溢出后软浮点未按合同拒绝。保持原后置finite/分量/角速检查，不提高或降低既有速度阈值，不增加合速度门禁，不限制raw绝对坐标，不改变正常估速的 delta*(1000/dt) 次序。它是纯适配Observation可用性检查，不是新增电机停车策略。

MATLAB有限浮点表达式与ARMCC软浮点库不是同一执行环境；前置只验证模型/边界预期，之后必须用实际C交叉。本说明不声称覆盖全部IEEE浮点状态，也不签署真实定位精度。

## 2. 恢复原源后先验的可核时序

| 阶段 | 实际时间（+08:00） | 证据 |
|---|---|---|
| v8首次产品编写前模型 | 01:32:22.886335~01:32:29.791944 | 740检查/120向量、exit0；前后C/H不存在；gate在01:34:31放行 |
| 过早预检修复run02 | 01:39:48.715062~01:40:02.246827 | PASS1521，但追加数值MATLAB先验尚缺，因此保留为历史，不作为最终时序合格证 |
| 恢复原C后MATLAB v9 | 01:41:48.585418~01:41:55.447476 | 784检查/120向量、exit0；运行前后C=bd94ea3e，H=9e344b6d |
| 原C仍保持时MATLAB v10 | 01:42:31.929467~01:42:40.161506 | 946检查/120向量、exit0；运行前后C/H仍同上 |
| 未采用的运算次序候选run03 | 01:43:17.785527~01:43:36.685969 | C=d8f59ae6、PASS2025，快照留在run03；不代表最终采用 |
| 恢复正常运算次序的最终run04 | 01:43:54.457102~01:44:13.513526 | C=33dcb514、H=9e344b6d，PASS2529断言，120向量在计数内 |

v9/v10本机版本均为 `9.8.0.1323502 (R2020a)`，实际可执行文件 `E:/Program Files/Polyspace/R2020a/bin/matlab.exe -batch`。`matlab_receipt_v9.json` 和 `matlab_receipt_v10.json` 明确保存原C/H运行前后完整SHA；两份快照相等，可证明最终重新应用修复前模型实际运行期间原源未被替换。run02过早写入的历史不隐藏，也不倒填成MATLAB在前。

## 3. 数值追加检查

v9在v8的740检查上追加44项（M5=79）：已float量化的0、±300、±1e8、±3e38七类anchor，对-200/0/+200mm请求保留可表示差分，检查既有分量域和anchor预检；极大anchor可能只能表示0位移，不将此当物理零速或精度证明；另验巨大反向差分必须拒绝且state不变。

v10追加162项（M5=241），遍历dt=20..100每个整数窗、正负分量边界，导出 `numeric_boundary_scan_v10.csv` 对照两种single运算次序；模型拒绝/接收预期保持。MATLAB single因求值/舍入次序显示的边界担忧，不能直接当作实际ARM失败：独立原版实际ARM边界扫描已通过该部分，随后才在巨大raw溢出反例失败；因此最终没有采用run03中“先乘1000再除dt”的修改。最终主run04也覆盖81窗正负边界和大anchor小差分。

| 产物 | SHA256 |
|---|---|
| verify_fusion_adapter_v9.m | `6722d69cf221dddac241240bc08b437cec9ecf667f67c9365e5cf11e2a37f76e` |
| verify_fusion_adapter_v10.m | `31a5fed336e691182e0fc8ca50acbec98fc34269d13e73e56cdf0fd5577def8f` |
| adapter_vectors_v10.csv（与v8/v9相同） | `31e85fe56dd8e6c2501bfe6241a977c0613a84ac619155b00cc969976a82ee8b` |
| 最终fusion_adapter.c | `33dcb51411f0fd9cc76ba977187a5d8f9f6aa19a20ad55a7fcf5e6733cc371b9` |
| 最终fusion_adapter.h | `9e344b6d37c10f8b0c69b09d0b552cae0ad677c2a45234fdd7f05c3ca9d8849d` |

`component_handoff_checks.json`复核v9/v10脚本、日志、summary、vectors、quantization均与回执hash一致；当前产品与run04快照一致。总数946为模型检查计数；2529为实际C断言计数，包含夹具重复边界组，不是独立场景数，也不与120向量重复相加。

## 4. 权限、规则与剩余工作

所有上述MATLAB/ARM均为本机离线。无硬件访问、枚举/轮询、下载、reset、运动、runtime caller、CAN交易或GitHub同步。根交接和工程引用由root另做，high最终同SHA独审由root另收口；本说明不抢先签最终独审/双目标通过。

R已核FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02；本次数值域与finite-window方案属于E，相关LOC-02/03/05/06/07、MOT-01/02/03/04、CAL-02、RUN-05/06。Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。真实源时延、漂移、量化、mixed校准、物理停车、A/J、轮端预算和双起点整场仍未闭环。
