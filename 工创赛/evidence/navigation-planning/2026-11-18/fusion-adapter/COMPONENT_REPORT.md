# fusion_adapter 组件交付报告

日期：2026-11-18。功能基线 `d023c37a`；本轮 MATLAB 完整前置阶段已由 root 保存为本地 `dedd8d56`。本报告只签组件源码及离线证据，不代替最终独审、唯一工程双目标构建、root 交接或实车验收。当前组件保持 pure copied-value / ESTIMATED / model_only；没有 runtime caller 或执行器接入。

## 1. 交付范围和接口

仅新增实际统一工程 `3.stm32f407工程/CHASSIS/USER/fusion_adapter.c/.h`。接口是 `FusionAdapter_Reset` 和 `FusionAdapter_Observe`：调用方先复制完整 `AppMapPose`，显式绑定六字段 `FusionIdentity`，传入 caller now 和独立 stop 证据。prepared plan 在绑定期间不可变，state 单 owner；非 NULL 指针有效且输入输出不别名由 caller 保证。可选 report 为 NULL；能安全写的 Observation 在 WAIT/拒绝时清零。

- 输出位置直接取已补偿 map.pose.x/y，heading 直接取地图 heading；不再次叠加 correction，不用 raw HWT yaw 替换地图坐标朝向。
- 速度仅用不含 map50% overlay 的 raw XY 在20..100ms有限窗差分；raw 仍含既有距离基础标定。角速度由地图 heading wrapped 差分取得。它们是 ESTIMATED 窗均值，不是真地速、瞬时角速或制动保证。
- 首样本/合法新绑定/begin变化/非倒退样本的 updates 回退只 prime；短窗保存 latest 但保留 anchor，长窗重新 prime，不跨 GAP 估速。身份、plan、旧sample和同sample核心冲突拒绝且不改状态；明确 map 健康失效清窗。
- 同sample复读完整 Observation，仅 now 可变；每次仍重验当前计划、身份、地图/逐源健康。metadata自然变化只影响当前report。当前map失效不能靠缓存续命。
- stopped只来自 caller 明确、同run且时效合法的stop事实；差分为0不能生成stopped。缓存stopped=0时新stop只pending；缓存stopped=1时原证据过期/撤销则整份Observation不可用，不能用新stop时刻延长旧缓存。estimated_stationary仅是report。
- 保留 Reference 既有分量±10000mm/s、角速±3600deg/s域；新增 raw 对 anchor±10*dt 的预检，在巨大差分减法溢出前拒绝。正常速度表达式仍为 delta*(1000/dt)，没有采用未证实必要的运算次序改动。数值范围及MATLAB时序见 `NUMERIC_PRECONDITION_REPORT.md`。

组件没有 getter、CAN交易、RTOS任务、运动命令、实时HWT分支、measured轮速、雷达bundle、停止策略或永久锁；不修改旧map50%、四向系数、PD、比例、heading owner或Reference产品源。layout/path/direction是caller绑定，不能声称由map/雷达回显。

## 2. 当前源码与文件格式

| 文件 | SHA256 |
|---|---|
| fusion_adapter.c | `33dcb51411f0fd9cc76ba977187a5d8f9f6aa19a20ad55a7fcf5e6733cc371b9` |
| fusion_adapter.h | `9e344b6d37c10f8b0c69b09d0b552cae0ad677c2a45234fdd7f05c3ca9d8849d` |
| fusion_reference.c（未改） | `07a9439f0435cb8ce35fc045b0a74f17e07e0362549336859760ae5921d02fb2` |
| fusion_reference.h（未改） | `8e0d855ae91a5ae6bc856aaf6af2300cd24ccdf8d91d251294087466fa0ab961` |

adapter C为8035字节，H为2398字节，均ASCII/CRLF。`component_handoff_checks.json` 于01:51核对当前产品与run04源码快照逐字相同；run04全部7个输入源hash仍匹配。11个旧产品源相对preparation保持，原6个dirty逐文件SHA保持。唯一uvprojx在此检查时仍为preparation原SHA；其后工程引用由root负责，不列入组件擅自改动。产品结构尺寸：state172/input168/report92字节，来自实际ARM编译，不能当作RTOS真实余栈证明。

## 3. MATLAB先验和保留的历史

- 初始产品编写前，本机MATLAB R2020a实际v8通过740检查/120向量（01:32:22.886335~01:32:29.791944，exit0）；`preimplementation_gate.json` 于01:34:31放行，明确C/H均不存在。脚本SHA `81888abffe3faee2fec80cfca6c5509bf02d361f2f6cf84cb4728522a1dcd356`。M1..M7范围见 `MATLAB_V7_REPORT.md`、`MATLAB_FINAL_PRECONDITION.md`。
- 编码后巨大raw反向差分在实际ARM首验暴露数值缺陷。早期修复run02虽通过1521断言，但补充数值MATLAB先验尚未完成，不能拿它作为最终时序合格证。原C被恢复并保留为 `before_numeric/fusion_adapter.c`；过早修复快照留在run02。
- 在原C `bd94ea3e...fee708` 保持不变时，实际MATLAB v9通过784检查，随后v10通过946检查/120向量（01:42:31.929467~01:42:40.161506，exit0）。回执逐项记载运行前后原C/H同SHA。之后才最终重应用预检修复并执行run04。
- v10分组M1=37/M2=12/M3=71/M4=27/M5=241/M6=22/M7=536；脚本SHA `31a5fed336e691182e0fc8ca50acbec98fc34269d13e73e56cdf0fd5577def8f`。v8~v10的120行向量均为SHA `31e85fe56dd8e6c2501bfe6241a977c0613a84ac619155b00cc969976a82ee8b`。初521及v2..v10脚本、回执和首错不覆盖。

MATLAB检查覆盖六身份/三map身份、prepared外壳、uint32回绕/零时间/半周期、逐源健康、完整缓存、updates冲突优先级、短窗/长gap、stop不可续命、raw/corrected分离、四向审计、量化和有限窗减速滞后；不是RTOS或硬件测量。

## 4. 实际ARM C验收

`check_adapter_c.py` 使用实际ARMCC编译adapter、真实Reference和chassis_kinematics，以Cortex-M3 soft-float ARM机器码在Unicorn执行；使用真实AppMapPose头文件，没有RTOS/getter/执行器provider。该隔离环境不是STM32F407板端运行，也不是完整工程构建。

| 运行 | 源C SHA前缀 | 结果及用途 |
|---|---|---|
| run01，01:37:26~39 | bd94ea3e | FAIL，252断言，unit第97行；raw +3e38→-3e38应拒绝但没有。原源/单元/机器码/回执保留 |
| run02，01:39:48~01:40:02 | 33dcb514 | PASS，1521断言；过早修复的历史，不代替之后MATLAB补验 |
| run03，01:43:17~36 | d8f59ae6 | PASS，2025断言；含不必要运算次序候选，仅保留证据，未采用 |
| run04，01:43:54.457102~01:44:13.513526 | 33dcb514 | PASS，2529断言/120 MATLAB向量；最终采用源码同SHA |

`actual_c_checks.json` 与 `actual-c-run04/receipt.json` 内容一致。2529是实际执行的断言计数，不是2529个独立场景；夹具内81个dt×正负边界和大anchor小差分组重复两遍，重复不作为新增覆盖。MATLAB向量在计数内，不应再把120相加称2649断言。

实际C还覆盖NULL/可选report、Observation失败清零、state逐字不变、plan/input逐字不变、源失效恢复、同sample缓存、raw巨大差分、分量合法合速度大于10000、全部81个整数窗正负边界、巨大anchor可表示小差分。真实Reference联验覆盖首窗不可用、重复不推进、零差分无stop不DONE、valid stop但仍高速不能推进初相位、恢复不跨gap。独立审查首轮巨大差分FAIL259与边界扫描后FAIL907也保留，不能把失败独审回执说成最终通过；最终high同SHA结论由root另收口。

## 5. 规则、未闭环和移交

核对R：FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02。start_id、场地F/mm和有向路径身份均保留；本组件没有规则物理验收或运行能力证明。相关E：LOC-02/03/05/06/07、MOT-01/02/03/04、CAL-02，以及RUN-05/06的有限恢复/出口原则。20..100ms、估计速度和cache策略均为工程选择。Q：FIELD-05、OBS-03、QR-04继续“待技术答疑确认”，无新官方答复、不改基线。

工程未闭环：真实停车点/运输包络/G-F布局、完整雷达bundle、停机仍可用真实轮速发布、真实source skew/时延、mixed校准、A/J制动和轮端预算、RTOS余栈、业务等待、双起点整场及物理停稳。后续先完成适用MATLAB再实现对应A/J和唯一CAN owner接入；不能把本报告当作这些能力已完成。

本收尾只新增/补充stage证据；不fetch/push、不Git写、不访问/枚举/轮询硬件、不下载/reset/发车。root负责最终独审结论、唯一工程引用/Integrated与Offline构建、根交接与本地保存；本组件未上传GitHub。
