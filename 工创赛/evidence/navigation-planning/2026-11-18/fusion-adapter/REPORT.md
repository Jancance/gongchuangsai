# fusion_adapter 最终软件阶段报告

日期：2026-11-18。目录 D:/Desktop/工创赛；分支 codex/chassis-navigation-plan。初始本地提交70b230c4，参考层功能基线d023c37a；本轮完整MATLAB初始前置已提交dedd8d56，最终产品阶段提交号见git log。

## 结论与范围

本阶段纯值数据适配软件完成：新增统一工程CHASSIS/USER/fusion_adapter.c/.h，并只向gongxun-f407.uvprojx的Integrated/Offline两目标各添加一份C/H引用。无runtime caller、getter/CAN调用或执行器接入。两目标均编译新源，随后因无caller被链接器移除；整机构建只证明编译兼容，动态行为由独立ARM二进制测试证明，不能称整车已接通融合。

位置直接复制corrected地图坐标，heading直接复制含场地offset的地图heading；估速仅用不含map50% overlay但仍含基础距离标定的raw差分，明确ESTIMATED。固定20..100ms窗、max_age100..1000ms；首窗/短窗不可冒充有效零速，长gap重新建窗。完整身份、模时间、当前逐源健康、同sample缓存和stop证据语义见PLAN及组件报告。estimated_stationary不是物理停稳，stopped只接受调用方已有自动停车证据；没有新增人工比赛确认或停车命令。

旧map50%、主旁系数、控制距离比例、trim、PD、heading owner、Reference及机械/视觉产品均未改。不从差分0、mask0、DONE、ACK、VM或desired0伪造停稳。

## 验证证据

| 检查 | 实际结果 | 权威证据 |
|---|---|---|
| 初次产品编码前MATLAB v8 | R2020a，740检查/120向量，exit0；C/H运行前后不存在 | MATLAB_FINAL_PRECONDITION.md、matlab_receipt_v8.json、冻结preimplementation_gate.json |
| 数值修复前MATLAB v10 | R2020a，946检查/120向量，exit0；运行前后修前C/H保持 | NUMERIC_PRECONDITION_REPORT.md、matlab_receipt_v10.json |
| 最终主ARM C run04 | 2529断言PASS，120向量已包含 | actual_c_checks.json、actual-c-run04/receipt.json |
| 最终high独立ARM C | 2114断言PASS，120向量已包含，8源执行前后同SHA | INDEPENDENT_REVIEW.md、independent-review/actual-arm-final-33dcb514/actual_arm_results.json |
| Vehicle_Integrated Rebuild | 0错误/0警告，编译fusion_adapter.c | integrated_build.log、build_receipts.json |
| Vehicle_Offline Rebuild | 0错误/0警告，编译fusion_adapter.c | offline_build.log、build_receipts.json |
| 保留核查 | 11旧受保护源/原6dirty SHA全部保持；工程LF/非ASCII保持；新C/H ASCII/CRLF | final_checks.json、project_receipt.json、delivery_checks.json |

ARM动态测试使用ARMCC真实产品/Reference/kinematics及实际AppMapPose头布局，在Unicorn执行Cortex-M3 soft-float机器码；无RTOS/getter/硬件provider桩。它不是F407板端或全RTOS动态测试。主2529是实际断言计数而非独立场景数，包含重复边界组；high2114是另一独立夹具，不能相加包装成独立覆盖量。120向量已包含在各自断言内。

两次真实M4F工程Rebuild均从唯一工程派生隔离副本，显式ICM45686_SAMPLE_ONLY=0、APP_CONTINUOUS_MAP_ENABLE=1、APP_MAP_NAV_RESIDUAL_ENABLE=1；未改统一工程默认宏。隔离输出位于工创赛/tmp/fusion-adapter-build-20261118，frozen保存项目/AXF/map/调用图/链接输入及SHA。

## 首错与数值修复

修前C bd94ea3e在raw +3e38→-3e38时，离线ARM软浮点差分溢出后未可靠拒绝；主FAIL252、高独立FAIL259及边界扫描后FAIL907均保留。最终只增加既有±10000mm/s分量域对应的anchor±10*dt差分前预检，正常乘除式及阈值未改。全部81整数窗口正负边界和巨大anchor小差分通过。

历史run02过早应用预检后通过，但未作为最终MATLAB时序合规证据；恢复修前C后实际运行v9/v10，再最终重应用预检并run04。未采用的运算次序候选run03、所有首错、初521及中间模型回执均保留。独审根目录旧actual_arm_results.json仍为修前FAIL，最终结论只指向actual-arm-final-33dcb514子目录。

## 最终源码与资源

- C SHA256：33dcb51411f0fd9cc76ba977187a5d8f9f6aa19a20ad55a7fcf5e6733cc371b9。
- H SHA256：9e344b6d37c10f8b0c69b09d0b552cae0ad677c2a45234fdd7f05c3ca9d8849d。
- ARM sizeof：state172/input168/report92字节（caller分配），AppMapPose128/Observation76字节。不是RTOS真实余栈证明。
- Integrated：Program Size: Code=144588 RO-data=6892 RW-data=3612 ZI-data=124004；AXF SHA256 f7a7b2e4d690678f8197d14f5c0229d3004307da034a7f8a0aaf79fad1798672。
- Offline：Program Size: Code=132780 RO-data=6884 RW-data=3416 ZI-data=123896；AXF SHA256 8d3ed833ffb421ce2fd873859a65eb1fe73b14211bf8741f0931449008d86473。
- 两目标均移除未调用adapter，不能据其最终RAM大小推断接入后的余栈或整车动态开销。

## R / E / Q、未闭环和下一步

已逐项核R FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02。相关E LOC-02/03/05/06/07、MOT-01/02/03/04、CAL-02及RUN-05/06；窗口、估速和缓存均为工程合同，不写成正式规则。Q FIELD-05、OBS-03、QR-04仍待技术答疑确认，无官方新答复，不改规则基线。

尚缺实际owner/持续轮速与HWT瞬时反馈发布、自动stop事实提供者、完整雷达占用/path bundle、G-F布局/真实停车点/运输包络、mixed补偿验收、源延迟/滑移漂移、A/J与轮端制动预算、RTOS余栈和双起点整场。pure值窗口可恢复不代表旧getter同run GAP自动恢复；该机制未改。累计轮/HWT地图仍是有漂移估计。

下一独立软件阶段先在本机MATLAB验证当前v/a、源延迟、向量jerk和轮端能力/制动，再实现A/J执行、接唯一CAN owner，最后非零过角blend。不能将desired cap直接送电机。本阶段未访问/枚举/轮询硬件，未下载/reset/发车，未核读当前板端状态；现场继续暂停。

仅本地完成/提交，不fetch/push。原6dirty与历史无关untracked保留，不混入提交；本stage临时.o/.pyc不作为交付，原始日志/生成证据保持字节不清洗，其空白不作为源码格式修改。
