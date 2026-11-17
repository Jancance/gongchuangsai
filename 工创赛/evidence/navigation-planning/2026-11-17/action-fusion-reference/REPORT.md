# 动作融合独立参考层交付（2026-11-18）

基线9fce01db；本阶段本地提交号见git log。目录沿用11-17/action-fusion-reference作为同阶段连续证据。用户已确认的访谈策略保持，本阶段先完成pure reference，不把尚无执行调用的模块称为整车动作融合已接通。

## 完成范围

新增活动CHASSIS/USER/fusion_reference.c/.h，并精确加入唯一gongxun-f407.uvprojx的Vehicle_Integrated和Vehicle_Offline两个目标；没有增加运行调用、RTOS任务、CAN交易、电源/许可、机械臂/视觉或雷达操作。旧地图50%、四向控制倍率/trim、航向PD保持。

Prepare/Reset/Step接收不可变有向场地折线、实际位姿/速度、run/start/map/layout/path身份及源时刻，输出当前段signed进度、横偏、沿原合法路径的剩余弧长、绝对heading和受轮速预算限制的desired世界/车体/四轮参考。目标不随实际欠转或当前位置重设，不直连终点穿黄区。

已实现：

- 32点/32附加障碍盒上限，超限拒绝不截断；2400场界及四黄色块内置。用固定heading矩形支持函数区间极值生成保守AABB，检查整段平移、actual姿态、回线段和短预览；障碍输入/车体尺寸/误差预算是调用者显式模型参数。
- 只顺序推进当前段及相邻同向共线段，可一帧跨过多个短共线节点且不要求命中端点小球；遇方向突变先真实停稳，不global-nearest跳到自交另一分支。普通共线节点保持非零desired；未实现非零几何blend。
- 后向传播下游低速/停止要求，持续按原路径remaining给沿程速度建议；保持原计划heading，以有界横移修回路线，实际heading用于world/body转换。总数学参考不超400mm/s，先扣heading轮速预留，再真实Chassis_Inverse同比限幅并回算已兑现参考。
- 沿程vt与横向vn分别检查，末端沿程cap0仍允许合法横向回线；严重沿程/横速及总速仍请求恢复。remaining0不等于done，终点位置、实际heading、速度及caller真实stopped要同时满足。
- 0/90/180绝对目标独立，90起点、180头尾90。旋转当前采用整圆外接框保守验证；可行时CW优先，否则拒绝。它不能精确区别仅CCW可行的停车空间，因此精确CW失败后CCW fallback尚未签署；不会强行在不确定空间旋转。
- 同sample按完整参考缓存重复返回，先查age/身份/健康；同timestamp但内容冲突拒绝。坏输入/恢复不消费状态，fresh合法后可继续；没有新增永久故障锁。Prepare失败不保留旧plan.valid。

## 实际验证与证据

本机MATLAB R2020a最终实际9034检查通过，SHA23073cb5d4914ac30c3b0064cff64d277043f20d2d0cb9b55150047df2ef4f76；最终00:22:19~26 exit0，回执绑定修前产品SHA，vt/vn模型先于对应C修复。历史8695/9010/9028各有副本；不能用初8695的代数检查代替后来状态机/预算前置。

主检查真实ARM C1364断言及96组MATLAB预算向量通过；编译实际fusion_reference/真实chassis_kinematics，ARMCC Cortex-M3 soft-float机器码由Unicorn执行，无RTOS/执行器桩。独立高审另在独立目录复制冻结源重新编译/执行1328断言、96向量通过，修前末端横移误判1312断言成功复现并留证。两套检查数量不同，绑定同一最终源，不混称一套总数。见actual_c_checks.json、independent-final/actual_c_checks.json、INDEPENDENT_REVIEW.md。

最终源SHA：

- C：07a9439f0435cb8ce35fc045b0a74f17e07e0362549336859760ae5921d02fb2
- H：8e0d855ae91a5ae6bc856aaf6af2300cd24ccdf8d91d251294087466fa0ab961

两真实目标均ICM45686_SAMPLE_ONLY=0、map及50%候选显式1，Keil-r重建0errors/0warnings，日志确有compiling fusion_reference.c。由于尚无运行调用，链接器移除了该模块未使用函数；证明的是编译兼容，实际模块执行由上述ARM测试证明，不冒称主程序已调用。未使用-f，未下载。

| 目标 | 静态大小 | AXF SHA256 |
|---|---|---|
| Vehicle_Integrated | Code=144588 RO-data=6892 RW-data=3612 ZI-data=124004   | 344bd4c07befc45452d81b412c36bf395f8f8d57d000edf3a1d8640cc3601c46 |
| Vehicle_Offline | Code=132780 RO-data=6884 RW-data=3416 ZI-data=123896   | ea30213a6100b17bdcfce4d84580503b61738164563c17fd8976dd6cd9955a94 |

最终final_checks核14个受保护旧文件、原6dirty SHA保持；工程只新增20行源引用，原UTF8/LF/非ASCII字节保持；两新源ASCII/CRLF。配置和旧控制器不改。FusionPlan/Input/State=1408/1012/120B，调用者分配，无静态大对象；不得放进已有窄任务栈而不另核。当前映像因无运行调用未新增该上下文，不把sizeof当真实栈余量。

## 发现并修复

1. 同源sample在阶段跳转后重复调用会输出不同参考：增加完整缓存及同timestamp内容一致性，bad/stale检查先于缓存。
2. 共线节点要求源恰好落进端点球，高速离散采样可能跳过而误恢复：改为有界、相邻、同向共线归一化，不跨折角。
3. 原用总反馈速度比较沿程cap，末端正常横移15mm/s会被误判：MATLAB与独立实际C都先复现首错，随后vt/vn分开，margin未放宽，严重400沿程/100横速仍拒绝。

保留draft_before_cache、before_lateral、lateral_failure与独立修前目录。早期夹具节点索引/几何端点错误、链接器移除测试size符号/UTF8读取错误及首次project原LF与脚本CRLF不符均保留；没有降低产品几何/源保护来制造通过。首次未加入新源的构建虽然0err，但compiled_reference=false，未算最终通过。

## 尚未完成及下一步

始终model_only=1/deployment_allowed=0。输出是desired速度及理想加速度hint下的cap，不保证速度/加速度/加加速度连续、真实制动或轮端动态跟随；轮速reserve仅数学余量。完整角格不停顿blend、精确CW/CCW空间选择、地图/有向雷达路径adapter、A/J有界实际执行和业务状态机尚未接。

下一软件小步先读原子AppMapPose及真实反馈构造身份/时间一致的只读adapter，并明确小幅横移/实际yaw导致混合速度时四向50%主方向候选的覆盖，避免两个分量叠加或静默全禁。之后用MATLAB先建当前v/a、源延迟和轮端余量的jerk-limited执行，再接唯一CAN owner；最后才做非零几何过角。停车点/运输包络/设备G-F映射未冻结处只作显式参数模型，不造实测。

硬件继续暂停，未下载/reset/发车、未fetch/push。实际精度、滑移/延迟、停车尾、混合补偿、余栈、负载与双起点3分钟整场均待新明确实测。现场运动后直接保存RAW要求保留，不自行循环重试。

R已核FIELD-01/02/06、OBS-01/04、RUN-01/02/03/04/07、PWR-01、MECH-01/02；E FIELD-08、LOC-02/03/05/06/07、MOT-01/02/03/04、CAL-02；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。算法/参数/400上限及保守几何为E，不冒充规则或实车验收。
