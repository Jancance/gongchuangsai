# 2026-11-16 三趟前向 BM 离线重放小计划

目标：判断今天三趟 ICM45686 RAW 经 Mahony Kp=0.5、Ki=0 与编码器融合 BM 的前向距离是否有足够证据称准确。BM 为主，B0 为旁证；横移以后。

1. 输入仅 inertial-field-run1、inertial-field-run2、inertial-field-run3-retry1 各自 capture/decoded.json，完整约200Hz RAW（1810/1810/1813条），按本趟trigger绑定。第三趟首失败目录保留，不作第四趟。不触硬件/控制，不fetch/commit/push，保留全部dirty。
2. 复用2026-11-15/fusion-b-mahony-comparison/compare_mahony_b.m的front、prepare_wheel、filter_intervals及frozen_plan原参数。固定Kp=.5/Ki0、轮约束interval=.2s/tau=.3s/history=.5s；不端点拟合、不补STOP零速、不臆造RAW/HWT后IMU、不按尺量选窗。verify：冻结输入/算法/参数SHA，运行用户电脑MATLAB，B0/BM同有效栅格并保留必要一致性检查。
3. 每趟输出前向BM/B0端位移、端速度、末0.5s位移变化与速度范围；列RAW末/HWT末/四轮括窗末/共同输出末/最后轮约束时刻，以及STOP、serial首次software_complete(DONE)、最终轮零时刻。verify：按原时间字段核同钟，接收时间标时序边界；量化共同窗及RAW末至后续轮零平台的可用轮程，不补入BM。STOP8.051/8.051/8.062s不等于地面停稳；RAW约8.546/8.546/8.559s结束，串口轮终点更晚，先判窗是否覆盖软件停稳。
4. run1尺量2100mm、run2尺量2102.5mm；测量点、手量不确定度和共同末窗对齐未知。只报BM共同窗末减最终尺量的表观差，明确混入时间窗差；run3无尺量，只报重放值/重复性。verify：不虚构误差区间，不称毫米级精度、BM优于B0或地面精度已验。

交付MATLAB脚本、实际回执/日志、机器可读结果、简短报告及前向时序图。执行交forward_replay，独立审查核端值、时间窗和结论；向用户仅展示关键汇总，异常才展开RAW。此文仅规划，无固件实现。

已读根长期记忆、ICM高频三趟接续顶部、两级AGENTS及规则基线。R已核RUN-01/PWR-01/MECH-01/MECH-02；E已核LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04仍待技术答疑确认。本次不改规则或签四向定位闭环。未闭环：同窗地面参考、尺量不确定度、独立地面停稳、第三趟尺量及横移/后退/旋转精度。BM仅MATLAB，未上车。