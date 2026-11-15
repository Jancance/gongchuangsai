# ICM辅助编码器/横向计算：冻结的小范围离线检查

2026-11-15。只用同half固件half-stop-run5/6/7，尺量已公开，仅作描述性比较，不称盲测或据终点调参。无固件/参数/控制写入，不连接硬件，不发车。

1. 重解原始raw的IR1 CRC及配置/采样时间/序号，审计真实约5Hz导出与200Hz采样间缺失；导出每轮原始计数/采样时刻与HWT角/角速率。固件源、输入及派生文件绑定SHA。
2. 本机MATLAB执行：按活动代码符号/轮序/前进比例重建编码器横向，分开“前进×偏航”和“麦轮横移”贡献；按安装记录sensor X右/Y后/Z下转换到body X前/Y左/Z上。每趟只用run_tick之前2.2～0.2s静止窗口求基线均值，不用停后数据校正前段。
3. 比较固定基线的平面加速度双积分、含HWT平面旋转/安装偏置r=(-20,+90)mm补偿的双积分、与编码器速度的简单互补估计（固定tau=0.1/0.3/1/3s，全部公开，不选终点最优参数）。不做终点强制零速/终点拟合/距离系数调节。平面模型不处理动态俯仰横滚；停后均值只诊断。
4. 重采样/中心差分/跨通道插值均是离线对齐，可用未来样点，不能声称在线已实现。直接积分仅固定起前均值，也不声称重建了缺失200Hz信号。HWT航向仍权威，不用本研究改变陀螺符号或参与控制。
5. 合成常加速度/零运动/纯前进转向及杠杆臂检查通过后，跑三趟实日志，输出曲线、灵敏度（1LSB/1mg/0.1°重力投影）及原始指标。验收是计算可复现/单位时间符号自洽、局限公开，不要求筛出能部署的融合候选。

R RUN-01/PWR-01；E LOC-01/02/03/05/07、MOT-02/04、RUN-06；Q FIELD-05/OBS-03/QR-04继续待技术答疑确认。现有前向测试与终点尺量不能闭合完整横移/地面沿程验证。

主要官方源：TDK DS-000577 rev1.0（±4g=8192LSB/g；±1000dps表列32.8LSB/(°/s)）；AN-000257用于偏置/重力投影概念。实际配置以源码/IR1为准，额定规格不当整车精度。

## Explicit follow-up after planar results

Planar integration failed on all three observed runs. Before concluding about ICM usefulness, add one fixed, untuned gyro-gravity transport model using all three gyro axes, initial gravity direction from pre mean, and existing HWT yaw/planar lever correction. No accelerometer feedback during motion, no endpoint correction, same fixed tau list. Unknown transverse accelerometer bias, installation height, sparse gyro loss, and future-sample alignment remain explicit. This is exploratory after seeing planar results, not a pre-registered held-out validation. Preserve planar output separately.
