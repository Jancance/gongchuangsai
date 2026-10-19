# 硬件与 STM32 接口合同

> 2026-10-06开发主责：[当前两组分工](GROUP_RESPONSIBILITIES_2026-10-06.md)。视觉组负责夹爪、270°三槽盘、机械臂动作组及搬运相关STM32模块；电控组负责公共F407平台、底盘与定位、雷达和连续运动。下文Orange Pi/STM32指观测端与执行端；执行模块使用统一通信/状态/定时器管理，工作包归属不改变物理接线。

> 2026-09-26当前变更：用户取消PAW3395和MaixCAM2；180°舵机升降雷达、ULN2003步进转动雷达；USART3改雷达，取消三夹紧及三占用检测旧用途，PB1保留270°三槽转盘。初始化成功蜂鸣次数可改。下文涉及旧硬件的内容仅供追溯，当前接口及待确认项以[雷达与手动修改计划](RADAR_AND_MANUAL_P1_PLAN_2026-09-26.md)和更新后的规则基线为准。外部F103工程不改，后续由用户手动修改，代码示例使用中文注释；实时X/Y定位尚未闭环。

> 2026-09-12载物机构变更：PB6/7/8释放、PB1改为270°车载载物转盘控制候选、PB9保留夹爪；摄像头固定。详见[转盘计划](ROTARY_CARRIER_PLAN.md)。新增转盘状态载荷仍为规划，未修改现有命令字和线上协议；动作到位确认、手臂互锁、库存验证按该计划迁移。

## 职责边界

- Orange Pi：任务码、物料/障碍识别、坐标转换、任务与路径规划。
- STM32F407：电机与机构通信、HWT101、雷达/扫描机构、待冻结X/Y位姿估计、吊塔归零限位和最终运动安全；PAW已取消，ToF首版不接入。
- Orange Pi 只发送速度/位姿/动作目标，不发送裸 PWM。

主通信物理层计划使用CH340 USB-UART：Orange Pi通过USB Host访问稳定的`/dev/serial/by-id/`或按USB端口固定的设备路径，CH340 TTL侧连接STM32 USART6 PC6/PC7。板上UART6 3.3V直连接口保留为救援备用，两条通道通过硬件选择避免双发送端冲突。CH340的TTL输出电平未实测为3.3V前不得连接STM32 RX。

## UART 帧

```text
A5 5A | version:u8 | payload_len:u16 BE | sequence:u16 BE |
command:u8 | payload | CRC16-CCITT-FALSE:u16 BE | 0D 0A
```

- CRC覆盖 `version` 到 `payload`。
- 最大载荷 512 字节，当前协议版本为 1。
- STM32解析器必须处理噪声、半帧、粘包、坏长度和坏 CRC。
- 新运动命令先返回 `ACCEPTED`；动作完成后返回 `DONE`；拒绝或故障返回 `ERROR`。
- STM32缓存最近至少 128 个已完成序号及结果；同一序号重发只返回缓存结果。
- 超时后 Orange Pi先查询状态，不能直接换新序号重复动作。

命令值已在 `src/smart_carrier/protocol.py` 固定。STM32 实现必须用相同的黄金测试向量验证。

## 心跳与失效

- STM32 200ms未收到有效心跳：平滑减速。
- 500ms未收到有效心跳：停车、禁止驱动、执行机构进入安全状态。
- 后续实际装配并验收ToF后，小于停止阈值时 STM32 可越过 Orange Pi命令直接停车；首版无ToF时不得伪造距离有效值。
- 上电默认电机禁止；断线恢复后不得自动重放历史命令。

## 坐标

- `camera_px`：图像 u向右、v向下，仅用于视觉内部。
- `robot_mm`：底盘旋转中心为原点，x向前、y向左、yaw逆时针为正。
- `field_mm`：赛场固定坐标；原点、轴向和 `start_id=1/2` 初始位姿必须配置化并冻结版本。
- `camera_ground`：相机工作平面毫米坐标；到 `robot_mm/field_mm` 的变换必须携带标定版本。
- STM32电机计数不暴露给视觉层；轮径、减速比、每转计数和轴符号保存在 STM32 参数区，仅用于各轮速度闭环和故障诊断。

## 原料盘高度与跨组合同（2026-10-02，待实现）

原料盘上表面离地高度由P1集中配置，用户现场准备覆盖80～100mm；车载270°盘与雷达高度独立。P2工作平面/标定必须匹配同一高度与geometry_revision，取放前核对source_id、work_plane_id、calibration_id、坐标单位/时钟/有效期；P1按机构零点和工具偏置转换目标Z并检查可达性。当前帧封装/命令码不变，相关高度载荷、校验和固件仍待实现，详见[高度及视觉整合计划](P1_P2_HEIGHT_AND_INTEGRATION_PLAN_2026-10-02.md)。高度或标定变化使旧目标失效；接口不匹配拒绝抓放，不将PWM命令或ACK当完成。

## PAW3395＋HWT101定位合同（历史，2026-09-26已取消PAW）

下述PAW字段保留历史追溯，当前不得作为必装器件或有效反馈要求；实时X/Y来源、HWT与视觉纠偏按LOC-01～07另行验证。

- 项目不使用ICM42688，不采购OPS-9作为主定位器件。
- PAW3395通过SPI接入STM32，负责车体平面相对位移；优先使用Motion Burst并同时读取 `delta_x/delta_y`、`SQUAL`、`Shutter`、`Lift_Stat` 和时间戳。
- HWT101通过UART接入STM32，作为唯一主航向传感器；只使用航向角和角速度，不使用加速度二次积分估算X/Y。
- STM32完成PAW安装偏角、X/Y比例、轴符号、非正交和旋转中心偏置补偿，再按HWT101中点航向积分到 `field_mm`。
- PAW/HWT数据过期、离地、表面质量不足、异常增量、校准ID不匹配或STM32复位时，`pose_valid=false`，停止盲积分并触发减速、停车或视觉重定位。
- Orange Pi只发送带时间戳、坐标系、标定ID、不确定度和来源的视觉位姿修正；STM32必须拒绝过期、跳变或坐标系不一致的修正。

定位上报至少包含：

```text
timestamp_ns, pose_seq, x_mm, y_mm, yaw_rad,
vx_mm_s, vy_mm_s, yaw_rate_rad_s,
paw_squal, paw_shutter, paw_lifted, hwt_status,
pose_valid, pose_age_ms, localization_source,
localization_calibration_id, rejection_reason
```

完整机械、标定、融合和替代门禁见 [PAW3395＋HWT101底盘定位专项计划](LOCALIZATION_PAW3395_HWT101_PLAN.md)。

## P1引脚与限位

STM32F407VET6的CAN、串口、PAW3395、舵机、启动键、吊塔原点、载物盘传感器及扩展口统一按[P1引脚资源规划](P1_STM32F407VET6_PIN_PLAN.md)执行。35升降轴使用PE8原点，28第二轴使用PE10原点；回零后依靠闭环编码器、软件行程和机械缓冲运行。DM4310先以绝对角度实物门禁决定是否免回零，PD3保留可选原点输入但首版DNP。

## ToF预留

首版不接入三个VL53L1X，相关I2C、XSHUT和INT只保留DNP焊位与测试点，软件状态固定为`NOT_PRESENT`。后续启用时，左/中/右分别使用独立XSHUT完成地址分配，上报距离必须包含有效标志和时间戳；超量程、信号弱或过期数据不能当作“前方无障碍”。启用前还必须处理与现有限位/按键信号的EXTI共享问题并完成黑色障碍实测。
