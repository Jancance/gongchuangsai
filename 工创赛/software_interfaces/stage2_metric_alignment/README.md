# 原料区毫米视觉观测接口（软件候选）

本接口仅接受、保存毫米偏差。**没有运动执行器、对准完成标记或UART驱动绑定**。不与现有`AA/CC`像素视觉协议混用；没有将新增模块导入视觉main.py，也未安装到D盘实车工程或烧录。当前仍运行之前的第1→2段距离联调版。

## 调用位置与状态

现有实际`VehicleMission_ReadAlignmentRequest`只在第2段停车、机构持位、无故障且等待毫米视觉时返回请求。本候选`metric_alignment_bridge.h`直接使用该头文件声明；测试时此函数由模型提供，接口必须在单一任务中调用。

1. 视觉目标/相机/内外参/安装姿态/工作平面高度经实测验证后，在版本化profile中填入非零ID；当前`profile.unconfigured.json`的calibrated=false/ID0，C端拒绝发起观测事务。
2. 调用`MetricAlignmentBridge_Open(inbox, profile, unique_request_id, now_ms)`。从现有第2段接口复制本轮run/stage/start/map/goal/名义目标，事务最大1000ms。request_id由未来适配器分配，**跨重启也必须唯一**（持久化epoch或经确认的nonce）；当前模块不生成nonce，不宣称具备未实现的掉电抗重放。
3. 将request结构传给未来传输适配器。请求线上编码/握手、物理串口归属与接线仍待接设备确定，当前未发送请求。SBC必须在收到该请求后取到新的、确认采集时间的图像，再应用匹配版本的毫米变换。
4. Python `MetricObservation`→`encode_observation`得到MA1帧，未来接收适配器完成有界分帧后调用`MetricAlignmentBridge_Submit`。这个函数只校验、保存观测，不执行底盘微调。
5. 使用结果前调用`MetricAlignmentBridge_Read`，会再核对当前轮次/停车/等待状态和帧年龄；取消、进入新轮、执行器启用或请求过期都会阻止读取。返回MA_OK只表示观测通过，不表示已对准或允许抓料。

旧像素程序和发送协议保持。毫米变换尚未接入，所以不能直接将旧X/Y像素强制填入本结构。配置里的示例限制是工程候选，不是本车精度或位移安全验收。

## 坐标、版本和时效

坐标为车身当前姿态下X向右、Y向前，正yaw逆时针；dx/dy/yaw表示视觉计算的**所需车身修正量**，不能混入“目标相对相机的位置”。计算这个修正量需要明确目标定义、相机到车身变换及停车期机械姿态。停车目标(2060,1200)是场地名义点，不是相机像素目标。

相机序列号/镜头/分辨率/裁剪/畸变流程映射到camera_id/calibration_id；机械臂/镜头安装姿态映射mount_id；实测工作平面高度映射height_id；视觉目标定义映射target_id。任一变化都更新对应ID并使旧标定失效；当前不替这些实测生成合法ID。

frame_sequence为该事务内严格递增的图像序号，不能用重复串口请求伪造新图像。capture_after_request_ms为SBC本地“收到请求→图像实际采集”的间隔，不是SBC或MCU绝对时刻；frame_age_ms为图像采集至发送时的已知年龄。C端额外用MCU事务已用时间减capture_after_request_ms作为保守年龄上界，覆盖传输积压，再与源报告年龄取大；不直接相减两台机器的启动时钟。

采集时间元数据必须可靠。当前project_profile中的host_read_completion不能证明曝光时间；相机缓冲和采集延时还未闭环前不将calibrated切true，也不宣称帧新鲜。基于请求起点的往返期限只提供通信约束，不能证明生产者诚实报告了图像时间。

## MA1观测帧

ASCII：`MA1,<19个十进制整数>,<4位大写CRC>\r\n`，最多256字节。CRC16/CCITT-FALSE（初值FFFF，多项式1021）覆盖MA1起至CRC前最后一个逗号。整数顺序：

| 下标 | 字段 | 单位/含义 |
|---|---|---|
| 0～5 | request_id,run_id,stage,start_id,map_version,goal_cell | 当前事务；stage2、goal14 |
| 6～10 | camera_id,calibration_id,mount_id,height_id,target_id | 与请求profile一致 |
| 11 | frame_sequence | 非零、事务内严格递增 |
| 12～13 | capture_after_request_ms,frame_age_ms | 毫秒，见上文 |
| 14 | confidence_permille | 0～1000；profile候选下限900 |
| 15 | units | 1=车身毫米修正；0或像素拒绝 |
| 16～18 | dx_milli_mm,dy_milli_mm,yaw_mdeg | 有符号int32，0.001mm/0.001deg |

候选观测包络±100mm/±10deg，帧年龄严格小于profile上限（最大200ms）。不是运动命令限幅，模块从不生成运动。拒绝码定义在metric_alignment.h：1参数/帧错误、2不满足停车条件、3未标定、4无有效请求、5过期、6轮次/目标不匹配、7标定/相机版本不匹配、8单位错误、9旧帧、10置信不足、11越界、12重复/乱序。

## 验证与下一步

运行：`python scripts/check_metric_alignment_interface_20261104.py`。Python编解码/输入检查13项、C接收/桥接/时效/CRC/边界137项通过，C编译使用`-Wall -Wextra -Werror`；ARMCC Cortex-M4单元编译通过，无诊断。C桥接用实际D盘vehicle_mission.h核对结构，但主状态机函数在测试中为模型，**不是完整固件构建或UART实物验收**。

证据在`evidence/integration/2026-11-04/metric-interface/`。后续需要设备在线、实际画面与相机身份/安装姿态、工作平面实测高度、独立毫米验证点、串口归属和时钟/缓冲验证；然后才接有界Emm原生位置微调、独立停车、视觉复测。当前不得打开位置执行器或直接抓料。
