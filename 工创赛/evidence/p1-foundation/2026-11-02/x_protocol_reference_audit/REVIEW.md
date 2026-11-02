# X协议与外部底盘实现审查（2026-11-02）

## 结论

不是全部代码都已迁移到X。根工程3.stm32f407工程/gongxun-f407.uvprojx仍为Emm协议；最近实车下载的是隔离X候选，最后下载记录为x_perimeter_comp008_20261101/ObjectsVehicle/gongxun-f407.axf，SHA256 6c8284e52e54bc6bda859a827a7b667ba491574fc889338edbae979f70353098。本次仅文件和既有日志审查，未连接硬件核对当前Flash，未下载或发车。

## 本地电机控制链路

- 根SYSTEM/CAN/can.c:393 CAN1_ReadSpeed以int32_t输出整数RPM，555 CAN1_SetEmmSpeedRaw发送F6七字节CAN数据。USER/chassis_auto_test.c:1241直接调用它，USER/main.c:574及3192也有Emm发送入口。根uvprojx没有APP_X_OFFLINE_ONLY覆盖，也没有挂接X诊断模块。不能用根工程构建去控制已配置X的四台驱动。
- X整圈候选SYSTEM/CAN/can.c:393转速float、调用ZdtX_DecodeSpeed；539 CAN1_SetXSpeedRaw用X八字节CAN；547系统快照输出0.1RPM；USER/chassis_auto_test.c:1074读X系统快照、1268下发X速度；main的点动和零速入口亦已改X。没有发现该候选还在发Emm七字节F6。
- X候选USER/chassis_kinematics.c:111仍叫Chassis_EncodeEmm，但127实际按RPM×10取整；这是遗留命名，不等于Emm报文。Chassis_EmmFeedbackToLogical同理。can.h仍有旧命名/注释，需在正式整合时澄清。
- X位置原生3600单位/圈由zdt_x_protocol.h:26一次转换为内部65536单位/圈，后续距离和里程统一用内部单位。不能因后端出现65536就断言仍按Emm解析，也不能再次改成3600造成二次缩放。
- 隔离候选普通uvprojx保留APP_X_OFFLINE_ONLY默认1；实车项目gongxun-f407-load-pd-vehicle.uvprojx显式定义0。应正式整合单一入口及明确固件身份，保留旧Emm基线，不应全仓替换字符串。

## 参考仓库与实际报文

用户指定 https://gitee.com/pinksakura/Logistics_Vehicle_F407_V2 。通过git ls-remote实时核对远程HEAD为8191479ab06eab8e9ed03ccb837f7137d049a0f8，与现有参考副本HEAD一致，参考副本干净。网页读取工具连接失败，但已直接读取Git副本源代码，未依赖搜索摘要或二手说明。

重点逐行阅读USER_Code/chassis_control.c/.h、position.c/.h、RTOS_APP/Chassis_task.c、Main_task.c/.h，核对Core/Src/freertos.c、usart.c、stm32f4xx_it.c、Visual_Identity/visual_identity.c、Remote_Control_RX.c、io_devices.c及tower/hcan.c的相关接口。不是对整个厂商HAL/视觉算法/机械资料逐文件验收。

参考USER_Code/chassis_control.c:54-79的SetMotorVoltageAndDirection使用UART4逐轮发8字节，完整串口布局为地址 F6 方向 速度高 速度低 加速度 同步 6B。加速度只有1字节，属于Emm式速度报文。它的同步字段为0，四次HAL_UART_Transmit之间osDelay(1)，不是真正同步启动。UART4配置115200、8N1；函数名含Voltage不代表裸PWM，实际发的是闭环电机速度命令。X对应串口完整9字节/CAN数据8字节，布局为地址 F6 方向 加速度高 加速度低 速度高 速度低 同步 6B。因此参考底盘不是我们当前X报文实现；不据此推断作者驱动屏幕固件版本。

参考tower里的CAN属于机构控制，不是上述四轮底盘的CAN通信，不能混为X底盘协议。

## 真正值得借鉴的控制逻辑

1. 场地X/Y/航向一起闭环。position.c:15-32从外部OPS帧读pos_x、pos_y、zangle；chassis_control.c:157-190计算目标与实际位置差，并用sin/cos将场地误差变换成车体速度，合成四轮。它能看见横向位置误差，我们当前仅单段距离加航向不能直接看见整车横向平移。转用这个架构必须先有健康、时间戳/坐标明确的雷达或视觉定位；单纯轮编码器里程仍可能把打滑一起积分，不能保证消除侧偏。
2. 粗定位加视觉精调。Main_task.c:90-100先粗视觉定位，再改变观察姿态并精调；Chassis_task.c:95-139切场地位置/视觉模式、完成后置零目标。可借鉴两级定位和模式职责，补充有界超时、有效数据与退出确认。
3. 到位持续确认。chassis_control.c:201-209要求位置/角度连续进入区间后in_pos，视觉也有持续计数。可借鉴防止偶然一帧误判的思路，改用真实经过时间和新鲜样本；它的普通阈值是X/Y±60mm、角度±15°，near阈值更大，不代表毫米精度，也不宜照搬。
4. 接近目标时按位置误差降低速度。chassis_move实际是比例项，vz=mKpz*devz，没有这里所期待的航向I或D。其位置反馈与我们的S曲线可组合成“规划速度+限幅位置修正”，但不能移植mKpz9、轮速0.238等与它的轮径、单位、周期绑定的值。
5. 路线以目标位姿表达，与机构动作解耦。Main_task.c用Chassis_Go_Pos(x,y,z)串联工位。可借鉴接口表达，但其阻塞while等待无统一超时，不能代替我们已有的有界状态机。

## 不宜照搬的具体部分

- 电机发送使用HAL_MAX_DELAY，未在该函数检查ACK/返回值；没有此路径的四轮速度闭环监测。
- chassis_move正负速度各按last_Speed±20限制的写法不按dt计算、不对称约束全部加减速，目标只比上次大一点时也可能直接加20；SpeedTarget_stop不清last_Speed。我们已有S曲线与输出斜率，应保留。
- numerical_limit名含dead_zone，实际仅控制是否叠加最小速度，没有把区间内值强制置零。不能当成熟低速死区直接套用。
- pos/visual解析的帧头尾检查没有形成全路径的长度、NaN、时效与健康门禁。视觉到位计数可能重复使用旧帧；一些等待不带超时；接收任务使用队列是结构参考，不证明并发与缓冲安全已经满足我们要求。
- 外部代码采用HAL/CMSIS任务层，我们现有工程使用SPL和原生FreeRTOS；不为借鉴控制思路整体换库。
- LICENSE.txt标明CC BY-NC-SA4.0；本轮只分析，没有复制其源代码进入我们的固件。

## 刚才整圈中止和调参顺序

已保存perimeter_fault_review.json。整圈只完成第1段，第二段RIGHT接近终点触发backend_cause8，distance_progress1797.651mm/target1800mm，source_valid1，姿态年龄26/27ms、无姿态串口错误；不是整圈完成。软件VM请求/PE2已0，但stop_mask0，故障后残留RPM非零，不能当成四轮零速确认或替现场确认停稳。

最后XD1在tick84370ms：左后ID1已确认目标-1.0RPM、目标年龄99ms，实际+0.3RPM，无驱动过流/过温/堵转标志；随后84380ms保护。chassis_auto_test.c:633-639在RUN阶段检测任意反号都会cut8；这些值与该分支完全相符。故障码8也供其他分支复用，首次快照未标独立轮号/分支ID，因此定为强证据的低速反向检测触发线索，不能排除真实轻微回弹，不能简单称误保护或删除保护。

推荐顺序（工程建议E，不是已实施改动）：
1. 将已验证X传输、反馈单位、身份检查正式整合到根工程入口，防止误编译/下载Emm；保留回滚版本与GBK/CRLF及现有故障保护。
2. 先补充反向检测的轮号、已ACK目标、采样时刻及判定分支诊断；离线复现0.1RPM分辨率下减速末端的符号变化，区分量化/短暂回弹与持续真实反转。在证据上设计低速窗口、持续时间/连续新鲜样本确认，真实反转/超速/反馈过期/驱动故障仍需触发停车。不要只放宽阈值或无条件跳过保护。
3. P1、D0.25、死区0.3°、I0保持不变，先完成补偿整圈及重复验证。前进+0.8%目前仅试验值，不能凭未完成整圈宣布有效；先短距离横移收尾验证再完整一圈。
4. 要进一步消除厘米级横移，接入可靠X/Y或视觉横向偏差反馈，而不是只加大航向P/I。先构建坐标/时效/速度限幅和停止门禁，再小幅修正；已有轮里程可辅助，不能代替真实地面位置。

本轮只读代码与日志，新增审查文档及交接；未改固件、未编译新固件、未下载、未发车、未提交推送。核对R FIELD-01/06 RUN-01 PWR-01；E LOC-03/05 MOT-02/04 RUN-06；官方Q保持未闭环。
