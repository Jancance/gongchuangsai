# F407 CAN2达妙只读识别：代码交付与板端下一项

> 2026-10-29 CAN2板端识别验收通过：用户Watch为state4/result1/valid7/ID1/Master0/MIT1、请求清0、home0，证据dm-can2-readonly/board-acceptance.json及原图。不重复本项。已继续在实际工程仅更新DM_CAN2三文件，新增read_request2，一次读本机P/V/T量程及CC状态快照并换算反馈角，保留请求1；无使能/写参/运动。协议检查、副本及实际编译0错误0警告，Code48352，备份/SHA/日志见evidence/p2-arm/2026-10-29/dm-can2-pose/，main/工程/CAN1/升降/伸缩文件不改。新版角度功能尚未烧录/实测；下一唯一项下载新版→VM请求1等3秒→read_request2一次，回量程/角度/状态Watch。详见[P2_DM_CAN2_POSE_2026-10-29.md](P2_DM_CAN2_POSE_2026-10-29.md)。已核R RUN-01/PWR-01/MECH-01/02、E RUN-06/LOC-03/CAL-05；机械零点/方向/范围/运动/固定抓放未闭环，官方Q不变。

## 本轮实读及实现状态

现机DM-J4310-2EC V1.1 DC24V；用户以CANable/COM16、经典CAN 1Mbps取得三条完整回复：

| 寄存器 | 回复标准帧ID | 完整数据 | 结论 |
|---|---|---|---|
| ESC_ID，08 | 000 | 01 00 33 08 01 00 00 00 | 电机CAN ID=1 |
| CTRL_MODE，0A | 000 | 01 00 33 0A 01 00 00 00 | MIT模式=1 |
| MST_ID，07 | 000 | 01 00 33 07 00 00 00 00 | Master ID=0 |

原始截图和协议解释保存在`evidence/p2-arm/2026-10-29/canable-readonly/`。开源参考的POZ位置速度配置与现机MIT不同，本次保留现机模式，不复制参考的0x101位置速度运动报文。

已接入用户实际工程`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`并实际编译0错误0警告；Code=47368、RO=604、RW=1816、ZI=25736。尚未向F407烧录，本板CAN2尚未验收。

## 文件与行为

- 新增`HARDWARE/DM_CAN2/dm_can2.c/.h`及`dm_can2_protocol.c`；按STM32F4标准库、原生FreeRTOS、中文注释实现。
- `USER/main.c`只增加头文件和现有唯一CAN任务内的Service调用；仅在当前机构隔离调试模式推进，未创建第二个CAN任务。
- CAN2 PB12/RX、PB13/TX、AF9，PCLK1=42MHz时分频3、BS1=11、BS2=2、SJW=1：42MHz÷3÷14=1Mbps。PCLK不符拒绝请求。
- 仅开启CAN1时钟以满足CAN2硬件依赖，不复位CAN1、不修改CAN1 BTR、CAN1 PD0/PD1及500kbps保留。
- 共享过滤器保持分界14；CAN1使用原bank0，CAN2使用bank14精确匹配标准ID0数据帧。不接受把“收到任意ID0”当完成，仍逐项核DLC、节点、0x33、RID及实读值。
- 默认请求0，不自动初始化或发送。显式请求1后，非阻塞地串行读取08、07、0A三个寄存器，每笔只有一帧，发送/回复/总事务有超时，忙时不排队，失败不自动重试。
- 机构运动/回零/伸缩忙或既有平台/VM请求未就绪时拒绝或终止；不含使能、保存零点、参数写入和运动命令。供电状态仍为原平台状态，不据软件请求宣称实测VM电压。
- 当前实际Keil项目文件与上次安装记录不同：伸缩源码和main引用仍在，但uvprojx漏掉EXTENSION组和包含路径。本次只恢复其工程引用，伸缩源码不改。

## 当前唯一操作

**在Keil重新打开上述实际工程，下载本次已编译版本并运行。保持`g_dm_can2_read_request=0`，先回报新版已运行。** 不重复USB驱动/刷CANable固件、供电或已通过的35顶部回零测试。

之后再进行一次板端只读识别：先停止cangaroo测量，保持机构停稳且升降/回零/伸缩请求为0；按既有机构调试流程`g_vm_test_request=1`并等待平台允许，再将`g_dm_can2_read_request`写1一次。CANable可留在总线上但不另发查询；主控和电机之间的终端/接线仍按实际总线核对。

| Watch变量 | 本轮成功条件/含义 |
|---|---|
| g_dm_can2_read_request | 请求写1一次，程序自动清0 |
| g_dm_can2_state | 4为本轮三项完成，5为失败 |
| g_dm_can2_result | 1为三项回复全部匹配 |
| g_dm_can2_valid_mask | 7表示本轮三项均有效，不能只凭Master值为0判断完成 |
| g_dm_can2_esc_id | 1 |
| g_dm_can2_master_id | 0 |
| g_dm_can2_ctrl_mode | 1（MIT） |
| g_dm_can2_replies[0..2] | 本轮三条原始回复，顺序ESC_ID/MST_ID/CTRL_MODE |

失败保留`state/result/init_result/last_rid/esr/last_rx`；结果2=时钟，3=初始化，4=发送，5=超时，6=溢出/报文，7=bus-off，8=互锁，9=过滤器/资源，10=参数不匹配。只读成功也不能作为角度、方向、到位、停止或固定抓放通过。

## 验证、来源与边界

- 主机协议检查覆盖三条只读编码、Master ID0、小端解析，以及错误ID/节点/操作码/RID/IDE/RTR/DLC拒绝。使用厂家SDK`DM_CAN.py`的0x7FF/0x33定义和用户实读值，不用示例默认值替代实测。
- 副本及实际Keil构建均0错误0警告；证据`evidence/p2-arm/2026-10-29/dm-can2-readonly/`中的host-protocol.log、stage-build.log、actual-build.log、manifest.json、installed.json。修改前文件已备份，安装前校验原文件未漂移。
- CAN1源码、app_config、升降/回零/行程配置和伸缩源码SHA256均保持。没有下载F407固件，没有由助手发送本次CAN2命令或运动。
- R核RUN-01/PWR-01/MECH-01/02；E核RUN-06/LOC-03/CAL-05。只读调试入口与正式运行隔离，官方Q保持待技术答疑确认。
- 未闭环：板端CAN2/实际1Mbps通信、反馈量程、角度/原点和允许范围、停止/到位/超时实机、固定动作组。下一阶段再按实际模式和当前角度确定运动方案；本次不切换MIT或开放运动。
