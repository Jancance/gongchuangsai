# STM32F407VET6 雷达控制端v6例程

本工程是HAL裸机、非阻塞的雷达应用协议v6控制端，已按雷达固件1.35和Flash保存状态v11验证。它采用“一套协议与客户端 + 六个独立流程Demo + 编译时选择”的结构。默认Demo只建立连接和读取状态，不选择工作区域，也不自动扫描。

公共函数说明见[雷达控制端函数使用说明](../STM32雷达上位机函数使用说明.md)，FreeRTOS移植见[FreeRTOS雷达任务移植示例](../FreeRTOS雷达任务移植示例.md)，旧v5控制端的最小升级步骤见[v5控制端适配v6修改说明](../v5控制端适配v6修改说明.md)。

## 最快使用方法

1. 按下方接线表连接雷达通信串口、调试串口和公共地，确认两端都是3.3 V TTL。
2. 打开`radar_example/demo_select.h`，第一次先保持`RADAR_DEMO_CONNECT`和正确的节点ID。
3. 使用Keil打开`MDK-ARM/F407VET6.uvprojx`，选择`F407VET6` Target并编译、下载。
4. 打开调试串口，设置为115200、8N1。正常时会看到`HELLO`、`PERSISTED`和`STATE`。
5. 需要执行业务时，将`RADAR_DEMO`改为对应Demo，并把`RADAR_DEMO_ZONE`明确改成雷达实际所在的1或2，然后重新编译、下载。
6. 单次Demo只执行一次；需要再次执行时复位控制板。持续扫描Demo可在调试串口发送`stop`终止。

最常修改的文件只有`radar_example/demo_select.h`。如果只是切换Demo、节点、区域或路径起终点，不需要修改协议源码和`main.c`。

## 工程配置

- MCU：STM32F407VET6
- 时钟：8 MHz HSE、168 MHz
- 雷达模块链路：USART1，115200 baud，8N1
- 调试输出：USART2，115200 baud，8N1
- USART1 RX为DMA2 Stream2 Channel4循环接收，TX为DMA2 Stream7 Channel4普通发送
- 裸机主循环，无RTOS

现有`.ioc`、串口引脚、DMA和时钟配置没有修改。

USART1 RX发生校验、帧、噪声、溢出或DMA错误后，中断只登记错误，主循环中的`RadarClient_Poll()`按100 ms间隔恢复。恢复会同时核对HAL状态、USART DMAR位和DMA Stream使能位，必要时重新初始化RX DMA，清除UART/DMA错误标志、丢弃残留协议半帧并重新HELLO；恢复过程不在中断中阻塞。

## 目录

~~~text
radar_example/
├─ protocol/
│  ├─ link_protocol.c/.h    COBS、CRC16和v6帧流解析
│  └─ radar_client.c/.h     握手、工作区域、事务和公开业务
├─ port/
│  ├─ radar_uart_port.c/.h  USART1与HAL适配
│  └─ demo_console.c/.h     USART2非阻塞调试输出
├─ demos/
│  ├─ demo_common.c/.h
│  ├─ demo_connect.c
│  ├─ demo_scan_once.c
│  ├─ demo_grid_scan.c
│  ├─ demo_path.c
│  ├─ demo_calibrate.c
│  └─ demo_continuous.c
└─ demo_select.h
~~~

`Core/Src/main.c`只初始化外设和模块，然后循环调用：

~~~c
RadarClient_Poll();
DemoConsole_Poll();
Demo_Poll();
~~~

## 接线

| STM32F407VET6 | 连接 |
|---|---|
| PA9 / USART1_TX | 雷达下位机PA10 / RX |
| PA10 / USART1_RX | 雷达下位机PA9 / TX |
| GND | 雷达下位机GND |
| PA2 / USART2_TX | 可选USB-TTL RX |
| PA3 / USART2_RX | 可选USB-TTL TX |

全部信号使用3.3 V TTL并共地。雷达下位机PA9为开漏输出，因此在“雷达PA9—本机PA10”接收线上只接一个4.7 kΩ上拉电阻到3.3 V；不要把该信号上拉到5 V。

## 选择Demo

编辑`radar_example/demo_select.h`：

~~~c
#define RADAR_DEMO RADAR_DEMO_CONNECT
#define RADAR_DEMO_NODE_ID 1U
#define RADAR_DEMO_ZONE 0U
#define RADAR_DEMO_PATH_START 0U
#define RADAR_DEMO_PATH_GOAL 24U
#define RADAR_DEMO_POINT_CLOUD_ENABLE 0U
~~~

可选Demo：

| 宏 | 行为 |
|---|---|
| `RADAR_DEMO_CONNECT` | 握手、读取保存状态，不启动扫描 |
| `RADAR_DEMO_SCAN_ONCE` | 14圈完整障碍扫描，打印坐标、方格和置信度 |
| `RADAR_DEMO_GRID_SCAN` | 14圈方格扫描，只打印障碍方格 |
| `RADAR_DEMO_PATH` | 读取Flash方格结果；有效则规划，无记录则拒绝 |
| `RADAR_DEMO_CALIBRATE` | 对所选工作区域执行一次两点校准 |
| `RADAR_DEMO_CONTINUOUS` | 持续接收障碍结果，调试口输入`stop`结束 |

除`RADAR_DEMO_CONNECT`外，其他Demo都必须把`RADAR_DEMO_ZONE`明确设为1或2，否则编译会报错。不能从Flash保存状态的`source_zone`猜测当前工作区域。

`RadarClient_Init()`只初始化通信客户端，不包含选区。`main.c`在它的下一行单独调用`RadarClient_RequestSelectZone(RADAR_DEMO_ZONE)`；尚未收到HELLO时，该请求会被非阻塞地登记，握手完成后再自动发出。

点云默认关闭。只有把`RADAR_DEMO_POINT_CLOUD_ENABLE`设为1后，例程才发送点云开关命令并接收最多120个显示点；障碍、方格、校准和路径功能不依赖显示点云。

调试串口使用32项非阻塞输出队列，可完整容纳一次最多25个障碍物的逐条打印。若持续输出速度长期超过115200串口带宽，例程会输出`[console] dropped ...`提示累计丢弃的日志行；该提示只表示调试文字来不及发送，不表示雷达协议数据或障碍结果丢失。

## 工作区域行为

连接后先读取HELLO：

- 未提交选区请求时，HELLO的`zone=1或2`会恢复雷达本次上电期间已有工作区域，`zone=0`则保持未选区。
- 已单独提交`RADAR_DEMO_ZONE=1或2`时，例程以该值为目标；即使HELLO返回另一区域，也会发送选择命令进行切换。
- SELECT_ZONE必须同时收到成功ACK和目标区域HELLO，随后读取保存状态，才进入READY。
- 普通STOP后不重复选择区域。
- 不要在每次业务前发送SELECT_ZONE，它会重新载入Flash位姿并覆盖临时会话位姿。

固件1.34将两个工作区域的位姿独立保存。选择区域1只加载区域1位姿，选择区域2只加载区域2位姿；两区之间不再进行自动对称换算。校准、保存位姿和恢复默认校准都只修改命令指定且当前已确认的区域，因此两个区域需要分别校准或分别保存。

`PERSISTED_STATE`中的`zone1_pose`和`zone2_pose`是两份独立记录，`source_zone`只表示最近一次修改的是哪一份记录。校准事务必须同时收到成功ACK、`source_zone`与本次区域一致的保存状态以及CALIBRATION结果后才算完成。

## 非阻塞接口

所有`RadarClient_Request...()`函数只入队并立即返回，最终结果从回调得到。请求不能从中断调用。回调由`RadarClient_Poll()`执行，回调中不得延时、再次进入Poll或进行长时间计算。

串口中断中只转发标志：

~~~c
RadarClient_OnUartTxCompleteInterrupt();
RadarClient_OnUartErrorInterrupt(huart->ErrorCode);
~~~

STOP使用独立高优先级帧，可以取消扫描或校准。心跳每500 ms发送一次，每2秒发送HELLO健康探测；当前节点连续5秒无有效回复时自动清理半帧并重新握手。

## 方格扫描写Flash提醒

`RADAR_DEMO_GRID_SCAN`会让雷达保存最新方格结果。它只能由明确业务条件触发，不能放入周期定时器或每次持续障碍回调中反复执行。

`RADAR_DEMO_PATH`不会自动扫描。连接和选区后，它检查雷达Flash返回的`grid_result_valid`：有有效记录就直接规划；没有有效记录就打印拒绝提示并结束。有效的空扫描同样允许规划。

`RADAR_DEMO_SCAN_ONCE`对应`SCAN_LOCKED`，返回完整障碍坐标，不用于替代方格扫描。

## 编译与测试

CubeMX重新生成工程后执行：

~~~powershell
powershell -ExecutionPolicy Bypass -File .\sync_keil_project.ps1
~~~

脚本会恢复v6源码组和包含路径。

运行主机测试：

~~~cmd
cd HostTests
run_host_tests.cmd
~~~

测试直接使用Visual Studio Build Tools，不依赖CMake，并检查协议固定向量、分块和坏帧重同步、节点过滤、工作区域门控，以及六种Demo的编译选择。

Keil Target：`STM32F407VET6`。当前默认Demo已使用ARMCC 5全量编译通过，0错误、0警告。

## 修改串口

句柄集中在`radar_example/port/radar_uart_port.h`：

~~~c
#define RADAR_UART_HANDLE       (&huart1)
#define RADAR_DEBUG_UART_HANDLE (&huart2)
~~~

宏只选择已存在的HAL句柄，不会自动配置引脚和DMA。更换串口前必须先在CubeMX配置115200 8N1、RX DMA Circular、TX DMA Normal，并生成对应初始化代码。

本次没有运行CubeMX、连接硬件或烧录固件。
