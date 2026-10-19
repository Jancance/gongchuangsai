# CAN电机状态只读查询：手动增量修改

> 最新实机接续（2026-10-12）：四台ID1～4状态读取均通过，result1/ESR0、对应ExtId0100～0400/DLC3、3A036B。基本状态门禁闭环，下一项按P1_TASK_SUPERVISION_MANUAL_GUIDE_2026-10-12.md进入任务监督/VM锁定；下方三文件修改及查询说明留作记录，不要求重做。原正式Rebuild/链接0错误0警告，Code13488/RO424/RW296/ZI22400。

四台共CAN1基本版本查询已过，类型均FW_Emm；复位默认关VM、CPU暂停硬件关断、软件断言关VM及临时注入清理已有证据。下一项只读取电机状态，不发送使能/速度/位置/校准命令；继续用户手动SPL/原生FreeRTOS/中文注释。

## 1. 手册依据

厂家原手册第73～74页5.5.15：串口表示Addr 3A 6B，返回Addr 3A Flags 6B。按已有CAN扩展ID映射，地址进ID，不进数据区，因此CAN请求DLC2、3A6B，回复DLC3、3A Flags 6B。地址仍1左后、2右后、3左前、4右前。

状态字常用位：0x01使能、0x02到达驱动器输入目标位置、0x04堵转、0x08堵转保护；另外0x10/0x20左右限位输入、0x80掉电标志、bit6保留。驱动器到位标志不等于底盘位姿/任务到位；尚未发运动命令时不假定状态必须某固定值。

## 2. SYSTEM/CAN/can.c复用已验收的短帧读取

保留初始化/回环/超时/匹配逻辑，只把原CAN1_ReadVersion函数体变成静态共用函数，再加两个公开入口。下面都是在原函数内定位替换，不能替换到回环部分。

把原函数声明

```c
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply)
```

改为：

```c
/* 共用短帧读取：校验地址、功能码、长度和固定校验尾 */
static uint32_t CAN1_ReadParameter(uint8_t address,
                                  uint8_t command,
                                  uint8_t reply_length,
                                  CanRxMsg *reply)
```

原参数检查替换为：

```c
if ((address == 0U) || (reply == NULL) ||
    (reply_length < 2U) || (reply_length > 8U))
{
    return 7U;
}
```

原tx.Data[0]赋值替换为：

```c
tx.Data[0] = command;
```

原接收的功能码检查和长度/尾字节检查，分别替换为：

```c
if ((rx.DLC != 0U) && (rx.Data[0] == command))
```

```c
if ((rx.DLC == reply_length) &&
    (rx.Data[reply_length - 1U] == 0x6BU))
```

其内部原return1/return6等保持。文件末尾增加两公开入口，原版本功能仍调用同一逻辑、命令和长度：

```c
/* 读取版本，保持原有接口 */
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply)
{
    return CAN1_ReadParameter(address, 0x1FU, 6U, reply);
}

/* 读取状态标志，不发送电机控制指令 */
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply)
{
    return CAN1_ReadParameter(address, 0x3AU, 3U, reply);
}
```

## 3. SYSTEM/CAN/can.h加声明

在原版本声明后加入：

```c
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply);
```

## 4. USER/main.c加一次性状态请求

在版本相关全局变量附近加入：

```c
/* 状态查询沿用g_can_version_address选择目标 */
volatile uint32_t g_can_status_request = 0U;
volatile uint32_t g_can_status_result = 0U;
volatile uint32_t g_can_status_esr = 0U;
volatile uint32_t g_can_status_flags = 0U;
volatile TickType_t g_can_status_tick = 0U;
CanRxMsg g_can_status_reply;
```

在任务A原版本请求if块之后、g_test_tick赋值之前加入：

```c
/* 只读电机状态，VM请求开启后由Watch触发一次 */
if ((g_can_status_request == 1U) &&
    (g_vm_test_request == 1U))
{
    uint32_t address = g_can_version_address;

    g_can_status_request = 0U;
    g_can_status_result = 0U;
    g_can_status_flags = 0U;
    g_can_status_tick = 0U;

    if ((address >= 1U) && (address <= 4U))
    {
        g_can_status_result =
            CAN1_ReadStatus((uint8_t)address,
                            &g_can_status_reply);
    }
    else
    {
        g_can_status_result = 7U;
    }

    g_can_status_esr = CAN1->ESR;

    if (g_can_status_result == 1U)
    {
        g_can_status_flags = g_can_status_reply.Data[1];
        g_can_status_tick = xTaskGetTickCount();
    }
}
```

只有result1的flags及该次tick是有效查询快照。未重新查询/切断VM后不得将旧快照视为当前状态，不用于本轮自动运动控制。查询失败只记录诊断，不新增RTOS致命停机或擅自复位驱动。现有静态查询仍每次初始化CAN，尚不是持续闭环采集架构。

## 5. 先读左后ID1

保留四台接入，不改菜单/接线/代码之外参数。保存/Rebuild/下载后先F5正常启动，恢复硬件软件控制允许档，将VM请求1/F5，确认四台电机启动完成。

Stop设置g_can_version_address=1、g_can_status_request=1，原版本请求保持0；F5约1秒再Stop。回报g_can_status_result、g_can_status_esr、g_can_status_flags及展开的g_can_status_reply。预期ExtId0100、DLC3、Data0为3A、Data2为6B，Flags中间值以实测解释；不把使能/到位等具体值预设为成功条件。

无需为了版本功能重做四台已过基本交易；共用函数参数化的正常路径由候选编译及实际新状态帧验证，遇新故障再针对性检查。下一步根据ID1实际状态再扩到2～4，不添加自动定时返回/运动命令。

## 需求与证据

检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E）；厂家原PDF73/74页完整视觉复核，原代码只读检查及私人候选编译/原源哈希另记。官方Q不变，实际状态帧/其他状态与持续采集、看门狗及完整安全/运动待验，助手不改真实固件或操作硬件。
