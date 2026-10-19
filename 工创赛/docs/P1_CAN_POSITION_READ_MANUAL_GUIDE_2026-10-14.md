> 最新2026-10-14：四台36单次静态位置均已验，sign0/raw1、1、3、3，对应0100～0400/DLC7/数据36符号幅值6B与POS日志一致，当前不再重复单节点查询。下一项P1_CAN_CONTINUOUS_FEEDBACK_MANUAL_GUIDE_2026-10-14.md静态持续采集。

> 2026-10-14实测：ID1 ExtID0100/DLC7/36 00 00 00 00 01 6B、result1/sign0/raw1与POS日志一致，首台通过。下一项ID2/3/4位置；可移除Watch已完成speed/reset项，保留position与健康监控。只移除观察表达式，源码接口全部保留。

# 四轮反馈第二步：Emm当前位置只读

> 最新2026-10-14：用户正式Rebuild/链接0错误0警告，19200/424/556/22612、2秒，实际位置接口和main只读核对正确；第2～5节构建完成。唯一下一项第6节左后ID1静态位置：下载后VM/原状态健康，位置请求1，核POS日志/符号与32位raw及ExtID0100/DLC7/36…6B；本日志无下载或实际回复证据，raw不要求0。

2026-10-14。四台35静态转速回帧已通过（各result1/RPM0、对应扩展ID及35 00 00 00 6B）。本步仅增量增加36实时位置读取，保留所有既有转速/状态保护/扫码/启动请求/监督/IWDG，不发运动/置零/校准命令。台架调试冻结宏保持用户已切回的1U。

## 1. 厂家格式和原始值

厂家ZDT_X42S V1.0.5_260527第72页5.5.13已核对：36读实时位置，符号00正/01负，四字节幅值00000000～FFFFFFFF。沿已验CAN1_MAP地址映射，扩展ID=地址左移8，DLC7，Data为36/符号/四字节幅值/6B。本步按高字节在前组装，保留sign和uint32 raw，避免把最大幅值直接装入int32溢出。

Emm角度=raw×360/65536，再应用符号；raw65536代表360度。X固件/10不适用于当前FW_Emm。这里是电机相对其坐标基准的实时位置，不能把它称作底盘X/Y或物理里程；不预设四台静止位置都为0，也不发送清零。

## 2. SYSTEM/CAN/can.h追加声明

在#endif之前追加：

```c
uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply,
                           uint8_t *sign, uint32_t *raw);
```

## 3. SYSTEM/CAN/can.c末尾追加

```c
/* Emm当前位置只读；符号与32位幅值分开，避免int32溢出 */
uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply,
                           uint8_t *sign, uint32_t *raw)
{
    uint32_t result;
    if (sign == NULL || raw == NULL) return 7U;
    *sign = 0U;
    *raw = 0U;
    result = CAN1_ReadParameter(address, 0x36U, 7U, reply);
    if (result == 1U)
    {
        if (reply->Data[1] > 1U)
        {
            result = 6U;
        }
        else
        {
            *sign = reply->Data[1];
            *raw = ((uint32_t)reply->Data[2] << 24) |
                   ((uint32_t)reply->Data[3] << 16) |
                   ((uint32_t)reply->Data[4] << 8) |
                   (uint32_t)reply->Data[5];
        }
    }
    if (result != 1U) CAN1_ResetSession();
    return result;
}
```

原共用ReadParameter和ReadSpeed/ReadStatus/ReadVersion全部保留。本接口result非1时sign/raw只是无效占位，原始回帧也可能是异常格式，仅result1方可解读。

## 4. USER/main.c两处

### 4.1 g_can_speed_address定义之前加

```c
/* 实时位置只读，不清零；原始幅值与符号分别保留 */
volatile uint32_t g_can_position_address = 1U;
volatile uint32_t g_can_position_request = 0U;
volatile uint32_t g_can_position_result = 0U;
volatile uint32_t g_can_position_sign = 0U;
volatile uint32_t g_can_position_raw = 0U;
volatile uint32_t g_can_position_esr = 0U;
CanRxMsg g_can_position_reply;
```

### 4.2 原转速查询if前插入新分支

```c
        /* 同一轮只执行一个查询，位置请求优先于转速/版本/状态 */
        if ((g_can_position_request == 1U) && (g_vm_test_request == 1U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U))
        {
            uint32_t address = g_can_position_address;
            uint8_t sign = 0U;
            uint32_t raw = 0U;
            char log[64];
            g_can_position_request = 0U;
            g_can_position_result = 0U;
            g_can_position_sign = 0U;
            g_can_position_raw = 0U;
            if (address >= 1U && address <= 4U)
            {
                g_can_position_result = CAN1_ReadPosition(
                    (uint8_t)address, &g_can_position_reply, &sign, &raw);
            }
            else
            {
                g_can_position_result = 7U;
            }
            g_can_position_esr = CAN1->ESR;
            if (g_can_position_result == 1U)
            {
                g_can_position_sign = sign;
                g_can_position_raw = raw;
            }
            /* 结果非1时SIGN/RAW字段只是无效占位 */
            sprintf(log, "POS:A%lu R%lu SIGN%lu RAW%lu\r\n",
                    (unsigned long)address,
                    (unsigned long)g_can_position_result,
                    (unsigned long)g_can_position_sign,
                    (unsigned long)g_can_position_raw);
            (void)USART1_SendString(log);
        }
```

同时把原转速分支开头的if改为else if：

```c
        else if ((g_can_speed_request == 1U) && (g_vm_test_request == 1U) &&
                 (g_safety_ready == 1U) && (g_can_guard_ready == 1U))
```

之后仍原版本/状态else if和正常轮询else，不删除任何旧分支。顺序为位置→转速→版本→状态→轮询，同轮只有一个CAN事务。原Refresh、Safety_CanReport、扫码/启动请求、APP心跳/栈堆/延时和默认VM0都保留，不在其他任务并发调用CAN。

## 5. 保存三个文件并Rebuild

Keil无需新增源文件或路径，用户保存Rebuild并发完整日志。私人can/main两单元ARMCC5.06 update5 build528 -c 0错0警告，三实际源SHA不变，无链接/烧录；新位置读写运行尚未验。构建通过后再正式下载，不凭本次私人编译签硬件位置回帧已通。

## 6. 构建后首先验ID1静态原始位置

用户下载后电机保持静止，VM硬件软件允许档、g_vm_test_request1；等待现有四台状态健康/两个ready1/fault0。暂停设置g_can_position_address=1、g_can_position_request=1，F5运行约1秒，再暂停截帧与串口。

预期POS:A1 R1 SIGN0 RAW某个数，符号也可能1，RAW不要求为0。对应ExtId0100/IDE4/RTR0/DLC7，Data[0]=36、Data[1]=00或01、Data[2..5]原始幅值、Data[6]=6B；零值候选36 00 00 00 00 00 6B。根据实际字节核对打印值，然后依次2/3/4。原Watch速度字段可折叠，新增position字段/原始reply及fault/ready/VM/APP余栈/heap用于本次。

g_can_position_sign为符号字节（不是方向接线结论），g_can_position_raw为幅值；接收/格式失败result不是1时RAW0不证明位置零点。静止重复读应关注一致性，未来运动反馈再验连续变化/正负/范围，不要求现在转动或强扭电机。台架宏1已确认可暂停截图；若将来切宏0则按新的运行状态调试流程，不静默混用。

## 7. 需求和未闭环

检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E），QR-04（Q）保持待技术答疑。当前静态反馈不证明轮速精度/里程/实时X/Y或整车READY，实际完整装机载物低速在反馈/几何/方向/限值与控制接口接入后做；原四轮状态保护不撤。用户抓放统计仍延后全程可跑后。

来源为已归档厂家手册第72页和当前CAN短帧实现；私人证据CAN_position_private_compile.json。正式构建/位置原始回帧/静态一致性及后续持续反馈、运动控制仍待用户实测。助手不修改实际三文件或执行CAN/烧录/硬件动作。
