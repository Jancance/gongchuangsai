> 最新实测2026-10-14：台架宏1源已确认，用户成功暂停操作；ID1原始扩展0100/IDE4/DLC5/35 00 00 00 6B、串口SPEED:A1 R1 RPM0，VM22V/两fault0/两ready1通过。唯一下一项保持本次静态供电，依次地址2/3/4、请求1，每次F5处理后暂停截raw及串口；完成后36位置读取，不重复ID1或旧看门狗门禁。

> 2026-10-14台架调试修订：用户报告Stop自动重启、无法设置VM请求；实际watchdog.c仍宏0，截图IWDG复位1/VM请求0。当前唯一先行修改watchdog.c宏为1U，Rebuild/下载并F5运行完成初始化后再Stop；CPU暂停时冻结IWDG，便于Watch设置和截图，正常运行监督/IWDG仍在。此前宏0已验结论保留，不重做全套看门狗测试。之后按本指南静态ID1速度读取。

# 四轮反馈第一步：Emm实时转速只读

> 2026-10-14用户正式构建/链接已过：0错误0警告，18900/424/532/22596、2秒，实际接口和互斥分支只读核对正确。第2～5节修改完成，唯一下一项第6节左后ID1静态转速：先VM供电/原四台状态健康，运行状态触发一次只读请求，核对串口及原始35回帧；该构建日志无下载/实际反馈证据。

2026-10-14。当前X42S_V2.0/FW_Emm/CAN1_MAP/500k/固定6B、地址1左后2右后3左前4右前沿用。厂家手册V1.0.5_260527第71页5.5.11：35读取转速、00正/01负、高低字节幅值；Emm直接RPM，X才除10。已提取并渲染查看第71/72页，当前位置36另一步做。地址按已验CAN映射进入ExtID，DLC5为35/符号/两字节速度/6B。

## 1. 本步范围

只新增一个读取接口及任务A手动分支，不发速度/使能/停止/置零/校准或位置控制命令。PC13只保留已验任务码请求；本步供电仍用现有g_vm_test_request静态测试，不自动把START变成电机运动。正式READY/业务合法性/运动控制仍待接。接收只返回本轮结果，result非1时RPM=0是无效占位，不判静止。

## 2. SYSTEM/CAN/can.h添加声明

在#endif前加：

```c
uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, int32_t *rpm);
```

## 3. SYSTEM/CAN/can.c末尾添加

保留原ReadParameter/版本/状态/缓存/有界超时：

```c
/* Emm实时转速只读：CAN回帧35、符号、速度高低字节、6B */
uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, int32_t *rpm)
{
    uint32_t result;
    uint32_t magnitude;
    if (rpm == NULL) return 7U;
    *rpm = 0;
    result = CAN1_ReadParameter(address, 0x35U, 5U, reply);
    if (result == 1U)
    {
        if (reply->Data[1] > 1U)
        {
            result = 6U;
        }
        else
        {
            magnitude = ((uint32_t)reply->Data[2] << 8) |
                        reply->Data[3];
            *rpm = (reply->Data[1] == 0U) ?
                   (int32_t)magnitude : -(int32_t)magnitude;
        }
    }
    if (result != 1U) CAN1_ResetSession();
    return result;
}
```

## 4. USER/main.c两处

### 4.1 g_can_version_address定义之后加

```c
/* 只读转速请求：地址1左后、2右后、3左前、4右前 */
volatile uint32_t g_can_speed_address = 1U;
volatile uint32_t g_can_speed_request = 0U;
volatile uint32_t g_can_speed_result = 0U;
volatile int32_t g_can_speed_rpm = 0;
volatile uint32_t g_can_speed_esr = 0U;
CanRxMsg g_can_speed_reply;
```

### 4.2 原版本查询if之前加新分支

在TaskA里原if(g_can_version_request...)之前插入下面分支，同时把原版本查询开头if改成else if，形成转速→版本→状态→轮询的互斥链。原后续CAN刷新/监督发布/扫码/启动门禁/心跳和延时全部保留。

```c
        /* 已有四台基本状态就绪后，每轮只发一个只读转速请求 */
        if ((g_can_speed_request == 1U) && (g_vm_test_request == 1U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U))
        {
            uint32_t address = g_can_speed_address;
            int32_t rpm = 0;
            char log[48];
            g_can_speed_request = 0U;
            g_can_speed_result = 0U;
            g_can_speed_rpm = 0;
            if (address >= 1U && address <= 4U)
            {
                g_can_speed_result = CAN1_ReadSpeed((uint8_t)address,
                                                  &g_can_speed_reply, &rpm);
            }
            else
            {
                g_can_speed_result = 7U;
            }
            g_can_speed_esr = CAN1->ESR;
            if (g_can_speed_result == 1U) g_can_speed_rpm = rpm;
            /* 结果非1时RPM字段不作为有效反馈 */
            sprintf(log, "SPEED:A%lu R%lu RPM%ld\r\n",
                    (unsigned long)address,
                    (unsigned long)g_can_speed_result,
                    (long)g_can_speed_rpm);
            (void)USART1_SendString(log);
        }
```

改后的原版本分支开头为：

```c
        else if ((g_can_version_request == 1U) &&
                 (g_vm_test_request == 1U))
```

不要在同轮追加多个CAN查询，不把CAN移到监督/按键/中断。初始化/Group4/IWDG和VM默认0不改；Keil不用新加文件或路径。

## 5. 用户Rebuild

只手改上述三个文件，保存Rebuild，提供完整日志。私人can/main两单元ARMCC5.06 update5 build528编译-c通过0错0警告，三实际源SHA不变；未链接/烧录，不签实际35回帧已通过。

## 6. 构建通过后首条反馈门禁

正式构建后再用户下载；保持电机静止，用已验硬件软件允许档，在程序运行状态Watch设置g_vm_test_request=1，等待原四台状态轮询/两个ready1及fault0。然后运行状态设置g_can_speed_address=1、g_can_speed_request=1，预期串口SPEED:A1 R1 RPM0，或对应静止附近数值并保留实际raw；扩展ID0100/DLC5/35 00 00 00 6B（零速度符号亦可能01）是静止候选。给串口与展开原始回帧、VM及fault/ready读数。

当前台架切宏1并下载/初始化后，可Stop修改Watch，再F5处理请求；查询自动消费为0。此调试设置不关闭正常运行的IWDG，实际部署前正式配置另行明确，不静默混用。先验证ID1，再依次2/3/4，不一次循环反复触发请求。读取失败时保留result/raw/ESR查原因，不以占位0RPM判通过。串口日志忙时可能未显示，Watch result及原始帧是另一证据；不要为看截图长暂停导致复位/VM关断混淆回帧。

## 7. 需求与后续

检查PWR-01/FIELD-02（R）、LOC-03/RUN-06/MOT-04（E）；QR-04（Q）保持待答疑。手册命令/单位是厂家事实，静态读取及分步验收为E，不代表整车轮速精度或运行中位姿已验。全四轮35基础回帧过后接36位置读取/原始符号幅值及角度，再更快独立采集/反馈时效，随后运动接口；实际从完整装机载物低速，X/Y和几何/方向仍需冻结实测。原状态3A保护监督不撤；抓放统计延后全程可跑。

来源evidence/p1-0/motor-manuals/ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527.pdf第71/72页、实际已验CAN短帧映射及当前源。私人证据CAN_speed_private_compile.json；实际回复/单位静态一致性/供电健康与完整运动均待用户实测。
