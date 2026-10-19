# 四轮CAN只读版本：可选目标地址（2026-10-11）

> 最新用户实机接续：四台单节点/共CAN1基本读取及复位默认VM0均通过。下一项维持VM请求1并暂停CPU，用现有电源开关硬件关断，测VM是否近0；不重复通信/复位或改代码。测后保持硬件关断并撤销软件请求。下面代码保留接入说明；完整故障安全/持续网络/运动待验。

当前左后ID1已读取通过：扩展ID0100，DLC6，1F00CA23146B，软件版本V2.0.2，FWType=FW_Emm。用户确认另外三台实际CAN1_MAP及FW_Emm；这些菜单反馈不等于通信通过。保持右后2、左前3、右前4原映射，先验证右后2。

以下由用户手动修改原工程USER/main.c；助手不编辑实际固件。只增加可选地址，仍使用原有单次版本查询、500k、固定6B、超时/帧校验和VM默认关闭，不新增任务或运动命令。

## 1. 全局变量加一项

在现有g_can_version_request声明前加入：

```c
/* 查询地址：1左后、2右后、3左前、4右前 */
volatile uint32_t g_can_version_address = 1U;
```

## 2. 替换任务A的版本请求if块

找到“请求VM供电后，手动读取左后ID1的版本”注释下的整个if块，用下面替换；后面的g_test_tick等保持原样。

```c
/* VM已请求开启后，手动读取指定电机版本 */
if ((g_can_version_request == 1U) &&
    (g_vm_test_request == 1U))
{
    /* 本次地址只取一次，限定为已分配的四个节点 */
    uint32_t address = g_can_version_address;

    g_can_version_request = 0U;
    g_can_version_result = 0U;

    if ((address >= 1U) && (address <= 4U))
    {
        g_can_version_result =
            CAN1_ReadVersion((uint8_t)address,
                             &g_can_version_reply);
    }
    else
    {
        /* 7表示地址无效，本次不发送 */
        g_can_version_result = 7U;
    }

    g_can_version_esr = CAN1->ESR;
}
```

项目已启用C99，块内uint32_t声明有效；32位Watch输入先检查1～4再转换uint8_t，避免误填地址截断后发给其他节点。请求一次消费，地址改动不会自行触发查询。无效地址时旧reply不作为本次有效回复，ESR是现状快照而非新交易结果。

## 3. 先读右后ID2

换接供电/通信线前全部断电。用户已确认三台菜单，不再重做左后验收；保持主控5V与3V3各自正确供电。右后先单节点接入，其他节点暂不接，减少首次失败时的排查范围。

1. 保存/Rebuild，用户工程0错误后下载；F5先运行到状态灯闪烁。
2. Stop将g_vm_test_request设1，F5，等待右后驱动启动完成。
3. Stop将g_can_version_address设2、g_can_version_request设1；F5约1秒，Stop。
4. 回报result/esr及展开reply/Data；右后预期扩展ID0200、DLC6、首1F尾6B，FW/HW中间字节以实测为准。

右后通过后再通过同一Watch地址选择3、4逐台验，不用为了换地址再次编辑/下载。仍遵守断电换接，不能把菜单确认或result1的单台证据当完整网络持续负载/运动通过。

## 需求与证据

检查PWR-01（R）、LOC-03/RUN-06（E）。原F407 SPL/原生FreeRTOS、中文注释、用户手动源码及八阶段不变，官方Q不变。候选只编译私人main副本，原源SHA前后比较，编译成功不代表实际右后交易通过；用户真实Rebuild/ID2～4帧仍待验。
