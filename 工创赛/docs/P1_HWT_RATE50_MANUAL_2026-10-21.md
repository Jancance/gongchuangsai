# 板内HWT101一次性设置50Hz：本轮只手改并构建

> 最新：设置后接收平均50Hz已由两图计数/tick差验证（同组约49.97～50.03Hz），区间无新增校验/UART/溢出错误。不要重复下方设置。当前下一项VM开关关闭，对主控/HWT实际断电、排除调试USB回供，约5秒后正常上电；当前静态固件Run≥5秒后只发rrate_request1读取，预期result1/raw8，展开原帧并同时记录VM/fault。不要用RST或重新设置代替断电保持检查；rate50_result因RAM初始化为0正常，本次看RRATE新读值。最大帧间隔/源连续性和运动尚未通过。证据hwt_rate50_measured_frequency.json。

> 最新板上结果：设置流程已完成，active0/result1/state0/command3/raw8，原帧55 5F 08 00 06 00 00 00 00 00 C2校验正确、VM与两fault0。不要再次执行下方设置步骤。下一项保持VM关闭/无任何请求，连续Run中两次约10秒截图angle/rate_count、checksum/uart/overflow错误及update_tick，验证实际接收频率；用户本轮两图计数和tick相同，不能据此算50Hz。断电保持和最大间隔/源连续有效性尚待后验。证据hwt_rate50_readback8_user.png/json。

> 最新：用户完整Rebuild40296/600/1524/25452、0错0警告/3秒，实际hwt.c/h完全匹配候选，RRATE声明亦已补齐，main/静态模式保持。下方手改与构建步骤已完成。当前下一项仅静态一次设置：VM外部开关保持关闭，下载复位，连续Run≥5秒，版本1/state0、VM/fault0后先rrate_request1；看到result1/raw6后再等2秒，于Run中rate50_request1一次，再Run约5秒Stop，记录rate50 active/result/state/command/raw及展开reply、VM/两fault。原读值非6或未成功时不设置，先发截图。候选成功active0/result1/raw8，之后还需测频与断电保持；原rrate_raw6是旧读取，不是当前动态参数。证据hwt_rate50_actual_rebuild_pass.json。

用户已读到RRATE：result1、raw6、reply_count1，原帧`55 5F 06 00 06 00 00 00 00 00 C0`，校验正确；版本result1/state0、VM请求及两fault0。首个16位6对应10Hz，与前轮151帧/15.1秒实测相符；后一个6是相邻BAUD寄存器，不把它合并进速率。接受此次受控只读验证，不等于已经设置50Hz。

厂家型号专用协议已取得：[HWT101 Protocol](https://wit-motion.gitbook.io/witmotion-sdk/hwt101-protocol/hwt101-protocol)，明确RRATE03的06=10Hz、08=50Hz、配置须先解锁并保存。厂家编写的同名协议文档转载[HWT101 Protocol PDF](https://www.scribd.com/document/1072777032/HWT101-Protocol)在配置注意事项明确列出解锁`FF AA 69 88 B5`和保存`FF AA 00 00 00`；前期未取得型号文档的阻点更新为本次已有型号协议。0x76是航向归零，绝不作为解锁或替代命令。旧资料与本轮协议不同之处保留来源，不扩展到A6/A7等未经本步确认的设置。

本步只有以下固定序列：解锁→RRATE写08→保存→只读RRATE。保持115200、全部100ms安全期限、现有输出和校准参数。50Hz是工程候选，不是竞赛规则。SAVE会保存模块当前配置，本代码本轮只改变RRATE，不主动改其他寄存器；不重启、不恢复出厂、不归零、不自动重试或回写10Hz。任何阶段失败后实际参数可能已改变，必须先读取再判断，不能只根据失败码认定设置未发生。

实际工程：`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx`。助手只生成说明/样本/隔离候选，实际源由用户手动修改。当前唯一操作如下，先关闭VM外部开关并退出Debug。

## 1. 头文件补齐声明

在实际USER/hwt101.h最后的`#endif`之前，先补上上轮尚未加入的[RRATE只读声明](code_samples/hwt_rrate_read_header.c.txt)，再添加[50Hz服务声明](code_samples/hwt_rate50_header.c.txt)。如果某组已经存在，只保留一组，不重复。原声明与最后#endif保留。

## 2. 增加完整50Hz服务

在实际USER/hwt101.c中找到`static void Hwt_VersionUpdate(uint32_t now)`的函数定义，紧接在它之前粘贴[完整50Hz服务代码](code_samples/hwt_rate50_service.c.txt)。原RRATE变量、读取函数及所有其他函数保留，不把新增代码贴入原函数体内。

## 3. 两个原服务加入互斥

在Hwt_VersionUpdate开头找到原局部变量声明：

```c
    volatile uint8_t *reply;
```

紧接其后增加：

```c
    if (g_hwt_rate50_active != 0U) return;
```

在Hwt_CalUpdate开头找到原声明：

```c
    uint32_t new_angle, trip_kind;
```

紧接其后也增加：

```c
    if (g_hwt_rate50_active != 0U) return;
```

两行只互斥50Hz事务。不要删原变量/赋值，也不要修改其他函数里的同名条件。

## 4. 中断允许新的发送所有者

在USART2_IRQHandler的TXE分支，找到整个原if条件：

```c
        if ((((s_tx_owner == 1U) && (g_hwt_version_state == 1U)) ||
             ((s_tx_owner == 2U) && (g_hwt_cal_active != 0U))) && (s_version_index < 5U))
```

只将这两行替换为：

```c
        if ((((s_tx_owner == 1U) && (g_hwt_version_state == 1U)) ||
             ((s_tx_owner == 2U) && (g_hwt_cal_active != 0U)) ||
             ((s_tx_owner == 3U) && (g_hwt_rate50_active != 0U) &&
              (g_hwt_rate50_state == 1U))) && (s_version_index < 5U))
```

后面的发送字节、计数、时间戳、屏障及TXE关闭代码全部保留。新服务只使用owner3；旧owner1/2行为保留。

## 5. TaskA调用链增加一次更新

在HWT101_Update末尾找到：

```c
    Hwt_VersionUpdate(now);
    Hwt_CalUpdate(now);
```

改为：

```c
    Hwt_Rate50Update(now);
    Hwt_VersionUpdate(now);
    Hwt_CalUpdate(now);
```

不改main、Keil文件或任务。配置继续STATIC1/SESSION0/TURN0/KEY1，不启用任何运动模式。

## 6. 本轮只保存、完整Rebuild

把完整日志发来，先不下载、不设置任何请求。默认g_hwt_rate50_request=0，不会自行写模块参数。前轮读取的6不会跨主控复位自动保留，因此后续下载后仍需先执行一次只读RRATE，确认6，再触发新服务；届时逐步指导，本轮先不操作。

## 服务约束与结果

仅静态VM请求0、无RTOS/Safety故障、无校准/版本/A7/RRATE请求或活动时可接受；必须版本读取成功、源valid3、最近RRATE读取result1/raw6、距最近5F≥1500ms。一次上电只接受一次设置，重复请求不排队；请求不合法/前置不满足返回9。版本/A7/RRATE/校准互斥，错误或条件丢失立即停止后续帧。

TXE仍连续发送5字节，单次发送及TC检查共200ms上界，命令间至少100ms，保存后1500ms再读，回复截止1000ms，事务总截止8000ms。没有阻塞等待，不增加TaskA延时。新服务result：1为候选读回08，2发送/TC超时，3读回超时，4错误/互斥/总截止，5读回不是08，9入口拒绝。状态0闲置/终态、1发送、2命令间等待、3等读回；command0解锁/1写速率/2保存/3读回。

观察变量：g_hwt_rate50_request/active/result/state/command/raw/reply。原g_hwt_rrate_raw仍保留此前读取值6，不是动态速率显示；不要把旧变量6误判为本次读回失败，应看rate50_raw及原帧。result1须结合接收帧率与断电后读回验证，未做到前不称50Hz或持久化通过。5F不回显地址，静默和计数时间检查不能消除全部身份歧义。

## 已做软件检查和需求边界

相同服务GCC C99 Wall/Wextra/Werror模拟通过：上电不发送、固定四帧顺序、读回08、原版本/RRATE保持、初始6/静默窗/VM条件拒绝、VM改变及UART错误/校准冲突中止、TX/TC/回复截止、非08失败和忙时/终态不自动重试。实际完整hwt模块隔离候选（含owner3及调用链/互斥修改）ARMCC Cortex-M4.fp编译无诊断。主机TXE/时间为模拟，不证明真实中断、电平或模块兼容，未整套链接/烧录。证据hwt_rate50_preparation.json；原RRATE图/JSON为hwt_rrate6_read_user_pass.png/json。实际固件未被助手修改。

已核对R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04；官方Q继续待技术答疑确认。用户实际构建、参数设置/读回、50Hz频率/最大间隔/断电保持、源失效/物理VM/运动停车与X/Y仍待闭环。
