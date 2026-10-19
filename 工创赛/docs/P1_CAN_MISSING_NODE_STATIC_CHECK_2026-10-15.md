# 缺失一个电机时的状态保护：静态检查

> 2026-10-15缺失ID1接回复位恢复通过：平台ready1/监督fault0、CAN原因0/坏节点0，用户实测VM仍0V；guard0、反馈0为未请求采集时正常状态。当前用户只要求精简Watch：移除旧单次位置六项、创建/回环两项、两循环计数，保留16项VM/故障/就绪/反馈/栈堆变量；只移除Watch表达式，不删源变量。暂不追加硬件测试。后续运动失效/重新允许状态机及运动控制仍未完成。

2026-10-15，工程测试（E）。独立CAN采集/扫码显示并行已由两张Watch截图和用户“屏幕正常更新”确认。本步不改源、不重建、不发运动指令，验证重新请求VM前ID1未接入时的真实状态保护；不能当成运行中断线或运动制动距离验收。

## 1. 先关VM，再处理接线

先取消power.c第33行原断点，避免正常VM0关断被重复截停。暂停时设：

```text
g_vm_test_request = 0
g_can_feedback_enable = 0
```

F5运行约1秒后Stop，用表确认VM已经降至0V，再拔掉左后ID1那一路电机连接线，其余三台保持接入。不要带着VM电压插拔。关闭反馈会清除反馈会话计数/掩码，属于原设计，不是新故障；此前已通过的证据已保存。

本次关闭速度/位置轮询，以状态查询独立验证缺失地址，防止一个缺失节点连续三类200ms等待让其余节点的年龄也被拖长。没有放宽任何原新鲜度或保护阈值。

## 2. 准备第一次故障现场

保持当前平台健康、g_can_poll_enable=1，在Watch增加：

```text
g_can_guard_cause
g_can_guard_bad_nodes
g_can_poll_result
```

展开poll_result四项，保留两fault/两ready/VM请求。到HARDWARE/POWER/power.c第33行结束大括号按F9添加断点，该位置已经执行GPIO关闭。

## 3. 请求VM并捕获保护

暂停时仅设g_vm_test_request=1，反馈仍0，F5运行。未接齐节点时不能到四节点就绪；现源启动宽限为3000ms，之后监督判定缺失。约3秒后应触发关断断点；不要把这一时限解释成已测得运动失效响应。

预期现场：

```text
g_rtos_fault          = 0
g_safety_fault        = 0x20
g_safety_ready        = 0
g_can_guard_ready     = 0
g_can_guard_cause     = 2
g_can_guard_bad_nodes = 0x01
g_can_poll_result[0]  = 5
```

另外三个状态结果预期1。g_vm_test_request仍为1并不表示输出仍被允许，监督故障优先。实际值不同就保留截图，不能按预期手填、清故障、改保护阈值。若立即停下但两fault都0，是暂停前的正常关断调用，可F5继续到真正非零故障。

## 4. 报告现场，不继续运行

停住后保持暂停，测VM应降至0V，给Watch截图和实际VM读数。调试冻结宏1已只读核对，暂停时IWDG冻结以保留现场。不要先F5等待复位或清故障，以免覆盖根因。恢复接线与复位的步骤在本次结果确认后继续，期间保持VM关闭。

## 5. 已捕获关断后的恢复（当前下一项）

缺失ID1关断及实测VM0已确认。保持当前暂停和VM0，把左后ID1原连接线接回；取消power.c第33行断点。点击Keil RST复位，再F5约2秒后Stop，给VM请求/两fault/平台ready/CANguard截图并实测VM仍0V。预期请求0、两fault0、平台ready1、CANguard0；CANguard0是VM尚未请求时的正常值，不要求此时四节点查询已恢复。不要在Watch手动把故障清零，也不先请求VM1。源码默认请求0/反馈0、Safety_Init复位清状态已只读核对；2026-10-15用户截图及实测VM0已确认本次恢复，见CAN_missing_ID1_restore_reset_VM0_pass.json；不等同正式运行重新允许状态机完成。

## 范围与证据

PWR-01（R，单内置电源保持）、LOC-03/RUN-06/MOT-04（E）。无官方规则更改，Q待答疑条目不因本测试闭环。缺失ID1静态关断、接线恢复后复位VM0均已有用户截图/电压证据。源核对：SAFETY_CAN_BOOT_MS3000/FRESH_MS1000、数据fault0x20、原因2/位0对应ID1、Power_Disable第32行关闭/第33行返回。已通过依据见CAN_task_feedback_scanner_parallel_pass.json及CAN_missing_ID1_fault20_VM0_pass.json；运行中反馈失效、重新允许互锁及运动控制仍未签完成。
