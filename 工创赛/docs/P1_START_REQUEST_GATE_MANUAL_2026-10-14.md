# PC13确认任务码：当前平台启动请求门禁

> 最新验收：用户正式Rebuild/链接0错误0警告，Code18232/RO424/RW512/ZI22576、2秒；实际main接入已只读核对。第2～5节修改构建完成，第6节第一例已通过：用户串口READY/SCAN:532+123+325+132无START，VM0V。有效扫码后按键确认也已通过：START:532+123+325+132只一条、VM0V。当前PC13为自恢复按键，松手即释放；现有按键逻辑适用。换码锁屏/不新增START亦经用户报告通过，VM0。唯一下一项自恢复键按住跨复位/重新扫码无自动START，松手再按才接受；其余完整READY/运动待接。

2026-10-14。扫码和多次换码更新已由用户实物报告通过；本步用户仅增量修改main.c。仅确认本轮任务码和记录启动请求，VM控制和电机CAN速度接口在后续底盘阶段接入。保留原监督/IWDG及扫码接口。g_start_gate_ready表示当前平台/基本任务码显示通过的“允许确认请求”，不是整车正式READY；运动能力、完整任务语义与抽签start_id仍待接。

## 1. 本步行为与工程边界

无有效码/平台未就绪/已有故障时按键拒绝，之后扫到码也不会自动启动，需重新释放后再按。符合条件时记录g_start_request=1、锁定g_start_task_code及当前page0；再扫码仍可解析诊断但不替换本轮任务对象/显示。本步不发送电机控制、不自动改g_vm_test_request；复位默认请求0。故障撤销请求，旧监督故障锁定仍在；清故障后的正式重新允许流程尚未签署完成。

Key_GetEvent原有消抖/先释放后按下不变，按键只提交事件；TaskA在实际扫码处理后再核对版本/平台和复制任务码。扫码版本在按键后改变则拒绝该次请求，避免确认另一个码。基本格式通过不是完整搬运合法性；QR-04继续待答疑，当前不能因本步request1启动比赛动作。

## 2. main.c新增门禁全局变量

在g_scan_log_send_ok定义后添加：

```c
/* 当前已接模块的启动请求门禁，尚未驱动底盘 */
volatile uint32_t g_start_gate_ready = 0U;
volatile uint32_t g_start_request = 0U;
volatile uint32_t g_start_accept_count = 0U;
volatile uint32_t g_start_deny_count = 0U;
char g_start_task_code[16] = {0};
static uint32_t s_start_gate_revision = 0U;
static uint32_t s_start_pending = 0U;
static uint32_t s_start_pending_revision = 0U;
```

## 3. main.c新增门禁更新函数

在App_TestTaskA之前（“任务A：每100毫秒计数一次”注释前）添加：

```c
/* 只有任务A调用；先检查当前平台，再确认按键对应的扫码版本 */
static void App_StartGateUpdate(void)
{
    uint32_t platform_ok;
    uint32_t code_ok;
    uint32_t accepted = 0U;
    uint32_t i;
    char log[24];

    taskENTER_CRITICAL();
    platform_ok = (g_rtos_fault == 0U && g_safety_fault == 0U &&
                   g_safety_ready == 1U) ? 1U : 0U;
    if (platform_ok == 0U)
    {
        g_start_request = 0U;
    }
    code_ok = (g_saoma_last_result == 1U &&
               g_saoma_accept_count != 0U &&
               g_scan_display_result == SCREEN_OK) ? 1U : 0U;
    g_start_gate_ready = (platform_ok != 0U && code_ok != 0U &&
                          g_start_request == 0U) ? 1U : 0U;
    s_start_gate_revision = g_saoma_accept_count;
    if (s_start_pending != 0U)
    {
        s_start_pending = 0U;
        if (g_start_gate_ready != 0U &&
            s_start_pending_revision == g_saoma_accept_count)
        {
            for (i = 0U; i < sizeof(g_start_task_code); i++)
            {
                g_start_task_code[i] = SAOMA_Data[i];
            }
            g_start_request = 1U;
            g_start_gate_ready = 0U;
            g_start_accept_count++;
            accepted = 1U;
        }
        else
        {
            g_start_deny_count++;
        }
    }
    taskEXIT_CRITICAL();
    if (accepted != 0U)
    {
        sprintf(log, "START:%s\r\n", g_start_task_code);
        (void)USART1_SendString(log);
    }
}
```

## 4. App_KeyTask提交事件

原if(Key_GetEvent()!=0U)内的g_start_key_count++一行替换为：

```c
            g_start_key_count++;
            /* 未就绪的按键当场拒绝，不留到扫码后自动启动 */
            taskENTER_CRITICAL();
            if (g_start_gate_ready != 0U && g_start_request == 0U)
            {
                s_start_pending_revision = s_start_gate_revision;
                s_start_pending = 1U;
            }
            else
            {
                g_start_deny_count++;
            }
            taskEXIT_CRITICAL();```

保留原if大括号、Key任务心跳/栈统计/1ms延时，不重复扫描按键、不在Key任务发送串口或显示。

## 5. TaskA两处接入

### 5.1 现有扫码显示if之前

在SAOMA_Update()后、SAOMA_DataReady判断前加入：

```c
        /* 确认启动请求后，保留本轮码和屏幕，不替换任务对象 */
        if (g_start_request != 0U)
        {
            SAOMA_DataReady = 0U;
        }
```

原扫码if/Screen_ShowTaskCode/SCAN日志全部保留。

### 5.2 g_test_tick赋值之前

```c
        App_StartGateUpdate();
```

之后原APP心跳/栈堆/延时全部保留。无需新增文件或改Keil路径，保存main后Rebuild并发完整日志。私人main.c ARMCC编译-c 0错0警告不等于真实链接/实机已经通过。

## 6. 构建后的实物测试顺序

保持VM请求0，下载后复位并让PC13释放稳定。串口助手仍115200/8N1。

1. 不扫码，按PC13一次：没有START日志/启动请求；后续扫码也不能自动出现START。
2. 扫有效码，等屏幕更新及约1秒平台稳定，再释放后按PC13：应出现START:对应完整任务码，g_start_request1、accept_count加1；VM仍0。
3. 此时换扫另一张码：SAOMA诊断可更新，但page0和g_start_task_code保持刚确认的码。
4. 按键保持按下不能反复接受，复位后请求回0且需重新扫码/释放再按。观察故障0、监督/喂狗继续进展和新增调用后的APP余栈。

提供串口日志/屏幕变化和VM读数。宏0下长时间Stop会复位，不用暂停改变量完成测试；两个START事件不能仅凭按键计数当整车真的开始行驶。

## 7. 需求与证据

R：QR-01/RUN-01/FIELD-02；E：QR-03/LOC-03/RUN-06/MOT-04；Q：QR-04仍待技术答疑。本步未实现两个批次业务对象/完整合法性、start_id输入、CAN上电就绪/运动启动、停止及正式重新允许互锁，不能签完整规则/整车READY。下一阶段把请求接到已验证的供电/CAN反馈与四轮控制，完整装机载物低速验证按原阶段顺序。抓放统计仍延后全程可跑后。

证据START_gate_private_compile.json；原main SHA不变，无实际源码修改/链接/烧录，私人两个文件按路径核对后清理。当前扫码实物通过依据用户报告，未补造构建日志、截图或VM/故障读数。
