# 独立看门狗IWDG：监督统一喂狗与复位默认关VM

> 最新接续2026-10-13：IWDG本轮基础门禁闭环，无调试串口READY仅一行/R21，LED持续/启动蜂鸣不反复/VM0V；宏0保留。下一项串口屏UART4：用户确认原屏TJC4832T135_011C_A01，原F103两行完整码/9600功能保留；成品U4图纸已核对，唯一先验实际屏VCC及TX空闲电压，再增量迁移。后续扫码UART5/正式READY，八阶段不变，完整安全待验。

CAN测试清理后正常启动已过，当前VM请求0/RTOS及监督fault0/平台ready1/CAN ready0、实测VM0。用户手动新增SYSTEM/WATCHDOG模块，并增量接入main/safety；保留现有SPL、FreeRTOS原生API、CAN和故障保持，不改PCB、option bytes或电机菜单。

## 1. 器件依据与本步参数

厂家事实：STM32F407 IWDG由LSI时钟驱动，启动后不能靠正常软件关闭，超时触发MCU复位；DBG_IWDG_STOP控制调试器暂停CPU时是否冻结计数，RCC的IWDGRST标志可用于识别复位。已核对本工程ST SPL V1.3.0的iwdg/dbgmcu/rcc源和头文件；官方参考为[RM0090](https://www.st.com/resource/en/reference_manual/DM00031020.pdf)第21章/调试支持章节及[DS8626 Rev12](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)第105页Table35。RM在线全文获取不稳定，本次使用其官方搜索摘录和本地ST驱动确认接口/调试行为，数据手册Table35已获取。

以下选择均为工程建议（E）：分频64、重装999，按LSI典型32kHz算名义超时为64×1000/32000=2秒。数据手册表条件下LSI频率17～47kHz，对应计算约1.36～3.76秒，不是本板实测值或官方比赛时限；后续记录实际复位和响应证据。

初始化放Power_Init/Safety_Init之后、任务创建之前，先保证输出默认禁止；运行期间只由监督任务在本轮检查通过后喂狗。VM关闭时不要求电机回复，上电3秒有限等待内允许平台健康进展，等待到期或已有健康后数据失效均由现有CAN guard锁定故障、关VM并停喂。看门狗是调度/程序卡住时的后备复位，不能代替已实现的更早故障关VM。

默认调试冻结为1，便于Stop查看Watch；正常运行时计数继续。正式无调试运行/不冻结行为另验，不能把调试冻结说成暂停CPU也会自动关VM。现有硬件开关独立关VM仍有效。

## 2. 新建SYSTEM/WATCHDOG/watchdog.h

```c
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

extern volatile uint32_t g_watchdog_started;
extern volatile uint32_t g_watchdog_feed_count;
extern volatile uint32_t g_reset_iwdg;
extern volatile uint32_t g_reset_flags;
/* 临时复位测试，验证后连同喂狗中的判断一起删除 */
extern volatile uint32_t g_watchdog_test_stop_feed;

uint32_t Watchdog_Init(void);
void Watchdog_Feed(void);

#endif
```

## 3. 新建SYSTEM/WATCHDOG/watchdog.c

```c
#include "watchdog.h"
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_dbgmcu.h"
#include "stm32f4xx_rcc.h"

/* 当前台架调试为1；不冻结测试/正式设置时改0并重新构建 */
#define WATCHDOG_DEBUG_FREEZE 1U
#define WATCHDOG_WAIT_LIMIT   1000000U
#define WATCHDOG_RELOAD       999U

volatile uint32_t g_watchdog_started = 0U;
volatile uint32_t g_watchdog_feed_count = 0U;
volatile uint32_t g_reset_iwdg = 0U;
volatile uint32_t g_reset_flags = 0U;
volatile uint32_t g_watchdog_test_stop_feed = 0U;

/* 调度前调用：1成功，2低速时钟未就绪，3寄存器更新超时 */
uint32_t Watchdog_Init(void)
{
    uint32_t wait_count = 0U;

    g_watchdog_started = 0U;
    g_watchdog_feed_count = 0U;
    /* 先保存上次复位原因，再清硬件标志，避免下次误认旧事件 */
    g_reset_flags = RCC->CSR & 0xFE000000U;
    g_reset_iwdg =
        (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) ? 1U : 0U;
    RCC_ClearFlag();

    if (WATCHDOG_DEBUG_FREEZE == 1U)
    {
        DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, ENABLE);
    }
    else
    {
        DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, DISABLE);
    }

    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
        wait_count++;
        if (wait_count >= WATCHDOG_WAIT_LIMIT)
        {
            return 2U;
        }
    }

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(WATCHDOG_RELOAD);
    IWDG_Enable();
    wait_count = 0U;
    while ((IWDG_GetFlagStatus(IWDG_FLAG_PVU) != RESET) ||
           (IWDG_GetFlagStatus(IWDG_FLAG_RVU) != RESET))
    {
        wait_count++;
        if (wait_count >= WATCHDOG_WAIT_LIMIT)
        {
            return 3U;
        }
    }
    IWDG_ReloadCounter();
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    g_watchdog_started = 1U;
    return 1U;
}

/* 正常运行只由监督检查通过后的分支调用 */
void Watchdog_Feed(void)
{
    if (g_watchdog_started != 1U)
    {
        return;
    }
    /* 临时测试：停止重新装载，其他任务仍正常运行 */
    if (g_watchdog_test_stop_feed == 1U)
    {
        return;
    }
    IWDG_ReloadCounter();
    g_watchdog_feed_count++;
}
```

等待循环只是有界次数，不当作精确毫秒延时；显式等待LSI就绪，并在启动后等待PR/RLR更新再装载。若启动失败保持VM默认关闭，main断言进入已有安全停机；已启动的硬件计数不能用返回错误来关闭。g_reset_flags保存reset状态位而非整个CSR，可能多个标志并存，不把原始值写死。

## 4. Keil加入三份源文件与头文件路径

工程已有两份标准库源文件，本轮不用复制或修改它们。在对应分组Add Existing Files加入：

- `.\SYSTEM\WATCHDOG\watchdog.c`
- `.\StdPeriph_Driver\src\stm32f4xx_iwdg.c`
- `.\StdPeriph_Driver\src\stm32f4xx_dbgmcu.c`

Options for Target → C/C++ → Include Paths追加 `.\SYSTEM\WATCHDOG`，原路径保持。watchdog.c已显式include所需外设头，本步无需修改stm32f4xx_conf.h或时钟/RTOS配置。

## 5. USER/main.c接初始化

头文件区添加：

```c
#include "watchdog.h"
```

在其他全局创建/诊断变量附近添加：

```c
/* 看门狗初始化结果，成功后才启动应用任务 */
volatile uint32_t g_watchdog_init_result = 0U;
```

main原Power_Init/Safety_Init之后、任何xTaskCreate之前添加：

```c
g_watchdog_init_result = Watchdog_Init();
configASSERT(g_watchdog_init_result == 1U);
```

保留VM请求默认0、采集默认1、五任务创建和断言、原CAN/串口/按键。不要在LED任务、SysTick、中断、任务A或无条件循环中加喂狗。

## 6. SYSTEM/SAFETY/safety.c统一健康喂狗

include区添加：

```c
#include "watchdog.h"
#include "freertos_hooks.h"
```

在Safety_Update函数末尾，原Power_Enable/Disable的if/else完整结束之后、函数最后大括号之前添加：

```c
/* 本轮任务/CAN检查与输出决策完成，只有健康才继续喂狗 */
if ((g_safety_ready == 1U) && (g_rtos_fault == 0U))
{
    Watchdog_Feed();
}
```

不能加在Safety_Init或Safety_CanGuardUpdate内，也不能加在每个任务心跳函数中。现有Safety_Update由唯一监督任务每20ms调用；CAN故障/任务失联先在同次检查中撤销ready与VM，随后不喂狗。RTOS致命hooks直接关VM并停机，不再有监督喂狗。上电等待是有限状态，超期会fault，不能无限依靠“等待”保持喂狗。

## 7. 本次先保存/Rebuild

用户完成2/3/4/5/6节步骤并提交完整构建日志。新增两自建文件、两标准库文件加入工程、main/safety小段接入，原实际源仍由用户操作。私人编译不能替代链接/下载与实际复位。

## 8. 正常运行与调试暂停门禁

构建通过后硬件先关VM，用户下载/F5约3秒再暂停，Watch观察：

```text
g_watchdog_init_result
g_watchdog_started
g_watchdog_feed_count
g_watchdog_test_stop_feed
g_reset_iwdg
g_reset_flags
g_vm_test_request
g_rtos_fault
g_safety_fault
g_safety_ready
g_can_guard_ready
g_safety_count
g_stack_a_words
g_safety_stack_words
g_heap_min_bytes
```

预期init_result1/started1/test_stop_feed0、两个fault0/平台ready1、VM请求0/CAN ready0/实际VM0。feed_count在恢复运行后应增长。reset_iwdg先记录基线，不将第一次读到历史1直接当成本轮故障；保存/清硬件标志是为了后续复位判断。新栈堆以实测为准。

确认正常喂狗后Stop保持约8秒，CPU不运行时feed_count不增长且不应因IWDG复位；再F5运行约2秒暂停，feed_count继续增而不是整轮启动重置。该检查只接受宏1下“调试暂停冻结”的行为，不说明无调试暂停/死机安全已经全过。

正常启动/喂狗增长及暂停后恢复计数的流程观察已取得：feed697→1024、监督703→1030，两fault0/IWDG复位0、VM请求0、用户VM0V。没有独立记录精确暂停时长或暂停内静态配对，不补造测量；推进第9节复位前健康供电基线。

## 9. 后续停止喂狗：实际IWDG复位与VM默认关闭

正常运行/调试暂停检查通过后，在test_stop_feed0的健康状态恢复硬件软件允许档，VM请求1/F5约4秒，确认四台新鲜/两个ready1/fault0且实际VM有电。保持开关允许和VM请求1，暂停设置g_watchdog_test_stop_feed=1，F5并保持运行约8秒后再暂停。正常结果是硬件复位，重启后：

```text
g_reset_iwdg = 1
g_watchdog_test_stop_feed = 0
g_vm_test_request = 0
g_rtos_fault = 0
g_safety_fault = 0
g_watchdog_started = 1
g_safety_ready = 1
g_can_guard_ready = 0
VM = 0 V
```

若调试器在复位入口停住，按F5让新启动完成，再观察保存的标志；不要把旧RAM值当已经完成的CRT初始化。需记录复位前供电、复位标志/计数重启和实际VM0；名义2秒不作为精确实测结论。测试变量默认0，正常重启会恢复喂狗，避免反复测试复位。

该例验证硬件计数和MCU复位后默认禁止，没有模拟真实高优先级死循环/全中断关闭/所有时钟异常；这些覆盖可针对性验证，不能代替完整安全层。已有任务/CAN故障关VM不重做；IWDG是后备复位，不是正常停机入口。

## 10. 停喂测试通过后清理临时入口

硬件关VM/退出调试，删除watchdog.h里的g_watchdog_test_stop_feed extern、watchdog.c里的该变量及Watchdog_Feed临时判断，保留正式started检查/ReloadCounter/count。用户Rebuild/下载后正常启动验证；测试Watch行可删除。

调试宏1不是临时故障入口，清理阶段保留便于教学。清理后构建及正常启动已过，现进入下一节不冻结配置验收；没有修改option bytes或外部看门狗硬件。本阶段通过后按既定优先级做UART4屏幕、UART5扫码和正式READY/启动/重新允许。

### 10.1 清理后不冻结配置：先只修改宏并构建

清理程序正常启动图16a3e64c/VM0V已过：初始化/started1、feed246、监督252、IWDG复位0、VM请求0、两fault0/平台ready1/CAN ready0。下一步用户硬件关VM/退出调试，在实际SYSTEM/WATCHDOG/watchdog.c顶部，将原宏和注释改为：

```c
/* 不冻结验证配置：CPU调试暂停时看门狗继续计时 */
#define WATCHDOG_DEBUG_FREEZE 0U
```

仅这处修改，保留所有正式喂狗和复位记录，不加回临时停喂变量。用户Rebuild并提供完整日志，构建过后再安排新配置运行、暂停时实际计时及无调试独立运行观察。宏0下长时间Stop可能导致复位，不沿用宏1的暂停冻结预期；本次配置验收通过前不签完整看门狗收尾。后续教学若需暂停检查，明确按调试配置另行切换，不静默改动已验配置。

### 10.2 宏0构建通过后：暂停CPU仍计时验证

2026-10-13用户宏0正式Rebuild/链接0错误0警告，Code15720/RO424/RW416/ZI22528、2秒，实际源只读核对宏0/无临时入口。仅配置构建通过，下一步实机：

1. 硬件先关VM，用户下载新程序进入调试，保持VM请求0，F5运行约5秒。
2. 点击Stop并保持约8秒，不手动Reset或重新下载。
3. 按F5让复位后的程序运行约3秒，暂停截图并测VM；若自动停复位入口，F5完成新启动后再观察。
4. 预期保存IWDG复位标志1、初始化/started1、VM请求0/两fault0/平台ready1/CAN ready0、实测VM0。复位入口未完成CRT初始化时RAM不是新启动结果，不直接读取旧值判定。

宏0暂停时不再冻结，CPU停住无法喂狗，故本轮预期是复位。精确时长不是实测保证，不重复临时停喂注入。通过后再安排无调试独立运行观察；正常教学若恢复宏1，必须明确记录配置，不把两种暂停行为混为同一门禁。

用户本轮观察补充（2026-10-13）：点击Stop等待期间自动重新运行；图6e57aaec显示IWDG复位1/flags24000000、初始化/started1、两fault0、VM请求0/平台ready1/CAN ready0，用户VM0V。本会话宏0暂停复位/默认关VM已过，自动继续时无需补按F5重复测试；不假定所有调试器都相同行为。

### 10.3 无调试正常运行观察

不再改代码或Rebuild。串口助手沿用已验收的USART1 115200 8N1，先清空接收区，退出Keil调试模式，复位主控一次，保持原供电与VM默认请求0，连续运行30秒，不再进入调试或点击Stop。

观察实际TaskA每次启动只发送一次的`F407 FreeRTOS READY`：这30秒内应只有本次启动的一行，不应每隔几秒重新出现；LED保持周期闪烁，启动蜂鸣不应反复，实际VM0。提供串口接收截图、LED/蜂鸣观察和VM读数。若复位键不可用先说明硬件现状，不把PC13启动/自锁键当复位。此基础提示不是正式整车READY，30秒只是本次基础观察，不代替全部任务负载/长期可靠性验收。

## 11. 需求、来源和未闭环

检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E），官方Q不变。IWDG/LSI/调试位/复位标志为厂家器件事实，不是竞赛规则；名义2秒、debug宏1、监督统一喂狗为E。来源为本地ST SPL V1.3.0驱动/当前用户源、ST数据手册Table35和RM0090官方摘录。私人编译结果另记；真实构建、健康喂狗、调试暂停后恢复观察及实际IWDG复位/默认关VM已过；测试入口清理后的正式构建已过，清理程序正常启动已过，最终不冻结配置下暂停CPU复位及无调试基础独立运行观察已过，本轮基础门禁闭环；完整异常/负载/长期可靠性及整车安全仍待验。助手不改真实源码/PCB或操纵硬件。
