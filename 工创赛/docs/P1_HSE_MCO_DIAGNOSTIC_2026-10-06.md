# 自制F407主控板HSE实际频率诊断：MCO1输出

日期：2026-10-06。状态：工程诊断（E）步骤准备，尚未执行。用户已确认有示波器和万用表；当前HSE输入PLL不锁，保留位修正后的HSI对照已在板上读到168MHz。实际晶振丝印/频率、PA8物理测点及是否空闲仍待确认，不把本说明作为已完成测量。

## 前置条件与来源

- 先读competition_requirements_2027.md，检查LOC-03、RUN-06、PWR-01；R和官方Q不变。
- 实际工程保持`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/`，用户手动改代码，新注释中文；本轮不写或修改`.c/.h`。
- 本机F4标准库的stm32f4xx_rcc.c说明MCO1使用PA8、GPIO必须为AF模式、HSE可作为源并分频1～5；GPIO_AF_MCO和RCC_MCO1Div_5已核对存在，GPIO/RCC模块已加入工程。
- 引脚计划PA8为首版DNP的ToF_R_XSHUT候选，软件未使用；这不是实物空闲的证明。用户必须确认PA8已引出且不接其他模块，再执行测频配置和接探头；不清楚时先看实际板照片/网点。不要求改PCB或猜某个焊盘。
- 晶振节点探测会增加负载，[ST AN2867](https://www.st.com/resource/en/application_note/an2867-oscillator-design-guide-for-stm8afals-stm32-mcus-and-mpus-stmicroelectronics.pdf)说明探头电容影响及低电容测量条件。当前选择缓冲MCO输出测频，不给普通探头直接接晶振脚的操作。

## 用户手动插入位置和代码（PA8确认后）

退出调试，在CORE/system_stm32f4xx.c的SetSysClock函数内，找到`if (HSEStatus == (uint32_t)0x01)`，在其大括号内最前面、PWR电压配置及PLL配置之前插入下列临时块。不能放main.c：当前程序尚未到main。

```c
/* 临时测频：PA8输出外部HSE的五分频，执行前确认该引脚空闲 */
{
    GPIO_InitTypeDef mco_gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* 时钟使能后读回，保证后续GPIO访问顺序 */
    (void)RCC->AHB1ENR;

    mco_gpio.GPIO_Pin   = GPIO_Pin_8;
    mco_gpio.GPIO_Mode  = GPIO_Mode_AF;
    mco_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    mco_gpio.GPIO_OType = GPIO_OType_PP;
    mco_gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &mco_gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

    RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_5);
}
```

局部块使声明位置兼容ARMCC5的C语法；只用自动局部变量和既有库接口，不依赖尚未完成的全局数据初始化。保留当前M8/N336/P2/Q7、HSE_VALUE=8000000及PLLCFGR字段掩码修正，不跳过PLL等待，不更换PLL频率。

保存、Rebuild、用户下载并完整复位后运行。代码先于PLL等待执行，即使再次停在该循环，已配置的MCO输出仍可测。MCO只更新RCC_CFGR的输出源/分频字段，不代表SYSCLK已切换PLL；当前若仍不锁，CFGR可为0x07409400，SWS低位仍0。

## 示波器测量

确认实际PA8测点后，CH1使用10×探头，示波器通道衰减也设10×，输入1MΩ/DC耦合；地夹只接板GND，探头尖接PA8测试点。参考起始设置为1V/div、200ns/div、上升沿触发约1.5V，打开CH1频率和周期测量。使用尽量短的地线；不直接夹细间距芯片脚或不明焊盘。

HSE实际频率=PA8实测频率×5：

| PA8读数 | 推算HSE | 判断范围 |
|---|---|---|
| 约1.6MHz、周期625ns | 约8MHz | 名义频率匹配；仍需观察稳定性/启动重复性，不能因此断言无虚焊 |
| 约5MHz | 约25MHz | 优先核对晶振实物型号及与8MHz声明的冲突，不先调整软件掩盖冲突 |
| 跳变、断续、异常脉冲 | 不直接给有效频率 | 检查信号/触发/连接和外部时钟质量，按波形继续诊断 |
| 没有信号 | 未知 | 先核对测点、探头、PA8 AF/MCO配置与HSE就绪，不直接认定晶振未起振 |

只取同一次启动的波形和调试读数，保存频率/周期、衰减设置、测点照片、RCC_CFGR和GPIOA配置。MCO可观察数字频率/间断，不能替代直接晶振幅度/驱动电平或模拟裕量测试。完成诊断后撤销临时PA8输出，恢复该引脚的项目用途再继续其他模块。

未闭环：PA8物理可用性、晶振标签/型号、实际波形、外部HSE PLL锁定与正式时钟验收。仪表存在不等于测量通过；本轮只读核对和步骤准备，没有源码修改或硬件操作。
