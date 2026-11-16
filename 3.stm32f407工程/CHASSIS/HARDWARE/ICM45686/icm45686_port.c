#include "icm45686_port.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t edge_sequence, edge_cycle;
uint32_t IcmPort_Cycles(void) { return DWT->CYCCNT; }
uint32_t IcmPort_CyclesPerUs(void) { return SystemCoreClock / 1000000U; }
uint32_t IcmPort_Millis(void) { return xTaskGetTickCount() * portTICK_PERIOD_MS; }
uint32_t IcmPort_Lock(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
void IcmPort_Unlock(uint32_t p) { __set_PRIMASK(p); }
void IcmPort_DelayMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms) + 1U); }
int IcmPort_DelayUs(uint32_t us)
{
    uint32_t start = IcmPort_Cycles(), loops = 100000U;
    while ((uint32_t)(IcmPort_Cycles() - start) < us * IcmPort_CyclesPerUs())
        if (--loops == 0U) return -1;
    return 0;
}
int IcmPort_Init(void)
{
    GPIO_InitTypeDef gpio;
    uint32_t start;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    start = IcmPort_Cycles();
    __NOP(); __NOP(); __NOP(); __NOP();
    if (!IcmPort_CyclesPerUs() || IcmPort_Cycles() == start) return -1;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_SYSCFG, ENABLE);
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_3; gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOE, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_4; gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN; GPIO_Init(GPIOE, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF; gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
    /* Mode3 /64，APB2=84MHz时1.3125MHz；本模块独占SPI1。 */
    SPI1->CR1 = (1U<<0) | (1U<<1) | (1U<<2) | (5U<<3) | (1U<<8) | (1U<<9);
    SPI1->CR2 = 0U; SPI1->CR1 |= 1U<<6;
    EXTI->IMR &= ~(1U<<4);
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~15U) | 4U;
    EXTI->RTSR |= 1U<<4; EXTI->FTSR &= ~(1U<<4);
    return 0;
}
static int wait_flag(uint32_t flag, uint32_t set, uint32_t start, uint32_t *loops)
{
    if (SPI1->SR & 0x170U) return -1;
    if ((uint32_t)(IcmPort_Cycles()-start) >= 1000U*IcmPort_CyclesPerUs()) return -1;
    while (((SPI1->SR & flag) != 0U) != set) {
        if ((SPI1->SR & 0x170U) || --*loops == 0U || (uint32_t)(IcmPort_Cycles()-start) >= 1000U*IcmPort_CyclesPerUs()) return -1;
    }
    return 0;
}
static int transfer(uint8_t reg, uint8_t *out, const uint8_t *in, uint32_t n)
{
    uint32_t start=IcmPort_Cycles(), loops=200000U, i;
    uint8_t discard;
    if (SPI1->SR & 0x171U) goto fail;
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);
    for (i=0U; i<=n; ++i) {
        if (wait_flag(2U,1U,start,&loops)) goto fail;
        *(__IO uint8_t *)&SPI1->DR = i ? (in ? in[i-1U] : 0U) : reg;
        if (wait_flag(1U,1U,start,&loops)) goto fail;
        discard=*(__IO uint8_t *)&SPI1->DR;
        if (i && out) out[i-1U]=discard;
    }
    if (wait_flag(128U,0U,start,&loops)) goto fail;
    GPIO_SetBits(GPIOE, GPIO_Pin_3); return 0;
fail:
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
    /* 超时后可能仍在途/RXNE/OVR，硬复位本模块独占SPI1，避免下笔吃旧DR。 */
    RCC->APB2RSTR |= 1U<<12;
    __DSB();
    RCC->APB2RSTR &= ~(1U<<12);
    SPI1->CR1 = (1U<<0) | (1U<<1) | (1U<<2) | (5U<<3) | (1U<<8) | (1U<<9);
    SPI1->CR2 = 0U; SPI1->CR1 |= 1U<<6;
    return -1;
}
int IcmPort_Read(uint8_t reg, uint8_t *data, uint32_t n) { return transfer(reg|128U,data,0,n); }
int IcmPort_Write(uint8_t reg, const uint8_t *data, uint32_t n) { return transfer(reg&127U,0,data,n); }
void IcmPort_Edge(uint32_t *seq, uint32_t *cycle) { *seq=edge_sequence; *cycle=edge_cycle; }
uint32_t IcmPort_Pending(void) { return EXTI->PR & (1U<<4); }
void IcmPort_EnableEdge(void)
{
    uint32_t saved=IcmPort_Lock();
    EXTI->PR=1U<<4; edge_sequence=0U; edge_cycle=0U;
    NVIC_ClearPendingIRQ(EXTI4_IRQn); NVIC_SetPriority(EXTI4_IRQn,4U);
    NVIC_EnableIRQ(EXTI4_IRQn); EXTI->IMR |= 1U<<4;
    IcmPort_Unlock(saved);
}
void EXTI4_IRQHandler(void)
{
    if (IcmPort_Pending()) {
        EXTI->PR=1U<<4; edge_cycle=IcmPort_Cycles(); ++edge_sequence;
    }
}
