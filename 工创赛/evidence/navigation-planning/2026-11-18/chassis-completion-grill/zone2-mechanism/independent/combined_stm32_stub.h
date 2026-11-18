#define GPIO_Pin_6 64U
#define GPIO_Pin_7 128U
#define GPIO_Pin_9 512U
#define GPIO_Mode_AF 2U
#define GPIO_PuPd_UP 1U
#define GPIO_Speed_50MHz 2U
#define GPIO_PinSource6 6U
#define GPIO_PinSource7 7U
#define GPIO_PinSource8 8U
#define GPIO_PinSource9 9U
#define GPIO_AF_TIM4 2U
#define GPIO_AF_USART3 7U
#define DISABLE 0U
#define RCC_APB1Periph_TIM4 4U
#define RCC_APB1Periph_USART3 64U
#define USART_IT_RXNE 1U
#define USART_IT_TXE 2U
#define USART_IT_TC 3U
#define USART_Mode_Rx 1U
#define USART_Mode_Tx 2U
#define USART3_IRQn 39U
static TIM_TypeDef tim4;
#define TIM4 (&tim4)
typedef struct {uint32_t USART_BaudRate, USART_Mode;} USART_InitTypeDef;
#define USART3 3U
static void GPIO_PinAFConfig(GPIO_TypeDef *gpio, uint32_t pin, uint32_t af)
{ (void)gpio; (void)pin; (void)af; }
static void USART_StructInit(USART_InitTypeDef *init) { init->USART_Mode = 0; }
static void USART_Init(uint32_t usart, USART_InitTypeDef *init) { (void)usart; (void)init; }
static void USART_ITConfig(uint32_t usart,uint32_t it,uint32_t state)
{ (void)usart; (void)it; (void)state; }
static void USART_Cmd(uint32_t usart,uint32_t state) { (void)usart; (void)state; }

typedef struct {
    uint16_t TIM_Prescaler;
    uint32_t TIM_Period, TIM_CounterMode;
} TIM_TimeBaseInitTypeDef;
typedef struct {
    uint32_t TIM_OCMode, TIM_OutputState, TIM_OCPolarity;
    uint16_t TIM_Pulse;
} TIM_OCInitTypeDef;
#define TIM_CounterMode_Up 0U
#define TIM_OCMode_PWM1 6U
#define TIM_OutputState_Enable 1U
#define TIM_OCPolarity_High 0U
#define TIM_OCPreload_Enable 1U
#define TIM_EventSource_Update 1U
static void TIM_Cmd(TIM_TypeDef *timer,uint32_t enabled) { timer->CR1=enabled; }
static void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef *init) { init->TIM_Prescaler=0; }
static void TIM_TimeBaseInit(TIM_TypeDef *timer,const TIM_TimeBaseInitTypeDef *init)
{ timer->PSC=init->TIM_Prescaler; timer->ARR=init->TIM_Period; }
static void TIM_OCStructInit(TIM_OCInitTypeDef *channel) { channel->TIM_Pulse=0; }
static void TIM_OC1Init(TIM_TypeDef *timer,const TIM_OCInitTypeDef *channel)
{ (void)timer; (void)channel; }
static void TIM_OC2Init(TIM_TypeDef *timer,const TIM_OCInitTypeDef *channel)
{ (void)timer; (void)channel; }
static void TIM_OC1PreloadConfig(TIM_TypeDef *timer,uint32_t value)
{ (void)timer; (void)value; }
static void TIM_OC2PreloadConfig(TIM_TypeDef *timer,uint32_t value)
{ (void)timer; (void)value; }
static void TIM_ARRPreloadConfig(TIM_TypeDef *timer,uint32_t value)
{ (void)timer; (void)value; }
static void TIM_SetCounter(TIM_TypeDef *timer,uint32_t counter) { timer->CNT=counter; }
static void TIM_GenerateEvent(TIM_TypeDef *timer,uint32_t event) { timer->EGR=event; }
static void TIM_SetCompare1(TIM_TypeDef *timer,uint16_t compare) { (void)timer; (void)compare; }
extern void Fixture_Pwm2(uint16_t compare);
static void TIM_SetCompare2(TIM_TypeDef *timer,uint16_t compare) { (void)timer; Fixture_Pwm2(compare); }
