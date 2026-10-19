#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hwt101_parser.h"
#define APP_HWT_BAUD 115200U
#define USART_SR_RXNE 32U
#define USART_SR_ORE 8U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_PE 1U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define __DMB() ((void)0)
typedef struct {uint32_t SR,DR;} MockUART;
static MockUART uart2;
#define USART2 (&uart2)
static uint32_t tick;
static uint32_t xTaskGetTickCount(void){return tick;}
static uint32_t xTaskGetTickCountFromISR(void){return tick;}
typedef struct { uint32_t tick; uint8_t data; } HwtByte;
static HwtByte s_rx[256];
static volatile uint16_t s_head = 0U, s_tail = 0U;
static volatile uint32_t s_error_epoch = 0U;
static uint32_t s_seen_error = 0U;
static Hwt101Parser s_parser;
volatile uint32_t g_hwt_baud = APP_HWT_BAUD;
volatile uint32_t g_hwt_rx_bytes = 0U, g_hwt_uart_errors = 0U, g_hwt_overflows = 0U;
volatile uint32_t g_hwt_angle_count = 0U, g_hwt_rate_count = 0U, g_hwt_checksum_errors = 0U;
volatile uint32_t g_hwt_valid_mask = 0U;
volatile uint32_t g_hwt_angle_age_ms = 0xFFFFFFFFU, g_hwt_rate_age_ms = 0xFFFFFFFFU;
volatile float g_hwt_yaw_deg = 0.0f, g_hwt_wz_dps = 0.0f;
volatile uint8_t g_hwt_last_frame[11] = {0U};
void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;
    uint8_t byte;
    uint16_t head;
    if (!(sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))) return;
    byte = (uint8_t)USART2->DR; /* SR后读DR清RXNE及错误 */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    { g_hwt_uart_errors++; s_error_epoch++; return; }
    g_hwt_rx_bytes++;
    head = s_head;
    if ((uint16_t)(head - s_tail) >= 256U)
    { g_hwt_overflows++; s_error_epoch++; return; }
    s_rx[head & 255U].data = byte;
    s_rx[head & 255U].tick = (uint32_t)xTaskGetTickCountFromISR();
    __DMB(); s_head = (uint16_t)(head + 1U);
}
/* 任务A每10ms调用；每轮最多128字节，ISR时间戳不以解析时间充新。 */
void HWT101_Update(void)
{
    uint32_t n, i, now;
    HwtByte item;
    for (n = 0U; n < 128U; n++)
    {
        taskENTER_CRITICAL();
        if (s_seen_error != s_error_epoch)
        {
            s_seen_error = s_error_epoch; s_tail = s_head;
            Hwt101Parser_Invalidate(&s_parser);
        }
        if (s_tail == s_head) { taskEXIT_CRITICAL(); break; }
        item = s_rx[s_tail & 255U]; s_tail = (uint16_t)(s_tail + 1U);
        taskEXIT_CRITICAL();
        Hwt101Parser_Feed(&s_parser, item.data, item.tick);
    }
    now = (uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();
    if (s_seen_error != s_error_epoch)
    { s_seen_error = s_error_epoch; s_tail = s_head; Hwt101Parser_Invalidate(&s_parser); }
    g_hwt_valid_mask = Hwt101Parser_Valid(&s_parser, now);
    g_hwt_angle_count = s_parser.angle_count; g_hwt_rate_count = s_parser.rate_count;
    g_hwt_checksum_errors = s_parser.checksum_errors;
    g_hwt_angle_age_ms = (s_parser.seen & 1U) ? now - s_parser.angle_tick : 0xFFFFFFFFU;
    g_hwt_rate_age_ms = (s_parser.seen & 2U) ? now - s_parser.rate_tick : 0xFFFFFFFFU;
    g_hwt_yaw_deg = (float)s_parser.yaw_raw * (180.0f / 32768.0f);
    g_hwt_wz_dps = (float)s_parser.wz_raw * (2000.0f / 32768.0f);
    for (i = 0U; i < 11U; i++) g_hwt_last_frame[i] = s_parser.last_frame[i];
    taskEXIT_CRITICAL();
}

static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL line %d checks %u\n",__LINE__,checks);return 1;}}while(0)
static void frame(uint8_t *f,uint8_t type,int32_t raw){unsigned i;uint16_t u=(uint16_t)raw;memset(f,0,11);f[0]=0x55;f[1]=type;f[6]=(uint8_t)u;f[7]=(uint8_t)(u>>8);for(i=0;i<10;i++)f[10]=(uint8_t)(f[10]+f[i]);}
static void feed(Hwt101Parser *p,uint8_t *f,uint32_t time){unsigned i;for(i=0;i<11;i++)Hwt101Parser_Feed(p,f[i],time);}
static void push(uint8_t v){uart2.SR=USART_SR_RXNE;uart2.DR=v;USART2_IRQHandler();uart2.SR=0;}
static void push_frame(uint8_t *f){unsigned i;for(i=0;i<11;i++)push(f[i]);}
static void reset_rx(void){s_head=s_tail=0;s_error_epoch=s_seen_error=0;g_hwt_rx_bytes=g_hwt_uart_errors=g_hwt_overflows=0;Hwt101Parser_Init(&s_parser);tick=100;HWT101_Update();}
int main(void){Hwt101Parser p;uint8_t f[11],g[11];unsigned i,j;int32_t v;uint32_t seed=7;
Hwt101Parser_Init(&p);CHECK(Hwt101Parser_Valid(&p,0)==0);Hwt101Parser_Feed(NULL,0,0);CHECK(Hwt101Parser_Valid(NULL,0)==0);
/* Signed bounds and randomized payloads including embedded 0x55. */
for(i=0;i<1000;i++){seed=seed*1664525U+1013904223U;v=(int32_t)(seed&65535U)-32768;Hwt101Parser_Init(&p);frame(f,0x53,v);feed(&p,f,100);CHECK(p.yaw_raw==v&&p.angle_count==1&&Hwt101Parser_Valid(&p,100)==1);frame(f,0x52,-v-(v==-32768));feed(&p,f,110);CHECK(p.wz_raw==-v-(v==-32768)&&Hwt101Parser_Valid(&p,110)==3);}
Hwt101Parser_Init(&p);frame(f,0x53,-32768);feed(&p,f,0);CHECK(p.yaw_raw==-32768);frame(f,0x52,32767);feed(&p,f,0);CHECK(p.wz_raw==32767);
CHECK(Hwt101Parser_Valid(&p,500)==3);CHECK(Hwt101Parser_Valid(&p,501)==0);frame(f,0x52,1);feed(&p,f,510);CHECK(Hwt101Parser_Valid(&p,510)==2);
Hwt101Parser_Init(&p);frame(f,0x53,16384);feed(&p,f,0xfffffff0U);CHECK(Hwt101Parser_Valid(&p,100)==1);CHECK(Hwt101Parser_Valid(&p,600)==0);
/* Bad checksum never updates value/tick; byte sliding recovers following frame. */
Hwt101Parser_Init(&p);frame(f,0x53,123);feed(&p,f,100);frame(g,0x53,456);g[10]^=1;feed(&p,g,110);CHECK(p.angle_count==1&&p.yaw_raw==123&&p.angle_tick==100&&Hwt101Parser_Valid(&p,110)==0);feed(&p,f,120);CHECK(p.angle_count==2&&Hwt101Parser_Valid(&p,120)==1);
Hwt101Parser_Init(&p);for(i=0;i<11;i++)if(i!=4)Hwt101Parser_Feed(&p,f[i],100);feed(&p,f,101);CHECK(p.angle_count==1);
Hwt101Parser_Init(&p);Hwt101Parser_Feed(&p,0x55,0);for(i=1;i<11;i++)Hwt101Parser_Feed(&p,f[i],100);CHECK(!p.angle_count);feed(&p,f,101);CHECK(p.angle_count==1);
Hwt101Parser_Init(&p);frame(g,0x51,1);feed(&p,g,100);CHECK(p.other_count==1&&!p.angle_count&&!Hwt101Parser_Valid(&p,100));
Hwt101Parser_Invalidate(&p);CHECK(p.length==0&&p.seen==0);
/* Actual RX ring/IRQ/Update: old arrival time cannot become fresh on drain. */
reset_rx();frame(f,0x53,16384);push_frame(f);frame(g,0x52,-16384);push_frame(g);HWT101_Update();CHECK(g_hwt_rx_bytes==22&&g_hwt_valid_mask==3&&g_hwt_yaw_deg==90.0f&&g_hwt_wz_dps==-1000.0f);
tick=601;HWT101_Update();CHECK(g_hwt_valid_mask==0&&g_hwt_angle_age_ms==501);
reset_rx();push_frame(f);tick=700;HWT101_Update();CHECK(g_hwt_valid_mask==0&&g_hwt_angle_age_ms==600);
/* Per-call budget, queued multiple frames, 16-bit ring counter rollover. */
reset_rx();for(j=0;j<22;j++)push_frame(f);HWT101_Update();CHECK(s_tail==128&&s_parser.angle_count==11);HWT101_Update();CHECK(s_tail==242&&s_parser.angle_count==22);
reset_rx();s_head=s_tail=65530U;push_frame(f);HWT101_Update();CHECK(s_parser.angle_count==1&&s_head==5&&s_tail==5&&g_hwt_valid_mask==1);
/* Every UART hardware error and overflow invalidates old/pending data. */
for(i=0;i<4;i++){static const uint32_t errors[4]={USART_SR_ORE,USART_SR_FE,USART_SR_NE,USART_SR_PE};reset_rx();push_frame(f);HWT101_Update();CHECK(g_hwt_valid_mask==1);uart2.SR=USART_SR_RXNE|errors[i];uart2.DR=0x55;USART2_IRQHandler();uart2.SR=0;HWT101_Update();CHECK(g_hwt_valid_mask==0&&g_hwt_uart_errors==1&&s_tail==s_head);push_frame(f);HWT101_Update();CHECK(g_hwt_valid_mask==1);}
reset_rx();push_frame(f);HWT101_Update();for(i=0;i<257;i++)push((uint8_t)i);HWT101_Update();CHECK(g_hwt_overflows==1&&g_hwt_valid_mask==0&&s_tail==s_head);push_frame(f);HWT101_Update();CHECK(g_hwt_valid_mask==1);
printf("PASS %u checks (actual parser and actual IRQ/ring/update, mocked UART/tick)\n",checks);return 0;}
