#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hwt101_parser.h"
#define APP_HWT_BAUD 115200U
#define USART_SR_RXNE 32U
#define USART_FLAG_TXE 128U
#define USART_FLAG_TC 64U
#define RESET 0U
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
static unsigned tx_budget=1000,tc_ready=1,tx_length;static uint8_t tx_log[256];
static uint32_t USART_GetFlagStatus(MockUART *p,uint32_t flag){return (flag==USART_FLAG_TXE)?(tx_budget!=0):tc_ready;}
static void USART_SendData(MockUART *p,uint16_t v){if(tx_length<sizeof(tx_log))tx_log[tx_length++]=(uint8_t)v;if(tx_budget)tx_budget--;}
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
/* 非阻塞单笔版本读取。5F无寄存器地址字段，因此此处不混入其他读请求。 */
volatile uint32_t g_hwt_version_request = 1U, g_hwt_version_result = 0U;
volatile uint32_t g_hwt_version_raw = 0U, g_hwt_version_low16 = 0U;
volatile uint32_t g_hwt_version_state = 0U, g_hwt_version_reply_count = 0U;
volatile uint32_t g_hwt_tx_bytes = 0U;
volatile uint8_t g_hwt_version_reply[11] = {0U};
static const uint8_t s_version_command[5] = {0xFFU, 0xAAU, 0x27U, 0x2EU, 0x00U};
static uint32_t s_version_index, s_version_tick, s_version_rx_base, s_version_error;
static uint32_t s_version_start_tick;

static void Hwt_VersionUpdate(uint32_t now)
{
    uint32_t i, errors;
    errors = s_error_epoch;
    if (g_hwt_version_state == 0U)
    {
        if (g_hwt_version_request != 1U) return;
        /* 给模块上电初始化留2秒，不依赖用户计时，不重复发设置。 */
        if ((uint32_t)(now - s_version_start_tick) < 2000U) return;
        g_hwt_version_request = 0U;
        g_hwt_version_result = 0U;
        s_version_index = 0U; s_version_tick = now;
        s_version_rx_base = s_parser.register_count;
        s_version_error = errors;
        g_hwt_version_state = 1U;
    }
    else g_hwt_version_request = 0U; /* 忙时丢弃重复请求，不排队重发 */
    if (errors != s_version_error)
    { g_hwt_version_result = 4U; g_hwt_version_state = 0U; return; }
    if (g_hwt_version_state == 1U)
    {
        if ((uint32_t)(now - s_version_tick) >= 200U)
        { g_hwt_version_result = 2U; g_hwt_version_state = 0U; return; }
        /* 每轮至多5字节；TXE不就绪立即返回，不忙等、不在临界区发串口。 */
        for (i = 0U; (i < 5U) && (s_version_index < 5U); i++)
        {
            if (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) return;
            USART_SendData(USART2, s_version_command[s_version_index++]);
            g_hwt_tx_bytes++;
        }
        s_version_tick = now;
        s_version_rx_base = s_parser.register_count;
        g_hwt_version_state = 2U;
    }
    if (g_hwt_version_state == 2U)
    {
        if ((uint32_t)(now - s_version_tick) >= 200U)
        { g_hwt_version_result = 2U; g_hwt_version_state = 0U; return; }
        if (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) return;
        g_hwt_version_state = 3U;
    }
    if (g_hwt_version_state == 3U)
    {
        if ((s_parser.register_count != s_version_rx_base) &&
            ((uint32_t)(s_parser.register_tick - s_version_tick) <= 1000U) &&
            ((uint32_t)(now - s_parser.register_tick) <= 1000U))
        {
            /* 用无符号类型组合，不把版本字节误作带符号角度。 */
            g_hwt_version_raw = (uint32_t)s_parser.register_data[0] |
                ((uint32_t)s_parser.register_data[1] << 8) |
                ((uint32_t)s_parser.register_data[2] << 16) |
                ((uint32_t)s_parser.register_data[3] << 24);
            g_hwt_version_low16 = g_hwt_version_raw & 0xFFFFU;
            g_hwt_version_reply[0] = 0x55U; g_hwt_version_reply[1] = 0x5FU;
            for (i = 0U; i < 8U; i++) g_hwt_version_reply[i + 2U] = s_parser.register_data[i];
            g_hwt_version_reply[10] = 0U;
            for (i = 0U; i < 10U; i++) g_hwt_version_reply[10] =
                (uint8_t)(g_hwt_version_reply[10] + g_hwt_version_reply[i]);
            g_hwt_version_reply_count++;
            g_hwt_version_result = 1U; g_hwt_version_state = 0U;
        }
        else if ((uint32_t)(now - s_version_tick) >= 1000U)
        { g_hwt_version_result = 3U; g_hwt_version_state = 0U; }
    }
}
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
    Hwt_VersionUpdate(now);
}
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL line %d checks %u\n",__LINE__,checks);return 1;}}while(0)
static void frame(uint8_t *f,uint8_t type,int32_t raw){unsigned i;uint16_t u=(uint16_t)raw;memset(f,0,11);f[0]=0x55;f[1]=type;f[6]=(uint8_t)u;f[7]=(uint8_t)(u>>8);for(i=0;i<10;i++)f[10]=(uint8_t)(f[10]+f[i]);}
static void feed(Hwt101Parser *p,uint8_t *f,uint32_t time){unsigned i;for(i=0;i<11;i++)Hwt101Parser_Feed(p,f[i],time);}
static void push(uint8_t v){uart2.SR=USART_SR_RXNE;uart2.DR=v;USART2_IRQHandler();uart2.SR=0;}
static void push_frame(uint8_t *f){unsigned i;for(i=0;i<11;i++)push(f[i]);}
static void reset_rx(void){g_hwt_version_request=g_hwt_version_state=g_hwt_version_result=0;g_hwt_version_raw=g_hwt_version_low16=g_hwt_version_reply_count=g_hwt_tx_bytes=0;s_version_start_tick=0;tx_budget=1000;tc_ready=1;tx_length=0;s_head=s_tail=0;s_error_epoch=s_seen_error=0;g_hwt_rx_bytes=g_hwt_uart_errors=g_hwt_overflows=0;Hwt101Parser_Init(&s_parser);tick=100;HWT101_Update();}
int rx_checks(void){Hwt101Parser p;uint8_t f[11],g[11];unsigned i,j;int32_t v;uint32_t seed=7;
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

static void start_version(void){reset_rx();s_version_start_tick=tick;g_hwt_version_request=1;tick+=2000;HWT101_Update();}
static void register_frame(uint8_t *f,uint32_t raw){unsigned i;memset(f,0,11);f[0]=0x55;f[1]=0x5f;for(i=0;i<4;i++)f[i+2]=(uint8_t)(raw>>(i*8));for(i=0;i<10;i++)f[10]=(uint8_t)(f[10]+f[i]);}
int main(void){uint8_t f[11];unsigned previous,i;CHECK(rx_checks()==0);previous=checks;
/* Startup read sends exactly documented five bytes; never sends A6/A7/unlock/save. */
reset_rx();g_hwt_version_request=1;s_version_start_tick=tick;tick+=1999;HWT101_Update();CHECK(tx_length==0&&g_hwt_version_request==1);tick++;HWT101_Update();CHECK(tx_length==5&&g_hwt_tx_bytes==5&&g_hwt_version_state==3&&!g_hwt_version_request);CHECK(memcmp(tx_log,"\xff\xaa\x27\x2e\x00",5)==0);
register_frame(f,0x89fe4101U);tick+=20;push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==1&&g_hwt_version_raw==0x89fe4101U&&g_hwt_version_low16==0x4101U&&g_hwt_version_reply_count==1&&g_hwt_version_state==0);CHECK(memcmp(f,(const void*)g_hwt_version_reply,11)==0);tick+=10000;HWT101_Update();CHECK(tx_length==5);
/* A preexisting register packet is not a reply to a new request. */
reset_rx();register_frame(f,7);push_frame(f);HWT101_Update();CHECK(s_parser.register_count==1);g_hwt_version_request=1;tick=2100;HWT101_Update();CHECK(g_hwt_version_state==3&&g_hwt_version_result==0);tick+=1000;HWT101_Update();CHECK(g_hwt_version_result==3&&tx_length==5);
/* No unbounded TX/RX waits, partial TX and stalled TC diagnosed separately. */
reset_rx();tx_budget=0;g_hwt_version_request=1;tick=2100;HWT101_Update();CHECK(g_hwt_version_state==1&&tx_length==0);tick+=200;HWT101_Update();CHECK(g_hwt_version_result==2&&g_hwt_version_state==0&&tx_length==0);
reset_rx();g_hwt_version_request=1;tick=2100;for(i=0;i<5;i++){tx_budget=1;HWT101_Update();tick+=10;}CHECK(tx_length==5&&g_hwt_version_state==3);
reset_rx();tc_ready=0;g_hwt_version_request=1;tick=2100;HWT101_Update();CHECK(g_hwt_version_state==2);tick+=200;HWT101_Update();CHECK(g_hwt_version_result==2&&g_hwt_version_state==0);
start_version();register_frame(f,123);f[10]^=1;tick+=10;push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==0&&g_hwt_checksum_errors==1);tick+=1000;HWT101_Update();CHECK(g_hwt_version_result==3);
/* Valid angle/gyro output is not mistaken for a version, and continues while waiting. */
start_version();frame(f,0x53,16384);tick+=10;push_frame(f);HWT101_Update();CHECK(g_hwt_valid_mask==1&&g_hwt_yaw_deg==90.0f&&g_hwt_version_result==0);tick+=1000;HWT101_Update();CHECK(g_hwt_version_result==3);
/* Duplicate request while busy is consumed; no automatic endless retries. */
start_version();g_hwt_version_request=1;tick+=20;HWT101_Update();CHECK(!g_hwt_version_request&&tx_length==5);tick+=1000;HWT101_Update();tick+=10000;HWT101_Update();CHECK(tx_length==5&&g_hwt_version_result==3);
/* Real RX-error epoch cancels even when a reply was buffered; explicit retry only. */
start_version();register_frame(f,42);push_frame(f);uart2.SR=USART_SR_ORE;USART2_IRQHandler();uart2.SR=0;HWT101_Update();CHECK(g_hwt_version_result==4&&g_hwt_version_raw==0&&g_hwt_version_state==0);g_hwt_version_request=1;HWT101_Update();CHECK(tx_length==10);register_frame(f,0xffffffffU);tick+=10;push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==1&&g_hwt_version_raw==0xffffffffU);
/* Timestamp wrap and late reply cannot masquerade as an in-window success. */
reset_rx();s_version_start_tick=0xfffff000U;tick=s_version_start_tick+2000U;g_hwt_version_request=1;HWT101_Update();tick+=100;register_frame(f,0x12345678U);push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==1&&g_hwt_version_raw==0x12345678U);
start_version();tick+=1100;register_frame(f,99);push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==3&&g_hwt_version_raw==0);
printf("PASS %u new version checks; %u total (mocked UART/tick, actual parser/transport/version logic)\n",checks-previous,checks);return 0;}
