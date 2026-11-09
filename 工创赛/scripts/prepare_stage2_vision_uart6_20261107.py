"""Stage2 mainline step 1: USART6 transport; no motion/calibration changes."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/integration12_stage2_vision_20261107/project'


def edit(relative, mutate):
    path = STAGE / relative
    raw = path.read_bytes()
    newline = b'\r\n' if b'\r\n' in raw else b'\n'
    text = raw.decode('latin1').replace('\r\n', '\n')
    result = mutate(text)
    path.write_bytes(result.replace('\n', newline.decode()).encode('latin1'))


def uart(source):
    assert 'void USART6_VisionConfig' not in source
    return source + r'''

/* Stage2 mainline: dedicated PC6/PC7 USART6, owned by Vision_Poll only.
 * USART1 and its existing UartRx task remain independent. */
static QueueHandle_t s_vision_rx_queue = NULL;
static uint8_t s_vision_tx[32];
static volatile uint16_t s_vision_tx_len = 0U, s_vision_tx_pos = 0U;
volatile uint32_t g_uart6_rx_errors = 0U;

void USART6_VisionConfig(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef serial;
    NVIC_InitTypeDef irq;
    s_vision_rx_queue = xQueueCreate(128U, sizeof(uint8_t));
    configASSERT(s_vision_rx_queue != NULL);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    GPIO_Init(GPIOC, &gpio);
    USART_StructInit(&serial);
    serial.USART_BaudRate = 115200U;
    serial.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART6, &serial);
    USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
    irq.NVIC_IRQChannel = USART6_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 6U;
    irq.NVIC_IRQChannelSubPriority = 0U;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    USART_Cmd(USART6, ENABLE);
}

void USART6_VisionEnableReceive(void)
{
    (void)USART6->SR;
    (void)USART6->DR;
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}

uint8_t USART6_VisionReadByte(uint8_t *data)
{
    if(data == NULL || s_vision_rx_queue == NULL)return 0U;
    return xQueueReceive(s_vision_rx_queue, data, 0U)==pdPASS ? 1U : 0U;
}

uint8_t USART6_VisionSend(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    if(data==NULL || !length || length>sizeof(s_vision_tx) || s_vision_tx_len)return 0U;
    for(i=0U;i<length;i++)s_vision_tx[i]=data[i];
    s_vision_tx_pos=0U;s_vision_tx_len=length;
    USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
    return 1U;
}

void USART6_IRQHandler(void)
{
    BaseType_t wake = pdFALSE;
    uint32_t status = USART6->SR;
    uint8_t data;
    if(status & (USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)){
        data=(uint8_t)USART_ReceiveData(USART6);
        if(status & (USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE))g_uart6_rx_errors++;
        else if(s_vision_rx_queue==NULL || xQueueSendFromISR(s_vision_rx_queue,&data,&wake)!=pdPASS)
            g_uart6_rx_errors++;
    }
    if(USART_GetITStatus(USART6,USART_IT_TXE)!=RESET){
        if(s_vision_tx_pos<s_vision_tx_len)USART_SendData(USART6,s_vision_tx[s_vision_tx_pos++]);
        else{USART_ITConfig(USART6,USART_IT_TXE,DISABLE);s_vision_tx_pos=s_vision_tx_len=0U;}
    }
    portEND_SWITCHING_ISR(wake);
}
'''


def header(source):
    assert 'USART6_VisionConfig' not in source
    return source.replace('#endif', '''/* Dedicated Orange Pi vision transport: PC6/PC7, 115200 8N1. */
void USART6_VisionConfig(void);
void USART6_VisionEnableReceive(void);
uint8_t USART6_VisionReadByte(uint8_t *data);
uint8_t USART6_VisionSend(const uint8_t *data, uint16_t length);
extern volatile uint32_t g_uart6_rx_errors;

#endif''')


def main(source):
    marker = '    USART1_Config();'
    assert source.count(marker) == 1
    return source.replace(marker, marker + '\n    if(VEHICLE_LIFT_RULER_ONLY)USART6_VisionConfig();')


def vision(source):
    assert 'USART6_VisionReadByte' not in source
    marker = ' g_src_slot<1U || g_src_slot>3U)'
    # The source wraps this test on one line; use the exact final clause.
    assert 'g_src_slot<1U || g_src_slot>3U)' in source
    source = source.replace('g_src_slot<1U || g_src_slot>3U)',
                            'g_src_slot<1U || g_src_slot>3U || !(g_src_extension_mm>=0.0f && g_src_extension_mm<=170.0f))')
    # The extension must be declared before SrcTxn_Take uses it.
    source = source.replace('volatile float g_src_pick_deg=180.0f,',
                            'volatile float g_src_extension_mm=0.0f;\nvolatile float g_src_pick_deg=180.0f,', 1)
    source = source.replace('volatile float g_src_extension_mm=0.0f;\nstatic uint8_t', 'static uint8_t', 1)
    marker = 'static uint8_t vis_buf[10];'
    source = source.replace(marker, '''volatile uint32_t g_vis_material_valid=0U,g_vis_center_valid=0U;
volatile uint32_t g_vis_mat_consumed=0U,g_vis_metric_valid=0U;
volatile uint32_t g_vis_command_request=0U,g_vis_command_sent=0U,g_vis_command_rejected=0U;
static uint32_t vis_last_byte_ms=0U,vis_errors_seen=0U;
''' + marker)
    marker = 'static void Vision_Byte(uint8_t b)\n{'
    assert source.count(marker) == 1
    source = source.replace(marker, marker + '''
 uint32_t tk=(uint32_t)xTaskGetTickCount();
 if(vis_len && (uint32_t)(tk-vis_last_byte_ms)>150U){vis_len=vis_expect=0U;g_vis_rx_bad++;}
 vis_last_byte_ms=tk;''')
    source = source.replace('  uint32_t tk=(uint32_t)xTaskGetTickCount();\n', '')
    old = '''   {g_vis_mat_left=vis_buf[2];g_vis_mat_right=vis_buf[3];g_vis_mat_down=vis_buf[4];
    g_vis_mat_tick=tk;g_vis_mat_cycle++;}'''
    new = '''   {
    if(vis_buf[2]>6U || vis_buf[3]>6U || vis_buf[4]>6U)g_vis_rx_bad++;
    else{g_vis_mat_left=vis_buf[2];g_vis_mat_right=vis_buf[3];g_vis_mat_down=vis_buf[4];
     g_vis_mat_tick=tk;g_vis_mat_cycle++;g_vis_material_valid=1U;}
   }'''
    assert old in source
    source = source.replace(old, new)
    source = source.replace('g_vis_center_tick=tk;g_vis_center_cycle++;}',
                            'g_vis_center_tick=tk;g_vis_center_cycle++;g_vis_center_valid=1U;}')
    begin = source.index('void Vision_Poll(void)')
    end = source.index('static uint32_t it_platform', begin)
    source = source[:begin] + '''void Vision_Poll(void)
{
 uint8_t b,command[3];uint32_t budget=128U,request;
 if(!g_vis_rx_armed){USART6_VisionEnableReceive();g_vis_rx_armed=1U;}
 if(vis_errors_seen!=g_uart6_rx_errors){
  vis_errors_seen=g_uart6_rx_errors;vis_len=vis_expect=0U;
  g_vis_material_valid=g_vis_center_valid=0U;
 }
 while(budget-- && USART6_VisionReadByte(&b)){g_vis_rx_bytes++;Vision_Byte(b);}
 if(g_vis_test_push){g_vis_test_push=0U;Vision_Byte((uint8_t)g_vis_test_byte);}
 request=g_vis_command_request;
 if(request){
  if(request!=1U && request!=2U && request!=5U && request!=0x14U && request!=0x15U){
   g_vis_command_request=0U;g_vis_command_rejected++;
  }else{
   command[0]=0xaaU;command[1]=(uint8_t)request;command[2]=0xbbU;
   if(USART6_VisionSend(command,3U)){
    g_vis_command_request=0U;g_vis_command_sent++;
    if(request==2U)g_vis_material_valid=0U;
    if(request==5U)g_vis_center_valid=0U;
   }
  }
 }
}

static uint32_t Vision_TakeTarget(uint32_t now)
{
 uint32_t region=g_vis_pick_region,color;
 float dx=0.0f,dy=0.0f;
 g_vis_txn_result=0U;g_vis_metric_valid=0U;
 if(region>2U || !(g_vis_taught_mask&(1U<<region))){g_vis_txn_result=6U;return 0U;}
 if(!g_vis_material_valid || !g_vis_mat_cycle || (uint32_t)(now-g_vis_mat_tick)>3000U){g_vis_txn_result=7U;return 0U;}
 if(g_vis_mat_cycle==g_vis_mat_consumed){g_vis_txn_result=8U;return 0U;}
 color=region==0U?g_vis_mat_left:(region==1U?g_vis_mat_right:g_vis_mat_down);
 if(!color){g_vis_txn_result=9U;return 0U;}
 if(g_vis_mm_per_px>0.0f && g_vis_center_valid && g_vis_center_cycle &&
    (uint32_t)(now-g_vis_center_tick)<=3000U){
  dx=((float)g_vis_center_x-g_vis_ref_x)*g_vis_mm_per_px;
  dy=((float)g_vis_center_y-g_vis_ref_y)*g_vis_mm_per_px;
  g_vis_metric_valid=1U;
 }
 g_src_pick_deg=g_vis_region_deg[region];g_src_extension_mm=g_vis_region_ext_mm[region];
 g_src_chassis_dx_mm=dx;g_src_chassis_dy_mm=dy;
 g_src_txn_seq++;g_src_txn_tick=now;g_src_txn_revision++;
 if(!SrcTxn_Take(now))return 0U;
 g_vis_mat_consumed=g_vis_mat_cycle;return 1U;
}
''' + source[end:]
    begin = source.index('   }else if(request==12U){')
    end = source.index('   }else if(request==11U){', begin)
    source = source[:begin] + '''   }else if(request==12U){
    if(Vision_TakeTarget(now))g_it_state=1U;else g_it_rejected=1U;
''' + source[end:]
    source = source.replace('/* =====', '/* =====', 1)
    return source


def run():
    # Mainline review: dedicated real vision transport, preserving actuator code.
    edit('CHASSIS/SYSTEM/USART/usart.c', uart)
    edit('CHASSIS/SYSTEM/USART/usart.h', header)
    edit('CHASSIS/USER/main.c', main)
    edit('CHASSIS/USER/integration10_complete.inc', vision)
    print('USART6 transport and vision observation consumption prepared; no motor changes.')


if __name__ == '__main__':
    run()
