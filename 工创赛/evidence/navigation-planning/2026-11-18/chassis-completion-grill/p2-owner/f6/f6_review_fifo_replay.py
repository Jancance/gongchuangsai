from pathlib import Path


replay = Path(__file__).with_name('f6_independent_replay.py')
source = replay.read_text(encoding='utf-8')
needle = "namespace['UNIT'] = unit + EXTRA.read_text(encoding='ascii')"
assert source.count(needle) == 1
source = source.replace(needle, '''
unit = replace(unit, 'return rx_head<rx_count;}',
    'return (uint8_t)(rx_count-rx_head);}')
unit = replace(unit, 'else {CanRxMsg *q=&rx_queue[rx_count++];', """else {
  if (inject==7U || inject==8U) {
      uint32_t n=inject==7U?1U:2U;
      while(n--){
          CanRxMsg *foreign=&rx_queue[rx_count++];
          memset(foreign,0,sizeof(*foreign));
          foreign->IDE=CAN_Id_Extended;foreign->RTR=CAN_RTR_Data;
          foreign->ExtId=0x200U;foreign->DLC=3U;
          foreign->Data[0]=f;foreign->Data[1]=2U;foreign->Data[2]=0x6bU;
      }
  }
  CanRxMsg *q=&rx_queue[rx_count++];""")
unit += EXTRA.read_text(encoding='ascii').replace('void test_entry(void)', 'static void inherited_cases(void)')
unit += r"""
void test_entry(void)
{
    CanRxMsg reply;
    CAN1_EmmSpeedReceipt receipt;
    uint32_t result;
    inherited_cases();

    begin_bounded(1U,100U,0U,20U);
    result=CAN1_StopMotor(1U,&reply);
    CK(result==1U && test_tick==120U && reply.Data[0]==0xfeU);

    begin_bounded(1U,100U,0U,CAN1_SINGLE_REPLY_WAIT_MS);
    result=CAN1_StopMotor(1U,&reply);
    CK(result==5U && test_tick==100U+CAN1_SINGLE_REPLY_WAIT_MS);
    CK(g_can_ctrl_fault_latched && !s_can1_normal_ready);

    begin_bounded(1U,100U,0U,0U);
    s_can1_normal_ready=0U;
    result=CAN1_SetEmmSpeedRawBounded(1U,0U,0U,5U,100U,&reply,&receipt);
    CK(result==CAN1_TIMING_WINDOW && !receipt.mailbox_accepted && !tx_count);

    begin_bounded(7U,100U,0U,6U);
    result=CAN1_SetEmmSpeedRawBounded(1U,0U,0U,5U,100U,&reply,&receipt);
    CK(result==1U && receipt.ack_valid && receipt.reply_ms==106U);
    CK(rx_head==rx_count && rx_count==2U);

    begin_bounded(8U,0xfffffffdU,0U,5U);
    result=CAN1_SetEmmSpeedRawBounded(1U,0U,0U,5U,0xfffffffdU,&reply,&receipt);
    CK(result==1U && receipt.ack_valid && receipt.reply_ms<=3U);
    CK(rx_head==rx_count && rx_count==3U);
}
"""
namespace['UNIT']=unit
''')
source = source.replace("prefix='f6_independent_'", "prefix='f6_review_fifo_'")
exec(compile(source, str(replay), 'exec'), {'__file__': str(replay), '__name__': '__main__'})
