
static void begin_bounded(uint32_t scenario,uint32_t slot,uint32_t offset,uint32_t ack_offset)
{
    reset_fixture();
    inject=scenario;test_tick=slot+offset;tx_done_ms=test_tick;
    first_visible_ms=slot+ack_offset;
    preload=pending_calls=force_rx_late=cancels=launch_depth=last_rx_tick=0U;
    launch_time=0U;
}

void test_entry(void)
{
    CanRxMsg reply;
    CAN1_EmmSpeedReceipt receipt;
    uint32_t slot,case_id,r;
    legacy_cases();
    begin_bounded(1U,100U,0U,0U);
    r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,100U,&reply,&receipt);
#if APP_MOTOR_OFFLINE_ONLY || !ZDT_EMM_SETTINGS_CONFIRMED
    CK(r!=1U && !tx_count && !receipt.mailbox_accepted && !receipt.ack_valid);
#if APP_MOTOR_OFFLINE_ONLY
    r=CAN1_SetEmmSpeedRawBounded(1U,0U,0U,5U,100U,&reply,&receipt);
    CK(r==1U && tx_count==1U && !nonzero_count && receipt.mailbox_accepted && receipt.ack_valid);
#endif
#else
    CK(r==1U && tx_count==1U && receipt.mailbox_accepted && receipt.tx_complete && receipt.ack_valid);
    CK(receipt.tx_ms==100U && receipt.reply_ms==100U && launch_depth==1U);
    CK(!test_depth && reply.ExtId==0x100U && reply.DLC==3U);
    for(case_id=0U;case_id<2U;case_id++){
        slot=case_id?0xfffffffeU:100U;
        begin_bounded(1U,slot,2U,6U);
        r=CAN1_SetEmmSpeedRawBounded(1U,1U,10U,255U,slot,&reply,&receipt);
        CK(r==1U && tx_count==1U && receipt.mailbox_accepted && receipt.tx_complete && receipt.ack_valid);
        CK(receipt.tx_ms==slot+2U && launch_time==receipt.tx_ms && receipt.reply_ms==slot+6U);
        CK(last_tx.ExtId==0x100U && last_tx.DLC==7U && last_tx.Data[4]==255U && !last_tx.Data[5]);

        begin_bounded(1U,slot,3U,3U);
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && !tx_count && !receipt.mailbox_accepted && !receipt.ack_valid);

        begin_bounded(1U,slot,0xffffffffU,0U);
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && !tx_count && !receipt.mailbox_accepted && !receipt.ack_valid);

        begin_bounded(1U,slot,0U,7U);
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && tx_count==1U && receipt.mailbox_accepted && receipt.tx_complete && !receipt.ack_valid);

        begin_bounded(1U,slot,0U,6U);
        force_rx_late=1U;
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && tx_count==1U && receipt.mailbox_accepted && !receipt.ack_valid);

        begin_bounded(1U,slot,0U,0U);
        tx_done_ms=slot+7U;
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && tx_count==1U && receipt.mailbox_accepted && !receipt.ack_valid && cancels==1U);

        begin_bounded(4U,slot,0U,0U);
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && tx_count==1U && !receipt.mailbox_accepted && !receipt.ack_valid && !cancels);

        begin_bounded(5U,slot,0U,0U);preload=4U;
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && !tx_count && !receipt.mailbox_accepted && !receipt.ack_valid);

        begin_bounded(6U,slot,0U,0U);
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && !tx_count && !receipt.mailbox_accepted && !receipt.ack_valid);

        begin_bounded(1U,slot,0U,0U);ack_code=0x12U;
        r=CAN1_SetEmmSpeedRawBounded(1U,0U,10U,5U,slot,&reply,&receipt);
        CK(r!=1U && tx_count==1U && receipt.mailbox_accepted && receipt.tx_complete && !receipt.ack_valid);
        CK(!test_depth);
    }
#endif
}
