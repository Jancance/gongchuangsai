
enum {
    READ_OK = 1U, READ_FOREIGN, READ_PARTIAL, READ_DUPLICATE,
    READ_OVERLONG, READ_BAD_TRAILER, READ_BAD_DLC
};

static uint32_t first_part_ms;

static void setup_read_frames(uint8_t command)
{
    uint32_t i;
    for (i=0U; i<rx_count; i++)
        read_visible[i]=(i+1U==rx_count) ? first_visible_ms : first_part_ms;
    if (read_case==READ_FOREIGN) {
        CanRxMsg last=rx_queue[rx_count-1U];
        rx_queue[rx_count-1U].ExtId+=0x100U;
        rx_queue[rx_count]=last;
        rx_queue[rx_count].Data[0]=0x1FU;
        read_visible[rx_count++]=first_visible_ms;
        rx_queue[rx_count]=last;
        read_visible[rx_count++]=first_visible_ms;
    }
    if (read_case==READ_PARTIAL) rx_count--;
    if (read_case==READ_DUPLICATE) rx_queue[rx_count-1U].ExtId--;
    if (read_case==READ_OVERLONG) rx_queue[rx_count-1U].DLC++;
    if (read_case==READ_BAD_TRAILER) rx_queue[rx_count-1U].Data[rx_queue[rx_count-1U].DLC-1U]=0U;
    if (read_case==READ_BAD_DLC) {
        if (command==0x3AU) rx_queue[0].DLC=2U;
        else rx_queue[0].DLC=1U;
    }
}

static void can_read_case(uint8_t system, uint32_t scenario, uint32_t slot, uint32_t offset,
                          uint32_t ending, uint32_t expected)
{
    CanRxMsg reply;
    CAN1_EmmSystemFeedback feedback;
    uint32_t r, reply_ms=123U;
    memset(&reply,0xcc,sizeof(reply));
    memset(&feedback,0xcc,sizeof(feedback));
    if (system) {
        r=CAN1_ReadEmmSystemFeedbackBounded(2U,slot,&reply,&feedback);
        if (r==1U) {
            CK(feedback.address==2U && feedback.position_sign==1U && feedback.valid);
            CK(feedback.request_ms==slot+offset && feedback.reply_ms==slot+ending);
            CK(feedback.position_counts==65536U && feedback.speed_decirpm==-500);
            CK(feedback.home_flags==3U && feedback.motor_flags==3U);
        } else {
            CAN1_EmmSystemFeedback zero;
            memset(&zero,0,sizeof(zero));
            CK(memcmp(&feedback,&zero,sizeof(zero))==0);
        }
    } else {
        r=CAN1_ReadStatusBounded(2U,slot,&reply,&reply_ms);
        CK(reply_ms==(r==1U ? slot+ending : 0U));
        if (r==1U) CK(reply.DLC==3U && reply.Data[1]==2U);
    }
    ck(r==expected,10000000U+system*1000000U+scenario*100000U+
       (offset&15U)*1000U+ending*100U+expected*10U+r);
    CK(test_depth==0U && s_can1_normal_ready==1U);
    if (r==1U) {
        CK(tx_count==1U && last_tx.ExtId==0x200U && launch_depth==1U);
        CK(launch_time==slot+offset && last_rx_tick==slot+ending);
        CK(last_tx.Data[0]==(system ? 0x43U : 0x3AU) && last_tx.DLC==(system ? 3U : 2U));
        CK(last_tx.Data[last_tx.DLC-1U]==0x6BU && (!system || last_tx.Data[1]==0x7AU));
    }
    CK(read_case==scenario);
}

static void read_begin(uint32_t scenario, uint32_t slot, uint32_t offset, uint32_t ending)
{
    begin_bounded(1U,slot,offset,ending);
    read_case=scenario;
    first_part_ms=slot+3U;
}

void test_entry(void)
{
    uint32_t pass, system, slot;
    CanRxMsg reply;
    CAN1_EmmSystemFeedback feedback;
    uint32_t reply_ms=111U;
    speed_cases();
    for (pass=0U; pass<2U; pass++) {
        slot=pass ? 0xfffffffeU : 100U;
        for (system=0U; system<2U; system++) {
            read_begin(READ_OK,slot,0U,4U);
            can_read_case(system,READ_OK,slot,0U,4U,1U);
            read_begin(READ_FOREIGN,slot,2U,6U);
            can_read_case(system,READ_FOREIGN,slot,2U,6U,1U);
            read_begin(READ_OK,slot,3U,3U);
            can_read_case(system,READ_OK,slot,3U,3U,CAN1_TIMING_WINDOW);
            CK(!tx_count);
            read_begin(READ_OK,slot,0xffffffffU,0U);
            can_read_case(system,READ_OK,slot,0xffffffffU,0U,CAN1_TIMING_WINDOW);
            CK(!tx_count);
            read_begin(READ_OK,slot,0U,7U);
            can_read_case(system,READ_OK,slot,0U,7U,CAN1_TIMING_WINDOW);
            CK(tx_count==1U && !cancels);
            read_begin(READ_OK,slot,2U,6U);
            tx_done_ms=slot+7U;
            can_read_case(system,READ_OK,slot,2U,6U,CAN1_TIMING_WINDOW);
            CK(tx_count==1U && cancels==1U);
            read_begin(READ_OK,slot,2U,6U);
            force_rx_late=1U;
            can_read_case(system,READ_OK,slot,2U,6U,CAN1_TIMING_WINDOW);
            CK(tx_count==1U);
            read_begin(READ_OK,slot,1U,6U);
            inject=4U;
            can_read_case(system,READ_OK,slot,1U,6U,4U);
            CK(tx_count==1U && !cancels);
            read_begin(READ_OK,slot,1U,6U);
            inject=5U;preload=4U;
            can_read_case(system,READ_OK,slot,1U,6U,CAN1_TIMING_WINDOW);
            CK(!tx_count);
            read_begin(READ_OK,slot,0U,6U);
            inject=6U;
            can_read_case(system,READ_OK,slot,0U,6U,CAN1_TIMING_WINDOW);
            CK(!tx_count);
            read_begin(READ_PARTIAL,slot,2U,6U);
            can_read_case(system,READ_PARTIAL,slot,2U,6U,CAN1_TIMING_WINDOW);
            read_begin(READ_OVERLONG,slot,2U,6U);
            can_read_case(system,READ_OVERLONG,slot,2U,6U,6U);
            read_begin(READ_BAD_TRAILER,slot,2U,6U);
            can_read_case(system,READ_BAD_TRAILER,slot,2U,6U,6U);
            read_begin(READ_BAD_DLC,slot,2U,6U);
            can_read_case(system,READ_BAD_DLC,slot,2U,6U,6U);
            if (system) {
                read_begin(READ_DUPLICATE,slot,2U,6U);
                can_read_case(system,READ_DUPLICATE,slot,2U,6U,6U);
            }
        }
    }
    read_begin(READ_OK,100U,0U,6U);
    s_can1_normal_ready=0U;
    memset(&feedback,0xcc,sizeof(feedback));
    CK(CAN1_ReadEmmSystemFeedbackBounded(2U,100U,&reply,&feedback)==CAN1_TIMING_WINDOW);
    CK(!tx_count && !s_can1_normal_ready && !feedback.valid && !feedback.request_ms);
    CK(CAN1_ReadStatusBounded(2U,100U,&reply,&reply_ms)==CAN1_TIMING_WINDOW);
    CK(!tx_count && !s_can1_normal_ready && !reply_ms);
    CK(CAN1_ReadStatusBounded(5U,100U,&reply,&reply_ms)==7U);
    CK(CAN1_ReadStatusBounded(2U,100U,&reply,0)==7U);
    CK(CAN1_ReadEmmSystemFeedbackBounded(5U,100U,&reply,&feedback)==7U);
    CK(CAN1_ReadEmmSystemFeedbackBounded(2U,100U,&reply,0)==7U);
}
