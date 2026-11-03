/* 只由startup_axes.c包含。先读取并匹配既有回零参数，再单次低速回零。
 * 不写驱动回零参数、不启用上电回零、不用历史闲置标志冒充本次完成。 */
volatile uint32_t g_startup_extension_home_state=0U;
volatile uint32_t g_startup_extension_home_flags=0U;
volatile uint32_t g_startup_extension_home_active=0U;
volatile uint32_t g_startup_extension_home_notice=0U;
volatile uint32_t g_startup_extension_home_abort_result=0U;
#if !APP_MOTOR_OFFLINE_ONLY
static uint8_t s_home_params[17];
static uint32_t home_u16(uint32_t i)
{return ((uint32_t)s_home_params[i]<<8)|s_home_params[i+1U];}
/* 0失败；1已完成；2本轮仍在执行。调用失败由公共启动层撤VM并清参考。 */
static uint32_t StartupExtension_HomeStep(uint32_t now)
{
    uint32_t flags,timeout;
    const uint8_t enable[5]={0xf3,0xab,1,0,0x6b};
    const uint8_t start[4]={0x9a,2,0,0x6b};
    if(s_home_done)return 1U;
    g_startup_extension_home_state=s_home_phase+1U;
    if(s_home_phase>=4U && now-s_home_begin>=STARTUP_EXTENSION_HOME_TIMEOUT_MS)return 0U;
    switch(s_home_phase) {
    case 0U:
        if(StartupAxis_ReadHomeParameters(s_home_params)!=1U)return 0U;
        timeout=((uint32_t)s_home_params[5]<<24)|((uint32_t)s_home_params[6]<<16)|
            ((uint32_t)s_home_params[7]<<8)|s_home_params[8];
        /* 保存的默认模式0..5合法；本次start报文明示模式2，不要求默认也为2。
         * 方向/速度/碰撞阈值仍须与已验证参数一致，不写驱动菜单。 */
        if(s_home_params[0]!=0x22U || s_home_params[1]>5U || s_home_params[2]!=0U ||
           home_u16(3)!=STARTUP_EXTENSION_HOME_RPM || timeout!=10000U ||
           home_u16(9)!=300U || home_u16(11)!=800U || home_u16(13)!=60U ||
           s_home_params[15]!=0U || s_home_params[16]!=0x6bU)return 0U;
        s_home_phase++;return 2U;
    case 1U:
        if(!control(enable,5U))return 0U;
        s_started=1U;s_home_phase++;return 2U;
    case 2U:
        if(query(0x3bU,3U)!=1U)return 0U;
        flags=s_reply.Data[1];g_startup_extension_home_flags=flags;
        if((flags&3U)!=3U || (flags&0x3cU))return 0U;
        s_home_phase++;return 2U;
    case 3U:
        if(query(0x35U,5U)!=1U || s_reply.Data[1]>1U || s_reply.Data[2] || s_reply.Data[3])return 0U;
        /* 丢应答也可能已经启动：发之前就置占用，失败必须退出并停机。 */
        s_home_begin=now;s_home_poll=now;g_startup_extension_home_notice=0U;
        g_startup_extension_home_active=1U;
        if(exchange(start,4U,3U)!=1U || (s_reply.Data[1]!=2U && s_reply.Data[1]!=0x9fU))return 0U;
        if(s_reply.Data[1]==0x9fU)g_startup_extension_home_notice=1U;
        s_home_phase++;return 2U;
    case 4U:
        if(now-s_home_poll<50U)return 2U;
        s_home_poll=now;
        if(query(0x3bU,3U)!=1U)return 0U;
        flags=s_reply.Data[1];g_startup_extension_home_flags=flags;
        if((flags&3U)!=3U || (flags&0x38U))return 0U;
        if(flags&4U)s_home_seen_running=1U;
        if(!(flags&0x0cU) && (s_home_seen_running || g_startup_extension_home_notice))s_home_phase++;
        return 2U;
    case 5U:
        if(query(0x35U,5U)!=1U || s_reply.Data[1]>1U)return 0U;
        if(s_reply.Data[2] || s_reply.Data[3]){s_home_phase=4U;return 2U;}
        if(!read_position() || !(g_startup_extension_current_deg>=-10000.0f &&
            g_startup_extension_current_deg<=10000.0f))return 0U;
        /* 驱动完成+新鲜零速+有效位置后，以当前停点建立工作零位。
         * 不写驱动坐标，不假定回零后数值必须为0。 */
        g_startup_extension_zero_deg=g_startup_extension_current_deg;
        s_start=g_startup_extension_zero_deg;
        s_target=g_startup_extension_zero_deg+STARTUP_EXTENSION_EXTEND_SIGN*STARTUP_EXTENSION_TARGET_MM*
            VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
        g_startup_extension_target_deg=s_target;
        g_startup_extension_reference_valid=1U;s_home_done=1U;
        g_startup_extension_home_active=0U;g_startup_extension_home_state=8U;
        return 1U;
    default:return 0U;
    }
}
#endif
