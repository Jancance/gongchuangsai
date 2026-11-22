#include <stdio.h>
#include "app_entry.h"
#include "heading_source.h"
#include "icm45686_raw.h"

enum {
    TRIAL_IDLE, TRIAL_PRIMING, TRIAL_SETUP, TRIAL_ACTIVE, TRIAL_STOPPING, TRIAL_FINISHED
};
enum {
    TRIAL_OK, TRIAL_CANCELLED, TRIAL_PREFLIGHT, TRIAL_OPTIONS, TRIAL_HEADING,
    TRIAL_COMMAND, TRIAL_CAN
};
enum { TRIAL_RUN_ID=1U, TRIAL_PREPARE_MS=5000U, TRIAL_HWT_AGE_MS=100U };

#if !VEHICLE_SPEED_TRIAL160_VM_SETUP_ONLY
/* 80 ms average of each wheel's next interval, starting 0/20/40/60 ms into
 * the batch. Quantized by validate_speed_trial160.m with a 77 mm wheel. */
static const uint16_t s_trial_rpm[][4] = {
    {  3,  5,  9, 14}, { 19, 26, 34, 43}, { 52, 63, 75, 87},
    { 99,112,124,136}, {149,161,174,186}, {198,211,223,236},
    {248,260,273,285}, {298,310,322,334}, {344,354,363,371},
    {378,383,388,392}, {394,396,397,397}, {397,397,397,397},
    {397,397,397,396}, {394,392,388,383}, {378,371,363,354},
    {344,334,322,310}, {298,285,273,260}, {248,236,223,211},
    {198,186,174,161}, {149,136,124,112}, { 99, 87, 75, 63},
    { 52, 43, 34, 26}, { 19, 14,  9,  5}, {  3,  1,  0,  0},
    {  0,  0,  0,  0}
};
#endif

typedef struct {
    uint32_t generation, slot_ms, tx_ms, ack_ms;
    uint8_t address, flags;
    int16_t target_rpm;
} SpeedTrialCommand;
typedef struct {
    uint32_t reply_ms, position_counts;
    int32_t speed_decirpm;
    uint8_t address, sign;
} SpeedTrialWheel;
typedef struct {
    uint32_t angle_ms, rate_ms;
    int32_t yaw_cdeg, wz_cdps;
    uint32_t valid_mask;
} SpeedTrialHeading;
typedef struct {
    uint32_t sample_ms, sequence;
    int16_t accel[3], gyro[3], temperature;
} SpeedTrialIcm;

static SpeedTrialCommand s_trial_commands[144];
static SpeedTrialWheel s_trial_wheels[160];
static SpeedTrialHeading s_trial_headings[384];
static SpeedTrialIcm s_trial_icm[384];
static ChassisMotorCommand s_trial_pending_commands[4];
static uint32_t s_trial_command_count, s_trial_wheel_count, s_trial_heading_count, s_trial_icm_count;
static uint32_t s_trial_command_mask, s_trial_command_slot, s_trial_overflow;
static uint32_t s_trial_wheel_ms[4], s_trial_hwt_sequence, s_trial_icm_sequence;
static uint32_t s_trial_phase, s_trial_reason, s_trial_tick, s_trial_record;
#if !VEHICLE_SPEED_TRIAL160_VM_SETUP_ONLY
static uint32_t s_trial_index;
#endif
static volatile uint32_t s_trial_start_request, s_trial_cancel_request;
static volatile uint32_t s_trial_export_ready;
static ChassisZone2CanStatus s_trial_terminal;

void AppChassis_SpeedTrialKey(void)
{
    if (s_trial_start_request) s_trial_cancel_request=1U;
    else s_trial_start_request=1U;
}

#if !VEHICLE_SPEED_TRIAL160_VM_SETUP_ONLY
static uint32_t SpeedTrial_Heading(uint32_t now,AppHeadingSource *heading)
{
    return App_HeadingSourceRead(heading) && heading->healthy &&
        App_HeadingFrameAge(heading,now,1U)<=TRIAL_HWT_AGE_MS &&
        App_HeadingFrameAge(heading,now,2U)<=TRIAL_HWT_AGE_MS;
}
#endif

static void SpeedTrial_Stop(uint32_t reason)
{
    if (s_trial_phase==TRIAL_STOPPING) return;
    s_trial_reason=reason;
    (void)ChassisAuto_Zone2Disarm();
    s_trial_phase=TRIAL_STOPPING;
}

static void SpeedTrial_CheckBeforeCan(void)
{
#if VEHICLE_SPEED_TRIAL160_VM_SETUP_ONLY
    if (s_trial_cancel_request &&
        (s_trial_phase==TRIAL_PRIMING || s_trial_phase==TRIAL_SETUP))
        SpeedTrial_Stop(TRIAL_CANCELLED);
#else
    AppHeadingSource heading;
    uint32_t now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    if (s_trial_phase!=TRIAL_PRIMING && s_trial_phase!=TRIAL_ACTIVE) return;
    if (s_trial_cancel_request) SpeedTrial_Stop(TRIAL_CANCELLED);
    else if (s_trial_record && !SpeedTrial_Heading(now,&heading)) SpeedTrial_Stop(TRIAL_HEADING);
#endif
}

static void SpeedTrial_Trace(const ChassisZone2CanStatus *status)
{
    const uint32_t command_capacity=sizeof(s_trial_commands)/sizeof(s_trial_commands[0]);
    const uint32_t wheel_capacity=sizeof(s_trial_wheels)/sizeof(s_trial_wheels[0]);
    const uint32_t heading_capacity=sizeof(s_trial_headings)/sizeof(s_trial_headings[0]);
#if ICM45686_RAW_ENABLE
    const uint32_t icm_capacity=sizeof(s_trial_icm)/sizeof(s_trial_icm[0]);
#endif
    AppHeadingSource heading;
#if ICM45686_RAW_ENABLE
    IcmRawStatus icm;
#endif
    uint32_t i;
#if ICM45686_RAW_ENABLE
    uint32_t publication;
#endif
    for (i=0U;i<4U;i++) {
        uint32_t bit=1UL<<i;
        if ((status->attempt_mask&bit) && !(s_trial_command_mask&bit)) {
            s_trial_command_mask|=bit;
            if (s_trial_command_count<command_capacity) {
                const ChassisMotorCommand *command=&s_trial_pending_commands[i];
                SpeedTrialCommand *event=&s_trial_commands[s_trial_command_count++];
                event->generation=status->attempt_generation;
                event->slot_ms=s_trial_command_slot+i*20U;
                event->tx_ms=status->tx_ms[i];
                event->ack_ms=status->reply_ms[i];
                event->address=command->address;
                event->target_rpm=command->direction ? -(int16_t)command->speed_raw : (int16_t)command->speed_raw;
                event->flags=((status->possible_mask&bit)?1U:0U) |
                    ((status->tx_complete_mask&bit)?2U:0U) | ((status->ack_mask&bit)?4U:0U);
            } else s_trial_overflow|=1U;
        }
        if ((status->position_mask&bit) && s_trial_wheel_ms[i]!=status->wheel_ms[i]) {
            s_trial_wheel_ms[i]=status->wheel_ms[i];
            if (s_trial_wheel_count<wheel_capacity) {
                SpeedTrialWheel *event=&s_trial_wheels[s_trial_wheel_count++];
                event->reply_ms=status->wheel_ms[i];
                event->position_counts=status->position_counts[i];
                event->speed_decirpm=status->wheel_decirpm[i];
                event->address=(uint8_t)(i+1U);
                event->sign=(uint8_t)status->position_sign[i];
            } else s_trial_overflow|=2U;
        }
    }
    if (App_HeadingSourceRead(&heading) && heading.sequence!=s_trial_hwt_sequence) {
        s_trial_hwt_sequence=heading.sequence;
        if (s_trial_heading_count<heading_capacity) {
            SpeedTrialHeading *event=&s_trial_headings[s_trial_heading_count++];
            event->angle_ms=heading.angle_ms;
            event->rate_ms=heading.rate_ms;
            event->valid_mask=heading.valid_mask;
            event->yaw_cdeg=(int32_t)(heading.yaw_deg*100.0f);
            event->wz_cdps=(int32_t)(heading.rate_dps*100.0f);
        } else s_trial_overflow|=4U;
    }
#if ICM45686_RAW_ENABLE
    if (AppIcm_ReadPublished(&icm,&publication) && icm.state==ICM_RAW_READY &&
        icm.fresh && icm.sample_sequence!=s_trial_icm_sequence) {
        s_trial_icm_sequence=icm.sample_sequence;
        if (s_trial_icm_count<icm_capacity) {
            SpeedTrialIcm *event=&s_trial_icm[s_trial_icm_count++];
            event->sample_ms=icm.sample_tick_ms;
            event->sequence=icm.sample_sequence;
            event->temperature=icm.raw_temperature;
            for (i=0U;i<3U;i++) {
                event->accel[i]=icm.raw_accel[i];
                event->gyro[i]=icm.raw_gyro[i];
            }
        } else s_trial_overflow|=8U;
    }
#endif
}

#if !VEHICLE_SPEED_TRIAL160_VM_SETUP_ONLY
static void SpeedTrial_Drive(const ChassisZone2CanStatus *status,uint32_t now)
{
    /* Submit the existing STOP only after the final zero batch is acknowledged. */
    const uint32_t batch_count=sizeof(s_trial_rpm)/sizeof(s_trial_rpm[0]);
    ChassisMotorCommand commands[4];
    AppHeadingSource heading;
    uint32_t i, profile;
    if (!status->ready) return;
    if (s_trial_index==batch_count && status->zero_confirmed) {
        SpeedTrial_Stop(TRIAL_OK);
        return;
    }
    if (!SpeedTrial_Heading(now,&heading)) {
        SpeedTrial_Stop(TRIAL_HEADING);
        return;
    }
    s_trial_record=1U;
    profile=s_trial_index<batch_count && (s_trial_index || status->zero_confirmed);
    for (i=0U;i<4U;i++) {
        /* LR, RR, LF, RF. Left uses + - - + in logical wheel coordinates. */
        uint32_t reverse=(APP_SPEED_TRIAL160_DIRECTION&1U) ^
            (APP_SPEED_TRIAL160_DIRECTION>=2U && (i==1U || i==2U));
        commands[i].address=(uint8_t)(i+1U);
        commands[i].direction=(uint8_t)(g_chassis_forward_direction[i]^reverse);
        commands[i].speed_raw=profile ? s_trial_rpm[s_trial_index][i] : 0U;
    }
    if (ChassisAuto_Zone2SubmitAt(TRIAL_RUN_ID,status->session,status->ack_generation+1U,
                                status->next_batch_ms,commands)) {
        for (i=0U;i<4U;i++) s_trial_pending_commands[i]=commands[i];
        s_trial_command_slot=status->next_batch_ms;
        s_trial_command_mask=0U;
        if (profile) s_trial_index++;
    } else SpeedTrial_Stop(TRIAL_COMMAND);
}
#endif

static void SpeedTrial_Step(void)
{
    ChassisZone2CanStatus status;
#if !VEHICLE_SPEED_TRIAL160_VM_SETUP_ONLY
    AppHeadingSource heading;
#endif
    uint32_t now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    if (!s_trial_start_request || s_trial_phase==TRIAL_FINISHED) return;
    if (s_trial_phase==TRIAL_IDLE) {
        s_trial_tick=now;
        s_trial_phase=TRIAL_PRIMING;
    }
    ChassisAuto_Zone2Read(&status);
    if (s_trial_record) SpeedTrial_Trace(&status);
#if VEHICLE_SPEED_TRIAL160_VM_SETUP_ONLY
    if (s_trial_phase==TRIAL_PRIMING) {
        if (s_trial_cancel_request) SpeedTrial_Stop(TRIAL_CANCELLED);
        else if (now-s_trial_tick>=TRIAL_PREPARE_MS ||
                 g_can_test_result>1U || g_safety_fault || g_rtos_fault ||
                 g_chassis_auto_cut_vm)
            SpeedTrial_Stop(TRIAL_PREFLIGHT);
        else if (ChassisAuto_Zone2Prime(TRIAL_RUN_ID) &&
            status.feedback_ready && g_can_guard_ready)
            s_trial_phase=TRIAL_SETUP;
    } else if (s_trial_phase==TRIAL_SETUP) {
        if (s_trial_cancel_request) SpeedTrial_Stop(TRIAL_CANCELLED);
        else if (status.cause || !status.feedback_ready || !g_can_guard_ready ||
                 !g_vm_test_request || g_can_test_result>1U || g_safety_fault || g_rtos_fault ||
                 g_chassis_auto_cut_vm)
            SpeedTrial_Stop(status.cause ? TRIAL_CAN : TRIAL_PREFLIGHT);
        else if (!ChassisAuto_Zone2Prime(TRIAL_RUN_ID)) SpeedTrial_Stop(TRIAL_PREFLIGHT);
    }
#else
    if (s_trial_phase==TRIAL_PRIMING) {
        if (s_trial_cancel_request) SpeedTrial_Stop(TRIAL_CANCELLED);
        else if (status.options_result) SpeedTrial_Stop(TRIAL_OPTIONS);
        else if (now-s_trial_tick>=TRIAL_PREPARE_MS || g_can_test_result>1U ||
                 g_safety_fault || g_rtos_fault || g_chassis_auto_cut_vm)
            SpeedTrial_Stop(TRIAL_PREFLIGHT);
        else if (SpeedTrial_Heading(now,&heading) && ChassisAuto_Zone2Prime(TRIAL_RUN_ID) &&
            status.options_mask==0x0FU && status.feedback_ready && g_can_guard_ready &&
            ChassisAuto_Zone2Begin(TRIAL_RUN_ID,g_chassis_auto_session))
            s_trial_phase=TRIAL_ACTIVE;
    } else if (s_trial_phase==TRIAL_ACTIVE) {
        if (status.cause || !g_chassis_auto_active || !g_vm_test_request || g_chassis_auto_cut_vm)
            SpeedTrial_Stop(TRIAL_CAN);
        else SpeedTrial_Drive(&status,now);
    }
#endif
    if (s_trial_phase==TRIAL_STOPPING && !g_chassis_auto_active) {
        (void)ChassisAuto_Zone2Disarm();
        if (s_trial_reason==TRIAL_OK && (!status.wheel_quiet || status.cause)) s_trial_reason=TRIAL_CAN;
        s_trial_terminal=status;
        s_trial_phase=TRIAL_FINISHED;
        __DMB();
        s_trial_export_ready=1U;
    }
}

/* Export from the lower-priority app task after STOP, so formatting and the
 * wireless backpressure cannot consume a CAN slot or block vehicle stopping. */
void AppChassis_SpeedTrialLog(void)
{
    static uint32_t section, cursor;
    char line[128];
    uint32_t count;
    if (!s_trial_export_ready || section>6U) return;
    __DMB();
    if (section==0U) {
        count=1U;
        (void)snprintf(line,sizeof(line),"T160_BEGIN,%u,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
            APP_SPEED_TRIAL160_DIRECTION,(unsigned long)s_trial_reason,
            (unsigned long)s_trial_terminal.state,(unsigned long)s_trial_terminal.cause,
            (unsigned long)s_trial_terminal.wheel_quiet,(unsigned long)s_trial_terminal.stop_ack_ms,
            (unsigned long)s_trial_terminal.options_result);
    } else if (section==1U) {
        count=4U;
#if VEHICLE_SPEED_TRIAL160_VM_SETUP_ONLY
        (void)snprintf(line,sizeof(line),"T160_OPT,%lu,%lu,0x%04X,%u\r\n",
            (unsigned long)(cursor+1U),(unsigned long)s_trial_terminal.options_ms[cursor],
            (unsigned)s_trial_terminal.options_flags[cursor],
            (unsigned)((s_trial_terminal.options_flags[cursor]&0x0086U)==0x0006U));
#else
        (void)snprintf(line,sizeof(line),"T160_OPT,%lu,%lu,%u,%lu\r\n",
            (unsigned long)(cursor+1U),(unsigned long)s_trial_terminal.options_ms[cursor],
            (unsigned)s_trial_terminal.options_flags[cursor],
            (unsigned long)((s_trial_terminal.options_mask>>cursor)&1U));
#endif
    } else if (section==2U && cursor<s_trial_command_count) {
        const SpeedTrialCommand *event=&s_trial_commands[cursor];
        count=s_trial_command_count;
        (void)snprintf(line,sizeof(line),"T160_C,%lu,%u,%d,%lu,%lu,%lu,%u\r\n",
            (unsigned long)event->generation,(unsigned)event->address,(int)event->target_rpm,
            (unsigned long)event->slot_ms,(unsigned long)event->tx_ms,(unsigned long)event->ack_ms,
            (unsigned)event->flags);
    } else if (section==3U && cursor<s_trial_wheel_count) {
        const SpeedTrialWheel *event=&s_trial_wheels[cursor];
        count=s_trial_wheel_count;
        (void)snprintf(line,sizeof(line),"T160_W,%u,%lu,%ld,%u,%lu\r\n",
            (unsigned)event->address,(unsigned long)event->reply_ms,(long)event->speed_decirpm,
            (unsigned)event->sign,(unsigned long)event->position_counts);
    } else if (section==4U && cursor<s_trial_heading_count) {
        const SpeedTrialHeading *event=&s_trial_headings[cursor];
        count=s_trial_heading_count;
        (void)snprintf(line,sizeof(line),"T160_H,%lu,%lu,%ld,%ld,%lu\r\n",
            (unsigned long)event->angle_ms,(unsigned long)event->rate_ms,(long)event->yaw_cdeg,
            (long)event->wz_cdps,(unsigned long)event->valid_mask);
    } else if (section==5U && cursor<s_trial_icm_count) {
        const SpeedTrialIcm *event=&s_trial_icm[cursor];
        count=s_trial_icm_count;
        (void)snprintf(line,sizeof(line),"T160_I,%lu,%lu,%d,%d,%d,%d,%d,%d,%d\r\n",
            (unsigned long)event->sample_ms,(unsigned long)event->sequence,
            (int)event->accel[0],(int)event->accel[1],(int)event->accel[2],
            (int)event->gyro[0],(int)event->gyro[1],(int)event->gyro[2],(int)event->temperature);
    } else if (section==6U) {
        count=1U;
        (void)snprintf(line,sizeof(line),"T160_END,%lu,%lu,%lu,%lu,%lu\r\n",
            (unsigned long)s_trial_command_count,(unsigned long)s_trial_wheel_count,
            (unsigned long)s_trial_heading_count,(unsigned long)s_trial_icm_count,
            (unsigned long)s_trial_overflow);
    } else {
        section++;
        return;
    }
    if (App_DebugSendString(line)==1U && ++cursor==count) {
        section++;
        cursor=0U;
    }
}
