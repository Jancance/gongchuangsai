#include "radar_observer.h"
#include "radar_servo.h"
#include "radar_client.h"
#include "link_protocol.h"

volatile unsigned failed, assertions, cases;
static uint32_t now_ms, scan_commands, plan_commands, calibration_commands;
static uint32_t scan_disabled, stow_disabled, stow_phase, pwm_changes;
static uint16_t path_token, pwm_value;
static uint8_t plan_start, plan_goal;
static link_decoder_t tx_decoder;

#define CHECK(value) do { assertions++; if (!(value)) { failed=__LINE__; return; } } while (0)

void StubPort_Reset(void);
void StubPort_Advance(uint32_t milliseconds);
void StubPort_Feed(const uint8_t *data, uint16_t size);

uint32_t xTaskGetTickCount(void) { return now_ms; }

void Fixture_Pwm2(uint16_t pwm)
{
    pwm_value = pwm;
    pwm_changes++;
    if (pwm == 2500U) {
        stow_disabled = !g_radar_lift_enable;
        stow_phase = g_radar_lift_phase_bits;
    }
}

static void on_tx(const link_message_t *message, void *context)
{
    (void)context;
    CHECK(message->node_id == 1U);
    if (message->type == LINK_CMD_SINGLE_MEASURE) {
        scan_commands++;
        scan_disabled = !g_radar_lift_enable && !g_radar_lift_phase_bits;
    }
    if (message->type == LINK_CMD_CALIBRATE) calibration_commands++;
    if (message->type == LINK_CMD_PLAN_PATH) {
        CHECK(message->payload_length == 2U);
        plan_commands++;
        path_token = message->sequence;
        plan_start = message->payload[0];
        plan_goal = message->payload[1];
    }
}

void Fixture_Transmit(const uint8_t *data, uint16_t size)
{
    link_decoder_feed(&tx_decoder, data, size, on_tx, 0);
}

static void feed(uint8_t type,const uint8_t *payload,uint16_t size)
{
    uint8_t encoded[LINK_MAX_ENCODED_FRAME];
    size_t encoded_size = link_encode_message(1U,type,7U,payload,size,encoded,sizeof(encoded));
    CHECK(encoded_size > 0U);
    StubPort_Feed(encoded, (uint16_t)encoded_size);
}

static void clock_at(uint32_t time)
{
    StubPort_Advance(time - now_ms);
    now_ms = time;
}

static void settle_service(uint32_t time)
{
    clock_at(time);
    RadarLift_Service(now_ms, 1U);
    RadarObserver_Service();
    RadarObserver_Service();
}

static uint32_t stage_is(uint32_t expected)
{
    RadarObserverStatus status;
    RadarObserver_GetStatus(&status);
    return status.stage == expected && status.error == 0U;
}

static void leg(uint32_t dir, uint32_t stage_before, uint32_t stage_after)
{
    uint32_t i, began = now_ms;
    RadarLift_Service(now_ms, 1U);
    CHECK(stage_is(stage_before) && direction == dir && total == 4200U && active_interval == 733U);
    CHECK(g_radar_lift_enable && timer_active && tim7.PSC == 83U && tim7.ARR == 732U);
    for (i=1; i<=4200; i++) {
        if (i%16U == 0U) {
            clock_at(began+i*733U/1000U);
            RadarLift_Service(now_ms, 1U);
        }
        dwt.CYCCNT += 168U*733U;
        tim7.SR |= TIM_SR_UIF;
        TIM7_IRQHandler();
        CHECK(g_radar_lift_done == i);
    }
    CHECK(!timer_active && g_radar_lift_enable && g_radar_lift_phase_bits);
    clock_at(began+3079U);
    RadarObserver_Service();
    CHECK(stage_is(stage_before) && g_radar_lift_enable);
    RadarLift_Service(now_ms, 1U);
    RadarObserver_Service();
    CHECK(stage_is(stage_after) && !g_radar_lift_enable && !g_radar_lift_phase_bits && claimed);
    CHECK(bank_d.BSRRL == 0U && bank_d.BSRRH == (GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12));
    CHECK(bank_b.BSRRL == 0U && bank_b.BSRRH == GPIO_Pin_8);
}

static void one_scan(void)
{
    const RadarZone2Config cfg = {1U,1U,4200U,1000U,500U,1000U,60000U,1U};
    uint8_t hello[13] = {1U,35U,6U,2U};
    uint8_t persisted[33] = {11U};
    uint8_t ack[2] = {LINK_CMD_GET_PERSISTED_STATE,0U};
    uint8_t grid[5] = {17U,0U,0U,0U,0U};
    uint8_t locked[5] = {4U,0U,0U,13U,0U};
    uint8_t path[8] = {0U,0U,0U,0U,24U,2U,4U,24U};
    uint32_t down_finished, up_finished;
    radar_client_state_t client;
    RadarObserverPath available;

    StubPort_Reset();
    link_decoder_init(&tx_decoder);
    rcc.CFGR = 0x1000U;
    CHECK(RadarServo_Init() && g_radar_servo_pulse_us[1] == 2500U);
    CHECK(RadarObserver_Configure(&cfg));
    settle_service(0U);
    CHECK(!RadarClient_IsReady() && !scan_commands);
    feed(LINK_EVT_HELLO,hello,sizeof(hello));
    settle_service(0U);
    feed(LINK_EVT_PERSISTED_STATE,persisted,sizeof(persisted));
    feed(LINK_EVT_ACK,ack,sizeof(ack));
    settle_service(0U);
    CHECK(RadarClient_IsReady() && RadarClient_GetState(&client) && client.selected_zone == 2U);
    CHECK(RadarObserver_StartZone2(0U,24U,true));
    CHECK(!RadarObserver_StartZone2(0U,24U,true));
    settle_service(0U);
    CHECK(stage_is(1U) && pwm_changes==1U && pwm_value==1000U && !g_radar_lift_enable);
    settle_service(999U);
    CHECK(stage_is(1U) && !timer_active && !g_radar_lift_enable);
    settle_service(1000U);
    leg(1U,2U,3U);
    if (failed) return;
    down_finished = now_ms;
    CHECK(!scan_commands && !RadarObserver_GetPath(&available));
    settle_service(down_finished+499U);
    CHECK(stage_is(3U) && !scan_commands);
    settle_service(down_finished+500U);
    CHECK(stage_is(4U) && scan_commands==1U && scan_disabled && !calibration_commands);

    ack[0] = LINK_CMD_SINGLE_MEASURE;
    feed(LINK_EVT_ACK,ack,sizeof(ack));
    feed(LINK_EVT_OBSTACLE_GRID,grid,sizeof(grid));
    feed(LINK_EVT_STATUS,locked,sizeof(locked));
    settle_service(now_ms);
    CHECK(stage_is(4U) && !g_radar_lift_enable && !plan_commands);
    locked[3] = 14U;
    feed(LINK_EVT_STATUS,locked,sizeof(locked));
    settle_service(now_ms);
    leg(0U,5U,6U);
    if (failed) return;
    up_finished = now_ms;
    CHECK(pwm_changes==2U && pwm_value==2500U && stow_disabled && stow_phase==0U && !plan_commands);
    CHECK(!RadarObserver_GetPath(&available) && !RadarObserver_StartZone2(0U,24U,true));
    settle_service(up_finished+999U);
    CHECK(stage_is(6U) && !plan_commands && claimed && !g_radar_lift_enable);
    settle_service(up_finished+1000U);
    CHECK(stage_is(7U) && plan_commands==1U && plan_start==0U && plan_goal==24U);
    path[0] = (uint8_t)path_token;
    path[1] = (uint8_t)(path_token >> 8);
    ack[0] = LINK_CMD_PLAN_PATH;
    feed(LINK_EVT_ACK,ack,sizeof(ack));
    settle_service(now_ms);
    CHECK(stage_is(7U) && !RadarObserver_GetPath(&available));
    feed(LINK_EVT_PATH_RESULT,path,sizeof(path));
    settle_service(now_ms);
    CHECK(stage_is(8U) && RadarObserver_GetPath(&available));
    CHECK(available.scan_sequence==17U && available.request_sequence==path_token &&
          available.start==0U && available.goal==24U && available.waypoint_count==2U);
    CHECK(!claimed && !g_radar_lift_enable && !g_radar_lift_phase_bits && !calibration_commands);
    settle_service(now_ms+100U);
    CHECK(scan_commands==1U && plan_commands==1U && pwm_changes==2U);
    cases++;
}

unsigned test_entry(void)
{
    one_scan();
    return failed;
}
