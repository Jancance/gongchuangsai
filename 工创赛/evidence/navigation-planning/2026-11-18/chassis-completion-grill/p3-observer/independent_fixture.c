#include "radar_observer.h"
#include "radar_uart_port.h"
#include "radar_lift.h"
#include "link_protocol.h"
#include <string.h>

volatile unsigned audit_checks, audit_failures, audit_failed_lines[48];
volatile uint32_t g_radar_lift_enable, g_radar_lift_interval_us;
static uint8_t *rx;
static uint16_t rx_size, read_pos, write_pos;
static uint32_t tick, lift_claim, lift_releases, servo_angle, tick_jump, hold_tx;
static RadarLiftSnapshot lift;
static uint32_t command_count[256];
static uint16_t command_sequence[256];
static const RadarZone2Config config = {7,0,8,5,5,5,1000,1};
static const uint8_t locked[5] = {4,0,0,14,0};
static const uint8_t unlocked[5] = {4,0,0,14,1};

#define CHECK(v) do { \
    audit_checks++; \
    if (!(v) && audit_failures < 48U) audit_failed_lines[audit_failures++] = __LINE__; \
} while (0)

static void sent_message(const link_message_t *message, void *context)
{
    (void)context;
    command_count[message->type]++;
    command_sequence[message->type] = message->sequence;
}

bool RadarUartPort_StartReceive(uint8_t *buffer, uint16_t length)
{
    rx = buffer;
    rx_size = length;
    read_pos = write_pos = 0;
    return true;
}
bool RadarUartPort_ReceiveIsActive(void) { return rx != 0; }
bool RadarUartPort_ReadByte(uint8_t *byte)
{
    if (read_pos == write_pos) return false;
    *byte = rx[read_pos];
    read_pos = (read_pos + 1U) % rx_size;
    return true;
}
bool RadarUartPort_StartTransmit(uint8_t *data, uint16_t length)
{
    link_decoder_t decoder;
    link_decoder_init(&decoder);
    link_decoder_feed(&decoder, data, length, sent_message, 0);
    if (!hold_tx) RadarClient_OnUartTxCompleteInterrupt();
    return true;
}
void RadarUartPort_AbortTransmit(void) {}
uint32_t RadarUartPort_GetTick(void) { return tick; }
uint32_t RadarUartPort_EnterCritical(void) { return 0; }
void RadarUartPort_ExitCritical(uint32_t key) { (void)key; }
uint32_t xTaskGetTickCount(void) { return tick_jump ? tick++ : tick; }
void vTaskDelay(uint32_t dt) { tick += dt; }
uint32_t RadarServo_SetAngle(uint32_t id, float angle)
{ CHECK(id == 2); servo_angle = (uint32_t)angle; return 1; }
uint32_t RadarLift_SafeIdle(uint32_t now) { return (uint32_t)(tick - now) < 50; }
uint32_t RadarLift_Claim(void) { ++lift_claim; return 1; }
void RadarLift_Unclaim(void) { --lift_claim; }
uint32_t RadarLift_Enable(uint32_t now)
{ CHECK(RadarLift_SafeIdle(now)); g_radar_lift_enable = 1; return 1; }
uint32_t RadarLift_Release(uint32_t sequence)
{
    CHECK(lift.sequence == sequence && lift.state == 2 && lift.result == 1);
    ++lift_releases;
    g_radar_lift_enable = 0;
    return 1;
}
uint32_t RadarLift_BeginBounded(uint32_t count, uint32_t dir, uint32_t budget, uint32_t *sequence)
{
    CHECK(count == 8 && dir <= 1 && budget > 0);
    *sequence = ++lift.sequence;
    lift.state = 1;
    lift.result = 1;
    lift.done = 0;
    return 1;
}
void RadarLift_Cancel(void) { lift.state = 3; g_radar_lift_enable = 0; }
void RadarLift_GetSnapshot(RadarLiftSnapshot *s) { *s = lift; }

static void queue(uint8_t node, uint8_t type, const uint8_t *payload, uint16_t length)
{
    uint8_t bytes[128];
    size_t i, n = link_encode_message(node, type, 123, payload, length, bytes, sizeof(bytes));
    CHECK(n > 0);
    for (i = 0; i < n; ++i) {
        rx[write_pos] = bytes[i];
        write_pos = (write_pos + 1U) % rx_size;
    }
}
static void frame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    queue(7, type, payload, length);
    RadarObserver_Service();
}
static void enqueue_ack(uint8_t command)
{
    uint8_t ack[2] = {command,0};
    queue(7, LINK_EVT_ACK, ack, sizeof(ack));
}
static void ack(uint8_t command)
{
    enqueue_ack(command);
    RadarObserver_Service();
}
static void enqueue_hello(uint8_t zone)
{
    uint8_t hello[13] = {1,36,6,zone};
    queue(7, LINK_EVT_HELLO, hello, sizeof(hello));
}
static void handshake(void)
{
    uint8_t persisted[33] = {11};
    enqueue_hello(0);
    RadarObserver_Service();
    RadarObserver_Service();
    ack(LINK_CMD_SELECT_ZONE);
    enqueue_hello(2);
    RadarObserver_Service();
    RadarObserver_Service();
    frame(LINK_EVT_PERSISTED_STATE, persisted, sizeof(persisted));
    ack(LINK_CMD_GET_PERSISTED_STATE);
    CHECK(RadarObserver_IsScanReady());
}
static uint32_t stage(void)
{
    RadarObserverStatus status;
    RadarObserver_GetStatus(&status);
    return status.stage;
}
static void leg_complete(void)
{
    lift.state = 2;
    lift.done = 8;
    RadarObserver_Service();
}
static void start_until_grid(uint32_t run)
{
    CHECK(RadarObserver_StartScan(run, 2, 9));
    tick_jump = 1;
    RadarObserver_Service();
    tick_jump = 0;
    CHECK(stage() == 1 && servo_angle == 45 && lift_claim == 1);
    tick += 5;
    RadarObserver_Service();
    CHECK(stage() == 2);
    leg_complete();
    CHECK(stage() == 3 && !g_radar_lift_enable);
    tick += 5;
    RadarObserver_Service();
    CHECK(stage() == 4);
}
static void finish_stow(void)
{
    leg_complete();
    CHECK(stage() == 6 && servo_angle == 180 && !g_radar_lift_enable);
    tick += 5;
    RadarObserver_Service();
    CHECK(stage() == 8 && lift_claim == 0);
}
static void drain_stop(void)
{
    RadarObserver_Service();
    ack(LINK_CMD_STOP);
}
static void reliable_scan_path(void)
{
    RadarObserverMap map;
    RadarObserverPathReply reply;
    uint8_t grid[5] = {9,0,0,0,0};
    uint8_t path[7] = {0,0,0,0,24,1,24};
    uint16_t seq;

    start_until_grid(7);
    RadarObserver_Service();
    CHECK(command_count[LINK_CMD_SINGLE_MEASURE] == 1);
    ack(LINK_CMD_SINGLE_MEASURE);
    frame(LINK_EVT_OBSTACLE_GRID, grid, sizeof(grid));
    frame(LINK_EVT_STATUS, unlocked, sizeof(unlocked));
    CHECK(stage() == 4 && !RadarObserver_GetMap(7, &map));
    frame(LINK_EVT_STATUS, locked, sizeof(locked));
    CHECK(stage() == 5 && !RadarObserver_GetMap(7, &map));
    leg_complete();
    CHECK(stage() == 6 && servo_angle == 180 && !g_radar_lift_enable);
    CHECK(!RadarObserver_GetMap(7, &map));
    tick += 4;
    RadarObserver_Service();
    CHECK(!RadarObserver_GetMap(7, &map));
    ++tick;
    RadarObserver_Service();
    CHECK(stage() == 8 && lift_claim == 0 && lift_releases == 2);
    CHECK(RadarObserver_GetMap(7, &map) && map.occupied_mask == 0 && map.identity.scan_sequence == 9);
    CHECK(RadarObserver_RequestPath(&map.identity, 0, 24));
    CHECK(!RadarObserver_GetPathReply(&map.identity, 0, 24, &reply));
    tick_jump = 1;
    RadarObserver_Service();
    tick_jump = 0;
    RadarObserver_Service();
    CHECK(stage() == 7 && !lift_claim && command_count[LINK_CMD_PLAN_PATH] == 1);
    seq = command_sequence[LINK_CMD_PLAN_PATH] - 1U;
    path[0] = (uint8_t)seq;
    path[1] = (uint8_t)(seq >> 8);
    ack(LINK_CMD_PLAN_PATH);
    frame(LINK_EVT_PATH_RESULT, path, sizeof(path));
    CHECK(!RadarObserver_GetPathReply(&map.identity, 0, 24, &reply));
    ++seq;
    path[0] = (uint8_t)seq;
    path[1] = (uint8_t)(seq >> 8);
    queue(6, LINK_EVT_PATH_RESULT, path, sizeof(path));
    RadarObserver_Service();
    CHECK(!RadarObserver_GetPathReply(&map.identity, 0, 24, &reply));
    frame(LINK_EVT_PATH_RESULT, path, sizeof(path));
    CHECK(RadarObserver_GetPathReply(&map.identity, 0, 24, &reply) && reply.path.request_sequence == seq);
    CHECK(!RadarObserver_GetPathReply(&map.identity, 24, 0, &reply));
    CHECK(RadarObserver_RequestPath(&map.identity, 24, 0));
    hold_tx = 1;
    RadarObserver_Service();
    RadarObserver_Service();
    CHECK(command_count[LINK_CMD_PLAN_PATH] == 2 && stage() == 7);
    RadarObserver_Cancel();
    CHECK(!RadarObserver_GetMap(7, &map));
    CHECK(!RadarObserver_GetPathReply(&map.identity, 0, 24, &reply));
    RadarObserver_Service();
    CHECK(stage() == 99 && !RadarObserver_IsScanReady() && !RadarObserver_StartScan(8, 2, 9));
    CHECK(!RadarObserver_Configure(&config)); /* STOP still pending. */
    CHECK(RadarClient_RequestStop() == RADAR_CLIENT_REQUEST_BUSY);
    ack(LINK_CMD_STOP);
    CHECK(command_count[LINK_CMD_STOP] == 0 && !RadarObserver_IsScanReady());
    RadarClient_OnUartTxCompleteInterrupt();
    hold_tx = 0;
    RadarObserver_Service();
    CHECK(command_count[LINK_CMD_STOP] == 1 && !RadarObserver_IsScanReady());
    drain_stop();
    if (!RadarObserver_IsScanReady()) handshake();
    CHECK(RadarObserver_IsScanReady() && !RadarObserver_GetMap(7, &map));
}
static void pretransmit_and_late_grid(void)
{
    RadarObserverMap map;
    uint8_t old_grid[6] = {10,0,0,0,1,18}, fresh_grid[5] = {11,0,0,0,0};
    uint32_t count = command_count[LINK_CMD_SINGLE_MEASURE];

    start_until_grid(8);
    CHECK(command_count[LINK_CMD_SINGLE_MEASURE] == count);
    enqueue_ack(LINK_CMD_SINGLE_MEASURE);
    queue(7, LINK_EVT_OBSTACLE_GRID, old_grid, sizeof(old_grid));
    queue(7, LINK_EVT_STATUS, locked, sizeof(locked));
    RadarObserver_Service();
    CHECK(stage() == 4); /* Pre-send input cannot complete the queued scan. */
    CHECK(command_count[LINK_CMD_SINGLE_MEASURE] == count + 1U);
    if (stage() == 4) {
        ack(LINK_CMD_SINGLE_MEASURE);
        frame(LINK_EVT_OBSTACLE_GRID, fresh_grid, sizeof(fresh_grid));
        frame(LINK_EVT_STATUS, locked, sizeof(locked));
    }
    CHECK(stage() == 5);
    finish_stow();
    CHECK(RadarObserver_GetMap(8, &map) && map.occupied_mask == 0 && map.identity.scan_sequence == 11);
    RadarObserver_Cancel();
    RadarObserver_Service();
    drain_stop();
}
static void lost_zone_same_poll(void)
{
    RadarObserverMap map;
    RadarObserverStatus status;
    uint8_t grid[5] = {12,0,0,0,0};

    start_until_grid(9);
    RadarObserver_Service();
    enqueue_hello(1);
    enqueue_hello(2);
    enqueue_ack(LINK_CMD_SINGLE_MEASURE);
    queue(7, LINK_EVT_OBSTACLE_GRID, grid, sizeof(grid));
    queue(7, LINK_EVT_STATUS, locked, sizeof(locked));
    RadarObserver_Service();
    CHECK(stage() == 99); /* The scan cannot survive an observed zone change. */
    if (stage() == 5) finish_stow();
    CHECK(!RadarObserver_GetMap(9, &map));
    if (stage() != 99) {
        RadarObserver_Cancel();
        RadarObserver_Service();
    }
    RadarObserver_Service();
    tick += 1499U;
    RadarObserver_Service();
    CHECK(!RadarObserver_IsScanReady() && !RadarObserver_GetMap(9, &map));
    ++tick;
    RadarObserver_Service();
    RadarObserver_GetStatus(&status);
    CHECK(status.stage == 99 && status.error != 0 && !status.map_valid && !status.path_valid);
    CHECK(RadarObserver_IsScanReady() && !RadarObserver_GetMap(9, &map));
}

unsigned test_entry(void)
{
    tick = 0xffffffffU;
    CHECK(RadarObserver_Configure(&config));
    RadarObserver_Service();
    CHECK(!RadarObserver_IsScanReady());
    handshake();
    reliable_scan_path();
    pretransmit_and_late_grid();
    lost_zone_same_poll();
    return audit_failures;
}
