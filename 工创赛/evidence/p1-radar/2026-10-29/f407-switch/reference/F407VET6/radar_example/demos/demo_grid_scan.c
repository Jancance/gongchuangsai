#include "demo_select.h"

#if RADAR_DEMO == RADAR_DEMO_GRID_SCAN

#include "demo_common.h"
#include "demo_console.h"

/*
 * 方格扫描流程：只提交一次14圈SINGLE_MEASURE。
 * 最终只返回0～24的障碍方格编号，不返回障碍坐标。
 */

static bool submitted;
static bool completed;
static bool aborted;

/* ======================== 业务回调函数 ======================== */

/*
 * 收到最终障碍方格事件时调用。
 * 有效编号位于data->grid_ids[0..count-1]；count=0也是有效空扫描。
 * 雷达在发送该结果前已经尝试把本次方格结果保存到自身Flash。
 */
static void on_grid(const radar_client_grid_result_t *data)
{
    DemoCommon_PrintGrid(data);
    /* count=0也是有效结果，表示本次没有检测到障碍方格。 */
    completed = true;
}

/* ======================== 回调表初始化 ======================== */

/* 先注册公共回调，再注册本Demo使用的最终方格回调。 */
void Demo_InitCallbacks(radar_client_callbacks_t *callbacks)
{
    DemoCommon_InitCallbacks(callbacks);
    callbacks->grid = on_grid;
}

/* ======================== Demo初始化与轮询 ======================== */

void Demo_Init(void)
{
    submitted = false;
    completed = false;
    aborted = false;
    DemoConsole_Printf("DEMO grid_scan: one 14-circle grid scan.\r\n");
}

void Demo_Poll(void)
{
    radar_client_state_t link;
    radar_client_request_result_t result;

    (void)RadarClient_GetState(&link);
    if (!link.connected) {
        submitted = false;
        completed = false;
        aborted = false;
        return;
    }
    if (DemoConsole_TakeStopRequest()) {
        aborted = true;
        DemoCommon_PrintRequest("stop", RadarClient_RequestStop());
        return;
    }
    if (aborted || completed || submitted || !link.ready) return;
    result = RadarClient_RequestGridScan();
    DemoCommon_PrintRequest("grid_scan", result);
    if (result == RADAR_CLIENT_REQUEST_ACCEPTED) submitted = true;
}

#endif
