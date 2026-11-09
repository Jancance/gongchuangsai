#include "demo_select.h"

#if RADAR_DEMO == RADAR_DEMO_PATH

#include "demo_common.h"
#include "demo_console.h"

/*
 * 路径规划使用雷达Flash中最近一次保存的方格扫描结果：
 * 1. 连接和选区流程会自动读取PERSISTED_STATE；
 * 2. grid_result_valid为真时直接发送起点和终点；
 * 3. 没有有效方格扫描记录时明确拒绝，不自动启动新扫描。
 *
 * 注意：grid_result_valid=true且grid_count=0表示“有效的空扫描”，
 * 此时仍允许使用永久障碍等保存设置进行路径规划。
 */

static bool persisted_seen;
static bool saved_grid_valid;
static bool path_submitted;
static bool completed;
static bool aborted;

/* ======================== 业务回调函数 ======================== */

/*
 * 收到雷达Flash保存状态时调用。
 * 本Demo只缓存“是否存在有效方格扫描”；data指针不能带出回调使用。
 * grid_result_valid=true且grid_count=0仍代表一次有效空扫描。
 */
static void on_persisted(const radar_client_persisted_state_t *data)
{
    DemoCommon_PrintPersisted(data);
    persisted_seen = true;
    saved_grid_valid = data->grid_result_valid;
}

/*
 * 收到路径规划最终事件时调用。
 * data->waypoints[0..waypoint_count-1]按行驶顺序保存拐点和终点，
 * 不包含起点；status必须为0才表示规划成功。
 */
static void on_path(const radar_client_path_result_t *data)
{
    DemoCommon_PrintPath(data);
    completed = true;
}

/* ======================== 回调表初始化 ======================== */

/*
 * 先注册公共回调，再覆盖persisted以检查Flash方格有效标志，最后注册
 * 路径结果回调。on_persisted内部仍调用公共打印函数，不会丢失状态输出。
 */
void Demo_InitCallbacks(radar_client_callbacks_t *callbacks)
{
    DemoCommon_InitCallbacks(callbacks);
    callbacks->persisted = on_persisted;
    callbacks->path = on_path;
}

/* ======================== Demo初始化与轮询 ======================== */

void Demo_Init(void)
{
    persisted_seen = false;
    saved_grid_valid = false;
    path_submitted = false;
    completed = false;
    aborted = false;
    DemoConsole_Printf("DEMO path: use saved grid, then plan %u -> %u.\r\n",
                       RADAR_DEMO_PATH_START, RADAR_DEMO_PATH_GOAL);
}

void Demo_Poll(void)
{
    radar_client_state_t link;
    radar_client_request_result_t result;

    (void)RadarClient_GetState(&link);
    if (!link.connected) {
        persisted_seen = false;
        saved_grid_valid = false;
        path_submitted = false;
        completed = false;
        aborted = false;
        return;
    }
    if (DemoConsole_TakeStopRequest()) {
        aborted = true;
        DemoCommon_PrintRequest("stop", RadarClient_RequestStop());
        return;
    }
    if (aborted || completed || !link.ready) return;
    if (!persisted_seen) return;
    if (!saved_grid_valid) {
        DemoConsole_Printf(
            "PATH refused: no valid grid scan is saved in radar Flash.\r\n");
        completed = true;
        return;
    }
    if (!path_submitted && !RadarClient_IsBusy()) {
        /* Flash中已有有效方格扫描，直接提交路径请求，不重新扫描。 */
        result = RadarClient_RequestPlanPath(RADAR_DEMO_PATH_START,
                                             RADAR_DEMO_PATH_GOAL);
        DemoCommon_PrintRequest("plan_path", result);
        if (result == RADAR_CLIENT_REQUEST_ACCEPTED) path_submitted = true;
    }
}

#endif
