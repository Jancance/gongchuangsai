#include "demo_select.h"

#if RADAR_DEMO == RADAR_DEMO_CONNECT

#include "demo_common.h"
#include "demo_console.h"

/*
 * 连接示例：只完成HELLO握手和保存状态读取，不启动扫描。
 * 建议第一次接线时先运行本Demo，确认能看到HELLO和PERSISTED输出；
 * 如果雷达本次上电已经选过区域，还会进入ready=1。
 */

/* ======================== 回调表初始化 ======================== */

/* 连接Demo没有专用业务回调，只注册公共的握手、状态和错误回调。 */
void Demo_InitCallbacks(radar_client_callbacks_t *callbacks)
{
    DemoCommon_InitCallbacks(callbacks);
}

/* ======================== Demo初始化与轮询 ======================== */

void Demo_Init(void)
{
    DemoConsole_Printf("DEMO connect: handshake and read persisted state only.\r\n");
    DemoConsole_Printf("If HELLO zone is 0, select zone 1 or 2 in your application.\r\n");
}

void Demo_Poll(void)
{
    /* 即使没有扫描，也保留STOP输入作为通信和高优先级命令示例。 */
    if (DemoConsole_TakeStopRequest())
        DemoCommon_PrintRequest("stop", RadarClient_RequestStop());
}

#endif
