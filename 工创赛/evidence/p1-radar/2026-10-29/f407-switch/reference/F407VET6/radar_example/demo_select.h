#ifndef DEMO_SELECT_H
#define DEMO_SELECT_H

#include "radar_client.h"

#define RADAR_DEMO_CONNECT    0 //只连接和读取状态
#define RADAR_DEMO_SCAN_ONCE  1	//14圈扫描，返回障碍坐标
#define RADAR_DEMO_GRID_SCAN  2	//14圈扫描，只返回方格编号
#define RADAR_DEMO_PATH       3 //使用Flash中已有方格结果规划路径
#define RADAR_DEMO_CALIBRATE  4	//校准当前区域
#define RADAR_DEMO_CONTINUOUS 5	//持续扫描，返回障碍坐标，输入stop终止

/*
 * 例程选择：这里只能选择一个Demo。
 * 初次使用保持RADAR_DEMO_CONNECT，确认串口通信正常后再切换其他流程。
 */
#ifndef RADAR_DEMO
#define RADAR_DEMO RADAR_DEMO_PATH //在这里修改demo演示流程
#endif

/* 雷达节点ID，必须与雷达中保存的ID一致，默认是1。 */
#ifndef RADAR_DEMO_NODE_ID
#define RADAR_DEMO_NODE_ID 1U
#endif

/*
 * 工作区域：1=启停区1，2=启停区2。
 * main.c会在RadarClient_Init下一行单独提交选区请求。
 * 0表示只连接、不提交选区请求；除连接Demo外必须明确填写1或2。
 */
#ifndef RADAR_DEMO_ZONE
#define RADAR_DEMO_ZONE 1U
#endif

/* 路径规划的起点、终点方格编号，范围都是0～24。 */
#ifndef RADAR_DEMO_PATH_START
#define RADAR_DEMO_PATH_START 4U
#endif

#ifndef RADAR_DEMO_PATH_GOAL
#define RADAR_DEMO_PATH_GOAL 11U
#endif

/*
 * 显示点云默认关闭以减少串口数据量。
 * 这里只影响显示点云，不影响障碍坐标、方格、校准和路径结果。
 */
#ifndef RADAR_DEMO_POINT_CLOUD_ENABLE
#define RADAR_DEMO_POINT_CLOUD_ENABLE 0U
#endif

#if RADAR_DEMO < RADAR_DEMO_CONNECT || RADAR_DEMO > RADAR_DEMO_CONTINUOUS
#error "RADAR_DEMO is invalid"
#endif

#if RADAR_DEMO_NODE_ID < 1U || RADAR_DEMO_NODE_ID > 254U
#error "RADAR_DEMO_NODE_ID must be 1..254"
#endif

#if RADAR_DEMO != RADAR_DEMO_CONNECT &&     RADAR_DEMO_ZONE != 1U && RADAR_DEMO_ZONE != 2U
#error "Set RADAR_DEMO_ZONE to 1 or 2 before compiling this operational demo"
#endif

#if RADAR_DEMO_PATH_START > 24U || RADAR_DEMO_PATH_GOAL > 24U
#error "Path grid IDs must be 0..24"
#endif

/* ======================== Demo公共入口 ======================== */

/*
 * 初始化当前Demo的回调表。应在RadarClient_Init()之前调用；函数会先
 * 注册公共回调，再注册当前流程专用回调，未使用的回调保持为NULL。
 */
void Demo_InitCallbacks(radar_client_callbacks_t *callbacks);
/* 初始化当前选中的Demo，不发送重复业务请求。 */
void Demo_Init(void);
/* 放在主循环中反复调用；函数内部是非阻塞状态机。 */
void Demo_Poll(void);

#endif
