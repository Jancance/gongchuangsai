#ifndef RADAR_OBSERVER_H
#define RADAR_OBSERVER_H
#include "radar_client.h"
typedef struct {
    uint8_t node_id,down_direction;
    /* 填实测总半步数，1..65535仅软件范围；不代表机械可用行程。 */
    uint16_t steps;
    /* 单程实测耗时加服务余量：stage_timeout须>ceil(steps*733us/1000)+250且<2^31。 */
    uint32_t deploy_ms,settle_ms,stow_ms,stage_timeout_ms;
    /* 节点ID必须实物确认；方向、半步数和等待时间均须实测，不设部署默认值。 */
    uint8_t measured;
} RadarZone2Config;
/* 估算命令数及733us基线名义耗时，未经实测；不能代替实际四圈/高度测量，无默认H。 */
typedef struct {
    uint16_t steps;
    uint32_t nominal_leg_ms;
    uint8_t estimated,measured;
} RadarFourTurnCandidate;
bool RadarObserver_FourTurnCandidate(uint32_t half_steps_per_turn,RadarFourTurnCandidate *candidate);
typedef struct {
    uint32_t scan_sequence;
    uint16_t request_sequence;
    uint8_t start,goal,status,waypoint_count;
    uint8_t waypoints[RADAR_CLIENT_MAX_WAYPOINTS];
} RadarObserverPath;
typedef struct {
    /* stage：0待机/1展开等待/2下降输出/3停稳/4扫描或校准/5回升输出/6收拢等待/7规划/8完成/99失败。
     * error：0无错/1参数或就绪拒绝/2输出拒绝/3雷达事务异常/4取消/5运行条件失效或阶段超时。
     * 阶段完成只表示开环输出与估计时间完成，不是机械高度/到位反馈。 */
    uint32_t sequence,stage,error;
    uint8_t map_valid,path_valid,calibration_complete;
} RadarObserverStatus;
typedef struct {
    uint32_t run_id,map_generation,layout_id,link_generation,scan_sequence;
    uint8_t start_id,node_id,zone;
} RadarObserverMapIdentity;
typedef struct {
    RadarObserverMapIdentity identity;
    uint32_t occupied_mask;
    uint8_t valid;
    RadarObserverStatus status;
} RadarObserverMap;
typedef struct {
    RadarObserverMapIdentity identity;
    radar_client_path_result_t path;
    RadarObserverStatus status;
} RadarObserverPathReply;
/* Watch/SWD独立机构邮箱，无node/measured依赖，不初始化雷达；8559为70mm单样本比例候选。
 * request 0默认关闭，1下(45→等待→dir1→释放)，2上(dir0→180→等待→释放)。
 * 先填正值wait_ms/budget_ms及interval_us=733..4000(默认733)，最后写request；一次消费并锁存。
 * result 1接受/2参数或忙拒绝/3失败保相/4正常完成释放；状态仍读observer快照。 */
extern volatile uint32_t g_radar_mechanism_request,g_radar_mechanism_wait_ms;
extern volatile uint32_t g_radar_mechanism_budget_ms,g_radar_mechanism_interval_us,g_radar_mechanism_result;
void RadarObserver_Service(void);
/* 仅登记配置或单次请求；true只表示已入队，最终结果读取状态快照。
 * Configure/Start可由其他任务调用，Client与Service只能由main的雷达任务调用。
 * manual_stowed_reference=true表示本次已由人工恢复固定收拢起点。 */
bool RadarObserver_Configure(const RadarZone2Config *config);
bool RadarObserver_StartZone2(uint8_t start,uint8_t goal,bool manual_stowed_reference);
/* 区2导航调用方负责锁存PC13、收拢基准及run_id唯一性；握手就绪后仅入队一次。 */
bool RadarObserver_IsScanReady(void);
bool RadarObserver_StartScan(uint32_t run_id,uint8_t start_id,uint32_t layout_id);
/* 两个读接口仅在结果匹配时写入调用方对象，状态与结果同次发布。 */
bool RadarObserver_GetMap(uint32_t run_id,RadarObserverMap *map);
bool RadarObserver_RequestPath(const RadarObserverMapIdentity *expected,uint8_t start,uint8_t goal);
bool RadarObserver_GetPathReply(const RadarObserverMapIdentity *expected,uint8_t start,uint8_t goal,RadarObserverPathReply *reply);
/* 调用本函数也表示人工已恢复固定收拢起点。必须显式配置、握手选区就绪；
 * 同一任务展开下降并停稳后仅请求一次校准，不自动重试写Flash，不是每场门禁。 */
bool RadarCalibration_StartZone2(void);
void RadarObserver_Cancel(void);
bool RadarObserver_GetPath(RadarObserverPath *path);
void RadarObserver_GetStatus(RadarObserverStatus *status);
#endif
