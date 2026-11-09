#ifndef APP_ENTRY_H
#define APP_ENTRY_H
#include <stdint.h>
#include "app_entry_config.h"
#include "motion_action_plan.h"
#define DEBUG_PLAN_DEFAULT 0U
#define DEBUG_PLAN_LEFT30 1U
#define DEBUG_PLAN_FORWARD30 2U
#define DEBUG_PLAN_RIGHT30 3U
#define DEBUG_PLAN_ROUTE_A 4U
#define DEBUG_PLAN_FORWARD237 5U /* 保留历史编号，当前禁用，不映射为其他距离。 */
#define DEBUG_PLAN_FORWARD210 6U
#define DEBUG_PLAN_FORWARD210_20 7U /* 仅专用构建允许的20cm/s候选。 */
#define DEBUG_PLAN_MAX DEBUG_PLAN_FORWARD210_20
/* 仅复制参数；许可、空闲和源新鲜度由调用适配层锁内检查。 */
uint32_t AppDebugPlan_Load(uint32_t id, MotionAction *actions, uint32_t *count);
#define COMP_ENTRY_IDLE 0U
#define COMP_ENTRY_BLOCKED 1U
#define COMP_REASON_PLAN_UNIMPLEMENTED 1U
#define COMP_REASON_INVALID_CODE 2U
#define COMP_REASON_PLATFORM 3U
/* 比赛任务调度尚未实现：只冻结按键确认的任务码，明确阻止发车。 */
typedef struct {
    uint32_t state, reason;
    char task_code[16];
} CompetitionEntryStatus;
extern volatile CompetitionEntryStatus g_competition_entry;
uint32_t AppCompetitionEntry_MotionAllowed(void);
void AppCompetitionEntry_Submit(const char *code, uint32_t platform_ready);
#endif
