#ifndef APP_ENTRY_CONFIG_H
#define APP_ENTRY_CONFIG_H
/* 发布入口只在构建时选择；禁止运行中切换成比赛/调试。 */
#define APP_ENTRY_DEBUG 1U
#define APP_ENTRY_COMPETITION 2U
#ifndef APP_ENTRY
#define APP_ENTRY APP_ENTRY_DEBUG
#endif
/* 旧诊断配置兼容；日常用3，单段/组合由运行时预设选择。 */
#ifndef APP_DEBUG_MODE
#define APP_DEBUG_MODE 3U
#endif
#if APP_ENTRY != APP_ENTRY_DEBUG && APP_ENTRY != APP_ENTRY_COMPETITION
#error Invalid_app_entry
#endif
#if APP_DEBUG_MODE > 3U
#error Invalid_debug_mode
#endif
#define APP_IS_DEBUG (APP_ENTRY == APP_ENTRY_DEBUG)
#define APP_IS_COMPETITION (APP_ENTRY == APP_ENTRY_COMPETITION)
/* APP_TEST_MODE仅为原共享运动引擎的内部兼容配置。 */
#define APP_TEST_MODE (APP_IS_COMPETITION ? 3U : APP_DEBUG_MODE)
#endif
