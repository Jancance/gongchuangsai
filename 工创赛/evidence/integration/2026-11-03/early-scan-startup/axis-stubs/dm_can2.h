#define DM_CAN2_POSVEL_SPEED_DEG_S 300.0f
extern volatile uint32_t g_dm_startup_go,g_dm_jog_first_fault;
extern uint32_t g_dm_point_enable,g_dm_jog_request,g_dm_jog_state;
extern uint32_t g_dm_can2_motor_state,g_dm_jog_result,g_dm_posvel_holding;
extern float g_dm_posvel_speed_deg_s;
void DM_CAN2_ModuleService(uint32_t,uint32_t);
uint32_t DM_CAN2_JogIsBusy(void);
