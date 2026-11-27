#ifndef SOURCE_STAGE2_BODY_PORT_H
#define SOURCE_STAGE2_BODY_PORT_H
#include "source_stage2_backend.h"
#include "source_stage2_owner.h"
#include "metric_alignment.h"
typedef struct {
 uint32_t body_frame_verified,route3_profile_verified;
 uint32_t tolerance_milli_mm,yaw_tolerance_mdeg,max_corrections;
 float correction_speed_mm_s;
} S2_BodyLimits;
/* Task A configures measured data. Zero/unconfigured profiles stay disabled. */
uint32_t Stage2Body_Configure(const MetricAlignmentProfile *,const S2_BodyLimits *);
uint32_t Stage2Body_GetHooks(S2_BackendHooks *,void *,uint32_t (*)(void *,const SPF_Command *));
uint32_t Stage2Body_Owns(void);
uint32_t Stage2Body_KeepsPower(void);
uint32_t Stage2Body_PositionEnabled(void);
uint32_t Stage2Body_ObservationWindow(void);
void Stage2Body_Cancel(void);
/* Called by the existing Task A before ordinary mission progression. */
uint32_t Stage2Body_Service(uint32_t now);
extern volatile uint32_t g_stage2_body_state,g_stage2_body_reason,g_stage2_body_session;
#endif
