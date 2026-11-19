#ifndef SOURCE_STAGE2_BACKEND_H
#define SOURCE_STAGE2_BACKEND_H
#include "source_stage2_runtime.h"
/* Real motor/servo drivers are bound in vehicle_arm.c. The route owner and
 * metric alignment executor stay in their own task; these hooks enqueue and
 * observe their real transactions, never return default success. */
typedef struct {
 void *context;
 uint32_t (*alignment_begin)(void *,const SPF_Command *,uint32_t);
 uint32_t (*alignment_poll)(void *,const SPF_Command *,S2_Event *,uint32_t);
 uint32_t (*route_begin)(void *,const SPF_Command *,uint32_t);
 uint32_t (*route_poll)(void *,const SPF_Command *,uint32_t);
 uint32_t (*route_stop)(void *,uint32_t);
 uint32_t (*path_verified)(void *,const SPF_Command *);
} S2_BackendHooks;

/* Values must be measured for the installed servos, not unverified defaults. */
typedef struct {uint32_t carrier_settle_ms,gripper_settle_ms;} S2_ServoTiming;
/* Metadata must come from actual tracking/time reconciliation, not UART
 * arrival time or a synthetic instance made from a color code. */
typedef struct {
 uint32_t run_id,pose_version,instance_id,captured_ms,usable_until_ms;
 uint32_t capture_verified,stationary,sweep_valid,safe_exit_budget_ms;
} S2_SourceObservation;
uint32_t SourceStage2Backend_Start(const SPF_Config *,const S2_BackendHooks *,
 const S2_ServoTiming *,uint32_t now);
uint32_t SourceStage2Backend_Target(const SPF_Target *,uint32_t slot,uint32_t now);
/* Read current task/region lookup from the real parser and submit one target. */
uint32_t SourceStage2Backend_Select(const S2_SourceObservation *,uint32_t now);
/* Called by the existing owner with physical evidence from the actual provider. */
uint32_t SourceStage2Backend_Observe(const S2_Event *,uint32_t now);
uint32_t SourceStage2Backend_ReadActive(SPF_Command *);
#endif
