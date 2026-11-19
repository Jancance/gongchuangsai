#ifndef SOURCE_STAGE2_OWNER_H
#define SOURCE_STAGE2_OWNER_H
#include "source_stage2_runtime.h"
/* Single existing CAN task owns these APIs. No Watch start flag, new thread,
 * implicit restart, invented calibration or default backend is supplied. */
uint32_t SourceStage2_Start(const SPF_Config *,const S2_IO *,uint32_t now);
uint32_t SourceStage2_Target(const SPF_Target *,uint32_t slot,uint32_t now);
uint32_t SourceStage2_IsActive(void);
void SourceStage2_Service(uint32_t now);
void SourceStage2_Cancel(uint32_t reason);
extern volatile uint32_t g_source_stage2_state,g_source_stage2_fault;
extern volatile uint32_t g_source_stage2_loaded_mask,g_source_stage2_completed;
#endif
