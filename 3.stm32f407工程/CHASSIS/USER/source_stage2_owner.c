#include "source_stage2_owner.h"
static S2_Runtime source;
volatile uint32_t g_source_stage2_state,g_source_stage2_fault;
volatile uint32_t g_source_stage2_loaded_mask,g_source_stage2_completed;

static void publish(void)
{
 g_source_stage2_state=source.flow.state;g_source_stage2_fault=source.flow.fault;
 g_source_stage2_loaded_mask=source.loaded_mask;g_source_stage2_completed=S2_Finished(&source);
}
uint32_t SourceStage2_Start(const SPF_Config *cfg,const S2_IO *io,uint32_t now)
{
 uint32_t accepted=S2_Begin(&source,cfg,io,now);publish();return accepted;
}
uint32_t SourceStage2_Target(const SPF_Target *target,uint32_t slot,uint32_t now)
{
 uint32_t accepted=S2_TargetReady(&source,target,slot,now);publish();return accepted;
}
uint32_t SourceStage2_IsActive(void)
{
 return source.configuration_loaded && (source.required_stop_mask || source.flow.stop_pending ||
  (source.flow.state!=SPF_IDLE && source.flow.state!=SPF_DONE && source.flow.state!=SPF_FAULT));
}
void SourceStage2_Service(uint32_t now){S2_Service(&source,now);publish();}
void SourceStage2_Cancel(uint32_t reason){S2_Cancel(&source,reason);publish();}
