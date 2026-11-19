#include "source_stage2_vision_bridge.h"
#include "source_vision_runtime.h"

uint32_t S2_VisionGrasp(void *unused,uint32_t run,uint32_t sequence,
 const S2_Event *e,uint32_t now)
{
 (void)unused;
 if(!e || e->run_id!=run)return 0U;
 return Vision_ReportGrasp(run,sequence,(e->evidence_flags&S2_SOURCE_CLEAR)?1U:0U,
  (e->evidence_flags&S2_HELD)?1U:0U,e->observed_ms,now);
}

uint32_t S2_VisionPlaced(void *unused,uint32_t run,uint32_t sequence,
 const S2_Event *e,uint32_t now)
{
 (void)unused;
 if(!e || e->run_id!=run)return 0U;
 return Vision_ReportPlaced(run,sequence,e->slot,
  (e->evidence_flags&S2_RELEASED)?1U:0U,(e->evidence_flags&S2_PRESENT)?1U:0U,
  (e->evidence_flags&S2_STABLE)?1U:0U,e->observed_ms,now);
}
