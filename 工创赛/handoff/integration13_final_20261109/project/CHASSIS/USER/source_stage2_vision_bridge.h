#ifndef SOURCE_STAGE2_VISION_BRIDGE_H
#define SOURCE_STAGE2_VISION_BRIDGE_H
#include "source_stage2_runtime.h"
/* Use as S2_IO evidence callbacks. These forward observed flags/timestamps
 * to the existing vision ledger, never generate evidence from command ACKs. */
uint32_t S2_VisionGrasp(void *,uint32_t run,uint32_t target_sequence,
 const S2_Event *,uint32_t now);
uint32_t S2_VisionPlaced(void *,uint32_t run,uint32_t target_sequence,
 const S2_Event *,uint32_t now);
#endif
