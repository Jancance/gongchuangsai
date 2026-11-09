#ifndef SOURCE_VISION_RUNTIME_H
#define SOURCE_VISION_RUNTIME_H
#include <stdint.h>
/* Exactly one CAN-task owner drains the dedicated vision queue. */
void Vision_Poll(void);
/* Called only by the existing mission/CAN owner, never an ISR.
 * run/sequence/evidence time must come from the actual current transaction.
 * Gripper PWM, driver ACK and a fixed delay are not grasp/place evidence. */
uint32_t Vision_SetTask(const char *code,uint32_t batch,uint32_t run);
uint32_t Vision_ReportGrasp(uint32_t run,uint32_t sequence,uint32_t source_clear,
 uint32_t held,uint32_t evidence_ms,uint32_t now);
uint32_t Vision_ReportPlaced(uint32_t run,uint32_t sequence,uint32_t slot,
 uint32_t released,uint32_t present,uint32_t stable,uint32_t evidence_ms,uint32_t now);
/* Read-only handoff to the route owner. Returns 1 only after all three
 * slots were committed by matching real evidence; this never starts a route. */
uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3]);
/* A planner rejected the selected target BEFORE any command was started.
 * Discards the pending transaction, retaining consumed-frame history. */
uint32_t Vision_RejectUnstarted(uint32_t run,uint32_t sequence);
#endif
