#ifndef SOURCE_STAGE2_RUNTIME_H
#define SOURCE_STAGE2_RUNTIME_H
#include "source_parallel_flow.h"

/* Nonblocking adapter contract. All methods run in the existing sole owner.
 * begin: 0 not ready/no side effects, 1 command accepted, 2 failed/uncertain.
 * poll: 0 pending, 1 new event, 2 failed. Events are not fabricated from ACKs.
 * stop: returns confirmed stopped axis/route bits, retrying until all agree.
 * ROUTE_AND_STOW is ONE begin call: route must not wait for arm completion. */
enum { S2_ALIGNED=1U,S2_SOURCE_CLEAR=2U,S2_HELD=4U,
       S2_RELEASED=8U,S2_PRESENT=16U,S2_STABLE=32U };
typedef struct {
 uint32_t run_id,sequence,completed_mask,observed_ms,evidence_flags;
 uint32_t instance_id,color_id,slot;
} S2_Event;

typedef struct {
 void *context;
 uint32_t (*begin)(void *,const SPF_Command *,uint32_t);
 uint32_t (*poll)(void *,const SPF_Command *,S2_Event *,uint32_t);
 uint32_t (*stop)(void *,uint32_t,uint32_t);
 /* Deliver validated physical evidence at the time it is observed. */
 uint32_t (*observe_grasp)(void *,uint32_t,uint32_t,const S2_Event *,uint32_t);
 /* Called after matching physical evidence, once per successful item. */
 uint32_t (*commit_loaded)(void *,uint32_t,uint32_t,const S2_Event *,uint32_t);
} S2_IO;

typedef struct {
 SPF_Flow flow;
 S2_IO io;
 SPF_Command active;
 uint32_t have_command,started,done_mask,required_stop_mask,stop_confirmed_mask;
 uint32_t command_ms,target_sequence,instance_id,color_id,slot,grasp_confirmed,loaded_mask;
 uint32_t rejected_events,committed,configuration_loaded;
} S2_Runtime;

/* Start only from IDLE/DONE with live, verified profiles. No default success. */
uint32_t S2_Begin(S2_Runtime *,const SPF_Config *,const S2_IO *,uint32_t now);
uint32_t S2_TargetReady(S2_Runtime *,const SPF_Target *,uint32_t slot,uint32_t now);
void S2_Service(S2_Runtime *,uint32_t now);
void S2_Cancel(S2_Runtime *,uint32_t reason);
uint32_t S2_Finished(const S2_Runtime *);
#endif
