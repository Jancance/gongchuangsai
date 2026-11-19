#ifndef SOURCE_BODY_STEP_H
#define SOURCE_BODY_STEP_H
#include <stdint.h>
/* 本次看护试验固定左移10mm/15mm每秒；不自动开局、不循环、不转角。 */
extern volatile uint32_t g_source_body_step_request,g_source_body_step_state;
extern volatile uint32_t g_source_body_step_reason,g_source_body_step_session;
extern volatile uint32_t g_source_align_rejected;
extern volatile uint32_t g_source_body_step_elapsed_ms,g_source_body_step_submitted;
uint32_t SourceBodyStep_KeepsPower(void);
uint32_t SourceBodyStep_WheelWindow(void);
uint32_t SourceBodyStep_Busy(void);
uint32_t SourceBodyStep_SmallDistanceWindow(void);
uint32_t SourceBodyStep_FastAlignmentWindow(void);
#endif
