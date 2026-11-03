#include "gripper_task.h"
#include "gripper.h"
#include "gripper_config.h"
#include "task.h"
volatile float g_jiazhua_debug_angle=GRIPPER_DEBUG_ANGLE_DEG;
volatile uint32_t g_jiazhua_debug_request=0U,g_jiazhua_debug_result=0U;
volatile BaseType_t g_gripper_task_create=pdFAIL;
volatile uint32_t g_gripper_signal_stop_count=0U;
static TaskHandle_t s_gripper_task=NULL;
static volatile uint32_t s_ready,s_other_busy,s_sequence_busy,s_pending,s_ticket,s_done,s_result,s_executing;
static volatile float s_angle;
void Gripper_SetAccess(uint32_t ready,uint32_t other_busy,uint32_t sequence_busy)
{taskENTER_CRITICAL();s_ready=ready;s_other_busy=other_busy;s_sequence_busy=sequence_busy;taskEXIT_CRITICAL();}
uint32_t Gripper_IsCommandBusy(void){return (s_pending || s_executing || g_jiazhua_debug_request==1U)?1U:0U;}
uint32_t Gripper_ControlledCommand(float angle)
{
    uint32_t ticket=0U;
    taskENTER_CRITICAL();
    if(s_ready==1U && !s_other_busy && !s_pending && angle>=0.0f && angle<=JIAZHUA_ANGLE_MAX_DEG)
    {s_ticket++;if(!s_ticket)s_ticket=1U;s_angle=angle;s_pending=1U;s_sequence_busy=1U;ticket=s_ticket;}
    taskEXIT_CRITICAL();return ticket;
}
uint32_t Gripper_ControlledResult(uint32_t ticket){return ticket && s_done==ticket?s_result:0U;}
void Gripper_CancelControlled(void)
{taskENTER_CRITICAL();if(s_pending){s_pending=0U;s_done=s_ticket;s_result=2U;}taskEXIT_CRITICAL();}
static void Gripper_Task(void *argument)
{
    uint32_t request,owned,ticket,allowed,debug_allowed;float angle,owned_angle;
    (void)argument;
    /* 沿用用户指定的调试启动180度张开，不由此宣称带料复位安全。 */
    g_jiazhua_debug_result=jiazhua(JIAZHUA_OPEN_ANGLE_DEG);
    for(;;)
    {
        taskENTER_CRITICAL();
        request=g_jiazhua_debug_request;angle=g_jiazhua_debug_angle;g_jiazhua_debug_request=0U;
        owned=s_pending;ticket=s_ticket;owned_angle=s_angle;s_pending=0U;
        s_executing=(owned || request==1U)?1U:0U;
        allowed=(s_ready==1U && !s_other_busy)?1U:0U;
        debug_allowed=(allowed && !s_sequence_busy && !owned)?1U:0U;
        taskEXIT_CRITICAL();
        if(owned){s_result=(allowed && jiazhua(owned_angle))?1U:2U;s_done=ticket;}
        if(request==2U){Gripper_Stop();g_gripper_signal_stop_count++;g_jiazhua_debug_result=2U;}
        else if(request==1U)g_jiazhua_debug_result=debug_allowed?jiazhua(angle):4U;
        else if(request!=0U)g_jiazhua_debug_result=3U;
        s_executing=0U;
        vTaskDelay(pdMS_TO_TICKS(GRIPPER_TASK_PERIOD_MS));
    }
}
void Gripper_TaskInit(void)
{
    if(s_gripper_task==NULL)
    {g_gripper_task_create=xTaskCreate(Gripper_Task,"Gripper",GRIPPER_TASK_STACK_WORDS,NULL,GRIPPER_TASK_PRIORITY,&s_gripper_task);configASSERT(g_gripper_task_create==pdPASS);}
}
