/* 本次单件接续：在唯一CAN任务，先完成带料升降再显式伸缩回零。 */
volatile uint32_t g_integration10_extension_request,g_integration10_extension_rejected;
static void Integration10_ExtensionService(void)
{
 uint32_t request=g_integration10_extension_request;
 g_integration10_extension_request=0U;
 if(request==2U)StartupAxes_Cancel();
 if(request==1U){
  if(g_lift_ruler_state==4U && g_lift_compare_operation==3U && g_lift_compare_top_valid==1U &&
     !g_lift_ruler_cause && !g_lift_ruler_stop_pending && !g_lift_ruler_request &&
     !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched && !g_chassis_auto_active &&
     g_startup_axes_state==0U){
   if(!StartupAxes_Begin())g_integration10_extension_rejected=2U;
  }else g_integration10_extension_rejected=1U;
 }
 StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);
}
