#include <stdint.h>
#include <stdio.h>
#include <string.h>
uint32_t g_lift_ruler_state;
uint32_t g_lift_compare_operation;
uint32_t g_lift_compare_top_valid;
uint32_t g_lift_ruler_cause;
uint32_t g_lift_ruler_stop_pending;
uint32_t g_lift_ruler_request;
uint32_t g_safety_fault;
uint32_t g_rtos_fault;
uint32_t g_can_ctrl_fault_latched;
uint32_t g_chassis_auto_active;
uint32_t g_startup_axes_state;
uint32_t g_safety_ready;
uint32_t g_can_test_result;
uint32_t g_can_guard_ready;
unsigned begin,service,cancel,checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d\n",__LINE__);}}while(0)
uint32_t StartupAxes_Begin(void){begin++;g_startup_axes_state=1;return 1;}
void StartupAxes_Cancel(void){cancel++;}
void StartupAxes_Service(uint32_t a,uint32_t b){(void)a;(void)b;service++;}
#include "integration10_extension.inc"
int main(void){
 unsigned k;uint32_t *bad[]={&g_lift_ruler_cause,&g_lift_ruler_stop_pending,&g_lift_ruler_request,&g_safety_fault,&g_rtos_fault,&g_can_ctrl_fault_latched,&g_chassis_auto_active,&g_startup_axes_state};
 g_lift_ruler_state=4;g_lift_compare_operation=3;g_lift_compare_top_valid=1;
 Integration10_ExtensionService();CK(begin==0 && service==1);
 for(k=0;k<8;k++){*bad[k]=1;g_integration10_extension_request=1;Integration10_ExtensionService();CK(!begin && g_integration10_extension_rejected==1);*bad[k]=0;}
 g_lift_ruler_state=3;g_integration10_extension_request=1;Integration10_ExtensionService();CK(!begin);g_lift_ruler_state=4;
 g_lift_compare_operation=2;g_integration10_extension_request=1;Integration10_ExtensionService();CK(!begin);g_lift_compare_operation=3;
 g_lift_compare_top_valid=0;g_integration10_extension_request=1;Integration10_ExtensionService();CK(!begin);g_lift_compare_top_valid=1;
 g_integration10_extension_request=1;Integration10_ExtensionService();CK(begin==1);
 Integration10_ExtensionService();CK(begin==1);
 g_integration10_extension_request=1;Integration10_ExtensionService();CK(begin==1);
 g_integration10_extension_request=2;g_safety_fault=1;Integration10_ExtensionService();CK(cancel==1);
 printf("EXTENSION_DISPATCH checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
