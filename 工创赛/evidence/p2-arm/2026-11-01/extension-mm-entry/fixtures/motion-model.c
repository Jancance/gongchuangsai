
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "extension.h"
static const char *scenario;
static TickType_t now;
static unsigned reads,moves,stops,enables,moved;
static int is(const char*s){return strcmp(scenario,s)==0;}
volatile uint32_t g_extension_home_request=0;
uint32_t ExtensionHome_IsBusy(void){return 0;}
void ExtensionHome_Service(uint32_t ready,uint32_t busy){(void)ready;(void)busy;assert(g_extension_home_request==0);}
TickType_t xTaskGetTickCount(void){return now;}
void vTaskDelay(TickType_t ticks){now+=ticks;}
uint32_t CAN1_ReadExtensionHomeParameters(uint8_t*p,CanRxMsg*r){(void)p;(void)r;assert(!"unexpected home read in motion regression");return 5;}
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg*r){assert(a==6);reads++;r->Data[0]=0x1F;return 1;}
uint32_t CAN1_ReadMotorOptions(uint8_t a,CanRxMsg*r){
 assert(a==6);reads++;r->Data[0]=0x1A;r->Data[2]=is("bad_options")?0:6;return 1;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){
 assert(a==6);reads++;
 if(is("read_fail")&&moved)return 5;
 r->Data[0]=0x3A;r->Data[1]=3;
 if(is("enable")&&!enables)r->Data[1]=2;
 if(is("fault")&&moved)r->Data[1]=5;
 if(is("transient_stall")&&moved&&now<700)r->Data[1]=5;
 if(is("persistent_stall")&&moved)r->Data[1]=5;
 if(is("wrap_stall")&&moved)r->Data[1]=5;
 if(is("preexisting_stall"))r->Data[1]=5;
 if(is("stall_then_protect")&&moved)r->Data[1]=now<700?5:9;
 if(is("stall_then_disable")&&moved)r->Data[1]=now<700?5:0;
 if(is("stall_then_read_fail")&&moved){if(now>700)return 5;r->Data[1]=5;}
 if((is("stall_then_abort")||is("stall_then_power_loss")||is("stall_then_gap"))&&moved)r->Data[1]=5;
 if(is("transient_no_arrival")&&moved)r->Data[1]=now<700?5:1;
 if(is("intermittent_stall")&&moved)r->Data[1]=now<4000?(((now/600)%2)?5:1):3;
 if(is("arrival_with_stall")&&moved)r->Data[1]=7;
 if(is("protect")&&moved)r->Data[1]=9;
 if(is("disable")&&moved)r->Data[1]=2;
 if((is("no_arrival")||(is("late_success")&&now<9500))&&moved)r->Data[1]=1;
 return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v){
 assert(a==6);reads++;r->Data[0]=0x35;
 *v=(is("pre_moving")|| (is("no_stop")&&moved))?1:0;return 1;}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*s,uint32_t*v){
 assert(a==6);reads++;r->Data[0]=0x36;
 *s=0;*v=1000;
 if(is("signed")){*s=moved?0:1;*v=20;return 1;}
 if(moved&&!is("ack_only"))*v=(is("negative")?980:1020);
 return 1;}
uint32_t CAN1_ExtensionExchange(const uint8_t*b,uint8_t len,CanRxMsg*r){
 r->Data[0]=b[0];r->Data[1]=2;r->Data[2]=0x6B;
 if(b[0]==0xF3){assert(len==5&&b[2]==1&&b[3]==0&&b[4]==0x6B);enables++;return 1;}
 if(b[0]==0xFE){assert(len==4&&b[1]==0x98&&b[2]==0&&b[3]==0x6B);stops++;return is("stop_fail")?5:1;}
 assert(len==12&&b[0]==0xFD&&b[1]==(is("negative")?1:0));
 assert(b[2]==0&&b[3]==5&&b[4]==20&&b[5]==0&&b[6]==0&&b[7]==6&&b[8]==64);
 assert(b[9]==2&&b[10]==0&&b[11]==0x6B);moves++;
 if(is("move_fail"))return 5;
 moved=1;return 1;}

#ifndef INTEGRATION
int main(int argc,char**argv){
 unsigned i,expected=1,before;uint32_t ready=1,busy=0;
 assert(argc==2);scenario=argv[1];
 if(is("wrap")||is("wrap_stall"))now=0xFFFFFFE0U;
 Extension_Service(1,0);Extension_MotionService(1,0);
 assert(reads==0&&moves==0&&stops==0&&enables==0);
 if(is("readonly")){
  g_extension_read_request=1;Extension_Service(1,0);Extension_Service(1,0);Extension_Service(1,0);
  assert(g_extension_state==4&&reads==3&&moves==0);puts("PASS readonly preserved");return 0;}
 g_extension_motion_request=is("negative")?2:1;
 for(i=0;i<1250;i++){
  now+=10;ready=!(is("not_ready")||(is("power_loss")&&moved)||(is("stall_then_power_loss")&&g_extension_stall_pending));
  busy=is("busy")||(is("busy_during")&&moved);
  if(is("repeat")&&Extension_MotionIsBusy())g_extension_motion_request=1;
  if(is("edit_direction")&&Extension_MotionIsBusy())g_extension_motion_direction=1;
  if((is("abort")||is("stop_fail"))&&moved)g_extension_motion_request=3;
  if(is("gap")&&moved)now+=600;
  if(is("stall_then_gap")&&g_extension_stall_pending)now+=600;
  if(is("stall_then_abort")&&g_extension_stall_pending)g_extension_motion_request=3;
  if(is("readonly_during")&&moved)g_extension_read_request=1;
  Extension_Service(ready,busy);Extension_MotionService(ready,busy);
  if(g_extension_motion_state>=7)break;
 }
 if(is("ack_only")||is("no_stop")||is("no_arrival"))expected=3;
 if(is("bad_options")||is("move_fail")||is("read_fail")||is("stop_fail"))expected=4;
 if(is("fault")||is("protect")||is("disable")||is("pre_moving"))expected=5;
 if(is("not_ready")||is("busy")||is("power_loss")||is("busy_during"))expected=6;
 if(is("gap"))expected=7;
 if(is("abort"))expected=2;
 if(is("persistent_stall")||is("wrap_stall")||is("preexisting_stall")||is("stall_then_protect")||is("stall_then_disable")||is("arrival_with_stall"))expected=5;
 if(is("stall_then_read_fail"))expected=4;
 if(is("stall_then_power_loss"))expected=6;
 if(is("stall_then_gap"))expected=7;
 if(is("stall_then_abort"))expected=2;
 if(is("transient_no_arrival"))expected=3;
 assert(g_extension_motion_result==expected);
 if(is("transient_stall")){assert(g_extension_stall_events==1&&g_extension_stall_max_ms>0&&g_extension_stall_max_ms<2000&&g_extension_stall_pending==0);}
 if(is("intermittent_stall")){assert(g_extension_stall_events>=2&&g_extension_stall_max_ms<2000);}
 if(is("fault")||is("persistent_stall")||is("wrap_stall")||is("arrival_with_stall")){assert(g_extension_stall_max_ms>=2000&&g_extension_stall_max_ms<=2400&&moves==1&&stops==1);}
 if(is("protect")){assert(g_extension_stall_events==0&&g_extension_motion_elapsed_ms<700&&stops==1);}
 if(is("stall_then_protect")||is("stall_then_disable")){assert(g_extension_stall_max_ms<2000&&g_extension_motion_elapsed_ms<1500&&stops==1);}
 if(is("preexisting_stall")){assert(moves==0&&stops==1);}
 if(expected!=1)assert(g_extension_stall_pending==0);
 if(expected==3)assert(g_extension_motion_elapsed_ms>=10000&&g_extension_motion_elapsed_ms<=10010);
 if(is("late_success"))assert(g_extension_motion_elapsed_ms>=9500&&g_extension_motion_elapsed_ms<10000);
 if(is("fault")||is("protect"))assert(moves==1&&stops==1);
 if(expected==1){assert(g_extension_motion_state==7&&moves==1&&stops==0);
  assert(g_extension_delta_raw==(is("negative")?-20:(is("signed")?40:20)));
  before=moves;for(i=0;i<8;i++){now+=10;Extension_Service(1,0);Extension_MotionService(1,0);}assert(moves==before);}
 else if(expected==2){assert(g_extension_motion_state==8&&stops==1);}
 else{assert(g_extension_motion_state==9);before=moves;
  g_extension_motion_request=1;Extension_MotionService(1,0);assert(moves==before);}
 if(is("not_ready")||is("busy"))assert(reads==0&&moves==0&&stops==0);
 if(is("bad_options"))assert(moves==0&&stops==0);
 if(is("pre_moving"))assert(moves==0&&stops==1);
 if(is("enable"))assert(enables==1);
 if(is("readonly_during"))assert(g_extension_read_request==0&&reads<20);
 if(is("stop_fail"))assert(g_extension_stop_result==5);
 printf("PASS motion %s result=%u moves=%u stops=%u\n",scenario,expected,moves,stops);return 0;
}
#else
unsigned g_can_test_result=1,g_safety_ready=1,g_vm_test_request=1,g_arm_menu_power_state=1;
unsigned g_lift_state=4,g_lift_motion_request,g_lift_home_request,g_lift_read_request,g_lift_home_read_request;
unsigned g_lift_motion_state,dm_busy,home_busy,dm_request,dm_started,lift_started;
uint32_t DM_CAN2_JogIsBusy(void){return dm_busy;}
uint32_t LiftHome_IsBusy(void){return home_busy;}
void Lift_Service(uint32_t ready){assert(ready);if(g_lift_motion_request==1){lift_started++;g_lift_motion_state=1;g_lift_motion_request=0;}}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){assert(ready);if(dm_request&&!busy)dm_started++;dm_request=0;}
void step(void){

}
int main(int argc,char**argv){
 assert(argc==2);scenario=argv[1];
 if(is("dm_busy")){dm_busy=1;g_extension_motion_request=1;step();assert(moves==0&&reads==0&&g_extension_motion_result==6);}
 else if(is("lift_busy")){g_lift_motion_state=6;g_extension_motion_request=1;step();assert(moves==0&&reads==0&&g_extension_motion_result==6);}
 else if(is("home_busy")){home_busy=1;g_extension_motion_request=1;step();assert(moves==0&&reads==0&&g_extension_motion_result==6);}
 else if(is("simultaneous_lift")){g_lift_motion_request=1;g_extension_motion_request=1;step();assert(lift_started==1&&moves==0&&g_extension_motion_result==6);}
 else if(is("simultaneous_dm")){dm_request=4;g_extension_motion_request=1;step();assert(Extension_MotionIsBusy()&&dm_started==0);}
 else if(is("readonly_busy")){g_extension_read_request=1;g_extension_motion_request=0;step();assert(g_extension_state==2&&moves==0);}
 else{g_extension_motion_request=1;step();assert(Extension_MotionIsBusy());
  g_lift_motion_request=1;g_lift_home_request=1;g_lift_read_request=1;g_lift_home_read_request=1;dm_request=4;
  step();assert(lift_started==0&&dm_started==0&&g_lift_home_request==0&&g_lift_read_request==0&&g_lift_home_read_request==0);}
 printf("PASS integration %s\n",scenario);return 0;
}
#endif
