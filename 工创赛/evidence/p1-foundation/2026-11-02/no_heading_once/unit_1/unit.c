
#include "no_heading_trial.h"
#include "heading_controller.h"
volatile unsigned checks, failures;
static void ck(unsigned good){checks++;if(!good)failures++;}
void test_entry(void){
 unsigned mode,d,i;float value,returned,error;HeadingHoldController hold={0};
 const float input[5]={-2.0f,-0.1f,0.0f,0.1f,2.0f};
 for(mode=0;mode<4;mode++)for(d=0;d<2;d++)for(i=0;i<5;i++){
  float expected=(APP_NO_HEADING_FORWARD_TRIAL && mode==0 && d==1)?0.0f:input[i];
  value=input[i];returned=NoHeadingTrialCorrection(mode,d?2100.0f:0.0f,&value);
  ck(returned==expected);ck(value==expected);
 }
 ck(HeadingController_HoldStep(&hold,0.0f,3601.0f,0.0f,100,&value,&error)==0);
 ck(HeadingController_HoldStep(&hold,0.0f,0.0f,3601.0f,100,&value,&error)==0);
}
