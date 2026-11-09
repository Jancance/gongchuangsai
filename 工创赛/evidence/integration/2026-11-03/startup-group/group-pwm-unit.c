#include <stdint.h>
#include <stdio.h>
#include "startup_pose.h"
static unsigned grip_calls,carrier_calls,last_pulse,fail_grip,fail_carrier;static float last_grip;
void Gripper_Init(void){}
uint8_t jiazhua(float angle){grip_calls++;last_grip=angle;return fail_grip?0:1;}
uint32_t CarrierPwm_Init(void){return 1;}
uint32_t CarrierPwm_Write(uint32_t pulse){carrier_calls++;last_pulse=pulse;return fail_carrier?0:1;}
int main(void){
 StartupPose_Init();
#if APP_MOTOR_OFFLINE_ONLY
 if(grip_calls || carrier_calls || StartupPose_GroupCommand())return 1;
#elif EXPECT_INVALID
 if(!grip_calls || carrier_calls || StartupPose_GroupValid() || StartupPose_GroupCommand() || carrier_calls)return 2;
#else
 if(grip_calls!=1 || last_grip!=145.0f || carrier_calls)return 3;
 if(!StartupPose_GroupValid() || !StartupPose_GroupCommand() || grip_calls!=2 || last_grip!=145.0f || carrier_calls!=1 || last_pulse!=EXPECT_PULSE)return 4;
 fail_grip=1;if(StartupPose_GroupCommand() || carrier_calls!=1)return 5;
 fail_grip=0;fail_carrier=1;if(StartupPose_GroupCommand())return 6;
#endif
 puts("group PWM passed");return 0;
}