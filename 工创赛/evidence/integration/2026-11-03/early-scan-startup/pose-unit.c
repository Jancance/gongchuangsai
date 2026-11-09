
#include <stdint.h>
#include <stdio.h>
#include "startup_pose.h"
extern volatile uint32_t g_startup_gripper_result,g_startup_carrier_result;
static uint32_t init_gripper,init_carrier,writes,last_pulse;
static float gripper_angle;
void Gripper_Init(void){init_gripper++;}
uint8_t jiazhua(float angle){gripper_angle=angle;return 1;}
uint32_t CarrierPwm_Init(void){init_carrier++;return 1;}
uint32_t CarrierPwm_Write(uint32_t pulse){writes++;last_pulse=pulse;return 1;}
int main(void){StartupPose_Init();
#if APP_MOTOR_OFFLINE_ONLY
 if(init_gripper || init_carrier || writes)return 1;
#else
 if(init_gripper!=1 || init_carrier!=1 || gripper_angle!=145.0f || g_startup_gripper_result!=1)return 2;
#if POSE_EXPECT_INVALID
 if(writes || g_startup_carrier_result!=3)return 3;
#elif POSE_EXPECT_PULSE
 if(writes!=1 || last_pulse!=POSE_EXPECT_PULSE || g_startup_carrier_result!=1)return 4;
#else
 if(writes || g_startup_carrier_result)return 5;
#endif
#endif
 puts("boot pose passed");return 0;}
