#include <math.h>
#include <stdint.h>
#include "chassis_odometry.h"
#include "chassis_kinematics.h"
#include "fusion_motion.h"

const FusionMotionLimits g_fusion_motion_limits = {1700.0f,2500.0f,12000.0f,90.0f,180.0f,1440.0f,640.0f,1000.0f,4500.0f};
volatile uint32_t odom_cases, odom_assertions;
volatile uint32_t distance_selftest_passed;

static uint32_t check_pose(const int8_t wheel_sign[4],float yaw_deg,
    float scale_x,float scale_y,uint32_t case_id)
{
    ChassisOdometry odom;
    const uint32_t total= (uint32_t)(1000.0f*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);
    const float encoder_mm=(float)total*3.14159265358979323846f*77.0f/65536.0f;
    const float yaw=yaw_deg*3.14159265358979323846f/180.0f;
    float dx=0.0f,dy=0.0f;
    uint32_t flags[4],i,k,now=1000U;
    ChassisOdometry_Init(&odom);
    if(!ChassisOdometry_SetPose(&odom,0.0f,0.0f,yaw_deg,yaw_deg)) return case_id;
    for(i=0U;i<4U;i++) {
        flags[i]=(wheel_sign[i]<0) ^ g_chassis_forward_direction[i];
        ChassisOdometry_Wheel(&odom,i,flags[i],0U,now);
    }
    ChassisOdometry_Yaw(&odom,yaw_deg,now);
    ChassisOdometry_Update(&odom,now);
    for(k=1U;k<=100U;k++) {
        now+=50U;
        for(i=0U;i<4U;i++)
            ChassisOdometry_Wheel(&odom,i,flags[i],(total*k)/100U,now);
        ChassisOdometry_Yaw(&odom,yaw_deg,now);
        ChassisOdometry_Update(&odom,now);
    }
    for(i=0U;i<4U;i++) {
        dx+=(float)wheel_sign[i]*encoder_mm*0.25f;
        dy+=(float)(i==0U||i==3U?1:-1)*wheel_sign[i]*encoder_mm*0.25f;
    }
    dx*=scale_x;dy*=scale_y;
    odom_assertions++;
    if(!odom.status.valid || fabsf(odom.status.x_mm-(dx*cosf(yaw)-dy*sinf(yaw)))>0.08f)
        return case_id;
    odom_assertions++;
    if(fabsf(odom.status.y_mm-(dx*sinf(yaw)+dy*cosf(yaw)))>0.08f)
        return case_id;
    odom_cases++;
    return 0U;
}

uint32_t test_entry(void)
{
    const int8_t forward[4]={1,1,1,1}, reverse[4]={-1,-1,-1,-1};
    const int8_t left[4]={1,-1,-1,1}, right[4]={-1,1,1,-1};
    uint32_t failure;
    distance_selftest_passed=Chassis_DistanceSelfTest();
    if(!distance_selftest_passed)return 9U;
    failure=check_pose(forward,0.0f,CHASSIS_ODOM_FORWARD_DISTANCE_SCALE,0.0f,1U);if(failure)return failure;
    failure=check_pose(reverse,0.0f,CHASSIS_ODOM_REVERSE_DISTANCE_SCALE,0.0f,2U);if(failure)return failure;
    failure=check_pose(left,0.0f,0.0f,CHASSIS_ODOM_LEFT_DISTANCE_SCALE,3U);if(failure)return failure;
    failure=check_pose(right,0.0f,0.0f,CHASSIS_ODOM_RIGHT_DISTANCE_SCALE,4U);if(failure)return failure;
    failure=check_pose(forward,90.0f,CHASSIS_ODOM_FORWARD_DISTANCE_SCALE,0.0f,5U);if(failure)return failure;
    failure=check_pose(reverse,90.0f,CHASSIS_ODOM_REVERSE_DISTANCE_SCALE,0.0f,6U);if(failure)return failure;
    failure=check_pose(left,90.0f,0.0f,CHASSIS_ODOM_LEFT_DISTANCE_SCALE,7U);if(failure)return failure;
    failure=check_pose(right,90.0f,0.0f,CHASSIS_ODOM_RIGHT_DISTANCE_SCALE,8U);if(failure)return failure;
    return 0U;
}
