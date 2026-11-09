
#include <stdio.h>
#include "chassis_kinematics.h"
#include "motion_route_trial.h"
int main(void){MotionProfile p;unsigned rpm,deadline;
 printf("{\"APP_DISTANCE_SCURVE_ACTIVE\":%u,\"APP_ROUTE_SPEED_TRIAL_ACTIVE\":%u,\"APP_TEST_MODE\":%u,",(unsigned)APP_DISTANCE_SCURVE_ACTIVE,(unsigned)APP_ROUTE_SPEED_TRIAL_ACTIVE,(unsigned)APP_TEST_MODE);
 if(!MotionRoute_Build(0,2100,400,&p,&rpm,&deadline))return 1;
 printf("\"distance_mm\":%.6f,\"peak_reference_mm_s\":%.6f,\"duration_s\":%.6f,\"accel_limit_mm_s2\":%.6f,\"jerk_limit_mm_s3\":%.6f,\"rpm_cap\":%u,\"boundary_s\":[",p.distance_mm,p.peak_speed,p.duration_s,p.limits.acceleration,p.limits.jerk,rpm);
 for(unsigned i=0;i<8;i++)printf("%s%.6f",i?",":"",p.boundary_s[i]);puts("]}");return 0;}
