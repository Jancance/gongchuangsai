
#include <stdio.h>
#include <math.h>
#include "app_config.h"
#include "motion_profile.h"
int main(void){
 float distances[]={150,900,960,1050,1920},speeds[]={200,300,400,450,500,550,600,700,800,1000};
 float f=3.14159265358979323846f*VEHICLE_WHEEL_DIAMETER_MM/60;
 FILE *p=fopen("sweep-profiles.csv","w"),*q=fopen("sweep-plans.csv","w");unsigned checks=0;
 if(!p||!q)return 1;
 fprintf(p,"distance_mm,cap_cm_s,time_s,position_mm,speed_mm_s,acceleration_mm_s2,approach_start_s,duration_s\n");
 fprintf(q,"distance_mm,cap_cm_s,peak_cm_s,duration_s,braking_mm,cruise_s,nominal_peak_rpm,production_profile_accepts\n");
 for(unsigned d=0;d<5;d++)for(unsigned v=0;v<10;v++){
  MotionProfile plan;MotionProfileLimits lim={speeds[v],VEHICLE_ROUTE_ACCEL_MM_S2-f*4,VEHICLE_ROUTE_JERK_MM_S3-f*12};
  if(!MotionProfile_Prepare(&plan,distances[d],&lim))return 2;
  fprintf(q,"%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u\n",distances[d],speeds[v]/10,plan.peak_speed/10,plan.duration_s,distances[d]-plan.position[4],plan.boundary_s[4]-plan.boundary_s[3],plan.peak_speed/f,speeds[v]<=VEHICLE_MAX_TRANSLATION_CM_S*10);
  for(unsigned k=0;k<=(unsigned)ceilf((plan.duration_s+4)*1000);k+=10){
   MotionProfileSample s;if(!MotionProfile_Sample(&plan,k*.001f,&s))return 3;
   if(s.speed_mm_s>speeds[v]+.01f || fabsf(s.acceleration_mm_s2)>VEHICLE_ROUTE_ACCEL_MM_S2+.1f)return 4;
   fprintf(p,"%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",distances[d],speeds[v]/10,k*.001f,s.position_mm,s.speed_mm_s,s.acceleration_mm_s2,plan.boundary_s[4],plan.duration_s);checks++;
  }
 }
 fclose(p);fclose(q);printf("Actual pure-C planner: 50 plans, %u samples. No production limit changed.\n",checks);return 0;
}
