"""Export actual C profile/controller model results for independent MATLAB checks."""
from pathlib import Path
import subprocess,json
R=Path(__file__).resolve().parents[1];S=R/'tmp/stage2_grid_route_20261103'
E=R/'evidence/integration/2026-11-04/stage2-speed';E.mkdir(parents=True,exist_ok=True)
unit=r'''
#define VEHICLE_PARAMETER_TABLES
#include <stdio.h>
#include <math.h>
#include "app_config.h"
#include "motion_route_trial.h"
#include "turn_brake.h"
#include "grid_route.h"
static unsigned failures,checks;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line %u\n",__LINE__);}}while(0)
int main(void){
 FILE *p=fopen("profiles.csv","w"),*r=fopen("rotation.csv","w"),*summary=fopen("rotation-summary.csv","w");
 CK(p&&r&&summary);if(!p||!r||!summary)return 1;
 fprintf(p,"stage,segment,direction,distance_mm,time_s,position_mm,speed_mm_s,acceleration_mm_s2,duration_s,peak_speed,approach_start_s\n");
 fprintf(r,"case_id,time_s,start_yaw,angle,tau,delay_s,yaw,rate,command_rpm\n");
 fprintf(summary,"case_id,angle,tau,delay_s,result,duration_s,error_deg,peak_dps\n");
 for(unsigned stage=2;stage<=8;stage++){
  RouteRow rows[8];unsigned count=0;FieldPoint end,start={1200,300};
  if(stage==2)CK(GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,vehicle_stage2_waypoints,sizeof(vehicle_stage2_waypoints),VEHICLE_STAGE2_FORWARD_CM_S,VEHICLE_STAGE2_LATERAL_CM_S,rows,8,&count,&end));
  else for(;count<8 && vehicle_drive[stage-1][count].distance_cm>0;count++){
   rows[count]=(RouteRow){MOTION_FORWARD_DISTANCE+vehicle_drive[stage-1][count].direction,vehicle_drive[stage-1][count].distance_cm,vehicle_drive[stage-1][count].speed_cm_s};
  }
  for(unsigned leg=0;leg<count;leg++){
   MotionProfile plan;unsigned rpm,deadline,dir=rows[leg].kind-MOTION_FORWARD_DISTANCE;
   CK(rows[leg].speed==40);CK(MotionRoute_Build(dir,rows[leg].amount*10,rows[leg].speed*10,&plan,&rpm,&deadline));
   for(unsigned k=0;k<=(unsigned)ceilf((plan.duration_s+2.5f)*1000);k+=10){
    MotionProfileSample v;CK(MotionProfile_Sample(&plan,k*.001f,&v));
    CK(v.speed_mm_s<=400.01f && fabsf(v.acceleration_mm_s2)<=1000.1f && rpm<=102);
    fprintf(p,"%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",stage,leg,dir,plan.distance_mm,k*.001f,v.position_mm,v.speed_mm_s,v.acceleration_mm_s2,plan.duration_s,plan.peak_speed,plan.boundary_s[4]);
   }
  }
 }
 unsigned id=0;
 for(unsigned startcase=0;startcase<3;startcase++)for(int sign=-1;sign<=1;sign+=2)
 for(unsigned ti=0;ti<3;ti++)for(unsigned di=0;di<3;di++){
  float yaw=startcase==0?0:(startcase==1?179:-179),start=yaw,rate=0,cmd=0,peak=0;
  float tau=(float[]){.04f,.08f,.15f}[ti],history_y[4000],history_r[4000];
  unsigned result=0,k,delay=di*2;TurnBrake ctrl;
  TurnBrake_Init(&ctrl,0,yaw,sign*90,APP_TURN_BRAKE_MAX_RPM*(6.0f*77/418));
  for(k=0;k<3000;k++){
   rate+=(cmd*(6.0f*77/418)-rate)*.01f/(tau+.01f);yaw=HeadingController_WrapDeg(yaw+rate*.01f);
   history_y[k]=yaw;history_r[k]=rate;unsigned idx=k>delay?k-delay:0;
   result=TurnBrake_Step(&ctrl,(k+1)*10,history_y[idx],history_r[idx],&cmd);
   CK(fabsf(cmd)<=APP_TURN_BRAKE_MAX_RPM+.001f && cmd*sign>=0);
   if(fabsf(rate)>peak)peak=fabsf(rate);
   fprintf(r,"%u,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g\n",id,(k+1)*.01f,start,sign*90,tau,delay*.01f,yaw,rate,cmd);
   if(result)break;
  }
  float error=HeadingController_ErrorDeg(start+sign*90,yaw);
  fprintf(summary,"%u,%d,%.9g,%.9g,%u,%.9g,%.9g,%.9g\n",id,sign*90,tau,delay*.01f,result,(k+1)*.01f,error,peak);
  CK(result==1 && fabsf(error)<=1.0f && peak<=60.01f);id++;
 }
 fclose(p);fclose(r);fclose(summary);printf("SPEED checks=%u failures=%u rotation_cases=%u\n",checks,failures,id);
 return failures?1:0;
}
'''
p=E/'speed-unit.c';p.write_text(unit,encoding='utf-8');exe=E/'speed-unit.exe'
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-DAPP_MOTOR_OFFLINE_ONLY=0','-I'+str(S/'CHASSIS/USER'),str(p)]
cmd += [str(S/'CHASSIS/USER'/n) for n in ['field_map.c','motion_profile.c','chassis_kinematics.c','heading_controller.c']]
cmd+=['-lm','-o',str(exe)]
f=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert f.returncode==0,f.stderr
f=subprocess.run([str(exe)],cwd=E,capture_output=True,text=True);print(f.stdout)
(E/'c-model.log').write_text(f.stdout+f.stderr,encoding='utf-8')
(E/'c-model.json').write_text(json.dumps(dict(passed=f.returncode==0,actual_C_profile_and_rotation_controller=True,plant_is_assumed=True,hardware_accessed=False,output=f.stdout),indent=2),encoding='utf-8')
assert f.returncode==0
