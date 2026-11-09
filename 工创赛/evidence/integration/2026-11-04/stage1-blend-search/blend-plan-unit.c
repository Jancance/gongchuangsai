
#include <stdio.h>
#include "stage1_blend.h"
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("fail line%u\n",__LINE__);}}while(0)
int main(void){
 FILE *f=fopen("blend-reference.csv","w"),*c=fopen("blend-c-model.csv","w");
 fprintf(f,"time_s,x_mm,y_mm,vx_mm_s,vy_mm_s,ax_mm_s2,ay_mm_s2,jx_mm_s3,jy_mm_s3,duration_s,y_duration_s,x_factor,y_factor\n");
 fprintf(c,"time_s,x_mm,y_mm,x_rpm,y_rpm,status\n");
 Stage1Blend b;CK(Stage1Blend_Build(1,1050,450,&b));
 for(unsigned i=0;i<=(unsigned)ceilf((b.duration+4)*100);i++){
  float t=i*.01f;MotionProfileSample x,y;CK(MotionProfile_Sample(&b.x,t,&x));CK(MotionProfile_Sample(&b.y,t,&y));
  CK(x.speed_mm_s<=450.01f && y.speed_mm_s<=200.01f);
  CK(fabsf(x.acceleration_mm_s2)+fabsf(y.acceleration_mm_s2)<=1000.1f);
  CK(fabsf(x.jerk_mm_s3)+fabsf(y.jerk_mm_s3)<=2000.1f);
  ChassisMotorCommand commands[4];CK(Stage1Blend_Commands(1,x.speed_mm_s/b.x_factor,y.speed_mm_s/b.y_factor,2,commands));
  fprintf(f,"%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",t,x.position_mm,y.position_mm,x.speed_mm_s,y.speed_mm_s,x.acceleration_mm_s2,y.acceleration_mm_s2,x.jerk_mm_s3,y.jerk_mm_s3,b.duration,b.y.duration_s,b.x_factor,b.y_factor);
 }
 float x=0,y=0,vx=0,vy=0,cx=0,cy=0;unsigned result=0;
 for(unsigned i=0;i<1500;i++){
  float t=(i+1)*.01f;vx+=(cx*b.x_factor-vx)*.01f/.06f;vy+=(cy*b.y_factor-vy)*.01f/.06f;x+=vx*.01f;y+=vy*.01f;
  result=Stage1Blend_Step(&b,t,t,x,y,(i+1)*10,&cx,&cy);
  fprintf(c,"%.9g,%.9g,%.9g,%.9g,%.9g,%u\n",t,x,y,cx,cy,result);
  if(result)break;
 }
 CK(result==1 && fabsf(x-1050)<=2 && fabsf(y-150)<=2);
 CK(!Stage1Blend_Build(0,1050,450,&b));CK(!Stage1Blend_Build(1,1050,451,&b));
 CK(Stage1Blend_Build(1,1050,450,&b));CK(Stage1Blend_Step(&b,1,1,1000,0,1000,&cx,&cy)==2);
 CK(Stage1Blend_Build(1,1050,450,&b));CK(Stage1Blend_Step(&b,1,1,0,200,1000,&cx,&cy)==2);
 CK(Stage1Blend_TrackLimit(0)==50 && Stage1Blend_TrackLimit(450)==65);
 for(unsigned mode=1;mode<=2;mode++){
  float side=mode==1?1:-1,aligned[4]={900+side*150,900-side*150,900-side*150,900+side*150},mx,my,spread;
  Stage1Blend_Project(mode,aligned,&mx,&my,&spread);CK(mx==900 && my==150 && spread==0);
  aligned[0]+=100;Stage1Blend_Project(mode,aligned,&mx,&my,&spread);CK(spread>50);
 }
 fclose(f);fclose(c);printf("PURE checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
