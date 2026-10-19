#include <stdio.h>
#include <float.h>
#include "chassis_kinematics.h"
static unsigned int checks,seed=1234567U;
static float ab(float x){return x<0?-x:x;}
static int near(float a,float b,float e){return ab(a-b)<=e;}
static float random_value(float r){seed=1664525U*seed+1013904223U;return (((float)(seed&65535U)/32767.5f)-1.0f)*r;}
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL line %d\n",__LINE__);return 1;}}while(0)
static int zero_wheels(const ChassisWheelSpeeds *w){return w->rpm[0]==0&&w->rpm[1]==0&&w->rpm[2]==0&&w->rpm[3]==0;}
int main(void){ChassisGeometry g=g_chassis_geometry;ChassisWheelSpeeds w;ChassisBodyVelocity b,r;float scale;unsigned int i,j;
const float one_rpm_mm_s=4.03171057f;union {unsigned int bits;float f;} special;
CHECK(g.wheel_diameter_mm==77&&g.track_width_mm==210&&g.wheelbase_mm==210);
/* Independent known vectors in CAN order LR,RR,LF,RF. */
b.vx_mm_s=one_rpm_mm_s;b.vy_mm_s=b.wz_rad_s=0;CHECK(Chassis_Inverse(&g,&b,100,&w,&scale)==1);for(i=0;i<4;i++)CHECK(near(w.rpm[i],1,0.00001f));CHECK(scale==1);
b.vx_mm_s=-one_rpm_mm_s;CHECK(Chassis_Inverse(&g,&b,100,&w,0)==1);for(i=0;i<4;i++)CHECK(near(w.rpm[i],-1,0.00001f));
b.vx_mm_s=0;b.vy_mm_s=one_rpm_mm_s;CHECK(Chassis_Inverse(&g,&b,100,&w,&scale)==1);CHECK(near(w.rpm[0],1,0.00001f)&&near(w.rpm[1],-1,0.00001f)&&near(w.rpm[2],-1,0.00001f)&&near(w.rpm[3],1,0.00001f));
b.vy_mm_s=-one_rpm_mm_s;CHECK(Chassis_Inverse(&g,&b,100,&w,&scale)==1);CHECK(near(w.rpm[0],-1,0.00001f)&&near(w.rpm[1],1,0.00001f)&&near(w.rpm[2],1,0.00001f)&&near(w.rpm[3],-1,0.00001f));
b.vy_mm_s=0;b.wz_rad_s=one_rpm_mm_s/210.0f;CHECK(Chassis_Inverse(&g,&b,100,&w,&scale)==1);CHECK(near(w.rpm[0],-1,0.00001f)&&near(w.rpm[1],1,0.00001f)&&near(w.rpm[2],-1,0.00001f)&&near(w.rpm[3],1,0.00001f));
b.wz_rad_s=-one_rpm_mm_s/210.0f;CHECK(Chassis_Inverse(&g,&b,100,&w,&scale)==1);CHECK(near(w.rpm[0],1,0.00001f)&&near(w.rpm[1],-1,0.00001f)&&near(w.rpm[2],1,0.00001f)&&near(w.rpm[3],-1,0.00001f));
b.vx_mm_s=b.vy_mm_s=b.wz_rad_s=0;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==1&&zero_wheels(&w)&&scale==1);
/* Mixed input has wheel peripheral speeds9,31,-11,51mm/s; preserve ratios. */
b.vx_mm_s=20;b.vy_mm_s=10;b.wz_rad_s=0.1f;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==1);CHECK(near(w.rpm[0],9.0f/51.0f,0.00001f));CHECK(near(w.rpm[1],31.0f/51.0f,0.00001f));CHECK(near(w.rpm[2],-11.0f/51.0f,0.00001f));CHECK(near(w.rpm[3],1,0.00001f));CHECK(scale>0&&scale<1);CHECK(Chassis_Forward(&g,&w,&r)==1);CHECK(near(r.vx_mm_s,b.vx_mm_s*scale,0.0001f)&&near(r.vy_mm_s,b.vy_mm_s*scale,0.0001f)&&near(r.wz_rad_s,b.wz_rad_s*scale,0.00001f));
/* Explicit sign-changing forward estimate and non-square geometry. */
for(i=0;i<4;i++)w.rpm[i]=1;CHECK(Chassis_Forward(&g,&w,&r)==1);CHECK(near(r.vx_mm_s,one_rpm_mm_s,0.00001f)&&r.vy_mm_s==0&&r.wz_rad_s==0);
g.wheelbase_mm=310;b.vx_mm_s=b.vy_mm_s=0;b.wz_rad_s=one_rpm_mm_s/260.0f;CHECK(Chassis_Inverse(&g,&b,100,&w,&scale)==1);CHECK(near(w.rpm[0],-1,0.00001f)&&near(w.rpm[1],1,0.00001f));g=g_chassis_geometry;
/* Deterministic round trips cover 3 independent dimensions and signed combinations. */
for(i=0;i<1000;i++){b.vx_mm_s=random_value(200);b.vy_mm_s=random_value(200);b.wz_rad_s=random_value(1);CHECK(Chassis_Inverse(&g,&b,1000,&w,&scale)==1&&scale==1);CHECK(Chassis_Forward(&g,&w,&r)==1);CHECK(near(r.vx_mm_s,b.vx_mm_s,0.001f)&&near(r.vy_mm_s,b.vy_mm_s,0.001f)&&near(r.wz_rad_s,b.wz_rad_s,0.00001f));}
for(i=0;i<200;i++){b.vx_mm_s=random_value(200);b.vy_mm_s=random_value(200);b.wz_rad_s=random_value(1);CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==1&&scale>0&&scale<=1);for(j=0;j<4;j++)CHECK(ab(w.rpm[j])<=1.000001f);CHECK(Chassis_Forward(&g,&w,&r)==1);CHECK(near(r.vx_mm_s,b.vx_mm_s*scale,0.001f)&&near(r.vy_mm_s,b.vy_mm_s*scale,0.001f)&&near(r.wz_rad_s,b.wz_rad_s*scale,0.00001f));}
/* Invalid values must not leak partial commands or stale outputs. */
b.vx_mm_s=1;b.vy_mm_s=b.wz_rad_s=0;
CHECK(Chassis_Inverse(0,&b,1,&w,&scale)==0&&zero_wheels(&w)&&scale==0);CHECK(Chassis_Inverse(&g,0,1,&w,&scale)==0&&zero_wheels(&w));CHECK(Chassis_Inverse(&g,&b,1,0,&scale)==0&&scale==0);
CHECK(Chassis_Inverse(&g,&b,0,&w,&scale)==0&&zero_wheels(&w));CHECK(Chassis_Inverse(&g,&b,-1,&w,&scale)==0&&zero_wheels(&w));
g.wheel_diameter_mm=0;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==0&&zero_wheels(&w));g=g_chassis_geometry;g.track_width_mm=-1;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==0);g=g_chassis_geometry;g.wheelbase_mm=0;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==0);g=g_chassis_geometry;
special.bits=0x7FC00000U;b.vx_mm_s=special.f;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==0&&zero_wheels(&w));b.vx_mm_s=1;CHECK(Chassis_Inverse(&g,&b,special.f,&w,&scale)==0);g.wheel_diameter_mm=special.f;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==0);g=g_chassis_geometry;
special.bits=0x7F800000U;b.wz_rad_s=special.f;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==0&&zero_wheels(&w));b.wz_rad_s=FLT_MAX;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==0&&zero_wheels(&w));b.wz_rad_s=0;b.vx_mm_s=b.vy_mm_s=FLT_MAX;CHECK(Chassis_Inverse(&g,&b,1,&w,&scale)==0&&zero_wheels(&w));
CHECK(Chassis_Forward(&g,0,&r)==0&&r.vx_mm_s==0&&r.vy_mm_s==0&&r.wz_rad_s==0);CHECK(Chassis_Forward(&g,&w,0)==0);CHECK(Chassis_Forward(0,&w,&r)==0&&r.vx_mm_s==0);w.rpm[0]=special.f;CHECK(Chassis_Forward(&g,&w,&r)==0&&r.vx_mm_s==0&&r.vy_mm_s==0&&r.wz_rad_s==0);
printf("PASS %u checks (actual chassis kinematics source, no actuator calls)\n",checks);return 0;}
