#include <stdio.h>
#include <string.h>
#include "chassis_kinematics.h"
static unsigned int checks;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %d\n",__LINE__);return 1;}}while(0)
static int stopped(const ChassisMotorCommand *c){unsigned int i;for(i=0;i<4;i++)if(c[i].address!=i+1||c[i].speed_raw!=0)return 0;return 1;}
static int zero(const ChassisWheelSpeeds *w){return !w->rpm[0]&&!w->rpm[1]&&!w->rpm[2]&&!w->rpm[3];}
int main(void){ChassisWheelSpeeds w,r;ChassisMotorCommand c[4];ChassisBodyVelocity b;int32_t feedback[4];unsigned int i,j,mask;union{unsigned int u;float f;} special;
const unsigned int forward[4]={0,1,0,1};const unsigned int patterns[3][4]={{0,1,0,1},{0,0,1,1},{1,1,1,1}};
for(i=0;i<4;i++)CHECK(g_chassis_forward_direction[i]==forward[i]);
for(mask=0;mask<16;mask++){for(i=0;i<4;i++)w.rpm[i]=(mask&(1U<<i))?-1.0f:1.0f;CHECK(Chassis_EncodeEmm(&w,1,c)==1);for(i=0;i<4;i++){CHECK(c[i].address==i+1&&c[i].speed_raw==1&&c[i].direction==(forward[i]^((mask>>i)&1)));feedback[i]=c[i].direction?-1:1;}CHECK(Chassis_EmmFeedbackToLogical(feedback,15,&r)==1);for(i=0;i<4;i++)CHECK(r.rpm[i]==w.rpm[i]);}
for(i=0;i<4;i++)w.rpm[i]=0;CHECK(Chassis_EncodeEmm(&w,1,c)==1&&stopped(c));
for(i=0;i<4;i++)w.rpm[i]=3000;CHECK(Chassis_EncodeEmm(&w,3000,c)==1);for(i=0;i<4;i++)CHECK(c[i].speed_raw==3000);
CHECK(Chassis_EncodeEmm(&w,1,c)==0&&stopped(c));CHECK(Chassis_EncodeEmm(&w,0,c)==0&&stopped(c));CHECK(Chassis_EncodeEmm(&w,3001,c)==0&&stopped(c));CHECK(Chassis_EncodeEmm(0,1,c)==0&&stopped(c));CHECK(Chassis_EncodeEmm(&w,1,0)==0);
for(i=0;i<4;i++)w.rpm[i]=1;w.rpm[3]=0.6f;memset(c,0xA5,sizeof(c));CHECK(Chassis_EncodeEmm(&w,1,c)==0&&stopped(c));w.rpm[3]=0.99999994f;CHECK(Chassis_EncodeEmm(&w,1,c)==1&&c[3].speed_raw==1);
w.rpm[3]=1.0002f;CHECK(Chassis_EncodeEmm(&w,1,c)==0&&stopped(c));special.u=0x7FC00000;w.rpm[3]=special.f;CHECK(Chassis_EncodeEmm(&w,1,c)==0&&stopped(c));special.u=0x7F800000;w.rpm[3]=special.f;CHECK(Chassis_EncodeEmm(&w,1,c)==0&&stopped(c));
for(i=0;i<4;i++)feedback[i]=1;for(mask=0;mask<16;mask++)if(mask!=15)CHECK(Chassis_EmmFeedbackToLogical(feedback,mask,&r)==0&&zero(&r));CHECK(Chassis_EmmFeedbackToLogical(feedback,31,&r)==0&&zero(&r));CHECK(Chassis_EmmFeedbackToLogical(0,15,&r)==0&&zero(&r));CHECK(Chassis_EmmFeedbackToLogical(feedback,15,0)==0);
feedback[3]=65536;CHECK(Chassis_EmmFeedbackToLogical(feedback,15,&r)==0&&zero(&r));feedback[3]=(-2147483647-1);CHECK(Chassis_EmmFeedbackToLogical(feedback,15,&r)==0&&zero(&r));feedback[3]=-65535;CHECK(Chassis_EmmFeedbackToLogical(feedback,15,&r)==1&&r.rpm[3]==65535);
for(j=0;j<3;j++){CHECK(Chassis_BasicTestCommands(&g_chassis_geometry,j,c)==1);for(i=0;i<4;i++){CHECK(c[i].address==i+1&&c[i].speed_raw==1&&c[i].direction==patterns[j][i]);feedback[i]=c[i].direction?-1:1;}CHECK(Chassis_EmmFeedbackToLogical(feedback,15,&w)==1);CHECK(Chassis_Forward(&g_chassis_geometry,&w,&b)==1);if(j==0)CHECK(b.vx_mm_s>4.03f&&b.vx_mm_s<4.04f&&b.vy_mm_s==0&&b.wz_rad_s==0);if(j==1)CHECK(b.vy_mm_s>4.03f&&b.vy_mm_s<4.04f&&b.vx_mm_s==0&&b.wz_rad_s==0);if(j==2)CHECK(b.wz_rad_s>0.019f&&b.wz_rad_s<0.020f&&b.vx_mm_s==0&&b.vy_mm_s==0);}
CHECK(Chassis_BasicTestCommands(&g_chassis_geometry,3,c)==0&&stopped(c));CHECK(Chassis_BasicTestCommands(0,0,c)==0&&stopped(c));CHECK(Chassis_BasicTestCommands(&g_chassis_geometry,0,0)==0);
printf("PASS %u checks (actual command/feedback adapters and three basic patterns; no hardware)\n",checks);return 0;}
