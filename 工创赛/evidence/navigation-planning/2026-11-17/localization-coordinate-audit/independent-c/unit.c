#include <stdint.h>
#include <math.h>
#include "chassis_odometry.h"
#include "chassis_kinematics.h"
const ChassisGeometry g_chassis_geometry={77.0f,212.0f,206.0f};
const uint8_t g_chassis_forward_direction[4]={0U, 1U, 0U, 1U};

static ChassisOdometry odom;
static uint32_t checks;
volatile float case_values[15];
#define CHECK(x) do {checks++;if(!(x))return 0x80000000U|__LINE__;}while(0)
#define NEAR(a,b,t) CHECK(fabsf((a)-(b))<(t))
#define PI 3.14159265358979323846f
static float wrap_test(float a){while(a>180)a-=360;while(a<=-180)a+=360;return a;}
static void wheel(uint32_t i,float mm,uint32_t ms){
 float driver=mm*(g_chassis_forward_direction[i]?-1.0f:1.0f);
 uint32_t raw=(uint32_t)(fabsf(driver)*65536.0f/(PI*g_chassis_geometry.wheel_diameter_mm)+0.5f);
 ChassisOdometry_Wheel(&odom,i,driver<0.0f,raw,ms);
}
static void sample(float x,float y,float turn,float yaw,uint32_t ms){
 ChassisOdometry_Yaw(&odom,wrap_test(yaw),ms);
 wheel(0,x+y-turn,ms);wheel(1,x-y+turn,ms);
 wheel(2,x-y-turn,ms);wheel(3,x+y+turn,ms);
 ChassisOdometry_Update(&odom,ms);
}
static void save(uint32_t c){case_values[c*3]=odom.status.x_mm;
 case_values[c*3+1]=odom.status.y_mm;case_values[c*3+2]=odom.status.heading_deg;}
uint32_t test_entry(void){
 uint32_t i,j;float turn,angle,x,y,k=(g_chassis_geometry.track_width_mm+g_chassis_geometry.wheelbase_mm)*0.5f;
 /* Case 1: HWT 89.8 degrees, then corrected 1000mm body-forward. */
 ChassisOdometry_Init(&odom);sample(0,0,0,0,1000);CHECK(odom.status.valid);
 for(i=1;i<=90;i++){angle=89.8f*(float)i/90.0f;turn=angle*PI/180.0f*k;
  sample(0,0,turn,angle,1000+i*40);CHECK(odom.status.valid);}
 turn=89.8f*PI/180.0f*k;
 for(i=1;i<=100;i++){sample((float)i*10.0f/CHASSIS_FORWARD_DISTANCE_SCALE,0,turn,89.8f,4600+i*20);CHECK(odom.status.valid);}
 NEAR(odom.status.x_mm,1000.0f*cosf(89.8f*PI/180.0f),0.08f);
 NEAR(odom.status.y_mm,1000.0f*sinf(89.8f*PI/180.0f),0.08f);
 NEAR(odom.status.heading_deg,89.8f,0.001f);save(0);
 /* Case 2: explicit field heading crossing +180, then forward. */
 CHECK(ChassisOdometry_SetPose(&odom,0,0,170,170));sample(0,0,0,170,10000);
 for(i=1;i<=20;i++){sample(0,0,(float)i*PI/180.0f*k,170+(float)i,10000+i*40);CHECK(odom.status.valid);}
 turn=20.0f*PI/180.0f*k;
 for(i=1;i<=10;i++)sample((float)i*10/CHASSIS_FORWARD_DISTANCE_SCALE,0,turn,-170,10800+i*20);
 CHECK(odom.status.valid);NEAR(odom.status.heading_deg,-170,0.001f);
 NEAR(odom.status.x_mm,100*cosf(-170*PI/180),0.08f);
 NEAR(odom.status.y_mm,100*sinf(-170*PI/180),0.08f);save(1);
 /* Case 3: both directions of pure turn leave centre at origin. */
 for(j=0;j<2;j++){
  ChassisOdometry_Init(&odom);sample(0,0,0,0,20000);
  for(i=1;i<=90;i++){angle=(j?-1.0f:1.0f)*(float)i;
   sample(0,0,angle*PI/180*k,angle,20000+i*40);CHECK(odom.status.valid);}
  NEAR(odom.status.x_mm,0,0.01f);NEAR(odom.status.y_mm,0,0.01f);
  NEAR(odom.status.heading_deg,j?-90.0f:90.0f,0.001f);
 }save(2);
 /* Case 4: gap freezes pose; later fresh samples cannot silently recover. */
 ChassisOdometry_Init(&odom);sample(0,0,0,0,30000);sample(2,0,0,0,30020);
 CHECK(odom.status.valid);x=odom.status.x_mm;y=odom.status.y_mm;
 ChassisOdometry_Update(&odom,30500);CHECK(!odom.status.valid&&odom.status.continuity_lost);
 sample(12,0,0,0,30520);sample(14,0,0,0,30540);
 CHECK(!odom.status.valid&&odom.status.continuity_lost);NEAR(odom.status.x_mm,x,0.001f);NEAR(odom.status.y_mm,y,0.001f);save(3);
 /* Case 5: explicit SetPose discards old history and restores stated frame. */
 CHECK(ChassisOdometry_SetPose(&odom,100,200,90,17));sample(14,0,0,17,30600);
 CHECK(odom.status.valid&&!odom.status.continuity_lost&&!odom.status.local_origin);
 sample(24,0,0,17,30620);CHECK(odom.status.valid);
 NEAR(odom.status.x_mm,100,0.02f);NEAR(odom.status.y_mm,200+10*CHASSIS_FORWARD_DISTANCE_SCALE,0.02f);
 NEAR(odom.status.heading_deg,90,0.001f);save(4);
 return checks;
}
