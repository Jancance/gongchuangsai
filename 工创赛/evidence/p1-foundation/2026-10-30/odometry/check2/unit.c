/* Offline only. Actual backend, route converter and extracted frontend;
 * CAN/VM/RTOS below are stubs. RUN setup is seeded, not hardware validation. */
#include "src/app_entry.c"
#include "src/chassis_auto_test.c"
#include "src/motion_control.h"
#include <math.h>
#include <string.h>
uint32_t test_tick,test_depth,test_primask;
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm,g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t checks,first_failure,model_mode,model_step,model_fault,model_delayed,model_count;
static float model_gain,position[4],velocity[4],target[4],history[4][32];
volatile uint32_t failure_context[6];
volatile uint32_t model_results[60][9];
volatile float trim_model_positions[60][3];
volatile uint32_t parity_outputs[160][4];
#define CHECK(x) do {checks++;if(!(x)&&!first_failure){first_failure=__LINE__;failure_context[0]=model_mode;failure_context[1]=model_fault;failure_context[2]=model_step;failure_context[3]=g_chassis_auto_cause;failure_context[4]=s_distance_fault_reason;failure_context[5]=model_count;}}while(0)
void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=1U;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){(void)a;r->Data[1]=1;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v)
{
    int32_t logical=(int32_t)(velocity[a-1]>=0 ? velocity[a-1]+0.5f : velocity[a-1]-0.5f);
    (void)r;
    if(model_fault==1U&&a==2U&&model_step>=100U)logical=81;
    if(model_fault==2U&&a==3U&&model_step>=100U)return 0U;
    if(model_fault==5U&&a==4U&&model_step>=100U)logical=-logical;
    *v=g_chassis_forward_direction[a-1] ? -logical : logical;return 1U;
}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*sign,uint32_t*raw)
{
    float driver=position[a-1]*(g_chassis_forward_direction[a-1] ? -1.0f : 1.0f);
    (void)r;
    if(model_fault==3U&&a==1U&&model_step>=100U)return 0U;
    *sign=driver<0;*raw=(uint32_t)(fabsf(driver)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);return 1U;
}
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t b,CanRxMsg*r){(void)a;(void)b;(void)r;return 1U;}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t direction,uint16_t rpm,uint8_t accel,CanRxMsg*r)
{
    (void)r;CHECK(test_depth==0U);CHECK(rpm<=54U);CHECK(accel==0U);
    target[a-1]=(direction==g_chassis_forward_direction[a-1]) ? (float)rpm : -(float)rpm;
    return 1U;
}
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg*r)
{
    (void)r;CHECK(test_depth==0U);
    if(model_fault==4U&&a==2U)return 0U;
    target[a-1]=0;return 1U;
}

#include "src/chassis_odometry.c"
static ChassisOdometry odom;
static uint32_t time_ms;
static void wheel(uint32_t i,float mm,uint32_t ms) {
 float driver=mm*(g_chassis_forward_direction[i] ? -1.0f : 1.0f);
 ChassisOdometry_Wheel(&odom,i,driver<0.0f,(uint32_t)(fabsf(driver)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f),ms);
}
static void sample(float x,float y,float turn,float yaw,uint32_t ms) {
 ChassisOdometry_Yaw(&odom,yaw,ms);
 wheel(0,x+y-turn,ms);wheel(1,x-y+turn,ms);
 wheel(2,x-y-turn,ms);wheel(3,x+y+turn,ms);
 ChassisOdometry_Update(&odom,ms);
}
static void reset_odom(float yaw) {
 ChassisOdometry_Init(&odom);time_ms=1000;
 sample(0,0,0,yaw,time_ms);CHECK(odom.status.valid);
}
volatile float odom_failure[4];
static uint32_t near_index;
static void near(float got,float expected) {
 near_index++;
 if(fabsf(got-expected)>=0.06f && !first_failure) {
  odom_failure[0]=got;odom_failure[1]=expected;odom_failure[2]=(float)near_index;
 }
 CHECK(fabsf(got-expected)<0.06f);
}
uint32_t test_entry(void) {
 uint32_t i,j,updates;
 ChassisOdometrySamples capture;
 /* Real backend capture survives per-action reset; no extra CAN calls. */
 ChassisAuto_Init();
 Auto_OdometrySample(0,1,0,123,300);ChassisAuto_ReadOdometrySamples(&capture);
 CHECK(capture.mask==1 && capture.raw[0]==123 && capture.ms[0]==300);
 Auto_PositionReset(1);ChassisAuto_ReadOdometrySamples(&capture);CHECK(capture.mask==1);
 Auto_OdometrySample(0,0,0,0,400);ChassisAuto_ReadOdometrySamples(&capture);CHECK(!capture.mask);
 /* Four-way translations including reverse and driver sign crossing. */
 for(j=0;j<4;j++) {
  reset_odom(177.0f);
  for(i=1;i<=100;i++) sample(j<2 ? (j==0?1:-1)*i*2.0f:0,
    j>=2 ? (j==2?1:-1)*i*2.0f:0,0,177,1000+i*20);
  CHECK(odom.status.valid);near(odom.status.x_mm,j<2?(j==0?1:-1)*200*CHASSIS_FORWARD_DISTANCE_SCALE:0);
  near(odom.status.y_mm,j>=2?(j==2?200:-200):0);
 }
 /* Pure rotation both ways crossing +/-180; centre must remain stationary. */
 for(j=0;j<2;j++) {
  reset_odom(170);
  for(i=1;i<=90;i++) {float deg=(j? -1.0f:1.0f)*i;
   sample(0,0,deg*3.14159265358979323846f/180.0f*209.0f,wrap(170+deg),1000+i*40);
  }
  CHECK(odom.status.valid);near(odom.status.x_mm,0);near(odom.status.y_mm,0);
  near(odom.status.heading_deg,j?-90:90);
 }
 /* Rotate then forward in fixed axes; no segment-reset API is involved. */
 reset_odom(0);
 for(i=1;i<=90;i++) sample(0,0,i*3.14159265358979323846f/180.0f*209.0f,(float)i,1000+i*40);
 for(i=1;i<=100;i++) sample(i*2.0f,0,3.14159265358979323846f*104.5f,90,4600+i*20);
 CHECK(odom.status.valid);near(odom.status.x_mm,0);near(odom.status.y_mm,200*CHASSIS_FORWARD_DISTANCE_SCALE);
 /* Constant simultaneous twist: exact SE2 endpoint for 90 degree arc. */
 reset_odom(0);
 for(i=1;i<=90;i++) sample((float)i,0,i*3.14159265358979323846f/180.0f*209,(float)i,1000+i*40);
 CHECK(odom.status.valid);near(odom.status.x_mm,90*CHASSIS_FORWARD_DISTANCE_SCALE*2/3.14159265358979323846f);
 near(odom.status.y_mm,90*CHASSIS_FORWARD_DISTANCE_SCALE*2/3.14159265358979323846f);
 /* Async 0/20/40/60ms wheels and 10ms yaw; never extrapolate. */
 ChassisOdometry_Init(&odom);
 for(i=0;i<=100;i++) {
  uint32_t now=1000+i*10;
  ChassisOdometry_Yaw(&odom,0,now);
  for(j=0;j<4;j++) if(i>=2*j && (i-2*j)%8==0) wheel(j,(now-1000)*0.2f,now);
  ChassisOdometry_Update(&odom,now);
 }
 CHECK(odom.status.valid);near(odom.status.x_mm,(odom.status.sample_ms-1060)*0.2f*CHASSIS_FORWARD_DISTANCE_SCALE);
 near(odom.status.y_mm,0);
 updates=odom.status.updates;ChassisOdometry_Update(&odom,2000);CHECK(updates==odom.status.updates);
 /* Staleness is sticky: recovery cannot silently repair missing displacement. */
 ChassisOdometry_Update(&odom,2500);CHECK(!odom.status.valid&&odom.status.continuity_lost);
 sample(200,0,0,0,2510);CHECK(!odom.status.valid);
 CHECK(ChassisOdometry_SetPose(&odom,100,200,90,0));
 sample(200,0,0,0,2600);CHECK(odom.status.valid&&!odom.status.local_origin);
 sample(202,0,0,0,2620);near(odom.status.x_mm,100);near(odom.status.y_mm,200+2*CHASSIS_FORWARD_DISTANCE_SCALE);
 /* Gap, encoder reset, repeated conflicting timestamp and bad sign. */
 reset_odom(0);sample(100,0,0,0,1020);CHECK(!odom.status.valid);
 reset_odom(0);wheel(0,1,1000);ChassisOdometry_Update(&odom,1000);CHECK(!odom.status.valid);
 reset_odom(0);ChassisOdometry_Wheel(&odom,0,2,0,1020);ChassisOdometry_Update(&odom,1020);CHECK(!odom.status.valid);
 reset_odom(0);ChassisOdometry_Yaw(&odom,100,1020);ChassisOdometry_Update(&odom,1020);CHECK(!odom.status.valid);
 reset_odom(0);ChassisOdometry_Yaw(&odom,0,999);ChassisOdometry_Update(&odom,1020);CHECK(!odom.status.valid);
 reset_odom(0);sample(2,0,0,0,1020);sample(0,0,0,0,1040);CHECK(odom.status.valid);near(odom.status.x_mm,0);
 /* True 32-bit clock wrap, large absolute counts, and no premature float cast. */
 ChassisOdometry_Init(&odom);sample(0,0,0,0,0xFFFFFFF0U);sample(2,0,0,0,4U);
 CHECK(odom.status.valid);near(odom.status.x_mm,2*CHASSIS_FORWARD_DISTANCE_SCALE);
 ChassisOdometry_Init(&odom);ChassisOdometry_Yaw(&odom,0,1000);
 for(i=0;i<4;i++)ChassisOdometry_Wheel(&odom,i,g_chassis_forward_direction[i],4000000000U,1000);
 ChassisOdometry_Update(&odom,1000);ChassisOdometry_Yaw(&odom,0,1020);
 for(i=0;i<4;i++)ChassisOdometry_Wheel(&odom,i,g_chassis_forward_direction[i],4000000100U,1020);
 ChassisOdometry_Update(&odom,1020);CHECK(odom.status.valid);
 near(odom.status.x_mm,100*3.14159265358979323846f*77/65536*CHASSIS_FORWARD_DISTANCE_SCALE);
 CHECK(!ChassisOdometry_SetPose(&odom,0,0,999,0));
 return first_failure ? (0x80000000U|first_failure) : checks;
}
