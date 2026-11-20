#include <stdint.h>
#include <string.h>
#include <math.h>
#include "app_odometry.h"
#include "app_icm_observer.h"
#include "chassis_auto_test.h"
#include "heading_source.h"
#include "power.h"
#include "app_motion_runtime.h"
#include "chassis_kinematics.h"
static uint32_t now,critical,vm,jog,state,completed,route_id,count=1,dummy;
static RouteHeadingReference reference={1U,0.0f};
static MotionAction actions[8];
static AppHeadingSource heading;
static ChassisOdometrySamples samples;
static ChassisAutoStatus motion;
static PowerCommandStatus power;
static AppMotionObservation observation={&count,&dummy,&dummy,&dummy,&dummy,&dummy,0,0,&dummy,&dummy,&dummy,actions,0};
volatile uint32_t g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_chassis_auto_active,g_wheel_test_active,g_debug_plan_id;
const ChassisGeometry g_chassis_geometry={77.0f,212.0f,206.0f};
const uint8_t g_chassis_forward_direction[4]={0U,1U,0U,1U};
static AppOdometryInputs inputs={&vm,&jog,&state,&completed,&reference,&route_id};
uint32_t xTaskGetTickCount(void){return now;}
void TestEnter(void){critical++;}
void TestExit(void){if(critical)critical--;}
uint32_t App_HeadingSourceRead(AppHeadingSource *v){*v=heading;return 1U;}
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *v){*v=samples;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *v){*v=motion;return 1U;}
void Power_ReadCommandStatus(PowerCommandStatus *v){*v=power;}
const AppMotionObservation *AppMotion_Live(void){return &observation;}
static uint32_t checks;
volatile uint32_t scope_cases,observer_cases;
volatile float values[8];
#define CHECK(x) do {checks++;if(!(x))return 0x80000000U|__LINE__;}while(0)
#define NEAR(a,b,t) CHECK(fabsf((a)-(b))<(t))
static void source(float F,float L){
 float mm[4],driver;uint32_t i;
 heading.healthy=1;heading.valid_mask=3;heading.sample_ms=heading.angle_ms=heading.rate_ms=now;heading.yaw_deg=0;heading.rate_dps=0;
 samples.mask=15U;mm[0]=F+L;mm[1]=F-L;mm[2]=F-L;mm[3]=F+L;
 for(i=0;i<4;i++) {driver=mm[i]*(g_chassis_forward_direction[i]?-1.0f:1.0f);
  samples.sign[i]=driver<0;samples.raw[i]=(uint32_t)(fabsf(driver)*65536.0f/(77.0f*3.14159265358979323846f)+.5f);samples.ms[i]=now;}
}
static void tick(float F,float L){now+=20;source(F,L);AppOdometry_Step(&inputs);}
static float pathF(const AppOdometryBaseline40 *x){float a=x->path_heading_deg*.0174532925199433f;return cosf(a)*(x->pose.x_mm-x->origin_x_mm)+sinf(a)*(x->pose.y_mm-x->origin_y_mm);}
static float pathL(const AppOdometryBaseline40 *x){float a=x->path_heading_deg*.0174532925199433f;return -sinf(a)*(x->pose.x_mm-x->origin_x_mm)+cosf(a)*(x->pose.y_mm-x->origin_y_mm);}
static uint32_t metadata(AppOdometryBaseline40 a,AppOdometryBaseline40 b){a.pose.x_mm=b.pose.x_mm;a.pose.y_mm=b.pose.y_mm;return memcmp(&a,&b,sizeof(a))==0;}
static void plan(uint32_t id,uint32_t kind,float mm){
 count=1;route_id=0;memset(actions,0,sizeof(actions));actions[0].kind=kind;actions[0].base_rpm=102;actions[0].distance_mm=mm;g_debug_plan_id=id;
}
static IcmRawStatus iraw;
static void imu(void){
 iraw.state=ICM_RAW_READY;iraw.last_error=ICM_RAW_OK;iraw.fresh=1;iraw.accel_config=.0f+0x38;iraw.gyro_config=0x28;
 iraw.sample_sequence++;iraw.edge_sequence++;iraw.sample_tick_ms=now;iraw.raw_accel[2]=-8192;
 heading.angle_ms=heading.sample_ms=heading.rate_ms=now;heading.healthy=1;heading.valid_mask=3;
 AppIcmObserver_Step(&iraw);
}
uint32_t test_entry(void){
 uint32_t i,v;float expected,oldX,oldY;AppOdometryBaseline40 raw,nav,again;AppIcmObserverStatus obs,obs2;
 CHECK(sizeof(ChassisOdometryStatus)==40U);CHECK(sizeof(AppOdometryBaseline40)==64U);
 AppOdometry_Init();
 /* Actual observer obtains calibration via 200 valid static samples. */
 for(i=0;i<200;i++){now+=5;imu();}
 AppIcmObserver_Read(&obs);CHECK(obs.cal_samples==200U);
 /* Actual successful-reference scope matrix, including unsupported left30. */
 for(v=0;v<16;v++) {
  AppOdometry_Init();state=1;completed=0;reference.valid=1;reference.origin_deg=0;plan(9,5,2100);
  if(v==1)plan(18,5,1500);if(v==2)plan(19,5,300);if(v==3)plan(10,6,2100);
  if(v==4)count=2;if(v==5)actions[0].base_rpm=101;if(v==6)actions[0].distance_mm=1500;
  if(v==7)actions[0].run_ms=1;if(v==8)actions[0].reference_mm_s=1;
  if(v==9)actions[0].relative_deg=1;if(v==10)route_id=1;if(v==11)actions[0].kind=4;
  if(v==12)g_debug_plan_id=8;if(v==13)count=0;if(v==14)actions[0].base_rpm=103;if(v==15)actions[0].distance_mm=2099;
  tick(0,0);tick(0,10);AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);scope_cases++;
  CHECK(metadata(raw,nav));
  if(APP_LEFT_NAV_RESIDUAL_ENABLE && v<2){CHECK(pathF(&nav)-pathF(&raw)>.08f);}
  else CHECK(memcmp(&raw,&nav,sizeof(raw))==0);
 }
 /* Exercise raw/nav getters and raw records through real odometry integration. */
 AppOdometry_Init();AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);CHECK(!raw.anchor_valid&&!nav.anchor_valid);
 state=1;completed=0;reference.valid=1;reference.origin_deg=0;plan(9,5,2100);motion.session++;
 tick(0,0);imu();
 for(i=1;i<=75;i++){tick(-.02f*(float)i*20.0f,(float)i*20.0f);imu();}
 AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);CHECK(raw.anchor_valid&&nav.anchor_valid&&raw.pose.valid);CHECK(metadata(raw,nav));
 expected=APP_LEFT_NAV_RESIDUAL_ENABLE?.5f*.017905538496259973f*fmaxf(pathL(&raw),0):0;
 NEAR(pathF(&nav)-pathF(&raw),expected,.002f);NEAR(pathL(&nav),pathL(&raw),.002f);
 CHECK(memcmp((const void*)&g_chassis_odometry,&nav.pose,sizeof(nav.pose))==0);
 for(i=0;i<100;i++){AppOdometry_ReadNavigation40(&again);CHECK(memcmp(&nav,&again,sizeof(nav))==0);}
 completed=1;tick(-30,1500);AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);
 CHECK(g_perimeter_records[1].captured);CHECK(memcmp((const void*)&g_perimeter_records[1].pose,&raw.pose,sizeof(raw.pose))==0);
 /* Actual ICM Step uses corrected anchors once; no end-of-predict residual. */
 imu();AppIcmObserver_Read(&obs);CHECK(obs.valid);observer_cases++;
 NEAR(obs.wheel_f_mm,pathF(&nav),.003f);NEAR(obs.wheel_r_mm,-pathL(&raw),.003f);
 NEAR(obs.position_f_mm,pathF(&nav),.003f);NEAR(obs.position_r_mm,-pathL(&raw),.003f);
 values[0]=pathF(&raw);values[1]=pathF(&nav);values[2]=obs.wheel_f_mm;values[3]=obs.position_f_mm;
 AppIcmObserver_Step(&iraw);AppIcmObserver_Read(&obs2);CHECK(memcmp(&obs,&obs2,sizeof(obs))==0);observer_cases++;
 now+=5;imu();AppIcmObserver_Read(&obs2);CHECK(obs2.t1_ms==obs.t1_ms);NEAR(obs2.wheel_f_mm,obs.wheel_f_mm,.0001f);observer_cases++;
 /* Raw invalid freezes numbers: nav keeps identical transform, flags unchanged. */
 now+=20;source(-30,1500);samples.mask=0;AppOdometry_Step(&inputs);AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&again);
 CHECK(!raw.pose.valid&&raw.pose.continuity_lost);CHECK(metadata(raw,again));NEAR(again.pose.x_mm,nav.pose.x_mm,.0001f);NEAR(again.pose.y_mm,nav.pose.y_mm,.0001f);
 imu();AppIcmObserver_Read(&obs);CHECK(!obs.valid&&(obs.reasons&ICM_OBS_BAD_WHEEL));observer_cases++;
 oldX=again.pose.x_mm;oldY=again.pose.y_mm;
 /* Immediate getter after ResetRecords: both anchors invalid, no XY withdrawal. */
 AppOdometry_ResetRecords();AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);CHECK(!raw.anchor_valid&&!nav.anchor_valid);
 NEAR(nav.pose.x_mm,oldX,.0001f);NEAR(nav.pose.y_mm,oldY,.0001f);
 state=0;completed=0;tick(-30,1500);AppOdometry_ReadNavigation40(&nav);NEAR(nav.pose.x_mm,oldX,.0001f);NEAR(nav.pose.y_mm,oldY,.0001f);
 /* Manual failure keeps scope; original raw anchor-invalid behavior remains. */
 vm=1;g_odometry_pose_x_mm=123;g_odometry_pose_y_mm=456;g_odometry_pose_heading_deg=37;g_odometry_pose_request=1;
 tick(-30,1500);CHECK(g_odometry_pose_result==2);AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);CHECK(!raw.anchor_valid&&!nav.anchor_valid);
 NEAR(nav.pose.x_mm,oldX,.0001f);NEAR(nav.pose.y_mm,oldY,.0001f);
 vm=0;g_odometry_pose_request=1;tick(-30,1500);CHECK(g_odometry_pose_result==1);
 AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);CHECK(memcmp(&raw,&nav,sizeof(raw))==0);NEAR(raw.pose.x_mm,123,.0001f);NEAR(raw.pose.y_mm,456,.0001f);
 /* New left150 and then new non-left frame rebuild explicit origins. */
 AppOdometry_ResetRecords();state=1;plan(18,5,1500);tick(0,0);tick(0,10);AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);
 CHECK(metadata(raw,nav));if(APP_LEFT_NAV_RESIDUAL_ENABLE)CHECK(pathF(&nav)>pathF(&raw)+.08f);
 AppOdometry_ResetRecords();plan(10,6,2100);tick(0,0);tick(0,-10);AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);CHECK(memcmp(&raw,&nav,sizeof(raw))==0);
 CHECK(critical==0);values[4]=(float)sizeof(AppOdometryBaseline40);return checks;
}
