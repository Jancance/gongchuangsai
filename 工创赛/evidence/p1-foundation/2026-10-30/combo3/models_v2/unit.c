/* Actual candidate backend + HeadingSession + odometry adapter.
 * The sequence driver and CAN/VM/RTOS/ground plant are offline models. */
#include "src/chassis_auto_test.c"
#include "src/chassis_odometry.c"
#include "src/heading_session.h"
#include "src/route_plan.h"
#include <math.h>
#include <string.h>
uint32_t test_tick,test_depth,test_primask;
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm,g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t checks,first_failure,scenario,action_index,plant_step,fault_kind,fault_action,power_active;
static float encoder[4],wheel_rpm[4],demand[4],commands[4][32],yaws[32],rates[32];
static float ground_x,ground_y,ground_yaw,ground_rate,observed_yaw,observed_rate,tau_value,gain_value,bias;
static uint32_t sensor_delay,drive_delay,transaction_count,action_begin_step,last_hs_cause;
volatile uint32_t failure_context[10];
volatile float results[48][18];
volatile float fault_details[48][16];
volatile uint32_t scenarios_run;
volatile float trace_data[12];
__attribute__((noinline)) void PerimeterTrace(void){__nop();}
#define CHECK(x) do {checks++;if(!(x)&&!first_failure){first_failure=__LINE__;failure_context[0]=scenario;failure_context[1]=action_index;failure_context[2]=test_tick;failure_context[3]=g_chassis_auto_state;failure_context[4]=g_chassis_auto_cause;failure_context[5]=s_distance_fault_reason;failure_context[6]=s_position_valid;failure_context[7]=plant_step;failure_context[8]=g_chassis_auto_result;failure_context[9]=s_odometry.status.reasons;}}while(0)

typedef struct {uint32_t sequence,sample_ms,angle_ms,rate_ms,healthy,valid_mask;float yaw_deg,rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static ChassisOdometry s_odometry;
volatile ChassisOdometryStatus g_chassis_odometry;
static struct {uint32_t state,completed;}g_combined_test;
volatile uint32_t g_odometry_pose_request,g_odometry_pose_result,g_vm_test_request;
volatile float g_odometry_pose_x_mm,g_odometry_pose_y_mm,g_odometry_pose_heading_deg;
static uint32_t s_can_jog_preparing;
static uint32_t App_HeadingFrameAge(const AppHeadingSource *s,uint32_t now,uint32_t bit){return (s->valid_mask&bit)?now-s->angle_ms:0xFFFFFFFFU;}
typedef struct {uint32_t captured,tick_ms;ChassisOdometryStatus pose;} PerimeterRecord;
volatile PerimeterRecord g_perimeter_records[4];
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
static const float s_perimeter_targets[4][3]={
{1800.0f,1800.0f,-90.0f},
{1800.0f,800.0f,-90.0f},
{800.0f,800.0f,-90.0f},
{800.0f,800.0f,-180.0f}
};
void Power_ReadCommandStatus(PowerCommandStatus *s){s->active=power_active;}
static uint32_t inject(void){return fault_kind&&action_index==fault_action&&plant_step-action_begin_step>=180U;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *r){transaction_count++;CHECK(!test_depth);r->Data[1]=s_status[a-1]&1U;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg *r,int32_t *v){
 float raw=wheel_rpm[a-1]*(g_chassis_forward_direction[a-1]?-1.0f:1.0f);
 (void)r;transaction_count++;CHECK(!test_depth);
 if(inject()&&fault_kind==1U&&a==3U)return 0U;
 if(inject()&&fault_kind==2U&&a==2U)raw=90;
 if(inject()&&fault_kind==7U&&a==3U)raw=-raw;
 *v=(int32_t)(raw>=0?raw+0.5f:raw-0.5f);return 1;
}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg *r,uint8_t *sign,uint32_t *raw){
 float value=encoder[a-1]*(g_chassis_forward_direction[a-1]?-1.0f:1.0f);
 (void)r;transaction_count++;CHECK(!test_depth);
 if(inject()&&fault_kind==3U&&a==1U)return 0U;
 if(inject()&&fault_kind==6U&&a==2U)value+=500.0f;
 *sign=value<0;*raw=(uint32_t)(fabsf(value)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);return 1;
}
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t enabled,CanRxMsg *r){(void)r;transaction_count++;CHECK(!test_depth);s_status[a-1]=enabled;return 1;}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t dir,uint16_t rpm,uint8_t accel,CanRxMsg *r){
 (void)r;transaction_count++;CHECK(!test_depth);CHECK(rpm<=54U);CHECK(!accel);
 demand[a-1]=dir==g_chassis_forward_direction[a-1]?(float)rpm:-(float)rpm;return 1;
}
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *r){
 (void)r;transaction_count++;CHECK(!test_depth);
 if(fault_kind==4U&&action_index==fault_action&&a==2U)return 0;
 demand[a-1]=0;return 1;
}
static void App_OdometryStep(void)
{
    AppHeadingSource source;
    ChassisOdometrySamples samples;
    uint32_t i,now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    taskENTER_CRITICAL(); source=s_heading_source; taskEXIT_CRITICAL();
#if APP_IS_DEBUG
    if(g_odometry_pose_request==1U) {
        PowerCommandStatus power;
        float x,y,heading;
        taskENTER_CRITICAL();
        x=g_odometry_pose_x_mm; y=g_odometry_pose_y_mm;
        heading=g_odometry_pose_heading_deg; g_odometry_pose_request=0U;
        taskEXIT_CRITICAL();
        Power_ReadCommandStatus(&power);
        g_odometry_pose_result=2U;
        if(!g_vm_test_request && !power.active && !g_chassis_auto_active &&
           !g_wheel_test_active && !s_can_jog_preparing && source.healthy &&
           App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS &&
           !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched)
            if(ChassisOdometry_SetPose(&s_odometry,x,y,heading,source.yaw_deg))
                g_odometry_pose_result=1U;
    }
#endif
    if(!s_perimeter_odom_started && g_combined_test.state==1U && source.healthy) {
        if(ChassisOdometry_SetPose(&s_odometry,1800.0f,1800.0f,-90.0f,source.yaw_deg))
            s_perimeter_odom_started=1U;
    }
    ChassisAuto_ReadOdometrySamples(&samples);
    if(source.healthy && App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS)
        ChassisOdometry_Yaw(&s_odometry,source.yaw_deg,source.angle_ms);
    else ChassisOdometry_Invalidate(&s_odometry);
    for(i=0U;i<4U;i++)
        if(samples.mask & (1UL<<i))
            ChassisOdometry_Wheel(&s_odometry,i,samples.sign[i],samples.raw[i],samples.ms[i]);
    if(samples.mask!=15U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)
        ChassisOdometry_Invalidate(&s_odometry);
    ChassisOdometry_Update(&s_odometry,now);
    taskENTER_CRITICAL();
    g_chassis_odometry=s_odometry.status;
    if(s_perimeter_odom_started && !g_perimeter_records[0].captured && s_odometry.status.valid &&
       g_combined_test.completed==0U) {
        g_perimeter_records[0].captured=1U;g_perimeter_records[0].tick_ms=now;
        g_perimeter_records[0].pose=s_odometry.status;
    }
    if(g_combined_test.completed>s_perimeter_completed_seen && g_combined_test.completed<=3U) {
        uint32_t slot=g_combined_test.completed;
        g_perimeter_records[slot].captured=1U;g_perimeter_records[slot].tick_ms=now;
        g_perimeter_records[slot].pose=s_odometry.status;
        s_perimeter_completed_seen=slot;
    }
    taskEXIT_CRITICAL();
}
static float angle_wrap(float x){return HeadingController_WrapDeg(x);}
static void plant(void){
 uint32_t w;ChassisWheelSpeeds wheels;ChassisBodyVelocity body;
 float before=ground_yaw,mid;
 for(w=0;w<4;w++) {
  uint32_t lag=drive_delay+w;
  float target=power_active&&plant_step>lag?commands[w][(plant_step+31U-lag)%32U]:0;
  float gain=gain_value*(1.0f+((w&1U)?bias:-bias));
  wheel_rpm[w]+=(target*gain-wheel_rpm[w])*(0.01f/(tau_value+0.01f));
  encoder[w]+=wheel_rpm[w]*3.14159265358979323846f*77.0f/60.0f*0.01f;
  wheels.rpm[w]=wheel_rpm[w];
 }
 CHECK(Chassis_Forward(&g_chassis_geometry,&wheels,&body));
 ground_rate=body.wz_rad_s*180.0f/3.14159265358979323846f;
 ground_yaw=angle_wrap(ground_yaw+ground_rate*0.01f);mid=(before+0.005f*ground_rate)*3.14159265358979323846f/180.0f;
 body.vx_mm_s*=CHASSIS_FORWARD_DISTANCE_SCALE;
 ground_x+=(body.vx_mm_s*cosf(mid)-body.vy_mm_s*sinf(mid))*0.01f;
 ground_y+=(body.vx_mm_s*sinf(mid)+body.vy_mm_s*cosf(mid))*0.01f;
 yaws[plant_step%32U]=angle_wrap(ground_yaw-100.0f);rates[plant_step%32U]=ground_rate;
 observed_yaw=yaws[(plant_step+32U-sensor_delay)%32U];observed_rate=rates[(plant_step+32U-sensor_delay)%32U];
 s_heading_source.healthy=1;s_heading_source.valid_mask=3;s_heading_source.sample_ms=test_tick;
 s_heading_source.angle_ms=test_tick-sensor_delay*10U;s_heading_source.rate_ms=s_heading_source.angle_ms;
 s_heading_source.sequence++;s_heading_source.yaw_deg=observed_yaw;s_heading_source.rate_dps=observed_rate;
}
static void finish_tick(void){
 uint32_t w;
 App_OdometryStep();CHECK(!test_depth);
 for(w=0;w<4;w++)commands[w][plant_step%32U]=demand[w];
 if(scenario==0U || (fault_kind==0U&&scenario==24U)) {
  trace_data[0]=(float)scenario;trace_data[1]=(float)test_tick;trace_data[2]=(float)action_index;
  trace_data[3]=ground_x;trace_data[4]=ground_y;trace_data[5]=ground_yaw;
  trace_data[6]=g_chassis_odometry.x_mm;trace_data[7]=g_chassis_odometry.y_mm;
  trace_data[8]=g_chassis_odometry.heading_deg;trace_data[9]=(float)g_chassis_odometry.valid;
  trace_data[10]=(float)g_chassis_auto_state;trace_data[11]=(float)g_chassis_auto_cause;
  if(!(plant_step%10U))PerimeterTrace();
 }
 test_tick+=10U;plant_step++;
}
static void reset(float tau,uint32_t delay,float wheel_bias){
 uint32_t i;
 test_tick=1000U;test_depth=test_primask=0;plant_step=0;power_active=1;
 memset(encoder,0,sizeof(encoder));memset(wheel_rpm,0,sizeof(wheel_rpm));memset(demand,0,sizeof(demand));memset(commands,0,sizeof(commands));
 for(i=0;i<32;i++){yaws[i]=170;rates[i]=0;}
 ground_x=ground_y=1800;ground_yaw=-90;ground_rate=0;observed_yaw=170;observed_rate=0;
 tau_value=tau;gain_value=1.0f;bias=wheel_bias;sensor_delay=delay;drive_delay=delay;
 g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=0;g_safety_ready=g_can_guard_ready=1;
 g_motion_diag_fault_latched=g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=g_wheel_test_active=g_wheel_test_cut_vm=0;
 ChassisAuto_Init();CHECK(ChassisAuto_BeginSequence());ChassisAuto_SafetyUpdate(1);
 ChassisOdometry_Init(&s_odometry);s_perimeter_odom_started=s_perimeter_completed_seen=0;
 memset((void*)g_perimeter_records,0,sizeof(g_perimeter_records));memset(&s_heading_source,0,sizeof(s_heading_source));
 g_combined_test.state=1;g_combined_test.completed=0;
}
static uint32_t action_run(const MotionAction *a){
 HeadingSession hs;HeadingSessionInput in={0};HeadingSessionOutput out={0};
 HeadingHoldController hold={0};ChassisAutoPreparedTranslation translation;ChassisAutoPreparedHeading heading;
 uint32_t start_ms=test_tick,turn=a->kind==MOTION_TURN,step,finished=0;
 float hold_target=observed_yaw;
 action_begin_step=plant_step;last_hs_cause=0U;
 in.now_ms=in.sample_ms=test_tick;in.sequence=1;in.healthy=in.platform_ok=1;in.max_rpm=27;in.yaw_deg=observed_yaw;
 if(turn){
  HeadingSession_Init(&hs);CHECK(HeadingSession_Start(&hs,&in,a->relative_deg,&out));
  CHECK(ChassisAuto_PrepareHeading(0,out.command_rpm,hs.initial_sign,&heading));
  CHECK(ChassisAuto_BeginPreparedHeading(&heading,g_chassis_auto_session));HeadingSession_ReportBegin(&hs,1,&out);
 }else{
  CHECK(ChassisAuto_PrepareRouteTranslation(a->kind-MOTION_FORWARD_DISTANCE,a->base_rpm,a->run_ms,a->distance_mm,a->reference_mm_s,&translation));
  CHECK(ChassisAuto_BeginPreparedTranslation(&translation,g_chassis_auto_session));
 }
 for(step=0;step<7500U;step++){
  uint32_t transactions_before=transaction_count;ChassisAutoStatus status;
  plant();ChassisAuto_ReadStatus(&status);
  if(turn){
   in.now_ms=test_tick;in.sample_ms=test_tick-sensor_delay*10U;in.sequence++;
   in.yaw_deg=observed_yaw;in.rate_dps=observed_rate;in.backend_active=status.active;
   in.backend_running=status.state==CHASSIS_AUTO_RUN;in.backend_result=status.result;in.backend_stop_mask=status.rotation_stop_mask;
   HeadingSession_Step(&hs,&in,&out);
   if(out.action==HS_ACTION_UPDATE)HeadingSession_ReportUpdate(&hs,ChassisAuto_UpdateHeading(out.command_rpm),&out);
   if(out.action==HS_ACTION_STOP)HeadingSession_ReportStop(&hs,ChassisAuto_RequestStop(),&out);
   if(hs.state==HS_FAULT||hs.state==HS_CANCELLED){
 if(!last_hs_cause){float *z=(float*)fault_details[scenario];z[0]=(float)action_index;z[1]=(float)test_tick;z[2]=(float)hs.cause;z[3]=(float)(in.now_ms-in.sample_ms);z[4]=(float)in.healthy;z[5]=in.yaw_deg;z[6]=in.rate_dps;z[7]=hs.brake.start_yaw;z[8]=hs.brake.direction;z[9]=hs.brake.direction*HeadingController_ErrorDeg(in.yaw_deg,hs.brake.start_yaw);z[10]=hs.brake.angle;z[11]=(float)(in.now_ms-hs.brake.tick);z[12]=(float)(in.now_ms-hs.brake.start_tick);z[13]=hs.brake.feedback_angle;z[14]=hs.brake.feedback_rate;z[15]=(float)in.backend_running;}
 last_hs_cause=hs.cause;ChassisAuto_Abort();}
  }else{
   int32_t correction=0;float error;
   CHECK(HeadingController_HoldStep(&hold,hold_target,observed_yaw,observed_rate,test_tick,&correction,&error));
   if(g_chassis_auto_active && !s_stop_due && !s_finish_requested &&
      g_chassis_auto_state<=CHASSIS_AUTO_RUN)CHECK(ChassisAuto_UpdateForwardHold(correction));
  }
  if(inject()&&fault_kind==5U){ChassisAuto_Abort();s_heading_source.healthy=0;}
  if(inject()&&fault_kind==8U)ChassisAuto_Abort();
  if(g_chassis_auto_active)ChassisAuto_CanStep();
  if(g_chassis_auto_result==2U || g_chassis_auto_cut_vm){power_active=0;}
  CHECK(transaction_count-transactions_before<=1U);
  if(!g_chassis_auto_active){
   if(g_chassis_auto_result!=1U) {finish_tick();break;}
   if(!turn || hs.state==HS_DONE){
    CHECK(g_chassis_auto_stop_mask[2]==15U);
    if(!turn)CHECK(s_distance_reached);
    g_combined_test.completed++;finished=1;
   }
  }
  finish_tick();
  if(finished)break;
 }
 CHECK(step<7500U);CHECK(test_tick-start_ms<(turn?35000U:a->run_ms+6000U));
 return finished;
}
static void pause_between(void){
 uint32_t i;
 g_combined_test.state=2;
 for(i=0;i<100;i++){plant();ChassisAuto_OdometryPoll();finish_tick();}
 CHECK(ChassisAuto_ReleaseCompleted(g_chassis_auto_session));g_combined_test.state=1;
}
static void scenario_run(uint32_t full,float distance,float speed,float tau,uint32_t delay,float wheel_bias,uint32_t fault){
 MotionAction actions[12];static const RouteRow rows[]={APP_ROUTE_ROWS(ROUTE_ROW)};RouteCheck rc;
 uint32_t i,count=full?3U:1U,finished=0,valid_mask=0U;MotionProfile profile;
 fault_kind=fault;fault_action=full?1U:0U;action_index=0;
 reset(tau,delay,wheel_bias);
 CHECK(Chassis_TranslationHoldSelfTest());CHECK(Chassis_DistanceSelfTest());
 if(full){CHECK(RoutePlan_Convert(rows,3,&g_chassis_geometry,actions,&rc));}
 else{
  actions[0]=(MotionAction){MOTION_BACKWARD_DISTANCE,0,0,0,distance,speed};
  CHECK(MotionRoute_Build(3,distance,speed,&profile,&actions[0].base_rpm,&actions[0].run_ms));
 }
 for(i=0;i<count;i++){
  action_index=i;
  if(!action_run(&actions[i]))break;
  finished++;
  if(g_perimeter_records[i+1].pose.valid)valid_mask|=1UL<<(i+1);
  if(i+1<count)pause_between();
 }
 if(g_perimeter_records[0].pose.valid)valid_mask|=1U;
 if(!fault && !(tau>0.25f || (tau>0.15f && delay))) {
  CHECK(finished==count);CHECK(g_chassis_auto_result==1U);CHECK(valid_mask==((1UL<<(count+1))-1U));
 }
 else if(!fault && finished==count) {CHECK(g_chassis_auto_result==1U);CHECK(valid_mask==((1UL<<(count+1))-1U));}
 else if(!fault) {CHECK(g_chassis_auto_cut_vm);CHECK(!power_active);CHECK(g_chassis_auto_result!=1U);}
 else{CHECK(finished<count);CHECK(g_chassis_auto_cut_vm);CHECK(!power_active);}
 results[scenario][0]=(float)full;results[scenario][1]=tau;results[scenario][2]=(float)delay;
 results[scenario][3]=wheel_bias;results[scenario][4]=(float)fault;results[scenario][5]=(float)finished;
 results[scenario][6]=(float)valid_mask;results[scenario][7]=g_chassis_odometry.x_mm;
 results[scenario][8]=g_chassis_odometry.y_mm;results[scenario][9]=g_chassis_odometry.heading_deg;
 results[scenario][10]=ground_x;results[scenario][11]=ground_y;
 results[scenario][12]=(float)g_chassis_auto_cause;results[scenario][13]=(float)s_distance_fault_reason;
 results[scenario][14]=(float)last_hs_cause;results[scenario][15]=(float)(test_tick-1000U);
 results[scenario][16]=(float)(tau>0.25f || (tau>0.15f && delay));
 results[scenario][17]=(float)g_chassis_auto_result;
 scenarios_run++;scenario++;
}
uint32_t test_entry(void){
 uint32_t d,s,t,l,f;
 const float taus[3]={0.08f,0.20f,0.40f};
 for(d=0;d<2;d++)for(s=0;s<2;s++)for(t=0;t<3;t++)for(l=0;l<2;l++)
  scenario_run(0,d?1800:150,s?200:50,taus[t],l?4:0,0,0);
 for(t=0;t<3;t++)for(l=0;l<2;l++)scenario_run(1,0,0,taus[t],l?4:0,l?0.003f:0,0);
 for(f=1;f<=6;f++)scenario_run(1,0,0,.20f,4,0,f);
 for(f=7;f<=8;f++)scenario_run(0,1800,50,.20f,4,0,f);
 for(f=7;f<=8;f++)scenario_run(1,0,0,.20f,4,0,f);
 return first_failure ? (0x80000000U|first_failure):checks;
}
