
#include "source_stage2_body_port.h"
#include "source_observer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define VEHICLE_LIFT_RULER_ONLY 0U
#define APP_MOTOR_OFFLINE_ONLY 0U
#define VEHICLE_STAGE_TIMEOUT_MS 120000U
#define VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS 2000U
#define VEHICLE_MIN_ROUTE_CM 1.0f
#define FIELD_BODY_HALF_MM 140.0f
#define MOTION_FORWARD_DISTANCE 100U
typedef struct{float x_mm,y_mm;}FieldPoint;
typedef struct{unsigned kind;float amount,speed;}RouteRow;
typedef struct{unsigned active,session,cause,rotation_stop_mask;}ChassisAutoStatus;
static ChassisAutoStatus backend;
static struct{unsigned state,index;}g_combined_test;
static unsigned g_vehicle_run_id=10,g_vehicle_stage=2,g_vehicle_state=8,g_vehicle_cause;
static unsigned g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_vehicle_stage_mask;
static unsigned g_vehicle_alignment_state;
volatile uint32_t g_source_stage2_completed;
static unsigned s_vehicle_queue_pending,s_vehicle_queue_begin,s_serial_start_permit,s_motion_session_open;
static unsigned g_vehicle_queue_wait_ms,g_vehicle_queue_attempts,g_vehicle_queue_last_mask;
static unsigned s_vehicle_handoff_valid=1,stopped=1,loaded=1,capture_ok=1,queue_ok=1,heading_mask;
static unsigned source_invalidates,metric_ready,queue_calls,route_calls,stop_calls,metric_services;
static FieldPoint s_vehicle_handoff={2060,1200};static RouteRow last_row;
static MetricAlignmentObservation observation;
static uint32_t holding=1;
static uint32_t StartupAxes_Holding(void){return holding;}
uint32_t SourceStage2_IsActive(void){return 0;}
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus*s){*s=backend;return 1;}
static uint32_t VehicleMission_Stopped(void){return stopped;}
static uint32_t VehicleMission_CaptureHandoff(void){return capture_ok;}
static uint32_t FieldMap_AxisClear(FieldPoint a,FieldPoint b,float half){(void)half;return a.x_mm>=0 && b.x_mm>=0 && b.x_mm<2400 && b.y_mm>=0 && b.y_mm<2400;}
static uint32_t VehicleMission_Queue(uint32_t stage,const RouteRow*rows,uint32_t n){
 (void)stage;(void)n;last_row=*rows;queue_calls++;if(!queue_ok)return 0;
 stopped=0;backend.active=1;backend.session++;backend.rotation_stop_mask=0;g_combined_test.state=1;return 1;}
static uint32_t VehicleMission_Load(uint32_t stage){route_calls++;if(!queue_ok)return 0;g_vehicle_stage=stage;g_vehicle_state=2;
 stopped=0;backend.active=1;backend.session++;backend.rotation_stop_mask=0;g_combined_test.state=1;g_combined_test.index=0;return 1;}
static uint32_t VehicleMission_PublishQueued(uint32_t stage){(void)stage;s_vehicle_queue_pending=0;return 1;}
static uint32_t App_SerialIdleBlocked(uint32_t now){(void)now;return heading_mask;}
static void MotionStop(void){stop_calls++;}
uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3]){(void)colors;return loaded && run==10 && batch==1;}
void SourceObserver_Init(const MetricAlignmentProfile*p){(void)p;metric_ready=0;}
void SourceObserver_Invalidate(void){metric_ready=0;source_invalidates++;}
void SourceObserver_Service(uint32_t now){(void)now;metric_services++;}
uint32_t SourceObserver_Read(uint32_t now,MetricAlignmentObservation*out){(void)now;if(!metric_ready)return MA_NO_REQUEST;*out=observation;return MA_OK;}
#include "source_stage2_body_port.inc"
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d state%u reason%u\n",__LINE__,s2bp.state,s2bp.reason);}}while(0)
static uint32_t path(void*p,const SPF_Command*c){(void)p;(void)c;return 1;}
static MetricAlignmentProfile profile(void){MetricAlignmentProfile p={1,1,2,3,4,5,200,900,20000,1000};return p;}
static S2_BodyLimits limits(void){S2_BodyLimits l={1,1,2000,500,3,30};return l;}
static void reset(void){
 memset(&s2bp,0,sizeof(s2bp));g_vehicle_run_id=10;g_vehicle_stage=2;g_vehicle_state=8;g_vehicle_cause=0;
 g_source_stage2_completed=0;backend=(ChassisAutoStatus){0,50,0,15};g_combined_test.state=3;g_combined_test.index=0;
 stopped=loaded=capture_ok=queue_ok=1;s_vehicle_queue_pending=0;metric_ready=0;queue_calls=route_calls=stop_calls=0;
 memset(&observation,0,sizeof(observation));}
static SPF_Command command(unsigned kind,unsigned seq){SPF_Command c;memset(&c,0,sizeof(c));c.run_id=10;c.sequence=seq;c.kind=kind;c.route_stage=3;return c;}
int main(void){MetricAlignmentProfile p;S2_BodyLimits l;S2_BackendHooks hooks;SPF_Command c;S2_Event e;unsigned old;
 reset();p=profile();l=limits();p.calibrated=0;CK(!Stage2Body_Configure(&p,&l));p.calibrated=1;
 CK(Stage2Body_Configure(&p,&l));CK(Stage2Body_GetHooks(&hooks,0,path));
 c=command(SPF_CMD_ALIGN,1);CK(hooks.alignment_begin(0,&c,100)==1);CK(Stage2Body_Owns() && Stage2Body_KeepsPower());
 holding=0;CK(!Stage2Body_KeepsPower());holding=1;
 g_rtos_fault=1;CK(!Stage2Body_KeepsPower());g_rtos_fault=0;
 g_safety_fault=1;CK(!Stage2Body_KeepsPower());g_safety_fault=0;
 g_can_ctrl_fault_latched=1;CK(!Stage2Body_KeepsPower());g_can_ctrl_fault_latched=0;
 CK(Stage2Body_Service(110)==1 && s2bp.state==S2BP_OBSERVE);metric_ready=1;CK(Stage2Body_Service(120)==1 && s2bp.state==S2BP_DONE);
 CK(hooks.alignment_poll(0,&c,&e,130)==1 && e.run_id==10 && e.sequence==1 && e.evidence_flags==S2_ALIGNED);
 CK(!Stage2Body_Owns() && !Stage2Body_KeepsPower());
 c=command(SPF_CMD_ROUTE_AND_STOW,2);CK(hooks.route_begin(0,&c,200)==1);CK(hooks.route_begin(0,&c,201)==1);
 CK(Stage2Body_Service(210)==1 && route_calls==1 && s2bp.state==S2BP_ROUTE && g_vehicle_stage_mask==2);
 Stage2Body_Service(220);CK(s2bp.session==backend.session && s2bp.session_segment==0);
 g_combined_test.index=1;backend.session++;Stage2Body_Service(230);CK(s2bp.session==backend.session && s2bp.session_segment==1);
 stopped=1;backend.active=0;backend.rotation_stop_mask=15;g_combined_test.state=3;Stage2Body_Service(240);
 CK(hooks.route_poll(0,&c,250)==1 && Stage2Body_Owns());
 g_source_stage2_completed=1;CK(!Stage2Body_Owns() && Stage2Body_Service(260)==0);
 CK(route_calls==1);

 reset();CK(Stage2Body_Configure(&p,&l));CK(Stage2Body_GetHooks(&hooks,0,path));c=command(SPF_CMD_ALIGN,3);
 hooks.alignment_begin(0,&c,100);Stage2Body_Service(110);observation.dx_milli_mm=15000;metric_ready=1;old=source_invalidates;Stage2Body_Service(120);
 CK(s2bp.state==S2BP_MOVE && queue_calls==1 && last_row.kind==MOTION_FORWARD_DISTANCE+2 && fabsf(last_row.amount-1.5f)<.001f);
 CK(fabsf(last_row.speed-3)<.001f && source_invalidates==old+1 && !metric_ready);
 Stage2Body_Service(130);stopped=1;backend.active=0;backend.rotation_stop_mask=15;g_combined_test.state=3;Stage2Body_Service(140);
 CK(s2bp.state==S2BP_OBSERVE && !metric_ready && hooks.alignment_poll(0,&c,&e,141)==0);
 observation.dx_milli_mm=1000;metric_ready=1;Stage2Body_Service(150);CK(hooks.alignment_poll(0,&c,&e,151)==1);

 reset();CK(Stage2Body_Configure(&p,&l));Stage2Body_GetHooks(&hooks,0,path);c=command(SPF_CMD_ALIGN,4);
 hooks.alignment_begin(0,&c,100);Stage2Body_Service(110);observation.dx_milli_mm=5000;metric_ready=1;Stage2Body_Service(120);
 CK(s2bp.state==S2BP_FAULT && s2bp.reason==S2BP_SMALL_STEP && queue_calls==0);

 reset();CK(Stage2Body_Configure(&p,&l));Stage2Body_GetHooks(&hooks,0,path);c=command(SPF_CMD_ROUTE_AND_STOW,5);
 hooks.route_begin(0,&c,100);Stage2Body_Service(110);Stage2Body_Service(120);backend.session++;Stage2Body_Service(130);
 CK(s2bp.state==S2BP_FAULT && s2bp.reason==S2BP_SESSION);
 CK(!hooks.route_stop(0,140));Stage2Body_Service(150);CK(s2bp.state==S2BP_STOPPING && stop_calls==1);
 backend.active=0;backend.rotation_stop_mask=7;Stage2Body_Service(160);CK(s2bp.state==S2BP_STOPPING);
 backend.rotation_stop_mask=15;Stage2Body_Service(170);CK(hooks.route_stop(0,180)==1 && s2bp.state==S2BP_STOPPED);

 reset();CK(Stage2Body_Configure(&p,&l));Stage2Body_GetHooks(&hooks,0,path);loaded=0;c=command(SPF_CMD_ROUTE_AND_STOW,6);
 hooks.route_begin(0,&c,100);Stage2Body_Service(110);CK(route_calls==0 && s2bp.state==S2BP_FAULT);
 printf("stage2 TaskA body-port checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
