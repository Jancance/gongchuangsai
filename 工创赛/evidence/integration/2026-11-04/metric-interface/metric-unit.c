#define WIRE "\115\101\061\054\067\054\062\054\062\054\061\054\062\054\061\064\054\061\060\054\062\060\054\063\060\054\064\060\054\065\060\054\061\054\061\060\054\062\060\054\071\065\060\054\061\054\055\061\062\065\060\060\054\062\065\060\060\054\055\066\060\060\054\070\065\067\106\015\012"
static const char *BAD[]={"\115\101\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\062\054\105\067\061\064\015\012","\115\101\061\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\071\054\104\101\064\104\015\012","\115\101\061\054\055\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\061\054\103\106\065\060\015\012"};

#include <stdio.h>
#include "metric_alignment_bridge.h"
static uint32_t fixture_ready=1;
static VehicleAlignmentRequest fixture_request={2,2,1,2,14,2060,1200,0};
uint32_t VehicleMission_ReadAlignmentRequest(VehicleAlignmentRequest *out)
{if(!fixture_ready)return 0;*out=fixture_request;return 1;}
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line %u\n",__LINE__);}}while(0)
static MetricAlignmentRequest req={7,2,2,1,2,14,100,1000,2060000,1200000,{1,10,20,30,40,50,200,900,100000,10000}};
int main(void){
 MetricAlignmentInbox c;MetricAlignmentObservation o,bad,saved;
 CK(MetricAlignment_Decode(WIRE,sizeof(WIRE)-1,&o)==MA_OK);
 CK(o.dx_milli_mm==-12500 && o.dy_milli_mm==2500 && o.yaw_mdeg==-600);
 CK(MetricAlignment_Begin(&c,&req,1)==MA_OK);
 CK(MetricAlignment_Accept(&c,&o,140,1)==MA_OK);
 CK(MetricAlignment_Read(&c,141,1,&saved)==MA_OK && saved.dx_milli_mm==-12500);
 CK(MetricAlignment_Accept(&c,&o,141,1)==MA_DUPLICATE);
 CK(MetricAlignment_Read(&c,321,1,&saved)==MA_EXPIRED);
 CK(MetricAlignment_Begin(&c,&req,1)==MA_OK);
 CK(MetricAlignment_Accept(&c,&o,1100,1)==MA_EXPIRED);
 CK(MetricAlignment_Begin(&c,&req,1)==MA_OK);
 CK(MetricAlignment_Accept(&c,&o,350,1)==MA_OLD_FRAME);
 CK(MetricAlignment_Begin(&c,&req,0)==MA_NOT_STOPPED && !c.pending);
 MetricAlignmentRequest uncal=req;uncal.profile.calibrated=0;
 CK(MetricAlignment_Begin(&c,&uncal,1)==MA_UNCALIBRATED && !c.pending);
 for(unsigned i=0;i<18;i++){
  CK(MetricAlignment_Begin(&c,&req,1)==MA_OK);bad=o;unsigned reason;
  switch(i){
   case 0:bad.run_id++;reason=MA_SESSION;break;
   case 1:bad.request_id++;reason=MA_SESSION;break;
   case 2:bad.map_version++;reason=MA_SESSION;break;
   case 3:bad.stage=1;reason=MA_SESSION;break;
   case 4:bad.start_id=2;reason=MA_SESSION;break;
   case 5:bad.goal_cell=10;reason=MA_SESSION;break;
   case 6:bad.camera_id++;reason=MA_PROFILE;break;
   case 7:bad.calibration_id++;reason=MA_PROFILE;break;
   case 8:bad.mount_id++;reason=MA_PROFILE;break;
   case 9:bad.height_id++;reason=MA_PROFILE;break;
   case 10:bad.target_id++;reason=MA_PROFILE;break;
   case 11:bad.units=0;reason=MA_UNITS;break;
   case 12:bad.frame_age_ms=201;reason=MA_OLD_FRAME;break;
   case 13:bad.capture_after_request_ms=999;reason=MA_OLD_FRAME;break;
   case 14:bad.confidence_permille=899;reason=MA_CONFIDENCE;break;
   case 15:bad.dx_milli_mm=100001;reason=MA_RANGE;break;
   case 16:bad.dy_milli_mm=INT32_MIN;reason=MA_RANGE;break;
   default:bad.yaw_mdeg=10001;reason=MA_RANGE;break;
  }
  CK(MetricAlignment_Accept(&c,&bad,140,1)==reason && !c.accepted);
 }
 CK(MetricAlignment_Begin(&c,&req,1)==MA_OK);
 CK(MetricAlignment_Accept(&c,&o,140,0)==MA_NOT_STOPPED && !c.pending);
 CK(MetricAlignment_Begin(&c,&req,1)==MA_OK);
 CK(MetricAlignment_Accept(&c,&o,140,1)==MA_OK);
 CK(MetricAlignment_Invalidate(&c)==MA_OK && !c.pending && !c.accepted);
 CK(MetricAlignment_Accept(&c,&o,141,1)==MA_NO_REQUEST);
 req.issued_ms=0xfffffff0U;
 CK(MetricAlignment_Begin(&c,&req,1)==MA_OK);
 CK(MetricAlignment_Accept(&c,&o,24,1)==MA_OK);
 CK(MetricAlignment_Read(&c,25,1,&saved)==MA_OK);
 for(unsigned n=0;n<sizeof(WIRE)-1;n++)CK(MetricAlignment_Decode(WIRE,n,&bad)==MA_BAD_ARGUMENT);
 char corrupt[sizeof(WIRE)];memcpy(corrupt,WIRE,sizeof(WIRE));corrupt[10]^=1;
 CK(MetricAlignment_Decode(corrupt,sizeof(WIRE)-1,&bad)==MA_BAD_ARGUMENT);
 for(unsigned k=0;k<3;k++)CK(MetricAlignment_Decode(BAD[k],(uint32_t)strlen(BAD[k]),&bad)==MA_BAD_ARGUMENT);
 CK(MetricAlignmentBridge_Open(&c,&req.profile,7,100)==MA_OK);
 CK(MetricAlignmentBridge_Submit(&c,WIRE,sizeof(WIRE)-1,140)==MA_OK);
 CK(MetricAlignmentBridge_Read(&c,141,&saved)==MA_OK);
 fixture_request.run_id++;
 CK(MetricAlignmentBridge_Read(&c,142,&saved)==MA_NOT_STOPPED && !c.pending);
 fixture_ready=0;
 CK(MetricAlignmentBridge_Open(&c,&req.profile,8,143)==MA_NOT_STOPPED);
 fixture_ready=1;fixture_request.position_executor_enabled=1;
 CK(MetricAlignmentBridge_Open(&c,&req.profile,8,144)==MA_NOT_STOPPED);
 printf("METRIC checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
