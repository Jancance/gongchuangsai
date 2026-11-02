#include "src/heading_controller.c"
#include <math.h>
#include <string.h>
volatile uint32_t check_count, failure_line;
volatile float metrics[12][8];
#define CHECK(x) do {check_count++;if(!(x)&&!failure_line)failure_line=__LINE__;}while(0)
static float ab(float x){return x<0?-x:x;}
static void plant_case(uint32_t id,float disturbance,uint32_t delay,uint32_t remove,float initial){
 HeadingHoldController h={0};float yaw=initial,rate=0,out=0,error=0,old=0,ys[64]={0},rs[64]={0};
 float sum=0,sum2=0,peak=0,peak_i=0,peak_u=0,tail_peak=0;uint32_t i,n=0;
 for(i=1;i<=(id<6U?3000U:6000U);i++){
  uint32_t k=i%64U,j=(i+64U-delay)%64U;
  float load=(remove && i>2000U)?0.0f:disturbance;
  if(id==10U && i>=2000U)load= -disturbance;
  ys[k]=yaw+((id==11U)?((i/5U)%2U?.02f:-.02f):0.0f);rs[k]=rate;HeadingController_HoldSetIntegralEnabled(&h,1U);
  CHECK(HeadingController_HoldStep(&h,0,ys[j],rs[j],i*10U,&out,&error));
  CHECK(ab(out)<=2.0001f);CHECK(ab(h.integral_rpm)<=0.3001f);
  CHECK(ab(out-old)<=0.1001f);old=out;
  rate+=(4.0f*out+load-rate)*(.01f/(.2f+.01f));yaw+=rate*.01f;
  if(ab(yaw)>peak)peak=ab(yaw);if(ab(out)>peak_u)peak_u=ab(out);
  if(ab(h.integral_rpm)>peak_i)peak_i=ab(h.integral_rpm);
  if(i>(id<6U?2000U:5000U)){sum+=ab(yaw);sum2+=yaw*yaw;n++;if(ab(yaw)>tail_peak)tail_peak=ab(yaw);}
 }
 metrics[id][0]=sum/n;metrics[id][1]=sqrtf(sum2/n);metrics[id][2]=peak;
 metrics[id][3]=tail_peak;metrics[id][4]=yaw;metrics[id][5]=h.integral_rpm;
 metrics[id][6]=peak_i;metrics[id][7]=peak_u;
}
uint32_t test_entry(void){
 HeadingHoldController h={0};float out=0,error=0,i_before;uint32_t n;union{uint32_t u;float f;}nan;
 HeadingController_HoldSetIntegralEnabled(&h,1U);
 for(n=1;n<=3000;n++)CHECK(HeadingController_HoldStep(&h,0,-.2f,0,n*10U,&out,&error));
 CHECK(ab(h.integral_rpm)<=.3001f);i_before=h.integral_rpm;
 CHECK(HeadingController_HoldStep(&h,0,-.2f,0,30000U,&out,&error));CHECK(h.integral_rpm==i_before);
 if(HEADING_INTEGRAL_TRIAL_KI>0.0f){CHECK(i_before>0);CHECK(HeadingController_HoldStep(&h,0,.2f,0,30010U,&out,&error));CHECK(h.integral_rpm<i_before);}
 HeadingController_HoldSetIntegralEnabled(&h,0);CHECK(h.integral_rpm==0 && h.integral_enabled==0);
 HeadingController_HoldSetIntegralEnabled(&h,1);h.integral_rpm=.2f;
 CHECK(HeadingController_HoldStep(&h,0,-5,0,30020U,&out,&error));CHECK(h.integral_rpm==.2f);CHECK(ab(out)<=2.0001f);
 CHECK(!HeadingController_HoldStep(&h,0,-.2f,0,30120U,&out,&error));CHECK(h.integral_rpm==0 && !h.initialized && out==0);
 HeadingController_HoldSetIntegralEnabled(&h,1);CHECK(HeadingController_HoldStep(&h,0,-.2f,0,30200U,&out,&error));h.integral_rpm=.1f;
 CHECK(!HeadingController_HoldStep(&h,1,-.2f,0,30210U,&out,&error));CHECK(h.integral_rpm==0 && out==0);
 nan.u=0x7fc00000U;HeadingController_HoldSetIntegralEnabled(&h,1);h.integral_rpm=.2f;
 CHECK(!HeadingController_HoldStep(&h,0,nan.f,0,30220U,&out,&error));CHECK(h.integral_rpm==0 && out==0);
 h.integral_rpm=.1f;HeadingController_HoldReset(&h);CHECK(!h.initialized && !h.integral_enabled && h.integral_rpm==0);
 HeadingController_HoldSetIntegralEnabled(&h,1);CHECK(HeadingController_HoldStep(&h,0,0,0,0xfffffff0U,&out,&error));CHECK(HeadingController_HoldStep(&h,0,0,0,4U,&out,&error));
 HeadingController_HoldReset(&h);HeadingController_HoldSetIntegralEnabled(&h,1);
 for(n=1;n<1000;n++)CHECK(HeadingController_HoldStep(&h,0,-.01f,0,n*10U,&out,&error));CHECK(h.integral_rpm==0);
 {
  HeadingHoldController cap={0};uint32_t dir;
  for(dir=0;dir<2;dir++) {
   HeadingController_HoldReset(&cap);HeadingController_HoldSetIntegralEnabled(&cap,1);
   for(n=1;n<=6000;n++)CHECK(HeadingController_HoldStep(&cap,0,dir?.2f:-.2f,0,n*20U,&out,&error));
   if(HEADING_INTEGRAL_TRIAL_KI>0)CHECK(ab(cap.integral_rpm-.3f*(dir?-1:1))<.0001f);
   else CHECK(cap.integral_rpm==0);
  }
 }
#if defined(HEADING_INTEGRAL_DWELL_MS) && HEADING_INTEGRAL_DWELL_MS>0
 {
  HeadingHoldController gate={0};HeadingController_HoldSetIntegralEnabled(&gate,1U);
  for(n=0;n<30;n++) {
   CHECK(HeadingController_HoldStep(&gate,0,-.2f,0,1000U+n*10U,&out,&error));
   CHECK(gate.integral_rpm==0.0f);
  }
  CHECK(HeadingController_HoldStep(&gate,0,-.2f,0,1300U,&out,&error));CHECK(gate.integral_rpm>0.0f);
  HeadingController_HoldReset(&gate);HeadingController_HoldSetIntegralEnabled(&gate,1U);
  gate.integral_rpm= -0.000001f;
  CHECK(HeadingController_HoldStep(&gate,0,-.2f,0,2000U,&out,&error));CHECK(gate.integral_rpm==0);
  CHECK(HeadingController_HoldStep(&gate,0,-.2f,0,2010U,&out,&error));CHECK(gate.integral_rpm==0);
  CHECK(HeadingController_HoldStep(&gate,0,-.01f,0,2020U,&out,&error));CHECK(gate.integral_sign_ms==0 && gate.integral_error_sign==0);
  HeadingController_HoldSetIntegralEnabled(&gate,0);CHECK(gate.integral_rpm==0 && !gate.integral_sign_ms && !gate.integral_error_sign);
 }
#endif
 plant_case(0,.12f,0,0,0);plant_case(1,-.12f,0,0,0);
 plant_case(2,.12f,4,0,0);plant_case(3,-.12f,4,0,0);
 plant_case(4,.12f,4,1,0);plant_case(5,0,4,0,.5f);
 plant_case(6,.25f,8,0,0);plant_case(7,-.25f,8,0,0);
 plant_case(8,.25f,8,1,0);plant_case(9,0,8,0,.5f);
 plant_case(10,.12f,4,0,0);plant_case(11,.12f,4,0,0);
 return failure_line?(0x80000000U|failure_line):check_count;
}
