#include "src/heading_controller.c"
#include <math.h>
#include <string.h>
volatile uint32_t check_count, failure_line;
volatile float metrics[6][8];
#define CHECK(x) do {check_count++;if(!(x)&&!failure_line)failure_line=__LINE__;}while(0)
static float ab(float x){return x<0?-x:x;}
static void plant_case(uint32_t id,float disturbance,uint32_t delay,uint32_t remove,float initial){
 HeadingHoldController h={0};float yaw=initial,rate=0,out=0,error=0,old=0,ys[64]={0},rs[64]={0};
 float sum=0,sum2=0,peak=0,peak_i=0,peak_u=0,tail_peak=0;uint32_t i,n=0;
 for(i=1;i<=3000;i++){
  uint32_t k=i%64U,j=(i+64U-delay)%64U;
  float load=(remove && i>2000U)?0.0f:disturbance;
  ys[k]=yaw;rs[k]=rate;HeadingController_HoldSetIntegralEnabled(&h,1U);
  CHECK(HeadingController_HoldStep(&h,0,ys[j],rs[j],i*10U,&out,&error));
  CHECK(ab(out)<=2.0001f);CHECK(ab(h.integral_rpm)<=0.3001f);
  CHECK(ab(out-old)<=0.1001f);old=out;
  rate+=(4.0f*out+load-rate)*(.01f/(.2f+.01f));yaw+=rate*.01f;
  if(ab(yaw)>peak)peak=ab(yaw);if(ab(out)>peak_u)peak_u=ab(out);
  if(ab(h.integral_rpm)>peak_i)peak_i=ab(h.integral_rpm);
  if(i>2000U){sum+=ab(yaw);sum2+=yaw*yaw;n++;if(ab(yaw)>tail_peak)tail_peak=ab(yaw);}
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
 plant_case(0,.12f,0,0,0);plant_case(1,-.12f,0,0,0);
 plant_case(2,.12f,4,0,0);plant_case(3,-.12f,4,0,0);
 plant_case(4,.12f,4,1,0);plant_case(5,0,4,0,.5f);
 return failure_line?(0x80000000U|failure_line):check_count;
}
