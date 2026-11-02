/* Offline plant only: live heading-controller math plus an unintegrated profile prototype.
 * No CAN/VM/RTOS, no hardware permission. Scalar profile units here are degrees. */
#include "heading_controller.h"
#include "motion_profile.h"
#include <math.h>
#include <stdint.h>
#define CASES 162U
#ifndef TURN_SIM_SPEED_DPS
#define TURN_SIM_SPEED_DPS 30.0f
#endif
#ifndef TURN_SIM_NOISE_DEG
#define TURN_SIM_NOISE_DEG 0.0f
#endif
#ifndef TURN_SIM_SENSOR_TICKS
#define TURN_SIM_SENSOR_TICKS 2U
#endif
volatile float results[CASES][15];
volatile float trace_values[12];
volatile uint32_t trace_count;
__attribute__((noinline)) void TurnTrace(void){trace_count++;}
static float clip(float x,float limit){return x>limit?limit:(x< -limit?-limit:x);}
static float wrap(float x){while(x>180)x-=360;while(x< -180)x+=360;return x;}
static int32_t round_rpm(float x){return x>=0?(int32_t)(x+0.5f):(int32_t)(x-0.5f);}
uint32_t test_entry(void)
{
    uint32_t variant,lag,tau_index,gain_index,dir_index,id=0,fail=0;
    const float coefficient=6.0f*77.0f/(212.0f+206.0f);
    const float taus[3]={0.08f,0.20f,0.40f},gains[3]={0.90f,1.0f,1.10f};
    for(variant=0;variant<3;variant++)for(lag=0;lag<3;lag++)
    for(tau_index=0;tau_index<3;tau_index++)for(gain_index=0;gain_index<3;gain_index++)
    for(dir_index=0;dir_index<2;dir_index++,id++) {
        HeadingController control;HeadingControllerOutput output;
        HeadingControllerConfig cfg=g_heading_controller_default_config;
        MotionProfile profile;MotionProfileSample reference;
        MotionProfileLimits limits={TURN_SIM_SPEED_DPS,15.0f,60.0f};
        float velocity[4]={0},commands[256]={0},angles[256]={0},rates[256]={0};
        float yaw=170.0f,rate=0,progress=0,dir=dir_index?-1.0f:1.0f;
        float planned_speed=0,planned_accel=0,old_accel=0,peak_rate=0,peak_accel=0;
        float peak_jerk=0,peak_overshoot=0,peak_jump=0,done_s=-1,stable_s=0;
        int32_t request=0,previous=0,last_sign=0;uint32_t reversals=0,step,w,done=0;
        uint32_t capture=(lag==1U&&tau_index==1U&&gain_index==1U&&dir_index==0U)||
            (lag==2U&&tau_index==2U&&gain_index==2U&&dir_index==0U);
        cfg.coarse_rpm=variant==1?27U:5U;cfg.timeout_ms=30000U;cfg.tolerance_deg=1.0f;
        control=(HeadingController){0};
        if(!HeadingController_Start(&control,&cfg,yaw+dir*90,yaw,0))return 0x80000001U;
        if(!MotionProfile_Prepare(&profile,90.0f,&limits))return 0x80000002U;
        for(step=0;step<4000U;step++) {
            float time=step*0.01f,observed_yaw=step>=TURN_SIM_SENSOR_TICKS?angles[(step-TURN_SIM_SENSOR_TICKS)%256U]:170.0f;
            float observed_rate=step>=TURN_SIM_SENSOR_TICKS?rates[(step-TURN_SIM_SENSOR_TICKS)%256U]:0.0f;
            float observed;
            observed_yaw+=TURN_SIM_NOISE_DEG*sinf((float)step*0.17f);
            observed_rate+=(TURN_SIM_NOISE_DEG>0.0f?0.5f:0.0f)*sinf((float)step*0.13f);
            observed=dir*(observed_yaw-170.0f);
            if(!done) {
                if(variant<2U) {
                    HeadingController_Step(&control,step*10U,wrap(observed_yaw),observed_rate,1,&output);
                    request=output.command_rpm;
                    if(output.state==HEADING_CTRL_DONE){done=1;done_s=time;request=0;}
                    else if(output.state==HEADING_CTRL_FAULT){done=2;done_s=time;request=0;}
                } else {
                    MotionProfile_Sample(&profile,time,&reference);
                    /* Test-only remaining-angle braking envelope. No reverse correction.
                     * 0.7s observed-rate lookahead plus0.8s transport/response allowance.
                     * These are conservative model assumptions, not measured vehicle bounds. */
                    float remaining=90.0f-observed;
                    float ahead=remaining-0.7f*(dir*observed_rate>0.0f?dir*observed_rate:0.0f);
                    float distance=ahead>0.25f?ahead-0.25f:0.0f;
                    float envelope=sqrtf(12.0f*12.0f+30.0f*distance)-12.0f;
                    float want=envelope;
                    float desired_accel;
                    if(want>2.0f*distance)want=2.0f*distance;
                    if(want>TURN_SIM_SPEED_DPS)want=TURN_SIM_SPEED_DPS;
                    desired_accel=clip((want-planned_speed)*5.0f,15.0f);
                    planned_accel+=clip(desired_accel-planned_accel,60.0f*0.01f);
                    planned_speed+=planned_accel*0.01f;
                    if(planned_speed<0.0f){planned_speed=0.0f;planned_accel=0.0f;}
                    if(planned_speed>TURN_SIM_SPEED_DPS){planned_speed=TURN_SIM_SPEED_DPS;planned_accel=0.0f;}
                    if(fabsf(remaining)<=0.8f&&fabsf(observed_rate)<=0.6f) {
                        stable_s+=0.01f;planned_speed=planned_accel=0.0f;
                    } else stable_s=0.0f;
                    if(stable_s>=0.7f){done=1;done_s=time;}
                    if(time>=30.0f||remaining< -1.0f){done=2;done_s=time;}
                    request=done?0:round_rpm(dir*planned_speed/coefficient);
                }
            } else request=0;
            commands[step%256U]=(float)request;
            if(fabsf((float)(request-previous))>peak_jump)peak_jump=fabsf((float)(request-previous));
            if(request&&last_sign&&((request>0)!=(last_sign>0)))reversals++;
            if(request)last_sign=request;previous=request;
            {
                float old_rate=rate,total=0;
                for(w=0;w<4;w++) {
                    /* One batch every70ms, four sequential wheel updates; 0/80/160ms added lag. */
                    uint32_t delayed=lag*8U+w*2U;
                    float target=step>=delayed?commands[(((step-delayed)/7U)*7U)%256U]:0;
                    float wheel_gain=gains[gain_index]*(1.0f+((int32_t)w-1.5f)*0.01f);
                    velocity[w]+=(target*wheel_gain-velocity[w])*(0.01f/(taus[tau_index]+0.01f));
                    total+=velocity[w]*0.25f;
                }
                rate=coefficient*total;yaw+=rate*0.01f;progress=dir*(yaw-170.0f);
                {
                    float accel=(rate-old_rate)/0.01f,jerk=(accel-old_accel)/0.01f;
                    if(fabsf(accel)>peak_accel)peak_accel=fabsf(accel);
                    if(fabsf(jerk)>peak_jerk)peak_jerk=fabsf(jerk);
                    old_accel=accel;
                }
            }
            angles[step%256U]=yaw;rates[step%256U]=rate;
            if(fabsf(rate)>peak_rate)peak_rate=fabsf(rate);
            if(progress-90>peak_overshoot)peak_overshoot=progress-90;
            if(capture) {
                trace_values[0]=(float)id;trace_values[1]=(float)variant;trace_values[2]=time;
                trace_values[3]=progress;trace_values[4]=dir*rate;trace_values[5]=(float)request*dir;
                trace_values[6]=90-progress;trace_values[7]=(float)done;
                trace_values[8]=90.0f;
                trace_values[9]=variant==2?planned_speed:request*dir*coefficient;
                trace_values[10]=peak_accel;trace_values[11]=peak_overshoot;TurnTrace();
            }
            if(done&&time>=done_s+1.5f)break;
        }
        results[id][0]=(float)variant;results[id][1]=lag*80.0f;
        results[id][2]=taus[tau_index]*1000;results[id][3]=gains[gain_index];results[id][4]=dir;
        results[id][5]=(float)done;results[id][6]=done_s;results[id][7]=progress-90;
        results[id][8]=peak_overshoot;results[id][9]=peak_rate;results[id][10]=peak_accel;
        results[id][11]=peak_jerk;results[id][12]=peak_jump;results[id][13]=(float)reversals;
        results[id][14]=(float)step;
        if(!done||step>=4000U)fail++;
    }
    return fail;
}
