#include "src/heading_session.h"
#include "src/route_heading_reference.h"
#include <math.h>
#include <stdlib.h>
#include "src/chassis_kinematics.h"
static uint32_t checks, failure;
#define CHECK(x) do {checks++;if(!(x)&&!failure)failure=__LINE__;}while(0)
static HeadingSessionInput fresh(float yaw) {
    HeadingSessionInput in={0};
    in.now_ms=in.sample_ms=1000;in.sequence=1;in.healthy=in.platform_ok=1;
    in.max_rpm=APP_TURN_BRAKE_MAX_RPM;in.yaw_deg=yaw;return in;
}
uint32_t test_entry(void) {
    static const float origins[]={-180,-179.9f,-90,0,90,179.9f,180};
    static const float drifts[]={-3,-1,-.1f,0,.1f,1,3};
    static const float offsets[]={0,0,-90,-180,-180,-90,0,0};
    uint32_t a,b,c,t;float target,delta;
    HeadingSession s;HeadingSessionInput in;HeadingSessionOutput out;
    RouteHeadingReference r={0};float sentinel=123.0f;
    CHECK(!RouteHeading_Target(&r,0,&sentinel));CHECK(sentinel==123);
    CHECK(!RouteHeading_Init(&r,NAN));CHECK(!RouteHeading_Init(&r,181));
    CHECK(RouteHeading_Init(&r,180));CHECK(r.origin_deg==-180);
    CHECK(!RouteHeading_Init(&r,0));CHECK(r.origin_deg==-180);
    CHECK(!RouteHeading_Target(&r,NAN,&target));CHECK(!RouteHeading_Target(&r,361,&target));
    for(a=0;a<7;a++) {
        r=(RouteHeadingReference){0};CHECK(RouteHeading_Init(&r,origins[a]));
        for(b=0;b<8;b++) {
            CHECK(RouteHeading_Target(&r,offsets[b],&target));
            CHECK(fabsf(HeadingController_ErrorDeg(target,origins[a]+offsets[b]))<.001f);
        }
        for(b=0;b<2;b++)for(c=0;c<7;c++) {
            float nominal=b?90.0f:-90.0f;
            target=HeadingController_WrapDeg(origins[a]+nominal);
            in=fresh(HeadingController_WrapDeg(origins[a]+drifts[c]));
            CHECK(RouteHeading_TurnDelta(nominal,target,in.yaw_deg,&delta));
            CHECK(fabsf(delta-(nominal-drifts[c]))<.001f);
            HeadingSession_Init(&s);
            CHECK(HeadingSession_StartReference(&s,&in,nominal,target,&out));
            CHECK(fabsf(HeadingController_ErrorDeg(out.target_deg,target))<.001f);
            CHECK(fabsf(s.brake.angle-fabsf(delta))<.001f);
            CHECK(s.initial_sign==(b?1:-1));CHECK(s.brake.lookahead_s==.85f);
            CHECK(!HeadingSession_StartReference(&s,&in,nominal,target,&out));
            HeadingSession_Cancel(&s,&out);CHECK(out.action==HS_ACTION_ABORT);
        }
    }
    in=fresh(0);
    for(a=0;a<2;a++) {
        float sign=a?1.0f:-1.0f;
        HeadingSession_Init(&s);CHECK(!HeadingSession_StartReference(&s,&in,90,90+sign*3.01f,&out));
        CHECK(!s.used);CHECK(s.state==HS_IDLE);
    }
    HeadingSession_Init(&s);CHECK(!HeadingSession_StartReference(&s,&in,89,90,&out));
    CHECK(!HeadingSession_StartReference(&s,&in,90,NAN,&out));
    CHECK(!HeadingSession_StartReference(&s,&in,90,INFINITY,&out));
    in.healthy=0;CHECK(!HeadingSession_StartReference(&s,&in,90,90,&out));
    in=fresh(0);in.sample_ms=900;CHECK(!HeadingSession_StartReference(&s,&in,90,90,&out));
    in=fresh(0);in.backend_active=1;CHECK(!HeadingSession_StartReference(&s,&in,90,90,&out));
    in=fresh(0);in.max_rpm=28;CHECK(!HeadingSession_StartReference(&s,&in,90,90,&out));
    /* Original high-speed entry remains exactly +/-90, original preview unchanged. */
    in=fresh(0);CHECK(!HeadingSession_Start(&s,&in,90.01f,&out));
    CHECK(HeadingSession_Start(&s,&in,90,&out));CHECK(s.brake.lookahead_s==.7f);
    /* Source loss, reverse, overshoot, timeout, and stop confirmation stay enforced. */
    for(a=0;a<5;a++) {
        HeadingSession_Init(&s);in=fresh(179);
        CHECK(HeadingSession_StartReference(&s,&in,90,-91,&out));
        HeadingSession_ReportBegin(&s,1,&out);
        in.now_ms=in.sample_ms=1020;in.sequence=2;in.backend_running=1;
        if(a==0)in.healthy=0;
        if(a==1)in.yaw_deg=176; /* reversed three degrees */
        if(a==2)in.yaw_deg=-89.9f; /* overshoot exceeds one degree */
        if(a==3)in.now_ms=in.sample_ms=1000+HS_RUN_MS;
        if(a==4)in.backend_result=2;
        HeadingSession_Step(&s,&in,&out);CHECK(s.state==HS_FAULT);CHECK(out.action==HS_ACTION_ABORT);
    }
    /* Prescribed angle trajectory tests target wrap and the complete stop handshake.
       It is not a mechanical simulation; the separate plant suite provides that. */
    for(a=0;a<2;a++) {
        float sign=a?1.0f:-1.0f;
        HeadingSession_Init(&s);in=fresh(179);
        target=HeadingController_WrapDeg(179+sign*90);
        CHECK(HeadingSession_StartReference(&s,&in,sign*90,target,&out));
        HeadingSession_ReportBegin(&s,1,&out);
        for(t=1;t<700 && s.state==HS_RUNNING;t++) {
            float progress=(float)t*.5f;if(progress>89.8f)progress=89.8f;
            in.now_ms=in.sample_ms=1000+t*20;in.sequence++;
            in.yaw_deg=HeadingController_WrapDeg(179+sign*progress);
            in.rate_dps=t<180?sign*25:0;in.backend_running=1;
            HeadingSession_Step(&s,&in,&out);
            CHECK(out.command_rpm*sign>=0);CHECK(fabsf(out.command_rpm)<=27);
        }
        CHECK(s.state==HS_STOPPING);CHECK(out.action==HS_ACTION_STOP);
        HeadingSession_ReportStop(&s,1,&out);
        in.backend_running=0;in.backend_result=1;in.backend_stop_mask=7;
        HeadingSession_Step(&s,&in,&out);CHECK(s.state==HS_STOPPING);
        in.backend_stop_mask=15;HeadingSession_Step(&s,&in,&out);CHECK(s.state==HS_DONE);
    }
    /* Refreshing four wheel commands must not freeze a turn after first RUN. */
    HeadingSession_Init(&s);in=fresh(0);
    CHECK(HeadingSession_StartReference(&s,&in,90,90,&out));HeadingSession_ReportBegin(&s,1,&out);
    in.now_ms=in.sample_ms=1020;in.sequence++;in.backend_running=0;in.backend_active=1;
    HeadingSession_Step(&s,&in,&out);CHECK(!s.turn_running_seen);CHECK(out.command_rpm==0);
    in.now_ms=in.sample_ms=1040;in.sequence++;in.backend_running=1;
    HeadingSession_Step(&s,&in,&out);CHECK(s.turn_running_seen);
    for(t=1;t<=8;t++) {
        in.now_ms=in.sample_ms=1040+20*t;in.sequence++;in.backend_running=0;
        in.yaw_deg=(float)t*.1f;in.rate_dps=5;
        HeadingSession_Step(&s,&in,&out);CHECK(s.state==HS_RUNNING);
    }
    CHECK(s.brake.feedback_angle>.1f);CHECK(s.brake.speed>0);
    in.healthy=0;HeadingSession_Step(&s,&in,&out);CHECK(s.state==HS_FAULT);
#ifdef APP_FINE_RPM_OFFLINE_ONLY
    {
        ChassisMotorCommand cmd[4];ChassisWheelSpeeds speeds={{0.1f,-0.1f,50.3f,-50.3f}};
        HeadingHoldController hold={0};float correction,error,effective;uint32_t k;
        CHECK(Chassis_EncodeEmm(&speeds,54,cmd));
        CHECK(cmd[0].speed_raw==1 && cmd[1].speed_raw==1);
        CHECK(cmd[2].speed_raw==503 && cmd[3].speed_raw==503);
        CHECK(Chassis_TranslationProfileCommands(0,10,.4f,cmd));
        CHECK(cmd[0].speed_raw==96 && cmd[1].speed_raw==104);
        CHECK(cmd[2].speed_raw==96 && cmd[3].speed_raw==104);
        CHECK(!Chassis_TranslationProfileCommands(0,10,NAN,cmd));
        CHECK(!Chassis_TurnBrakeCommands(NAN,cmd));
        CHECK(!Chassis_TurnBrakeCommands(27.1f,cmd));
        CHECK(Chassis_TurnBrakeCommands(.1f,cmd));
        for(k=0;k<4;k++)CHECK(cmd[k].speed_raw==1);
        CHECK(Chassis_ProfileCorrectionRpm(0,2,&effective));CHECK(effective==0);
        CHECK(!Chassis_ProfileCorrectionRpm(1,2.01f,&effective));
        for(k=0;k<100;k++)CHECK(HeadingController_HoldStep(&hold,0,-.5f,0,1000+20*k,&correction,&error));
        CHECK(fabsf(correction-.4f)<.001f);
        for(k=0;k<100;k++)CHECK(HeadingController_HoldStep(&hold,0,0,0,3000+20*k,&correction,&error));
        CHECK(correction==0);CHECK(hold.quantized_deci_rpm==0);
        CHECK(!HeadingController_HoldStep(&hold,0,NAN,0,5000,&correction,&error));
    }
#endif
    return failure?(0x80000000U|failure):checks;
}
