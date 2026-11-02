#define REQUIRE(x) do { if(!(x)) return 0x80000000U|__LINE__; } while(0)
static uint32_t check_turn_guards(void)
{
    HeadingSession s;HeadingSessionInput in={0};HeadingSessionOutput out;
    ChassisMotorCommand commands[4];MotionAction actions[8];RouteCheck check;
    RouteRow rows[2]={{MOTION_TURN,90.0f,(float)APP_TURN_BRAKE_TRIAL_DPS},{MOTION_TURN,90.0f,(float)APP_TURN_BRAKE_TRIAL_DPS}};
    int32_t rpm;
    REQUIRE(RoutePlan_Convert(rows,1,&g_chassis_geometry,actions,&check)==1);
    REQUIRE(actions[0].base_rpm==APP_TURN_BRAKE_MAX_RPM && actions[0].relative_deg==90);
    REQUIRE(!RoutePlan_Convert(rows,2,&g_chassis_geometry,actions,&check));
    rows[0].speed=45;REQUIRE(!RoutePlan_Convert(rows,1,&g_chassis_geometry,actions,&check));
    rows[0].speed=15;rows[0].amount=45;
    REQUIRE(!RoutePlan_Convert(rows,1,&g_chassis_geometry,actions,&check));
    for(rpm=-28;rpm<=28;rpm++) {
        REQUIRE(Chassis_HeadingCommands(rpm,commands)==(rpm>=-5&&rpm<=5));
        REQUIRE(Chassis_TurnBrakeCommands(rpm,commands)==(rpm>=-(int32_t)APP_TURN_BRAKE_MAX_RPM&&rpm<=(int32_t)APP_TURN_BRAKE_MAX_RPM));
    }
    in.max_rpm=APP_TURN_BRAKE_MAX_RPM+1U;in.healthy=in.platform_ok=1;in.yaw_deg=177;
    HeadingSession_Init(&s);REQUIRE(!HeadingSession_Start(&s,&in,90,&out));
    in.max_rpm=APP_TURN_BRAKE_MAX_RPM;HeadingSession_Init(&s);
    REQUIRE(!HeadingSession_Start(&s,&in,45,&out));
    HeadingSession_Init(&s);REQUIRE(HeadingSession_Start(&s,&in,90,&out));
    REQUIRE(out.command_rpm==0 && s.turn_trial);
    HeadingSession_ReportBegin(&s,1,&out);
    in.backend_active=1;in.backend_running=0;in.sequence=1;in.now_ms=in.sample_ms=50;
    HeadingSession_Step(&s,&in,&out);REQUIRE(out.command_rpm==0);
    in.now_ms=in.sample_ms=150;
    HeadingSession_Step(&s,&in,&out);
    REQUIRE(out.state==HS_FAULT && out.command_rpm==0); /* stale sequence */
    in.now_ms=in.sample_ms=0;in.sequence=0;in.backend_active=0;
    HeadingSession_Init(&s);REQUIRE(HeadingSession_Start(&s,&in,90,&out));
    HeadingSession_ReportBegin(&s,0,&out);REQUIRE(out.state==HS_FAULT);
    HeadingSession_Init(&s);REQUIRE(HeadingSession_Start(&s,&in,90,&out));
    HeadingSession_Cancel(&s,&out);REQUIRE(out.state==HS_CANCELLED && out.command_rpm==0);
    HeadingSession_Init(&s);REQUIRE(HeadingSession_Start(&s,&in,90,&out));
    in.healthy=0;HeadingSession_Step(&s,&in,&out);REQUIRE(out.state==HS_FAULT);
    in.healthy=1;HeadingSession_Init(&s);REQUIRE(HeadingSession_Start(&s,&in,90,&out));
    in.backend_result=2;HeadingSession_Step(&s,&in,&out);REQUIRE(out.state==HS_FAULT);
    in.backend_result=0;HeadingSession_Init(&s);REQUIRE(HeadingSession_Start(&s,&in,90,&out));
    in.now_ms=in.sample_ms=30000;in.sequence=1;
    HeadingSession_Step(&s,&in,&out);REQUIRE(out.state==HS_FAULT);
    in.now_ms=in.sample_ms=0;HeadingSession_Init(&s);
    REQUIRE(HeadingSession_Start(&s,&in,90,&out));s.state=HS_STOPPING;s.stop_accepted=1;
    in.backend_result=1;in.backend_stop_mask=7;
    HeadingSession_Step(&s,&in,&out);REQUIRE(out.state!=HS_DONE);
    in.backend_stop_mask=15;HeadingSession_Step(&s,&in,&out);REQUIRE(out.state==HS_DONE);
    {
        TurnBrake b;
        TurnBrake_Init(&b,0,177,90,14.36842f);
        REQUIRE(TurnBrake_Step(&b,10,-91,0,&rpm)==2); /* +92deg, crossing wrap */
        REQUIRE(rpm==0);
        TurnBrake_Init(&b,0,177,90,14.36842f);
        REQUIRE(TurnBrake_Step(&b,100,177,0,&rpm)==2);
    }
    return 0;
}
#undef REQUIRE
