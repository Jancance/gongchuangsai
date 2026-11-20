from pathlib import Path
import ast,json
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';old=R/'工创赛/evidence/navigation-planning/2026-11-16/right-reverse210-40-prep/check_actual_c.py';oldtext=old.read_text('utf-8')
prefix=next(ast.literal_eval(n.value) for n in ast.parse(oldtext).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='prefix' for t in n.targets))
# Keep real headers, but no hardware routines from the old harness.
prefix=prefix[:prefix.index('static uint32_t clock_ms;')]+'#include "heading_session.h"\n#include <math.h>\n'
unit=r'''
volatile uint32_t review_assertions;
volatile float results[54][7];
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,n,j,tauid,lagid,gid,di,ix=0;MotionAction action[8];uint32_t count;ChassisMotorCommand cmds[4];
 const float taus[3]={.04f,.08f,.15f},gains[3]={.9f,1.0f,1.1f};
 for(id=0;id<=22;id++){
  uint32_t good=(id>=8 && id<=21);CK(AppDebugPlan_Load(id,action,&count)==good);
  if(id==20||id==21){CK(count==1 && action[0].kind==MOTION_TURN && action[0].base_rpm==82);CK(action[0].relative_deg==(id==20?90.0f:-90.0f));CK(!action[0].run_ms && !action[0].distance_mm && !action[0].reference_mm_s);}
 }
 CK(!AppDebugTurn90Allowed(20,81,90));CK(!AppDebugTurn90Allowed(20,82,-90));CK(!AppDebugTurn90Allowed(21,82,90));CK(!AppDebugTurn90Allowed(20,82,89));
 CK(Chassis_TurnBrakeCommands(81.4f,cmds));for(j=0;j<4;j++)CK(cmds[j].speed_raw==814);
 CK(!Chassis_TurnBrakeCommands(82.1f,cmds));
 for(tauid=0;tauid<3;tauid++)for(lagid=0;lagid<3;lagid++)for(gid=0;gid<3;gid++)for(di=0;di<2;di++,ix++){
  HeadingSession session;HeadingSessionInput in={0};HeadingSessionOutput out={0};
  float theta=0,rate=0,command=0,peak=0,dir=di?1.0f:-1.0f;float angles[64]={0},rates[64]={0},commands[64]={0};uint32_t stable=0,done=0;
  HeadingSession_Init(&session);in.max_rpm=82;in.healthy=in.platform_ok=1;in.sequence=1;
  CK(HeadingSession_Start(&session,&in,dir*90,&out));CK(session.brake.profile.valid && fabsf(session.brake.profile.peak_speed-90)<.001f);
  HeadingSession_ReportBegin(&session,1,&out);in.backend_active=in.backend_running=1;
  for(n=0;n<3000;n++){
   uint32_t source=n>=4?n-4:0,delay=lagid==0?0:(lagid==1?9:18);float observed=n>=4?angles[source%64]:0,velocity=n>=4?rates[source%64]:0;
   in.now_ms=(n+1)*5;in.sample_ms=in.now_ms;in.sequence++;in.yaw_deg=HeadingController_WrapDeg(observed);in.rate_dps=velocity;
   HeadingSession_Step(&session,&in,&out);CK(out.action!=HS_ACTION_ABORT && session.state!=HS_FAULT);CK(dir*out.command_rpm>=-.001f && fabsf(out.command_rpm)<=81.401f);
   if(out.action==HS_ACTION_STOP){HeadingSession_ReportStop(&session,1,&out);in.backend_active=in.backend_running=0;in.backend_result=1;in.backend_stop_mask=15;in.now_ms+=5;in.sample_ms=in.now_ms;in.sequence++;HeadingSession_Step(&session,&in,&out);CK(out.action==HS_ACTION_COMPLETE);done=1;break;}
   if(n%9==0)command=out.command_rpm;commands[n%64]=command;
   angles[n%64]=theta;rates[n%64]=rate;
   {float target=(n>=delay?commands[(n-delay)%64]:0)*(6.0f*77/418)*gains[gid];rate+=(target-rate)*(1-expf(-.005f/taus[tauid]));theta+=rate*.005f;}
   if(fabsf(rate)>peak)peak=fabsf(rate);
  }
  CK(done && fabsf(90-dir*theta)<=1.0f);results[ix][0]=theta;results[ix][1]=peak;results[ix][2]=n*.005f;results[ix][3]=taus[tauid];results[ix][4]=lagid*.044f;results[ix][5]=gains[gid];results[ix][6]=dir;
 }
 {HeadingSession a;HeadingSessionInput in={0};HeadingSessionOutput out={0};in.max_rpm=82;in.healthy=in.platform_ok=1;in.sequence=1;HeadingSession_Init(&a);CK(HeadingSession_Start(&a,&in,90,&out));HeadingSession_Cancel(&a,&out);CK(out.action==HS_ACTION_ABORT);
  HeadingSession_Init(&a);CK(HeadingSession_Start(&a,&in,90,&out));in.now_ms=101;in.sample_ms=0;in.sequence++;HeadingSession_Step(&a,&in,&out);CK(out.action==HS_ACTION_ABORT);
  in.now_ms=in.sample_ms=0;in.sequence=1;HeadingSession_Init(&a);CK(HeadingSession_Start(&a,&in,90,&out));HeadingSession_ReportBegin(&a,1,&out);in.backend_running=in.backend_active=1;in.now_ms=in.sample_ms=5;in.sequence++;in.yaw_deg=91.1f;HeadingSession_Step(&a,&in,&out);CK(out.action==HS_ACTION_ABORT);
 }
 return 0;
}
'''
# Reuse only the repository's established compiler/emulator loader (RAM initializers included).
script='from pathlib import Path\nimport ctypes,hashlib,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET\nR=Path.cwd();E=Path(__file__).resolve().parent;U=R/"3.stm32f407工程/CHASSIS/USER";B=Path("D:/Keil_v5/ARM/arm5_compiler/Bin");T=E/"actual-c";T.mkdir(exist_ok=True)\ndef short(p):\n b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b));return b.value\n'
script+='prefix='+repr(prefix)+'\ntests='+repr(unit)+'\nsource=""\n(T/"unit.c").write_text(prefix+tests,encoding="ascii")\n'
tail=oldtext[oldtext.index('control=ET.parse'):]
tail=tail.replace("for mode,flag in [('trim_on',1),('trim_off',1),('inactive',0)]:","for mode,flag in [('turn90',1)]:")
tail=tail.replace("'-DCAPTURE_UNIT_TEST=1'","'-DAPP_TURN90_90_ENABLE=1','-DAPP_TURN_BRAKE_TRIAL_DPS=90'")
tail=tail.replace("U/'motion_profile.c']:","U/'motion_profile.c',U/'heading_session.c',U/'heading_controller.c']:")
script+=tail;(E/'check_actual_c.py').write_text(script,encoding='utf-8')
print('Prepared actual ARM C fixture')
