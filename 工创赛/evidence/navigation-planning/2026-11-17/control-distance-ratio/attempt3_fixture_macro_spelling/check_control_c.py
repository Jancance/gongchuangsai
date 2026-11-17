from pathlib import Path
import ast,ctypes,hashlib,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin');T=E/'actual-c';T.mkdir(exist_ok=True)
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b));return b.value
old=(R/'工创赛/evidence/navigation-planning/2026-11-16/right-reverse210-40-prep/check_actual_c.py').read_text('utf8')
prefix=next(ast.literal_eval(n.value) for n in ast.parse(old).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='prefix' for t in n.targets))
prefix=prefix[:prefix.index('static uint32_t clock_ms;')]+'#include <math.h>\n#include "chassis_direction_guard.h"\nvolatile uint32_t g_debug_plan_id;\n'
auto=(U/'chassis_auto_test.c').read_bytes().decode('cp936')
def fn(name):
 m=re.search(r'(?m)^(?:static )?(?:uint32_t|float) '+name+r'\(',auto);assert m,name
 start=m.start();opening=auto.index('{',m.end());level=1;i=opening+1
 while level:
  if auto[i]=='{':level+=1
  if auto[i]=='}':level-=1
  i+=1
 return auto[start:i]+'\n'
source=fn('Auto_ControlTargetMm')+fn('Auto_PrepareBasicCommands')+fn('Auto_PrepareTranslationEx')+fn('ChassisAuto_PrepareTranslation')
validate=fn('ChassisAuto_BeginPreparedTranslation');validate=validate[:validate.index('    taskENTER_CRITICAL();')]+'    return 1U;\n}\n';validate=validate.replace('ChassisAuto_BeginPreparedTranslation','validate_only');source+=validate
marker='#if APP_BASELINE40_ACTIVE && APP_RIGHT40_FORWARD_TRIM_ENABLE';start=auto.index(marker)+len(marker);expression=auto[start:auto.index(' ?',start)]
source+='static uint32_t selected_right(uint32_t g_debug_plan_id,uint32_t s_translation_mode,uint32_t s_distance_max_rpm,float s_distance_target_mm,uint32_t s_route_trial){\n'+marker+'\nreturn '+expression+';\n#else\nreturn 0U;\n#endif\n}\n'
start=auto.index('if (mean >= s_distance_profile.distance_mm ||');end=auto.index('\n        {',start);pred=auto[start+4:end].strip()[:-1].replace('s_distance_profile.distance_mm','target').replace('s_profile_done','done');source+='static uint32_t actual_arrival(float mean,float target,uint32_t done){return '+pred+';}\n'
tests=r'''
volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 float arrival=APP_DISTANCE_CURVE_ARRIVAL_MM;
 uint32_t id,d,r,k,i,n,b,z,route;MotionAction a[8];ChassisMotorCommand cmd[4];
 float scale[4]={0.99604743083f,1.00047641734f,1.00015875536f,0.998573466476f};
 uint32_t dirs[12]={0,1,2,3,2,2,0,0,3,3,1,1};
 float mm[12]={2100,2100,2100,2100,1500,300,1500,300,1500,300,1500,300};
 uint32_t rpms[5]={99,100,101,102,103};float ds[6]={299,300,1500,1501,2100,2101};
 for(id=0;id<=22;id++)for(d=0;d<4;d++)for(r=0;r<5;r++)for(k=0;k<6;k++){
  uint32_t good=APP_BASELINE40_ACTIVE && id>=8 && id<=19 && d==dirs[id-8] && ds[k]==mm[id-8] && rpms[r]==((d==0||d==3)?100:102);
  CK(AppDebugPlan40Allowed(id,d,rpms[r],ds[k])==good);
  g_debug_plan_id=id;
  {float expected=ds[k];if(APP_CONTROL_DISTANCE_RATIO_ENABLE&&good&&ds[k]==2100)expected*=scale[d];
   CK(fabsf(Auto_ControlTargetMm(d,rpms[r],ds[k],0)-expected)<.001f);
   CK(Auto_ControlTargetMm(d,rpms[r],ds[k],400)==ds[k]);}

  for(route=0;route<2;route++)CK(selected_right(id,d,rpms[r],ds[k],route)==(good && APP_RIGHT40_FORWARD_TRIM_ENABLE && !route && d==2 && (id==10||id==12||id==13)));
 }
 for(id=8;id<=19;id++){g_debug_plan_id=id;
  CK(AppDebugPlan_Load(id,a,&n)==APP_BASELINE40_ACTIVE);
  if(APP_BASELINE40_ACTIVE){ChassisAutoPreparedTranslation p,bad;
   CK(n==1 && a[0].kind==dirs[id-8]+4 && a[0].distance_mm==mm[id-8]);
   CK(ChassisAuto_PrepareTranslation(dirs[id-8],a[0].base_rpm,12779,mm[id-8],&p));
   {float expected=mm[id-8];if(APP_CONTROL_DISTANCE_RATIO_ENABLE&&id<=11)expected*=scale[dirs[id-8]];
   CK(p.valid && p.profile.valid && fabsf(p.profile.distance_mm-expected)<.001f);CK(p.distance_mm==mm[id-8]);}
   CK(p.profile.duration_s*1000+2000<12779);CK(validate_only(&p,0));
   CK(actual_arrival(p.profile.distance_mm,p.profile.distance_mm,0));
   CK(!actual_arrival(p.profile.distance_mm-.1f,p.profile.distance_mm,0));
   CK(actual_arrival(p.profile.distance_mm - arrival+.01f,p.profile.distance_mm,1));
   CK(!actual_arrival(p.profile.distance_mm - arrival-.01f,p.profile.distance_mm,1));
   bad=p;bad.profile.limits.acceleration+=.1f;CK(!validate_only(&bad,0));
   bad=p;bad.profile.limits.jerk+=.1f;CK(!validate_only(&bad,0));
   bad=p;bad.profile.distance_mm+=1;CK(!validate_only(&bad,0));
   for(i=0;i<4;i++)CK(p.commands[2][i].speed_raw==0);
  }
 }
 CK(!ChassisAuto_PrepareTranslation(2,102,12779,2101,0));CK(!validate_only(0,0));
#if APP_BASELINE40_ACTIVE
 for(d=0;d<4;d++)for(k=0;k<3;k++){
  float dist[3]={300,1500,2100};uint32_t cap=(d==0||d==3)?100:102;
  float denom=(d==1&&APP_LEFT40_DRIFT_TRIM_ENABLE)?1.02f:((d==2&&APP_RIGHT40_FORWARD_TRIM_ENABLE)?1.01f:1.0f);
  CK(MotionDistanceAcceleration(d,cap,dist[k])==150.0f/denom);
  CK(MotionDistanceJerk(d,cap,dist[k])==300.0f/denom);
  CK(MotionDistanceFeedbackLimit(d,cap,dist[k])==130 && MotionDistanceRunMsFor(d,cap,dist[k])==12779);
 }
 for(b=0;b<=102;b++)for(z=0;z<=160;z++){
  float requested=((int32_t)z-80)*.025f,limit=(b<2?b:2),h=requested,t=.01f*b,w[4],ideal[4];
  if(h>limit)h=limit;if(h< -limit)h=-limit;
  ideal[0]=-((float)b)+t-h;ideal[1]=(float)b+t+h;ideal[2]=(float)b+t-h;ideal[3]=-((float)b)+t+h;
  CK(Chassis_Right40ForwardCommands(b,requested,cmd));
  for(i=0;i<4;i++){
   CK(cmd[i].speed_raw<=1060);w[i]=(float)cmd[i].speed_raw*.1f;
   if(cmd[i].direction!=g_chassis_forward_direction[i])w[i]=-w[i];
   CK(fabsf(w[i]-ideal[i])<=.0501f);if(!b)CK(cmd[i].speed_raw==0);
  }
  CK(fabsf((w[0]+w[1]+w[2]+w[3])*.25f-t)<=.0501f);
  CK(fabsf((w[0]-w[1]-w[2]+w[3])*.25f+(float)b)<=.0501f);
  CK(fabsf((-w[0]+w[1]-w[2]+w[3])*.25f-h)<=.0501f);
 }
 CK(!Chassis_Right40ForwardCommands(103,0,cmd));CK(!Chassis_Right40ForwardCommands(102,2.01f,cmd));CK(!Chassis_Right40ForwardCommands(102,-2.01f,cmd));CK(!Chassis_Right40ForwardCommands(102,0,0));
 {union{uint32_t u;float f;} q;q.u=0x7fc00000;CK(!Chassis_Right40ForwardCommands(102,q.f,cmd));for(i=0;i<4;i++)CK(!cmd[i].speed_raw);q.u=0x7f800000;CK(!Chassis_Right40ForwardCommands(102,q.f,cmd));}
#else
 {ChassisAutoPreparedTranslation p;CK(!ChassisAuto_PrepareTranslation(2,102,12779,2100,&p));}
#endif
 return 0;
}
'''
(T/'unit.c').write_bytes((prefix+source+tests).encode('cp936'))
tail=old[old.index('control=ET.parse'):]
tail=tail.replace("for mode,flag in [('trim_on',1),('trim_off',1),('inactive',0)]:","for mode,flag in [('ratio_on',1),('ratio_off',1),('inactive',0)]:")
tail=tail.replace("'-DAPP_LEFT40_DRIFT_TRIM_ENABLE='+str(int(mode=='trim_on'))","'-DAPP_LEFT40_DRIFT_TRIM_ENABLE=1','-DAPP_RIGHT40_FORWARD_TRIM_ENABLE='+str(flag),'-DAPP_CONTROL_DISTANCE_RATIO_ENABLE='+str(int(mode=='ratio_on'))")
tail=tail.replace("['app_entry.c','app_entry.h','chassis_kinematics.c','app_fusion_capture.c','motion_trial20.h','motion_trial40.h']","['app_entry.c','app_entry.h','app_config.h','chassis_kinematics.c','chassis_auto_test.c','motion_trial20.h','motion_trial40.h']")
exec(compile(tail,'established_arm_loader','exec'))
report['source_sha256'].update({name:hashlib.sha256((U/name).read_bytes()).hexdigest() for name in ['app_config.h','chassis_auto_test.c','chassis_kinematics.h','motion_profile.c']})
report['fixture_sha256']=hashlib.sha256((T/'unit.c').read_bytes()).hexdigest()
report['fixture_scope']='Actual prepare and precritical validation only; real right selector condition plus actual AppDebugPlan40Allowed; no RTOS backend'
(E/'actual_c_checks.json').write_text(json.dumps(report,indent=2),'utf8')
