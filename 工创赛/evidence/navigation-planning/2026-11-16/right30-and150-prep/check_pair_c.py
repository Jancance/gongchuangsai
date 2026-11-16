from pathlib import Path
import ast,ctypes,hashlib,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).resolve().parent
U=R/'3.stm32f407工程/CHASSIS/USER'
B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin')
T=E/'actual-c';T.mkdir(exist_ok=True)
def short(p):
    b=ctypes.create_unicode_buffer(32768)
    assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b))
    return b.value
old=R/'工创赛/evidence/navigation-planning/2026-11-16/right-reverse210-40-prep/check_actual_c.py'
oldtext=old.read_text('utf-8')
prefix=next(ast.literal_eval(n.value) for n in ast.parse(oldtext).body
    if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='prefix' for t in n.targets))
serial=(U/'app_serial_runtime.c').read_bytes().decode('cp936')
start=serial.index('else if ((strncmp(p,"PLAN"')
end=serial.index('\r\n    {',start)
guard=serial[start+len('else if '):end]
assert 'strcmp(p,"PLAN12")==0' in guard and 'strcmp(p,"PLAN13")==0' in guard
parser='static uint32_t parse_plan(const char *p){if'+guard+'return p[5] ? (10U+(uint32_t)(p[5]-\'0\')) : (uint32_t)(p[4]-\'0\');return 99U;}\n'
tests=parser+r'''
volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,d,r,k,i,n,b,z;MotionAction a[8];ChassisMotorCommand cmd[4];
 uint32_t rpms[]={0,99,100,101,102,103};
 float distances[]={0,299,300,301,1499,1500,1501,2099,2100,2101};
 const char *bad[]={"PLAN","PLAN14","PLAN013","PLAN13x","PLAN130","PLAN012","PLAN12x","PLAN120","PLAN+12","PLAN-1","PLAN00","PLAN1 "};
 CK(DEBUG_PLAN_RIGHT150_40==12 && DEBUG_PLAN_RIGHT30_40==13 && DEBUG_PLAN_MAX==13);
 CK(parse_plan("PLAN8")==8 && parse_plan("PLAN9")==9 && parse_plan("PLAN10")==10 && parse_plan("PLAN11")==11 && parse_plan("PLAN12")==12 && parse_plan("PLAN13")==13);
 for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++) CK(parse_plan(bad[i])==99);
 for(id=0;id<=14;id++)for(d=0;d<4;d++)for(r=0;r<6;r++)for(k=0;k<10;k++){
  uint32_t expected=APP_BASELINE40_ACTIVE &&
    ((distances[k]==2100 && ((id==8&&d==0&&rpms[r]==100)||(id==9&&d==1&&rpms[r]==102)||
      (id==10&&d==2&&rpms[r]==102)||(id==11&&d==3&&rpms[r]==100)))||
     (id==12&&d==2&&rpms[r]==102&&distances[k]==1500)||
     (id==13&&d==2&&rpms[r]==102&&distances[k]==300));
  CK(AppDebugPlan40Allowed(id,d,rpms[r],distances[k])==expected);
 }
 for(d=0;d<4;d++)for(r=0;r<6;r++)for(k=0;k<10;k++){
  uint32_t expected=APP_BASELINE40_ACTIVE &&
    ((distances[k]==2100 && (((d==0||d==3)&&rpms[r]==100)||((d==1||d==2)&&rpms[r]==102)))||
     (d==2&&rpms[r]==102&&(distances[k]==1500||distances[k]==300)));
  CK(MotionTrial40Allowed(d,rpms[r],distances[k])==expected);
 }
 for(id=8;id<=13;id++){
  CK(AppDebugPlan_Load(id,a,&n)==APP_BASELINE40_ACTIVE);
  if(APP_BASELINE40_ACTIVE){
   CK(n==1 && a[0].kind==(id>=12?6U:id-4U));
   CK(a[0].base_rpm==((id==8||id==11)?100U:102U));
   CK(a[0].distance_mm==(id==12?1500.0f:(id==13?300.0f:2100.0f)));
   CK(!a[0].run_ms && a[0].relative_deg==0);
  }
 }
 CK(!AppDebugPlan_Load(12,0,&n));CK(!AppDebugPlan_Load(12,a,0));CK(!AppDebugPlan_Load(14,a,&n));
#if APP_BASELINE40_ACTIVE
 CK(!AppDebugPlan_Load(0,a,&n));CK(!AppDebugPlan_Load(5,a,&n));
 CK(MotionDistanceAllowed(2,102,1500));CK(MotionDistanceAllowed(2,102,300));
 CK(MotionDistanceAcceleration(2,102,300)==150 && MotionDistanceJerk(2,102,300)==300);
 CK(MotionDistanceFeedbackLimit(2,102,300)==130 && MotionDistanceRunMsFor(2,102,300)==12779);
 CK(MotionDistanceAcceleration(2,102,1500)==150 && MotionDistanceJerk(2,102,1500)==300);
 CK(MotionDistanceFeedbackLimit(2,102,1500)==130 && MotionDistanceRunMsFor(2,102,1500)==12779);
 CK(MotionDistanceJumpCountsPerMs(2,102,1500)==175);
 CK(MotionDistanceAcceleration(1,102,2100)==(APP_LEFT40_DRIFT_TRIM_ENABLE?150.0f/1.02f:150.0f));

 {
  MotionProfile p;MotionProfileLimits lim;MotionProfileSample sample;
  float factor=3.14159265358979323846f*77.0f/60.0f,t,previous,ds[3]={300,1500,2100};
  lim.speed=400;lim.acceleration=150-factor*4;lim.jerk=300-factor*12;
  for(k=0;k<3;k++){
   CK(MotionProfile_Prepare(&p,ds[k],&lim));
   CK(p.valid && p.limits.speed==400 && p.peak_speed<=400);
   if(k==0){CK(fabsf(p.peak_speed-167.930894808f)<.01f);CK(fabsf(p.duration_s-3.572898249f)<.001f);}
   else CK(fabsf(p.peak_speed-400)<.001f);
   previous=0;
   for(t=0;t<p.duration_s;t+=.02f){
    CK(MotionProfile_Sample(&p,t,&sample)==MOTION_PROFILE_RUNNING);
    CK(sample.position_mm>=previous-.003f && sample.position_mm<=ds[k]+.003f);
    CK(sample.speed_mm_s>=-.003f && sample.speed_mm_s<=p.peak_speed+.03f);
    CK(fabsf(sample.acceleration_mm_s2)<=lim.acceleration+.03f);
    CK(fabsf(sample.jerk_mm_s3)<=lim.jerk+.03f);previous=sample.position_mm;
   }
   CK(MotionProfile_Sample(&p,p.duration_s,&sample)==MOTION_PROFILE_DONE);
   CK(sample.position_mm==ds[k] && sample.speed_mm_s==0 && sample.acceleration_mm_s2==0);
  }
 }
 for(b=0;b<=102;b++)for(z=0;z<3;z++){
  int32_t logical[4];float h=(float)((int32_t)z*2-2);
  CK(Chassis_TranslationProfileCommands(2,b,h,cmd));
  for(i=0;i<4;i++){
   CK(cmd[i].speed_raw<=1040);
   logical[i]=cmd[i].direction?-(int32_t)cmd[i].speed_raw:(int32_t)cmd[i].speed_raw;
   if(g_chassis_forward_direction[i])logical[i]=-logical[i];
  }
  CK(logical[0]+logical[1]+logical[2]+logical[3]==0);
  CK(-logical[0]+logical[1]+logical[2]-logical[3]==(int32_t)b*40);
  if(!b)for(i=0;i<4;i++)CK(cmd[i].speed_raw==0);
 }
#endif
 return 0;
}
'''
prefix += '#include <math.h>\n'
source=''
(T/'unit.c').write_bytes((prefix+tests).replace('\r\n','\n').encode('ascii'))
tail=oldtext[oldtext.index('control=ET.parse'):]
# Same ARM compiler and Unicorn runner; only the localized tests above execute.
exec(compile(tail,str(old),'exec'))
