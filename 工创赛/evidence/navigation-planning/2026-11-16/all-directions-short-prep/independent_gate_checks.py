from pathlib import Path
import ast,ctypes,hashlib,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin');T=E/'independent-gates-c';T.mkdir(exist_ok=True)
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b));return b.value
old=R/'工创赛/evidence/navigation-planning/2026-11-16/right-reverse210-40-prep/check_actual_c.py';oldtext=old.read_text('utf8');prefix=next(ast.literal_eval(n.value) for n in ast.parse(oldtext).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='prefix' for t in n.targets))
auto=(U/'chassis_auto_test.c').read_bytes().decode('gbk');fw=auto.rsplit('s_lateral_enabled=',1)[1].split(';',1)[0]
assert fw.count('AppDebugPlan40Allowed')==1 and fw.count('AppMotion_Live')==1
fw=fw.replace('(*AppMotion_Live()->action_count)','action_count')
lt=auto.split('#if APP_BASELINE40_ACTIVE && APP_LEFT40_DRIFT_TRIM_ENABLE',1)[1].split('?',1)[0].strip();assert 'Chassis_Left40DriftCommands' not in lt and 'AppDebugPlan40Allowed' in lt
wrapper="static uint32_t forward_gate(uint32_t g_debug_plan_id,ChassisAutoPreparedTranslation *prepared,uint32_t action_count){\n#if APP_BASELINE40_ACTIVE\n return ("+fw+");\n#else\n return 0;\n#endif\n}\nstatic uint32_t left_gate(uint32_t g_debug_plan_id,uint32_t s_translation_mode,uint32_t s_route_trial,uint32_t s_distance_max_rpm,float s_distance_target_mm){\n#if APP_BASELINE40_ACTIVE && APP_LEFT40_DRIFT_TRIM_ENABLE\n return ("+lt+");\n#else\n return 0;\n#endif\n}\n"

tests=r"""
volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,d,r,k,i,b,z;uint32_t rpms[]={99,100,101,102,103};float mm[]={299,300,301,1499,1500,1501,2100,2101};
 ChassisAutoPreparedTranslation p;ChassisMotorCommand cmd[4];memset(&p,0,sizeof(p));p.profile.valid=1;p.run_ms=12779;
 for(id=0;id<=20;id++)for(d=0;d<=4;d++)for(r=0;r<5;r++)for(k=0;k<8;k++){
  uint32_t forward=(id==8&&mm[k]==2100)||(id==14&&mm[k]==1500)||(id==15&&mm[k]==300);
  uint32_t left=(id==9&&mm[k]==2100)||(id==18&&mm[k]==1500)||(id==19&&mm[k]==300);
  p.motion_mode=d;p.base_rpm=rpms[r];p.distance_mm=mm[k];
  CK(forward_gate(id,&p,1)==(APP_BASELINE40_ACTIVE && forward && d==0 && rpms[r]==100));
  CK(left_gate(id,d,0,rpms[r],mm[k])==(APP_BASELINE40_ACTIVE && APP_LEFT40_DRIFT_TRIM_ENABLE && left && d==1 && rpms[r]==102));
 }
 for(i=0;i<3;i++){
  id=i==0?8:(i==1?14:15);p.motion_mode=0;p.base_rpm=100;p.distance_mm=i==0?2100:(i==1?1500:300);p.run_ms=12779;p.profile.valid=1;p.route_reference_mm_s=0;
  CK(forward_gate(id,&p,1)==APP_BASELINE40_ACTIVE);
  CK(!forward_gate(id,&p,0));CK(!forward_gate(id,&p,2));
  p.route_reference_mm_s=1;CK(!forward_gate(id,&p,1));p.route_reference_mm_s=0;
  p.profile.valid=0;CK(!forward_gate(id,&p,1));p.profile.valid=1;
  p.run_ms=12778;CK(!forward_gate(id,&p,1));p.run_ms=12779;
  id=i==0?9:(i==1?18:19);CK(!left_gate(id,1,1,102,p.distance_mm));
 }
#if APP_BASELINE40_ACTIVE
 for(b=0;b<=102;b++)for(z=0;z<3;z++){
  int32_t logical[4];uint32_t trim=b<25?0:(b<75?1:2);float h=(float)((int32_t)z*2-2);
  CK(Chassis_Left40DriftCommands(b,h,cmd));
  for(i=0;i<4;i++){logical[i]=cmd[i].direction?-(int32_t)cmd[i].speed_raw:(int32_t)cmd[i].speed_raw;if(g_chassis_forward_direction[i])logical[i]=-logical[i];CK(cmd[i].speed_raw<=1060);}
  CK(logical[0]+logical[1]+logical[2]+logical[3]==-(int32_t)trim*40);
  CK(logical[0]-logical[1]-logical[2]+logical[3]==(int32_t)b*40);
  if(!b)for(i=0;i<4;i++)CK(cmd[i].speed_raw==0);
 }
#endif
 return 0;
}
"""
prefix+=wrapper;source='';(T/'unit.c').write_bytes((prefix+tests).replace('\r\n','\n').encode('ascii'))
tail=oldtext[oldtext.index('control=ET.parse'):].replace("E/'actual_c_checks.json'","E/'independent_gate_checks.json'")
exec(compile(tail,str(old),'exec'))
