from pathlib import Path
import ctypes,hashlib,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET,csv
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin');T=E/'actual-c';T.mkdir(exist_ok=True)
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b));return b.value
unit=r"""
#include "fusion_reference.h"
#include <math.h>
#include <string.h>
volatile uint32_t review_assertions;
volatile uint32_t size_plan=sizeof(FusionPlan),size_state=sizeof(FusionState),size_input=sizeof(FusionInput);
static FusionInput input,backup;static FusionPlan plan;static FusionCheck check;static FusionState state,saved;static FusionObservation obs,bad;static FusionOutput output,old;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
#define CLOSE(a,b,e) CK(fabsf((a)-(b))<(e))
static void setup(void){
 FusionConfig *c=&input.config;memset(&input,0,sizeof(input));
 input.model_only=1;input.count=3;input.identity.run_id=input.identity.map_version=input.identity.layout_id=input.identity.path_id=1;input.identity.start_id=1;
 c->body_length_mm=270;c->body_width_mm=250;c->heading_budget_deg=3;c->position_tolerance_mm=5;c->max_error_mm=40;
 c->stop_speed_mm_s=2;c->stop_omega_deg_s=.5f;c->acceleration_hint_mm_s2=200;c->cross_gain_s=.5f;c->lateral_cap_mm_s=20;
 c->preview_s=.1f;c->wheel_limit_rpm=80;c->heading_reserve_rpm=5;c->speed_state_margin_mm_s=2;c->max_age_ms=100;
 c->geometry.wheel_diameter_mm=77;c->geometry.track_width_mm=212;c->geometry.wheelbase_mm=206;
 input.points[0].x_mm=300;input.points[0].y_mm=300;input.points[1].x_mm=1200;input.points[1].y_mm=300;input.points[2].x_mm=1200;input.points[2].y_mm=1200;
 input.speed_mm_s[0]=200;input.speed_mm_s[1]=80;
 memset(&obs,0,sizeof(obs));obs.identity=input.identity;obs.position=input.points[0];obs.valid=1;obs.feedback_source=FUSION_ESTIMATED_FEEDBACK;obs.stopped=1;obs.sample_ms=obs.now_ms=1000;
 FusionReference_Reset(&state);
}
static uint32_t step(void){return FusionReference_Step(&plan,&state,&obs,&output);}
static void tick(void){obs.sample_ms+=10;obs.now_ms=obs.sample_ms;}
static uint32_t zero(void){return output.world_vx_mm_s==0 && output.world_vy_mm_s==0 && output.body.vx_mm_s==0 && output.body.vy_mm_s==0 && output.wheels.rpm[0]==0&&output.wheels.rpm[1]==0&&output.wheels.rpm[2]==0&&output.wheels.rpm[3]==0;}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t i,j,status;float nan_value;union{uint32_t u;float f;} bits;bits.u=0x7fc00000U;nan_value=bits.f;
 CK(size_plan>0&&size_state>0&&size_input>0);setup();input.config.start_heading_deg=0;input.config.goal_heading_deg=180;CK(FusionReference_Prepare(&plan,&input,&check));CK(plan.turn_direction==-1);CLOSE(plan.hold_heading_deg,-90,.001f);
 CK(step()==FUSION_TURN_START);CK(state.phase==0);CK(zero());
 obs.heading_deg=-90;tick();CK(step()==FUSION_TURN_START&&state.phase==1);old=output;saved=state;CK(step()==old.status);CK(memcmp(&output,&old,sizeof(old))==0&&memcmp(&state,&saved,sizeof(state))==0);
 tick();CK(step()==FUSION_FOLLOW);CK(output.world_vx_mm_s>0);CK(output.mixed==0);CLOSE(output.heading_target_deg,-90,.001f);
 old=output;saved=state;CK(step()==old.status&&memcmp(&output,&old,sizeof(old))==0);
 obs.position=input.points[1];obs.stopped=0;tick();CK(step()==FUSION_FOLLOW&&state.edge==0);CK(zero());
 obs.stopped=1;tick();CK(step()==FUSION_FOLLOW&&state.edge==1);CK(zero());saved=state;old=output;CK(step()==old.status&&memcmp(&output,&old,sizeof(old))==0&&memcmp(&state,&saved,sizeof(state))==0);
 obs.position=input.points[2];tick();CK(step()==FUSION_TURN_END&&state.phase==2);CLOSE(output.heading_target_deg,-180,.001f);old=output;CK(step()==FUSION_TURN_END&&memcmp(&output,&old,sizeof(old))==0);
 obs.heading_deg=-180;tick();CK(step()==FUSION_DONE&&state.phase==3);CK(zero());old=output;CK(step()==FUSION_DONE&&memcmp(&output,&old,sizeof(old))==0);
 CK(output.model_only==1&&!output.deployment_allowed&&!output.calibration_verified);
 /* Each bad fresh observation leaves state unchanged and zeroes old desired values. */
 setup();CK(FusionReference_Prepare(&plan,&input,&check));CK(step()==FUSION_TURN_START);tick();CK(step()==FUSION_FOLLOW);saved=state;bad=obs;
 for(i=0;i<16;i++){
  obs=bad;tick();
  switch(i){case 0:obs.identity.run_id++;break;case 1:obs.identity.start_id=2;break;case 2:obs.identity.map_version++;break;case 3:obs.identity.layout_id++;break;case 4:obs.identity.path_id++;break;case 5:obs.identity.direction=1;break;case 6:obs.valid=0;break;case 7:obs.continuity_lost=1;break;case 8:obs.reasons=1;break;case 9:obs.feedback_source=3;break;case 10:obs.now_ms=obs.sample_ms+101;break;case 11:obs.sample_ms=obs.now_ms+1;break;case 12:obs.position.x_mm=nan_value;break;case 13:obs.heading_deg=nan_value;break;case 14:obs.vx_mm_s=nan_value;break;default:obs.stopped=2;}
  memset(&output,0x5a,sizeof(output));CK(step()==FUSION_INVALID);CK(zero());CK(memcmp(&state,&saved,sizeof(saved))==0);
 }
 obs=bad;tick();CK(step()==FUSION_FOLLOW);CK(output.world_vx_mm_s>0);
 /* Same timestamp but conflicting pose/velocity is invalid; cached stale also invalid. */
 saved=state;bad=obs;obs.position.x_mm+=1;CK(step()==FUSION_INVALID&&zero());CK(memcmp(&state,&saved,sizeof(saved))==0);obs=bad;obs.now_ms+=101;CK(step()==FUSION_INVALID&&zero());obs=bad;CK(step()==FUSION_FOLLOW);
 /* Signed progress, backwards motion not masked, terminal side/overshoot/no-stop never DONE. */
 obs.position.x_mm=280;tick();CK(step()==FUSION_FOLLOW);CLOSE(output.signed_s_mm,-20,.001f);CLOSE(output.remaining_mm,1820,.001f);
 obs.position=input.points[1];obs.vx_mm_s=400;obs.stopped=0;tick();CK(step()==FUSION_RECOVERY&&zero());CK(output.reason==FUSION_SPEED_STATE);
 obs.vx_mm_s=0;obs.stopped=1;tick();CK(step()==FUSION_FOLLOW&&state.edge==1);
 obs.position=input.points[2];obs.position.x_mm+=30;obs.stopped=0;tick();CK(step()!=FUSION_DONE&&state.phase==1);
 obs.position=input.points[2];obs.position.y_mm+=30;tick();CK(step()==FUSION_RECOVERY&&zero());CLOSE(output.endpoint_residual_mm,-30,.001f);
 obs.position=input.points[2];tick();CK(step()==FUSION_FOLLOW&&state.phase==1);
 obs.stopped=1;obs.vy_mm_s=40;tick();CK(step()==FUSION_RECOVERY&&zero());
 /* Fresh actual lateral feedback at q=L remains a bounded sideways correction. */
 setup();CK(FusionReference_Prepare(&plan,&input,&check));step();obs.position=input.points[1];tick();CK(step()==FUSION_FOLLOW&&state.edge==1);
 obs.position=input.points[2];obs.position.x_mm+=30;obs.stopped=0;tick();CK(step()==FUSION_FOLLOW&&output.world_vx_mm_s<0);
 for(i=0;i<3;i++){obs.position.x_mm=i==0?1228:(i==1?1220:1210);obs.vx_mm_s=-15;tick();CK(step()==FUSION_FOLLOW&&state.phase==1&&output.world_vx_mm_s<0);}
 saved=state;obs.vx_mm_s=0;obs.vy_mm_s=400;tick();CK(step()==FUSION_RECOVERY&&output.reason==FUSION_SPEED_STATE&&memcmp(&state,&saved,sizeof(saved))==0);
 obs.vx_mm_s=-100;obs.vy_mm_s=0;tick();CK(step()==FUSION_RECOVERY&&output.reason==FUSION_SPEED_STATE);
 /* High speed close to a stop requests recovery, never fabricated instantaneous braking. */
 setup();CK(FusionReference_Prepare(&plan,&input,&check));step();obs.position.x_mm=1190;obs.vx_mm_s=400;obs.stopped=0;tick();CK(step()==FUSION_RECOVERY&&output.reason==FUSION_SPEED_STATE);
 /* Bound cursor across adjacent collinear short nodes, stop at corner; no nearest jump. */
 setup();input.count=5;input.points[1].x_mm=310;input.points[2].x_mm=320;input.points[2].y_mm=300;input.points[3].x_mm=330;input.points[3].y_mm=300;input.points[4].x_mm=330;input.points[4].y_mm=500;
 for(i=0;i<4;i++)input.speed_mm_s[i]=100;CK(FusionReference_Prepare(&plan,&input,&check));step();obs.position.x_mm=327;obs.stopped=0;tick();CK(step()==FUSION_FOLLOW&&state.edge==2);CK(output.world_vx_mm_s>0);
 saved=state;obs.position.x_mm=337;tick();CK(step()==FUSION_RECOVERY&&state.edge==2&&memcmp(&state,&saved,sizeof(state))==0);
 setup();input.points[1].x_mm=600;input.points[2].x_mm=900;input.points[2].y_mm=300;input.speed_mm_s[1]=200;CK(FusionReference_Prepare(&plan,&input,&check));step();obs.stopped=0;
 for(i=0;i<3;i++){obs.position.x_mm=i==0?597:(i==1?607:620);tick();CK(step()==FUSION_FOLLOW);CK(output.world_vx_mm_s>0);}CK(state.edge==1);
 /* Backward cap propagates across multiple collinear nodes. */
 setup();input.count=6;for(i=0;i<6;i++){input.points[i].x_mm=300+100*i;input.points[i].y_mm=300;if(i<5)input.speed_mm_s[i]=i==4?80:400;}
 CK(FusionReference_Prepare(&plan,&input,&check));CK(plan.node_cap[1]<400);CLOSE(plan.node_cap[1],sqrtf(80*80+2*200*300),.001f);
 /* Full rectangle sweep / built-in yellow, bounds and actual correction clearance. */
 setup();input.count=2;input.config.heading_budget_deg=.01f;input.points[0].y_mm=input.points[1].y_mm=410;CK(FusionReference_Prepare(&plan,&input,&check));obs.position=input.points[0];step();obs.position.x_mm=700;obs.position.y_mm=430;tick();CK(step()==FUSION_RECOVERY&&output.reason==FUSION_GEOMETRY&&zero());
 obs.position.y_mm=390;tick();CK(step()==FUSION_FOLLOW&&output.world_vy_mm_s>0);
 setup();input.count=2;input.points[0].y_mm=input.points[1].y_mm=775;CK(!FusionReference_Prepare(&plan,&input,&check)&&check.reason==FUSION_GEOMETRY);
 setup();input.count=2;input.points[0].x_mm=100;CK(!FusionReference_Prepare(&plan,&input,&check));
 setup();input.count=2;input.box_count=1;input.boxes[0].min_x=700;input.boxes[0].max_x=750;input.boxes[0].min_y=250;input.boxes[0].max_y=350;CK(!FusionReference_Prepare(&plan,&input,&check));
 /* Legal fixed-heading diagonal, conservative unsupported turn near a boundary. */
 setup();input.count=2;input.points[1].x_mm=input.points[1].y_mm=400;CK(FusionReference_Prepare(&plan,&input,&check));
 setup();input.count=2;input.points[0].x_mm=2250;input.points[0].y_mm=150;input.points[1].x_mm=2100;input.points[1].y_mm=150;input.config.heading_budget_deg=.01f;CK(FusionReference_Prepare(&plan,&input,&check));input.config.goal_heading_deg=90;CK(!FusionReference_Prepare(&plan,&input,&check)&&check.reason==FUSION_GEOMETRY);
 /* Invalid plan/model and all dangerous numeric domains fail without output reuse. */
 for(i=0;i<14;i++){
  setup();backup=input;
  switch(i){case 0:input.count=33;break;case 1:input.box_count=33;break;case 2:input.model_only=0;break;case 3:input.identity.run_id=0;break;case 4:input.points[1]=input.points[0];break;case 5:input.speed_mm_s[0]=401;break;case 6:input.speed_mm_s[0]=nan_value;break;case 7:input.config.goal_heading_deg=37;break;case 8:input.config.body_length_mm=nan_value;break;case 9:input.config.heading_reserve_rpm=80;break;case 10:input.config.acceleration_hint_mm_s2=0;break;case 11:input.config.geometry.wheel_diameter_mm=0;break;case 12:input.config.max_age_ms=0;break;default:input.points[0].x_mm=nan_value;}
  CK(!FusionReference_Prepare(&plan,&input,&check));memset(&output,0x5a,sizeof(output));CK(step()==FUSION_INVALID&&zero());
 }
 /* Time wrap and different plan cannot reuse state. */
 setup();CK(FusionReference_Prepare(&plan,&input,&check));obs.sample_ms=obs.now_ms=0xfffffff0U;CK(step()==FUSION_TURN_START);obs.sample_ms=obs.now_ms=4;CK(step()==FUSION_FOLLOW);saved=state;input.identity.path_id++;CK(FusionReference_Prepare(&plan,&input,&check));obs.identity=input.identity;tick();CK(step()==FUSION_INVALID&&memcmp(&state,&saved,sizeof(state))==0);
"""
rows=list(csv.reader((E/'budget_vectors.csv').open()))
for row in rows:
 v=[float(x) for x in row];lit=lambda x:format(x,'.10g')+'f' if ('.' in format(x,'.10g') or 'e' in format(x,'.10g')) else format(x,'.10g')+'.0f'
 h,wx,wy,reserve,bx,by,sx,sy,*rest=v;wheels=rest[:4];scale=rest[4]
 unit+='\n setup();input.count=2;input.config.body_length_mm=input.config.body_width_mm=10;input.config.acceleration_hint_mm_s2=1000;'
 unit+='input.config.start_heading_deg=input.config.goal_heading_deg='+lit(h)+';input.config.heading_reserve_rpm='+lit(reserve)+';obs.heading_deg='+lit(h)+';'
 unit+='input.points[1].x_mm=300+'+lit(wx)+';input.points[1].y_mm=300+'+lit(wy)+';input.speed_mm_s[0]='+lit((wx*wx+wy*wy)**.5)+';CK(FusionReference_Prepare(&plan,&input,&check));step();tick();CK(step()==FUSION_FOLLOW);'
 for field,val in [('body.vx_mm_s',bx),('body.vy_mm_s',by),('world_vx_mm_s',sx),('world_vy_mm_s',sy),('wheel_scale',scale)]+[('wheels.rpm['+str(j)+']',wheels[j]) for j in range(4)]:unit+='CLOSE(output.'+field+','+lit(val)+',.003f);'
 unit+='CK(output.deployment_allowed==0&&output.calibration_verified==0);'
unit+='\n /* Independent opposite-sign terminal correction; along overspeed still rejected. */\n setup();CK(FusionReference_Prepare(&plan,&input,&check));step();obs.position=input.points[1];tick();CK(step()==FUSION_FOLLOW&&state.edge==1);\n obs.position.x_mm=1170;obs.position.y_mm=1200;obs.stopped=0;tick();CK(step()==FUSION_FOLLOW);CK(output.world_vx_mm_s>14&&output.world_vx_mm_s<16);\n obs.position.x_mm+=.15f;obs.vx_mm_s=15;tick();CK(step()==FUSION_FOLLOW&&output.world_vx_mm_s>0);CK(state.phase==1);\n saved=state;obs.vy_mm_s=400;tick();CK(step()==FUSION_RECOVERY&&output.reason==FUSION_SPEED_STATE&&zero());CK(memcmp(&state,&saved,sizeof(state))==0);\n /* Every fixed yellow block is present without caller boxes. */\n for(i=0;i<3;i++){setup();input.count=2;input.config.heading_budget_deg=.01f;input.points[0].y_mm=input.points[1].y_mm=(i==1?775:1625);if(i>0){input.points[0].x_mm=1100;input.points[1].x_mm=2300;}CK(!FusionReference_Prepare(&plan,&input,&check)&&check.reason==FUSION_GEOMETRY);}\n /* A future self-intersection never selects the globally nearest edge. */\n setup();input.count=6;input.points[3].x_mm=300;input.points[3].y_mm=1200;input.points[4]=input.points[0];input.points[5]=input.points[1];for(i=0;i<5;i++)input.speed_mm_s[i]=200;CK(FusionReference_Prepare(&plan,&input,&check));step();obs.position.x_mm=500;tick();CK(step()==FUSION_FOLLOW&&state.edge==0);saved=state;obs.position.x_mm=300;obs.position.y_mm=1200;tick();CK(step()==FUSION_RECOVERY&&memcmp(&state,&saved,sizeof(state))==0);\n'
unit+='\n return 0;\n}\n'
(T/'unit.c').write_bytes(unit.encode('ascii'))
control=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
opts=['--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','--diag_suppress=177,550','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0','-DAPP_VM_CALIBRATION_ONLY_ENABLE=0','-DAPP_VM_ONLY_ENABLE=0','-DAPP_WHEEL_ENABLE_HOLD_ENABLE=0','-DAPP_BASELINE40_ENABLE=0']
opts+=['-I',short(E)]
opts+=['-D'+d for d in control.findtext('Define').split(',')]
for inc in control.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/inc).resolve())]
def run(cmd,log):
 cp=subprocess.run(cmd,cwd=T,capture_output=True,timeout=90);(T/log).write_bytes(cp.stdout+cp.stderr);assert cp.returncode==0,(cp.stdout+cp.stderr).decode('gbk',errors='replace')
objects=[]
for p in [T/'unit.c',E/'fusion_reference.c',U/'chassis_kinematics.c']:
 obj=p.stem+'.o';objects.append(obj);run([str(B/'armcc.exe'),*opts,'-c',short(p),'-o',obj],p.stem+'_compile.log')
(T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x50000 { *(+RW,+ZI) } }\n')
run([str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',*objects,'--library_type=microlib','-o','unit.axf'],'link.log')
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
from unicorn import arm_const as reg
data=(T/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',data);machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
for addr,size in [(0x08000000,0x100000),(0x20000000,0x50000),(0xE0000000,0x100000)]:machine.mem_map(addr,size)
for n in range(elf[12]):
 _,kind,flags,addr,offset,size,*_=struct.unpack_from('<IIIIIIIIII',data,elf[6]+n*elf[11])
 if flags&2 and size:machine.mem_write(addr,b'\0'*size if kind==8 else data[offset:offset+size])
stop=0x080ff000;machine.reg_write(reg.UC_ARM_REG_SP,0x2004fff0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1);machine.emu_start(elf[4]|1,stop,timeout=60000000,count=200000000)
assert machine.reg_read(reg.UC_ARM_REG_PC)==stop
result=machine.reg_read(reg.UC_ARM_REG_R0);symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=T).decode(errors='replace')
def symbol_value(name):
 address=int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16);return struct.unpack('<I',machine.mem_read(address,4))[0]
report={'status':'PASS' if not result else 'FAIL','failed_line':result,'assertions':symbol_value('review_assertions'),'budget_vectors':len(rows),'hardware_accessed':False,'actual_product_sources':True,'execution':'ARMCC Cortex-M3 soft-float machine code in Unicorn; no actuator/RTOS providers','sizes':{x:symbol_value('size_'+x) for x in ['plan','state','input']},'source_sha256':{n:hashlib.sha256(((E if n.startswith('fusion_reference.') else U)/n).read_bytes()).hexdigest() for n in ['fusion_reference.c','fusion_reference.h','chassis_kinematics.c']}}
(E/'actual_c_checks.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))
if result:print(unit.splitlines()[result-1]);raise SystemExit(result)
