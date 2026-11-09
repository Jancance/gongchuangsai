from pathlib import Path
import shutil,json,hashlib,subprocess,re,sys
R=Path(__file__).resolve().parents[1]; A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程'); S=R/'tmp/stepper_reference_speed_20261101/project'; E=R/'evidence/p2-arm/2026-11-01/stepper-reference-speed'; E.mkdir(parents=True,exist_ok=True)
F=['HARDWARE/LIFT/lift_config.h','HARDWARE/EXTENSION/extension_config.h']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(m): (E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
def build(d,n):
 log=E/n
 z=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(d/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],capture_output=True,timeout=180)
 t=log.read_text(encoding='utf-8',errors='replace');assert z.returncode==0 and '0 Error(s), 0 Warning(s)' in t,t[-3000:]
 print(t[-450:],flush=True)
if '--install' in sys.argv:
 m=json.loads((E/'manifest.json').read_text(encoding='utf-8'));assert m['host_pass'] and m['stage_build_pass']
 for f in F:assert sha(A/f)==m['before'][f] and sha(S/f)==m['after'][f]
 for f,h in m['protected'].items():assert sha(A/f)==h,f
 for f in F:shutil.copy2(S/f,A/f)
 m['installed']=True;save(m);build(A,'actual-build.log')
 for f,h in m['protected'].items():assert sha(A/f)==h,f
 m['actual_build_pass']=True;save(m);print('Installed and built; no flash or hardware commands.');sys.exit()



assert S.parent.name == "stepper_reference_speed_20261101"
shutil.copytree(A,S,dirs_exist_ok=True,ignore=shutil.ignore_patterns('Objects','Listings','*.uvguix.*','.git'))
m={'before':{f:sha(A/f) for f in F},'protected':{str(p.relative_to(A)).replace(chr(92),'/'):sha(p) for p in A.rglob('*') if p.is_file() and p.suffix.lower() in ['.c','.h','.uvprojx'] and str(p.relative_to(A)).replace(chr(92),'/') not in F},'host_pass':False,'stage_build_pass':False,'installed':False,'actual_build_pass':False,'flash_performed':False,'physical_speed_accepted':False}
for f in F:
 b=E/'original'/f;b.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(A/f,b)
p=S/F[0];t=p.read_text(encoding='utf-8')
for old,new in [('#define LIFT_DOWN_JOG_RPM     300U','#define LIFT_DOWN_JOG_RPM     1846U'),('#define LIFT_JOG_RPM          3000U','#define LIFT_JOG_RPM          1846U'),('#define LIFT_JOG_ACCELERATION 250U','#define LIFT_JOG_ACCELERATION 255U')]:
 assert t.count(old)==1;t=t.replace(old,new,1)
t=t.replace('#define LIFT_DOWN_JOG_RPM','/* 参考X报文常用速度1846.4RPM，Emm以整数RPM明确设1846，不复制uint16溢出。 */\n#define LIFT_DOWN_JOG_RPM',1)
p.write_bytes(t.replace('\r\n','\n').replace('\n','\r\n').encode('utf-8'))
p=S/F[1];t=p.read_text(encoding='utf-8')
for old,new in [('#define EXTENSION_JOG_RPM           5U','#define EXTENSION_JOG_RPM           106U'),('#define EXTENSION_JOG_ACCELERATION  20U','#define EXTENSION_JOG_ACCELERATION  255U')]:
 assert t.count(old)==1;t=t.replace(old,new,1)
t=t.replace('/* 28伸缩轴首次方向试验；脉冲数不表示已标定的毫米行程。 */','/* 普通运动按参考X报文等效106RPM，适配本车Emm；回零参数不改。 */')
p.write_bytes(t.replace('\r\n','\n').replace('\n','\r\n').encode('utf-8'))
m['after']={f:sha(S/f) for f in F};save(m)
H=R/'tmp/stepper_reference_speed_20261101/host';H.mkdir(parents=True,exist_ok=True)
shutil.copytree(R/'tmp/extension_mm_entry_20261101/host-check',H,dirs_exist_ok=True,ignore=shutil.ignore_patterns('*.exe'))
model=r"""#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "lift_protocol.h"
#include "lift_config.h"
#include "extension.c"
int main(void){
 uint8_t out[12]; unsigned dir;uint32_t samples[]={1,5000,32000,50000};unsigned i;
 uint16_t reference35=(uint16_t)(2800U*30U);uint16_t reference28=(uint16_t)(2000*.53);
 assert(reference35==18464 && reference28==1060);
 assert(LIFT_JOG_RPM==1846&&LIFT_DOWN_JOG_RPM==1846&&LIFT_JOG_ACCELERATION==255);
 assert(EXTENSION_JOG_RPM==106&&EXTENSION_JOG_ACCELERATION==255);
 for(dir=0;dir<2;dir++)for(i=0;i<4;i++){
  assert(LiftProtocol_Move(dir,samples[i],out));
  assert(out[0]==0xFD&&out[1]==dir&&out[2]==0x07&&out[3]==0x36&&out[4]==255);
  assert(out[9]==2&&out[10]==0&&out[11]==0x6B);
  assert((((uint32_t)out[5]<<24)|((uint32_t)out[6]<<16)|((uint32_t)out[7]<<8)|out[8])==samples[i]);
 }
 for(dir=0;dir<2;dir++){
  s_extension_target_pulses=320;g_extension_motion_direction=dir;Extension_BuildMove(out);
  assert(out[0]==0xFD&&out[1]==dir&&out[2]==0&&out[3]==106&&out[4]==255);
  assert(out[7]==1&&out[8]==64&&out[9]==2&&out[10]==0&&out[11]==0x6B);
 }
 assert(!LiftProtocol_Move(2,100,out));assert(!LiftProtocol_Move(0,0,out));
 assert(!LiftProtocol_Move(0,50001,out));
 assert(LIFT_DOWN_MAX_PULSES==50000&&EXTENSION_PULSES_PER_MM==32.f&&EXTENSION_JOG_PULSES==1600);
 puts("PASS reference overflow/0.1RPM conversion;8 lift packets,2 extension packets,existing pulse boundaries and mm scale unchanged");return 0;
}
"""
source=(S/'HARDWARE/EXTENSION/extension.c').read_text(encoding='utf-8')
a=source.index('static void Extension_BuildMove(');b=source.index('void Extension_MotionService(',a)
builder=source[a:b]
model=model.replace('#include "extension.c"','#include "extension_config.h"\nstatic uint32_t s_extension_target_pulses;\nvolatile uint32_t g_extension_motion_direction;\n'+builder)
(H/'speed-check.c').write_text(model,encoding='utf-8')
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror','-ffunction-sections','-fdata-sections','-I'+str(H),'-I'+str(S/'HARDWARE/LIFT'),'-I'+str(S/'HARDWARE/EXTENSION'),str(S/'HARDWARE/LIFT/lift_protocol.c'),str(H/'speed-check.c'),'-Wl,--gc-sections','-lm','-o',str(H/'speed-check.exe')]
z=subprocess.run(cmd,capture_output=True,text=True);assert z.returncode==0,z.stderr
z=subprocess.run([str(H/'speed-check.exe')],capture_output=True,text=True,timeout=10);assert z.returncode==0,z.stderr;(E/'host-check.log').write_text(z.stdout,encoding='utf-8');m['host_pass']=True;save(m);print(z.stdout,flush=True)
build(S,'stage-build.log');m['stage_build_pass']=True;save(m);print('Two-config reference speed adaptation staged.',flush=True)
