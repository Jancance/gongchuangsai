from pathlib import Path
import shutil,json,hashlib,subprocess,re,sys
R=Path(__file__).resolve().parents[1]; A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程'); S=R/'tmp/dm_smooth_20261101/project'; E=R/'evidence/p2-arm/2026-11-01/dm-smooth'; E.mkdir(parents=True,exist_ok=True)
F=['HARDWARE/DM_CAN2/dm_can2.h','HARDWARE/DM_CAN2/dm_can2_protocol.c','HARDWARE/DM_CAN2/dm_can2_motion.c','USER/main.c']
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
assert S.parent.name == "dm_smooth_20261101"
shutil.copytree(A,S,dirs_exist_ok=True,ignore=shutil.ignore_patterns('Objects','Listings','*.uvguix.*','.git'))
m={'before':{f:sha(A/f) for f in F},'protected':{str(p.relative_to(A)).replace(chr(92),'/'):sha(p) for p in A.rglob('*') if p.is_file() and (p.suffix.lower() in ['.c','.h','.uvprojx']) and str(p.relative_to(A)).replace(chr(92),'/') not in F},'host_pass':False,'stage_build_pass':False,'installed':False,'actual_build_pass':False,'flash_performed':False,'physical_smoothness_accepted':False}
for f in F:
 b=E/'original'/f;b.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(A/f,b)
def edit(f,fn):
 p=S/f;t=p.read_bytes().decode('utf-8').replace('\r\n','\n');t=fn(t);p.write_bytes(t.replace('\n','\r\n').encode('utf-8'))
def once(t,a,b):assert t.count(a)==1,a[:100];return t.replace(a,b,1)
def header(t):
 t=once(t,'float DM_CAN2_SmoothStep(float ratio);',"""float DM_CAN2_SmoothStep(float ratio);
/* 五次位置曲线的一阶导数；单位为归一化时间的倒数。 */
float DM_CAN2_SmoothVelocity(float ratio);
uint32_t DM_CAN2_BuildMitTrajectory(float position, float velocity, float pmax,
                                   float vmax, float tmax, CanTxMsg *tx);""")
 return t
edit(F[0],header)
def proto(t):
 t=once(t,'uint32_t DM_CAN2_BuildMitHold(float position, float pmax, float vmax,\n                             float tmax, CanTxMsg *tx)','uint32_t DM_CAN2_BuildMitTrajectory(float position, float velocity, float pmax,\n                                   float vmax, float tmax, CanTxMsg *tx)')
 t=once(t,'!(position >= -pmax && position <= pmax))','!(position >= -pmax && position <= pmax) ||\n        !(velocity >= -0.5F && velocity <= 0.5F) ||\n        !(velocity >= -vmax && velocity <= vmax))')
 t=once(t,'v = 2047U;','v = (uint32_t)((velocity + vmax) * 4095.0F / (2.0F * vmax));')
 t=once(t,'/* v_des、t_ff均为0的量化近似，避免失联后维持非零速度参考。 */','/* 运动速度匹配位置曲线；静态/预置仍用零速度，力矩前馈保持零。 */')
 marker='float DM_CAN2_SmoothStep(float ratio)'
 t=once(t,marker,"""uint32_t DM_CAN2_BuildMitHold(float position, float pmax, float vmax,
                             float tmax, CanTxMsg *tx)
{
    return DM_CAN2_BuildMitTrajectory(position, 0.0F, pmax, vmax, tmax, tx);
}

float DM_CAN2_SmoothVelocity(float ratio)
{
    if (!(ratio > 0.0F && ratio < 1.0F)) return 0.0F;
    return 30.0F * ratio * ratio * (1.0F - ratio) * (1.0F - ratio);
}

"""+marker)
 return t
edit(F[1],proto)
def motion(t):
 t=once(t,'static void JogHold(float reference, TickType_t now)','static void JogReference(float reference, float velocity, TickType_t now)')
 t=once(t,'DM_CAN2_BuildMitHold(reference, s_pmax, s_vmax, s_tmax, &tx)','DM_CAN2_BuildMitTrajectory(reference, velocity, s_pmax, s_vmax, s_tmax, &tx)')
 t=t.replace('JogHold(s_target, now);','JogReference(s_target, 0.0F, now);')
 t=once(t,'else JogHold(s_start + (s_target - s_start) * DM_CAN2_SmoothStep(ratio), now);',"""else JogReference(s_start + (s_target - s_start) * DM_CAN2_SmoothStep(ratio),
                          (s_target - s_start) * DM_CAN2_SmoothVelocity(ratio) *
                          (1000.0F / (float)s_duration_ms), now);""")
 return t
edit(F[2],motion)
p=S/F[3]; data=p.read_bytes()
old=b'vTaskDelay(pdMS_TO_TICKS(10U));\r\n            continue;'
assert data.count(old)==1
new='/* 旋转时提高服务频率，其他机构保持原节拍。 */'.encode('gb18030')+b'\r\n            vTaskDelay(pdMS_TO_TICKS(DM_CAN2_JogIsBusy() ? 2U : 10U));\r\n            continue;'
p.write_bytes(data.replace(old,new,1))

H=R/'tmp/dm_smooth_20261101/host';shutil.copytree(R/'tmp/dm_can2_points_20261029/host-check',H,ignore=shutil.ignore_patterns('*.exe'))
p=H/'motion.c';t=p.read_text(encoding='utf-8');t=once(t,'assert((((unsigned)t->Data[2]<<4)|(t->Data[3]>>4))==2047);','assert((((unsigned)t->Data[2]<<4)|(t->Data[3]>>4))>=2012 && (((unsigned)t->Data[2]<<4)|(t->Data[3]>>4))<=2082);');p.write_text(t,encoding='utf-8')
mod=S/'HARDWARE/DM_CAN2';gcc='C:/mingw/mingw64/bin/gcc.exe';common=[gcc,'-std=c99','-Wall','-Wextra','-Werror','-I'+str(H),'-I'+str(mod)]
z=subprocess.run(common+[str(mod/'dm_can2_protocol.c'),str(mod/'dm_can2_motion.c'),str(H/'motion.c'),'-lm','-o',str(H/'motion.exe')],capture_output=True,text=True);assert z.returncode==0,z.stderr
cases=['positive','negative','stale','txfail','noreply','fault','disabled','tracking','limit','busy','abort','repeat','gap','badprofile','stoplost']+['point_'+n for n in ['pick','place','minus_turn','edit','gated','gate_drop','badtarget','nantarget','outside','wrong_direction','abort','repeat','stale','noreply','fault','gap','stoplost']]
logs=[]
for c in cases:
 z=subprocess.run([str(H/'motion.exe'),c],capture_output=True,text=True);logs.append(z.stdout+z.stderr);assert z.returncode==0,(c,z.stdout,z.stderr)
(E/'host-check.log').write_text('\n'.join(logs),encoding='utf-8')
m['host_pass']=True;m['motion_host_cases']=len(cases);m['after']={f:sha(S/f) for f in F};save(m)
build(S,'stage-build.log');m['stage_build_pass']=True;save(m)
print('Stage ready, '+str(len(m['protected']))+' protected source/project files; 32 synthetic cases, no physical acceptance.',flush=True)
