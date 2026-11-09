from pathlib import Path
import shutil,json,hashlib,subprocess,re,sys
R=Path(__file__).resolve().parents[1]; A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程'); S=R/'tmp/carrier_boot_zero_20261101/project'; E=R/'evidence/p2-arm/2026-11-01/carrier-boot-zero'; E.mkdir(parents=True,exist_ok=True)
F=['HARDWARE/CARRIER/carrier.c','HARDWARE/CARRIER/carrier.h','HARDWARE/CARRIER/carrier_config.h','USER/main.c']
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

assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','Listings','*.uvguix.*','.git'))
m={'before':{f:sha(A/f) for f in F},'protected':{str(p.relative_to(A)).replace(chr(92),'/'):sha(p) for p in A.rglob('*') if p.is_file() and p.suffix.lower() in ['.c','.h','.uvprojx'] and str(p.relative_to(A)).replace(chr(92),'/') not in F},'host_pass':False,'stage_build_pass':False,'installed':False,'actual_build_pass':False,'flash_performed':False,'physical_boot_zero_accepted':False}
for f in F:
 b=E/'original'/f;b.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(A/f,b)
def once(t,a,b):assert t.count(a)==1,a[:100];return t.replace(a,b,1)
def edit(f,fn):
 p=S/f;t=p.read_bytes().decode('utf-8').replace('\r\n','\n');p.write_bytes(fn(t).replace('\n','\r\n').encode('utf-8'))
def carrier(t):
 t=once(t,'static uint32_t s_session = 0U, s_started = 0U, s_last_service = 0U;','static uint32_t s_session = 0U, s_started = 0U, s_last_service = 0U;\nstatic uint32_t s_boot = 0U, s_boot_timed = 0U;')
 t=once(t,'    g_carrier_init_result = CarrierPwm_Init();',"""    g_carrier_init_result = CarrierPwm_Init();
    /* 用户要求：初始化后立即发0度，不依赖VM。启动阶段占用机构互锁。 */
    if (g_carrier_init_result != 1U) { g_carrier_state = 9U; g_carrier_result = 4U; return; }
#if CARRIER_BOOT_ZERO_ENABLE
    if (CarrierPwm_Write(CARRIER_PULSE_MIN_US) != 1U)
    { CarrierPwm_Off(); g_carrier_state = 9U; g_carrier_result = 6U; return; }
    g_carrier_debug_angle = CARRIER_SLOT_1_DEG;
    g_carrier_latched_angle = CARRIER_SLOT_1_DEG;
    g_carrier_pulse_us = CARRIER_PULSE_MIN_US;
    g_carrier_signal_active = 1U;
    g_carrier_state = 1U;
    s_session = 1U;
    s_boot = 1U;
#endif""")
 t=once(t,'    CarrierPwm_Off();\n    g_carrier_signal_active = 0U;','    CarrierPwm_Off();\n    g_carrier_signal_active = 0U;\n    s_boot = 0U;')
 t=once(t,'    if (s_session)\n',"""    /* 启动0度独立于手动VM准入；窗口后继续PWM保持，不冒充到位反馈。 */
    if (s_boot)
    {
        if (other_busy != 0U) { Carrier_Fail(5U); return; }
        if (!s_boot_timed) { s_started = now_ms; s_last_service = now_ms; s_boot_timed = 1U; }
        if (now_ms - s_last_service > CARRIER_SERVICE_GAP_MS)
        { Carrier_Fail(8U); return; }
        s_last_service = now_ms;
        g_carrier_elapsed_ms = now_ms - s_started;
        if (request == 1U) g_carrier_rejected_count++;
        if (g_carrier_elapsed_ms >= CARRIER_TEST_WINDOW_MS)
        {
            s_boot = 0U;
            s_session = 0U;
            g_carrier_state = 4U;
            g_carrier_result = 1U;
        }
        return;
    }
    if (s_session)
""")
 return t
edit(F[0],carrier)
edit(F[1],lambda t:t.replace('/* 0空闲，1输出窗口，2窗口结束待人工确认，3信号撤销，9失败。 */','/* 0空闲，1输出窗口，2手动窗口结束待确认，3撤销，4启动0度保持，9失败。 */'))
edit(F[2],lambda t:once(t,'/* 只有显式enable/request才输出；首次0度摆位须在固定盘之前完成。 */','/* 用户要求上电直接输出0度，独立于VM；手动分度仍须显式准入。 */\n#define CARRIER_BOOT_ZERO_ENABLE 1U'))
p=S/F[3];t=p.read_bytes();t=once(t,b'/* PB1 carrier uses the same serialized arm scheduler; no boot motion. */',b'/* PB1 boot zero is independent of VM; manual indexing keeps arm guards. */')
t=once(t,b'    Carrier_Service(ready, other_busy,',b'    if (g_rtos_fault != 0U || g_safety_fault != 0U) g_carrier_request = 2U;\r\n    Carrier_Service(ready, other_busy,');p.write_bytes(t)
H=R/'tmp/carrier_boot_zero_20261101/host';H.mkdir(parents=True)

model=(R/'tmp/carrier_zero_slots_20261030/host-check/model.c').read_text(encoding='utf-8')
a=model.index('static void App_CarrierStep(void)');b=model.index('static void start(',a)
main=(S/'USER/main.c').read_bytes();x=main.index(b'static void App_CarrierStep(void)');y=main.index(b'static void App_CanTask(void *argument)',x)
wrapper=main[x:y].decode('ascii');model=model[:a]+wrapper+model[b:]
model=model.replace('static unsigned mode=1','unsigned g_rtos_fault,g_safety_fault;\nstatic unsigned mode=1')
model=once(model,' Carrier_Init();',""" if(is(s,"boot_write_fail"))write_ok=0;
 Carrier_Init();
 if(strncmp(s,"boot_",5)==0){
  if(is(s,"boot_write_fail")){assert(g_carrier_state==9&&!g_carrier_signal_active);puts(s);return 0;}
  assert(writes==1&&last_pulse==500&&g_carrier_signal_active&&Carrier_IsBusy());
  assert(g_carrier_test_enable==0&&g_carrier_request==0);
  now=is(s,"boot_wrap")?0xffffff00U:100U;
  unsigned begin=now; g_vm_test_request=0;g_lift_state=0;
  App_CarrierStep();assert(g_carrier_state==1&&Carrier_IsBusy());
  if(is(s,"boot_stop")){g_carrier_request=2;now+=10;App_CarrierStep();assert(g_carrier_state==3&&!Carrier_IsBusy()&&!g_carrier_signal_active);puts(s);return 0;}
  if(is(s,"boot_gap")){now+=501;App_CarrierStep();assert(g_carrier_state==9&&g_carrier_result==8&&Carrier_IsBusy()&&!g_carrier_signal_active);puts(s);return 0;}
  if(is(s,"boot_conflict")){g_dm_jog_request=4;now+=10;App_CarrierStep();assert(g_carrier_state==9&&g_carrier_result==5);puts(s);return 0;}
  if(is(s,"boot_fault")){g_safety_fault=1;now+=10;App_CarrierStep();assert(g_carrier_state==3&&!g_carrier_signal_active&&!Carrier_IsBusy());puts(s);return 0;}
  for(unsigned i=1;i<=15;i++){
   now=begin+i*100U;
   if(is(s,"boot_repeat"))g_carrier_request=1;
   App_CarrierStep();
  }
  assert(g_carrier_state==4&&g_carrier_result==1&&!Carrier_IsBusy()&&g_carrier_signal_active&&writes==1);
  now+=10000;App_CarrierStep();assert(writes==1&&g_carrier_signal_active);
  if(is(s,"boot_repeat"))assert(g_carrier_rejected_count==15);
  puts(s);return 0;
 }
 if(init_ok){
  assert(writes==1&&last_pulse==500&&Carrier_IsBusy());
  g_carrier_request=2;Carrier_Service(0,0,0);
  assert(!Carrier_IsBusy()&&!g_carrier_signal_active);
  writes=offs=0;
 }""")
(H/'model.c').write_text(model,encoding='utf-8')
z=subprocess.run(['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror','-I'+str(S/'HARDWARE/CARRIER'),str(H/'model.c'),str(S/'HARDWARE/CARRIER/carrier.c'),'-o',str(H/'check.exe')],capture_output=True,text=True);assert z.returncode==0,z.stderr
cases=['boot','disabled','init_fail','not_ready','nan','inf','low','high','min','max','write_fail','unknown','window','busy','no_repeat','post_window_pause','gap','power_loss','disable','conflict','abort','stop_idle','wrap','rearm','mode','safety','vm','identity','guard_starts','guard_stops']+['block_'+s for s in ['dm','lhome','ehome','emove','lmove','dmreq','lmreq','lhreq','emreq','ehreq','lfault','lhfault','efault','ehfault']]+['slot0','slot120','slot240','beyond240','wrap360']+['boot_'+s for s in ['vm_off','wrap','stop','gap','conflict','fault','repeat','write_fail']]
logs=[]
for case in cases:
 z=subprocess.run([str(H/'check.exe'),case],capture_output=True,text=True);assert z.returncode==0,(case,z.stdout,z.stderr);logs.append(z.stdout+z.stderr)
(E/'host-check.log').write_text('\n'.join(logs),encoding='utf-8');m['host_pass']=True;m['host_cases']=cases;m['after']={f:sha(S/f) for f in F};save(m)
build(S,'stage-build.log');m['stage_build_pass']=True;save(m);print(str(len(cases))+' cases passed; stage ready',flush=True)
