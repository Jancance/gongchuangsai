"""Compare original/instrumented actual C helper; no hardware access."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
from prepare_home_direction_diag_20261117 import BASE, P, E
from check_stage2_extension_mirror_20261117 import function


def main():
    signature = 'uint32_t StartupAxis_SetHomeDirection('
    old = function((BASE/'CHASSIS/SYSTEM/CAN/can.c').read_text(encoding='gb18030'), signature)
    source = (P/'CHASSIS/SYSTEM/CAN/can.c').read_text(encoding='gb18030')
    new = function(source, signature)
    prefix = source[source.index('/* Passive sole-CAN-task evidence.'):source.index(signature)]
    fixture = (E.parent/'stage2-extension-mirror/actual-mirror-home.c').read_text(encoding='utf-8')
    assert fixture.count(old) == 1
    active = (P/'CHASSIS/USER/extension_home_only.inc').read_text(encoding='utf-8')
    assert active == (BASE/'CHASSIS/USER/extension_home_only.inc').read_text(encoding='utf-8')
    assert active.strip() in fixture
    fixture = fixture.replace('static unsigned checks,tick=', 'static unsigned no_mailbox,failed_tx,timeout_tx,drop_ack;\nstatic unsigned checks,tick=')
    fixture = fixture.replace('return 0;}\nstatic unsigned CAN_TransmitStatus', 'return framecount==no_mailbox?4:0;}\nstatic unsigned CAN_TransmitStatus')
    fixture = fixture.replace('pending=1;if(ack==2)', 'pending=!drop_ack;if(ack==2)')
    fixture = fixture.replace('(void)a;(void)b;return CAN_TxStatus_Ok;', '(void)a;(void)b;if(framecount==failed_tx)return CAN_TxStatus_Failed;if(framecount==timeout_tx)return 0;return CAN_TxStatus_Ok;')
    fixture = fixture.replace('memcpy(driver,p,17);', 'no_mailbox=failed_tx=timeout_tx=drop_ack=0;DIAG_RESET();memcpy(driver,p,17);')
    extra = r'''
 /* Same actual helper under every meaningful newly observed failure class. */
 for(i=0;i<33;i++){
  reset(i==1);memcpy(before,driver,17);
  g_startup_axes_state=2;g_startup_axes_vm_request=1;
  if(i==2)read_fail=1;
  if(i==3)read_fail=2;
  if(i==4)ack=0xe2;
  if(i==5)drop_ack=1;
  if(i>=6 && i<=8)no_mailbox=i-5;
  if(i>=9 && i<=11)failed_tx=i-8;
  if(i>=12 && i<=14)timeout_tx=i-11;
  if(i>=15 && i<=17)cancel_frame=i-14;
  if(i>=18 && i<=30)corrupt=i-18;
  if(i==31)before[4]^=1;
  if(i==32)g_safety_fault=1;
  r=StartupAxis_SetHomeDirection(before,after);
  printf("TRACE %u %u %u %u %u %u %u %u",i,r,framecount,reads,home_calls,abort_calls,stop_calls,g_startup_axes_state);
  for(j=0;j<17;j++)printf(" %u",driver[j]);
  for(j=0;j<framecount;j++){unsigned k;printf(" %u %u",frames[j].ExtId,frames[j].DLC);for(k=0;k<frames[j].DLC;k++)printf(" %u",frames[j].Data[k]);}
  puts("");
  DIAG_CHECK(i,r);
 }
 DIAG_FREEZE_TEST();
'''
    fixture = fixture.replace(' printf("PASS actual mirror helper', extra+'\n printf("PASS actual mirror helper')
    diag_macros = r'''
#define DIAG_RESET() memset((void*)&g_home_direction_diag,0,sizeof(g_home_direction_diag))
static void diag_check(unsigned scenario,unsigned result){
 CHECK(g_home_direction_diag.result==result);
 CHECK(g_home_direction_diag.frozen==(result!=1));
 if(result==1){CHECK(g_home_direction_diag.stage==11);CHECK(g_home_direction_diag.after_valid);}
 if(scenario==1)CHECK(!g_home_direction_diag.write_tx_submit_count && !g_home_direction_diag.ack_seen);
 if(scenario==2)CHECK(g_home_direction_diag.stage==3 && !g_home_direction_diag.fresh_valid);
 if(scenario==3)CHECK(g_home_direction_diag.stage==9 && !g_home_direction_diag.after_valid);
 if(scenario==4)CHECK(g_home_direction_diag.stage==7 && g_home_direction_diag.ack_seen && g_home_direction_diag.ack_dlc==3 && g_home_direction_diag.ack_data[1]==0xe2);
 if(scenario==5)CHECK(g_home_direction_diag.stage==7 && !g_home_direction_diag.ack_seen && result==5);
 if(scenario>=6 && scenario<=14)CHECK(g_home_direction_diag.stage==6 && result==4);
 if(scenario>=18 && scenario<=30)CHECK(g_home_direction_diag.stage==10 && g_home_direction_diag.after_valid && result==7);
 if(scenario==31)CHECK(g_home_direction_diag.stage==4 && result==7);
 if(scenario==32)CHECK(g_home_direction_diag.stage==2 && result==9);
}
#define DIAG_CHECK(s,r) diag_check(s,r)
#define DIAG_FREEZE_TEST() do{HomeDirectionDiag saved;uint8_t a[17],b[17];unsigned rr; \
 reset(0);ack=0xe2;CHECK(set()==6);memcpy(&saved,(const void*)&g_home_direction_diag,sizeof(saved)); \
 ack=2;pending=framecount=reads=0;memcpy(a,driver,17);g_startup_axes_state=2;g_startup_axes_vm_request=1; \
 rr=StartupAxis_SetHomeDirection(a,b);CHECK(rr==1 && framecount==3); \
 CHECK(!memcmp(&saved,(const void*)&g_home_direction_diag,sizeof(saved)));}while(0)
'''
    results = {}
    for name, helper, macros in (
        ('original', old, '#define DIAG_RESET() ((void)0)\n#define DIAG_CHECK(s,r) ((void)0)\n#define DIAG_FREEZE_TEST() ((void)0)\n'),
        ('instrumented', prefix+new, diag_macros),
    ):
        code = fixture.replace(old, helper)
        # Definitions must follow the helper and precede reset/main.
        code = code.replace('static void reset(unsigned direction)', macros+'\nstatic void reset(unsigned direction)')
        if name == 'instrumented':
            code = code.replace('static void diag_check(', 'static unsigned set(void);\nstatic void reset(unsigned direction);\nstatic void diag_check(', 1)
        path = E/('actual-home-direction-'+name+'.c')
        path.write_text(code, encoding='utf-8')
        exe = path.with_suffix('.exe')
        args = ['gcc','-std=c99','-O1','-Wall','-Wextra','-Wno-misleading-indentation','-I'+str(P/'USER'),str(path),'-lm','-o',str(exe)]
        build = subprocess.run(args,capture_output=True,text=True)
        (E/(path.stem+'-build.log')).write_text(build.stdout+build.stderr)
        assert build.returncode == 0, build.stderr
        run = subprocess.run([str(exe)],capture_output=True,text=True)
        (E/(path.stem+'.log')).write_text(run.stdout+run.stderr)
        assert run.returncode == 0, run.stdout+run.stderr
        results[name] = run.stdout
    a = [x for x in results['original'].splitlines() if x.startswith('TRACE ')]
    b = [x for x in results['instrumented'].splitlines() if x.startswith('TRACE ')]
    assert len(a)==33 and a==b, 'Instrumentation changed ordered CAN effects or exit values'
    report = dict(passed=True,differential_scenarios=len(a),actual_original_checks=int(re.search(r': (\d+) checks',results['original'])[1]),actual_instrumented_checks=int(re.search(r': (\d+) checks',results['instrumented'])[1]),first_failure_freeze_verified=True,original_stop_and_no9a_tests_preserved=True,hardware_access=False,helper_sha256=hashlib.sha256(new.encode()).hexdigest(),matlab=json.loads((E/'matlab-result.json').read_text()))
    (E/'checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':
    main()
