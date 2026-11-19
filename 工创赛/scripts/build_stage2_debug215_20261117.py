"""Actual formal scanner and debug key/task/owner branches, then dual Keil."""
from pathlib import Path
import subprocess,shutil,json,hashlib,datetime
from prepare_stage2_debug215_20261117 import R,BASE,P,E
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def fixture():
 prior=R/'evidence/integration/2026-11-16/source-real-qr'
 for p in prior.iterdir():
  if p.suffix in ('.h','.inc','.c'):shutil.copyfile(p,E/p.name)
 shutil.copyfile(R/'evidence/integration/2026-11-17/stage2-sync-opt/all-regions-owner.c',E/'base-commit-fixture.c')
 (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(b'#include "low_readonly_diag.inc"',b'/* separate diagnostics */'))
 cycle=(P/'CHASSIS/USER/source_mcu_cycle.inc').read_text(encoding='utf-8')
 boundary=' if(state!=1U || !run || g_source_scan_owner_run!=run || g_source_scan_pending)return;'
 assert cycle.count(boundary)==1
 (E/'tested-source-cycle.inc').write_text(cycle.replace(boundary,' SC_TestPreempt(); /* fixture scheduler hook only */\n'+boundary),encoding='utf-8')
 (E/'tested-region-dispatch.inc').write_bytes((P/'CHASSIS/USER/region_dispatch.inc').read_bytes())
 s=(prior/'actual-real-qr.c').read_text(encoding='utf-8')
 s=s.replace('static uint32_t g_start_key_count,g_vehicle_key,key_event;',r'''
static uint32_t g_stage3_state,stage3_keys;
static void Stage3_Arrive(void){}
static void Stage3_KeyEvent(void){stage3_keys++;}
static void Stage3_Service(uint32_t t){(void)t;}
static uint32_t Stage3_SharedByte(uint8_t b,uint32_t n,uint32_t x){(void)b;(void)n;(void)x;return 0;}
static void Stage3_Telemetry(uint32_t n){(void)n;}
#include "tested-region-dispatch.inc"
static uint32_t g_start_key_count,g_vehicle_key,key_event;''')
 key=(prior/'tested-key-branch.inc').read_text(encoding='utf-8').replace('SourceCycle_KeyEvent();','RegionDispatch_KeyEvent();')
 main=(P/'CHASSIS/USER/main.c').read_text(encoding='gb18030')
 assert key in main
 (E/'tested-key-branch.inc').write_text(key,encoding='utf-8')
 pos=s.rindex('int main(void){');s=s[:pos]+s[pos:].replace('int main(void){','static int formal_qr_main(void){',1)
 s+=r'''
int main(void){
#if !SOURCE_CYCLE_TEST_215
 CHECK(g_region_dispatch_area==0 && !g_source_cycle_request);press_actual_key();CHECK(stage3_keys==1 && g_region_dispatch_area==3 && !g_source_cycle_request);
 return formal_qr_main();
#else
 unsigned i;static const unsigned requests[6]={1,2,7,6,3,3};
 CHECK(g_region_dispatch_area==1 && !g_source_cycle_request && !g_source_cycle_state);
 round_reset();g_region_dispatch_area=1;press_actual_key();CHECK(g_source_cycle_request==1 && !stage3_keys && !g_it_request);
 SourceCycle_Service(tick);CHECK(g_source_cycle_state==1 && g_it_request==1);
 CHECK(g_source_cycle_colors[0]==2 && g_source_cycle_colors[1]==1 && g_source_cycle_colors[2]==5);
 CHECK(!g_source_scan_pending && !g_source_scan_locked_run && !g_source_scan_owner_run && !g_vis_task_valid);
 g_source_scan_owner_run=77;g_source_scan_pending=33;g_source_scan_locked_run=88;SAOMA_DataReady=1;
 for(i=0;i<4;i++)SourceCycle_ScanService();
 CHECK(g_source_scan_owner_run==77 && g_source_scan_pending==33 && g_source_scan_locked_run==88 && SAOMA_DataReady==1 && !displays);
 CHECK(SC_ScanTask()==1 && g_source_scan_pending==33 && g_source_scan_locked_run==88 && !g_vis_task_valid);
 for(i=0;i<6;i++){
  CHECK(g_it_request==requests[i]);g_it_request=0;g_it_state=2;g_it_action=requests[i];SourceCycle_Service(tick);
 }
 CHECK(g_source_cycle_state==2 && g_source_cycle_prep==6 && !g_it_request);
 CHECK(g_source_cycle_colors[0]==2 && g_source_cycle_colors[1]==1 && g_source_cycle_colors[2]==5);
 /* Live observed region colors remain incoming OC fields, not fixed task order. */
 g_source_cycle_state=6;sample(g_source_cycle_pose,1,0,0,0,1,5,2,1,0);
 CHECK(sc_colors[0]==5 && sc_colors[1]==2 && sc_colors[2]==1);
 sc_valid=0;SourceCycle_Service(tick);CHECK(g_source_cycle_state==6 && !g_low_request && !g_source_cycle_scheduled);
 press_actual_key();CHECK(g_source_cycle_request==9);SourceCycle_Service(tick);CHECK(g_source_cycle_state==9 && g_source_cycle_fault==2);
 CHECK(g_source_scan_pending==33 && g_source_scan_locked_run==88 && !g_vis_task_valid && !displays);
 printf("PASS actual debug215: %u checks; PC13 existing dispatcher, prep6, real task215, untouched scan/fullQR, OC region observations, freshness and STOP\n",checks);return 0;
#endif
}
'''
 return s
def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed']
 s=fixture();results={}
 for mode in [0,1]:
  name='actual-formal-qr' if mode==0 else 'actual-debug215';path=E/(name+'.c');path.write_text('#define SOURCE_CYCLE_TEST_215 '+str(mode)+'\n'+s,encoding='utf-8');exe=E/(name+'.exe')
  cmd=['gcc','-std=c99','-Wall','-Wextra','-Wno-unused-function','-Wno-misleading-indentation','-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(path),str(P/'CHASSIS/USER/source_parallel_flow.c'),'-lm','-o',str(exe)]
  subprocess.run(cmd,check=True);r=subprocess.run([str(exe)],capture_output=True,text=True);(E/(name+'.log')).write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True);assert r.returncode==0;results[name]=r.stdout.strip()
 changes={};manifest={};unchanged={}
 for path in P.rglob('*'):
  if not path.is_file() or any(x in ('Objects','Listings') for x in path.relative_to(P).parts):continue
  n=path.relative_to(P).as_posix();manifest[n]=sha(path)
  if path.read_bytes()!=(BASE/n).read_bytes():changes[n]=sha(path)
 assert set(changes)=={'CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/region_dispatch.inc','USER/vehicle_parameters.h','gongxun-f407.uvprojx'}
 for n in ['CHASSIS/USER/pzq_commit_overlap.inc','CHASSIS/USER/vehicle_source_parallel.inc','HARDWARE/DM_CAN2/dm_can2_motion.c','CHASSIS/USER/stage3_cycle.inc','CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/lift_ruler_calibration.c','CHASSIS/USER/main.c','CHASSIS/HARDWARE/SAOMA/saoma.c']:
  assert (P/n).read_bytes()==(BASE/n).read_bytes();unchanged[n]=sha(P/n)
 c=json.loads((R/'evidence/integration/2026-11-17/stage2-sync-opt/candidate.json').read_text());images={}
 for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
  for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
  log=E/(target+'-build.log');r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=240);data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-600:],flush=True)
  assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
  axf=P/'Objects'/target/(target+'.axf');images[target]={'axf':str(axf),'sha256':sha(axf)}
 c.update(base=str(BASE),project=str(P),changes=changes,images=images,unchanged_sha256=unchanged,matlab_latest=proof,matlab_timeline={'before_debug215_implementation':proof['completed_local']},source_rebuild_byte_identical=False,
  source_cycle_test_215=True,source_cycle_task_source='explicit debug215, not scanned QR',debug_task_colors=[2,1,5],default_region_entry='area1; physical PC13 existing SourceCycle_KeyEvent required',actual_c=results,build_passed=True,flashed=False,hardware_access=False,completed_local=datetime.datetime.now().isoformat())
 c['review_passed']=False
 c['requirements']=list(dict.fromkeys(['QR-01','QR-05','QR-06','DSP-01']+c['requirements']))
 c['debug_not_formal_scan_acceptance']=True;c['debug_task_source_is_full_qr']=False
 (E/'candidate.json').write_text(json.dumps(c,indent=2)+'\n');(E/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(E/'checks.json').write_text(json.dumps({'passed':True,'actual_c':results,'dual_build':True,'hardware_access':False,'flashed':False},indent=2)+'\n')
if __name__=='__main__':main()
