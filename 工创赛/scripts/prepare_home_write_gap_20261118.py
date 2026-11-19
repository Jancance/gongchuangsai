"""Isolate a 20ms post-ACK gap and read-only failure evidence."""
from pathlib import Path
import json,shutil,subprocess
from check_stage2_extension_mirror_20261117 import function
R=Path(__file__).resolve().parents[1];BASE=R/'tmp/home_readback250_20261118/project';P=R/'tmp/home_write_gap_20261118/project';E=R/'evidence/integration/2026-11-18/home-write-gap'
PROBE=r'''
typedef struct {uint32_t called,status_result,status_flags,read_result,read_valid,wait_ms;uint8_t after[17];} HomeAfterWriteProbe;
volatile HomeAfterWriteProbe g_home_after_write_probe;
static void HomeAfterWrite_ReadOnlyProbe(void)
{
 uint8_t q[2]={0x3aU,0x6bU},data[17]={0U};CanRxMsg rx;uint32_t r;
 if(g_home_direction_diag.frozen || !g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return;
 memset(&rx,0,sizeof(rx));g_home_after_write_probe.called=1U;
 r=StartupAxis_Exchange(q,2U,3U,&rx);g_home_after_write_probe.status_result=r;
 if(r==1U)g_home_after_write_probe.status_flags=rx.Data[1];
 r=StartupAxis_ReadHomeParameters(data);g_home_after_write_probe.read_result=r;g_home_after_write_probe.read_valid=(r==1U);
 memcpy((void*)g_home_after_write_probe.after,data,17U);
}
'''


def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed'];assert not P.exists()
 p=Path('CHASSIS/SYSTEM/CAN/can.c');b=(BASE/p).read_bytes();sig='uint32_t StartupAxis_SetHomeDirection('
 old=function(b.decode('gb18030'),sig);new=old
 marker='if(result!=1U)return HomeDirectionDiag_Return(result); /* Never retry an uncertain parameter write. */'
 assert new.count(marker)==1
 new=new.replace(marker,marker+'\r\n        vTaskDelay(pdMS_TO_TICKS(20U)); /* Yield after accepted configuration write. */\r\n        if(!g_home_direction_diag.frozen)g_home_after_write_probe.wait_ms=20U;')
 marker='HD_SET(after_valid,result==1U);if(result!=1U)return HomeDirectionDiag_Return(result);'
 assert new.count(marker)==1
 new=new.replace(marker,'HD_SET(after_valid,result==1U);if(result!=1U){HomeAfterWrite_ReadOnlyProbe();return HomeDirectionDiag_Return(result);}')
 # The original first failure remains the returned failure, even if a later
 # read-only diagnostic query succeeds. No parameter rewrite or 9A in probe.
 prefix=PROBE.replace('\n','\r\n')
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 (P/p).write_bytes(b.replace(old.encode('ascii'),(prefix+new).encode('ascii'),1))
 fixture=(E.parent.parent/'2026-11-17/home-direction-diag/actual-home-direction-instrumented.c').read_text(encoding='utf-8')
 original_fixture_helper=function(fixture,sig)
 fixture=fixture.replace(original_fixture_helper,prefix+new)
 fixture=fixture.replace('#define DIAG_RESET() memset((void*)&g_home_direction_diag,0,sizeof(g_home_direction_diag))','#define DIAG_RESET() do{memset((void*)&g_home_direction_diag,0,sizeof(g_home_direction_diag));memset((void*)&g_home_after_write_probe,0,sizeof(g_home_after_write_probe));}while(0)')
 # CAN event comparison now includes deliberate read-only diagnostics; keep
 # original safety/assertions and add direct checks for the new failure probe.
 needle='if(scenario==3)CHECK(g_home_direction_diag.stage==9 && !g_home_direction_diag.after_valid);'
 assert needle in fixture
 fixture=fixture.replace(needle,needle+'\n if(scenario==3)CHECK(g_home_after_write_probe.called && g_home_after_write_probe.read_valid && g_home_direction_diag.result==5 && !home_calls);\n if(scenario==0)CHECK(g_home_after_write_probe.wait_ms==20 && !g_home_after_write_probe.called);\n if(scenario==1 || scenario==4)CHECK(!g_home_after_write_probe.wait_ms && !g_home_after_write_probe.called);')
 src=E/'actual-write-gap.c';src.write_text(fixture,encoding='utf-8');exe=src.with_suffix('.exe')
 run=subprocess.run(['gcc','-std=c99','-O1','-Wall','-Wextra','-Wno-misleading-indentation','-I'+str(P/'USER'),str(src),'-lm','-o',str(exe)],capture_output=True,text=True);(E/'actual-write-gap-build.log').write_text(run.stdout+run.stderr);assert run.returncode==0,run.stderr
 run=subprocess.run([str(exe)],capture_output=True,text=True);(E/'actual-write-gap.log').write_text(run.stdout+run.stderr);assert run.returncode==0,run.stdout+run.stderr
 (E/'checks.json').write_text(json.dumps(dict(passed=True,actual_tests=run.stdout.splitlines()[-1],probe_read_only=True,probe_never_releases_original_failure=True,hardware_access=False),indent=2)+'\n')
 print(run.stdout.splitlines()[-1],flush=True)


if __name__=='__main__':main()
