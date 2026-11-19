"""Exercise actual view-entry owner with simulated scanner and IT feedback."""
from pathlib import Path
import json,re,subprocess
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-11/source-view-entry'
P=R/'tmp/source_view_entry_20261111/project'
assert json.loads((E/'matlab-result.json').read_text())['passed']
inc=P/'CHASSIS/USER/source_view_entry.inc';code=inc.read_text(encoding='utf-8')
names=sorted(set(re.findall(r'\bg_\w+',code)))
floats={'g_it_dm_deg','g_it_extension_mm','g_startup_extension_current_deg','g_extension_home_reference_deg'}
names=[s for s in names if not s.startswith('g_source_view_')]
unit='#include <stdint.h>\n#include <string.h>\n#include <math.h>\n#include <stdio.h>\n#include <stdlib.h>\n'
unit+='\n'.join(('float ' if n in floats else 'uint32_t ')+n+';' for n in names)
unit+=r'''
char SAOMA_Data[16];
static int healthy=1,top=1,refs=1,armbusy=0,fastbusy=0,sourcebusy=0,scanenabled=0;
#define SPF_DM 1U
#define SPF_EXT 2U
#define VEHICLE_STAGE1_SCAN_FRESH_MS 1000U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static int it_platform(void){return healthy;}
static int it_at_top(void){return top;}
static int SingleTeach_ReferencesReady(void){return refs;}
static int VehicleArm_Busy(void){return armbusy;}
static int Stage2Fast_IsActive(void){return fastbusy;}
static int SourceStage2_IsActive(void){return sourcebusy;}
static void SAOMA_EnableReceive(void){scanenabled=1;}
static void SAOMA_DisableReceive(void){scanenabled=0;}
static int FieldMap_TaskCodeValid(const char *s){return !strcmp(s,"146+123+641+231");}
#include "source_view_entry.inc"
#define CK(x) do{if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
int main(void){
 g_it_state=2;g_it_auto_state=2;g_saoma_accept_count=3;
 g_source_view_request=2;SourceView_Service(10);CK(!g_it_request && !g_source_view_state);
 g_source_view_request=1;SourceView_Service(100);CK(g_source_view_state==1 && scanenabled);
 /* Old scanner frame and early arrival never move. */
 strcpy(SAOMA_Data,"146+123+641+231");g_saoma_last_result=1;
 g_saoma_frame_start_ms=90;g_saoma_frame_end_ms=95;g_saoma_accept_count=4;
 g_source_view_request=2;SourceView_Service(110);CK(g_source_view_state==1 && !g_it_request);
 g_saoma_frame_start_ms=120;g_saoma_frame_end_ms=130;
 SourceView_Service(140);CK(g_source_view_state==2 && !scanenabled && !g_it_request);
 g_source_view_request=2;SourceView_Service(150);
 CK(g_source_view_state==3 && g_it_request==3 && g_it_axes==SPF_DM && g_it_dm_deg==180);
 /* A stale IT DONE cannot advance this transaction. */
 g_it_request=0;SourceView_Service(160);CK(g_source_view_state==3);
 /* The owner's own in-progress arm move must remain allowed. */
 armbusy=1;g_it_state=1;SourceView_Service(170);CK(g_source_view_state==3);
 armbusy=0;g_it_sequence++;g_it_state=2;g_it_action=3;SourceView_Service(180);
 CK(g_source_view_state==4 && g_it_request==3 && g_it_axes==SPF_EXT && g_it_extension_mm==65);
 g_it_request=0;SourceView_Service(190);CK(g_source_view_state==4);
 g_it_sequence++;SourceView_Service(200);CK(g_source_view_state==5 && g_vis_command_request==5);
 g_vis_center_cycle++;g_vis_center_valid=1;g_vis_center_tick=199;
 SourceView_Service(210);CK(g_source_view_state==5);
 g_vis_center_tick=211;SourceView_Service(220);CK(g_source_view_state==6);
 g_source_view_request=9;SourceView_Service(230);CK(g_source_view_state==9 && g_it_request==9);
 g_it_request=0;g_source_view_request=1;healthy=0;SourceView_Service(240);
 CK(g_source_view_state==9 && !g_it_request);
 puts("PASS actual view-entry: fresh real-scan window, explicit arrival, ordered180/65, fresh IT/center, stop and no body/ledger writes; simulated backends.");
 return 0;
}
'''
owner=re.sub(r'^#include "(?:saoma|field_map|vehicle_tuning)\.h"\n','',code,flags=re.M)
unit=unit.replace('#include "source_view_entry.inc"',owner)
src=E/'view-owner.c';exe=E/'view-owner.exe';src.write_text(unit,encoding='utf-8')
p=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(inc.parent),str(src),'-o',str(exe)],capture_output=True,text=True)
log=p.stdout+p.stderr
if p.returncode==0:
    p=subprocess.run([str(exe)],capture_output=True,text=True,timeout=10);log+=p.stdout+p.stderr
(E/'owner-check.log').write_text(log,encoding='utf-8');print(log);assert p.returncode==0
(E/'checks.json').write_text(json.dumps(dict(passed=True,hardware_access=False,actual_owner=True,simulated_backends=True),indent=2)+'\n')
