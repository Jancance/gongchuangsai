"""Actual scan/arrival choreography and correction planner, simulated events."""
from pathlib import Path
import json,subprocess
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-10/source-arrival-policy'
assert json.loads((E/'matlab-result.json').read_text())['passed']
source=r'''
#include <stdio.h>
#include <stdlib.h>
#include "source_arrival_policy.h"
#define CK(x) do{if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
int main(void){SourceArrivalPolicy p={0};SourceCorrectionStep s;
 CK(SourceArrival_Begin(&p,8,10));
 CK(!SourceArrival_Arrived(&p,8,1,1,1,20)); /* scan first */
 CK(!SourceArrival_Scanned(&p,7,1,1,20));CK(!SourceArrival_Scanned(&p,8,1,0,20));
 CK(SourceArrival_Scanned(&p,8,123,1,20));
 CK(!SourceArrival_Arrived(&p,8,0,1,1,30));CK(!SourceArrival_Arrived(&p,8,1,0,1,30));
 CK(SourceArrival_Arrived(&p,8,1,1,1,30) && p.state==SAP_PREPARE);
 CK(!SourceArrival_ViewReady(&p,8,1,0,40));CK(SourceArrival_ViewReady(&p,8,1,1,40));
 CK(!SourceArrival_Aligned(&p,8,0,1,50));CK(SourceArrival_Aligned(&p,8,1,1,50));
 CK(p.state==SAP_READY && p.qr_revision==123);
 s=SourceCorrection_Plan(100000,2000);CK(s.delta_milli_mm==40000 && s.speed_mm_s==60);
 s=SourceCorrection_Plan(-30000,2000);CK(s.delta_milli_mm==-20000 && s.speed_mm_s==35);
 s=SourceCorrection_Plan(15000,2000);CK(s.delta_milli_mm==10000 && s.speed_mm_s==15);
 s=SourceCorrection_Plan(5000,2000);CK(s.reason==2 && !s.delta_milli_mm);
 s=SourceCorrection_Plan(1999,2000);CK(s.reason==1);
 s=SourceCorrection_Plan(-2147483647-1,2000);CK(s.reason==3);
 puts("PASS: actual scan->explicit arrival->view180/65 completion->fresh alignment policy;40/20/10mm bands, signed limits andunsupported sub10mm. SIMULATED EVENTS.");return 0;}
'''
src=E/'policy-test.c';exe=E/'policy-test.exe';src.write_text(source)
p=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(R/'software_interfaces/source_station'),str(src),'-o',str(exe)],capture_output=True,text=True)
log=p.stdout+p.stderr
if p.returncode==0:
 p=subprocess.run([str(exe)],capture_output=True,text=True,timeout=10);log+=p.stdout+p.stderr
(E/'host-check.log').write_text(log);print(log);assert p.returncode==0
(E/'host-checks.json').write_text(json.dumps({'passed':True,'hardware_access':False,'result':log.strip()},indent=2)+'\n')
