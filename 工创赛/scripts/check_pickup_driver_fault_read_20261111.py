"""Exercise actual fault-idle read admission and bounded CAN reader; no devices."""
from pathlib import Path

R = Path(__file__).resolve().parents[1]
P = R / 'tmp/pickup_driver_fault_read_20261111/project'
U = P / 'CHASSIS/USER'
E = R / 'evidence/integration/2026-11-11/driver-fault-read'
startup = (U / 'startup_axes.c').read_text(encoding='utf-8')
helper = startup[startup.index('uint32_t StartupAxes_DiagnosticReadReady(void)'):]

source = (R / 'scripts/check_pickup_driver_fifo_fix_20261111.py').read_text(encoding='utf-8')
source = source.replace('tmp/pickup_driver_fifo_fix_20261111/project', 'tmp/pickup_driver_fault_read_20261111/project')
source = source.replace('2026-11-11/driver-fifo-fix', '2026-11-11/driver-fault-read')
source = source.replace('static uint32_t fifo_model,fifo_overruns;', 'static uint32_t fifo_model,fifo_overruns;\nstatic uint32_t eh_stop_pending,eh_started,powered=1;\n#define VEHICLE_EXTENSION_HOME_ONLY 1\n#define VEHICLE_STARTUP_GROUP_ENABLE 0\ntypedef struct {uint32_t active;} PowerCommandStatus;\nstatic void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=powered;}')
source = source.replace('g_startup_axes_vm_request=0;CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==9 && tx_calls==before);g_startup_axes_vm_request=1;', 'powered=0;CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==9 && tx_calls==before);powered=1;')
extra = r'''
 for(uint32_t state=0;state<10;state++)for(uint32_t pending=0;pending<2;pending++)for(uint32_t started=0;started<2;started++){
  g_startup_axes_state=state;eh_stop_pending=pending;eh_started=started;
  uint32_t ready=!pending && !started && (state==0 || state==4 || state==5 || state==6);
  CK(StartupAxes_DiagnosticReadReady()==ready);
  CK(g_startup_axes_state==state && eh_stop_pending==pending && eh_started==started);
 }
 eh_stop_pending=eh_started=0;g_startup_axes_vm_request=0;
 for(uint32_t state=0;state<=6;state++)if(state==0 || state>=4){
  g_startup_axes_state=state;uint32_t before=tx_calls;
  CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==1 && tx_calls==before+1 && n==18);
  CK(g_startup_axes_state==state && g_startup_axes_vm_request==0);
 }
 g_startup_axes_state=4;g_startup_axes_vm_request=1;
'''
marker = ' uint8_t commands[7]={0x1f,0x21,0x24,0x26,0x27,0x42,0x43};'
assert source.count(marker) == 1
source = source.replace(marker, extra + marker, 1)
source = source.replace("c.write_text(prefix+", "c.write_text(prefix+" + repr(helper) + "+", 1)
exec(compile(source, 'actual-fault-idle-reader', 'exec'), {'__file__': str(__file__), '__name__': '__main__'})

source = (R / 'scripts/check_extension_readonly_diag_20261110.py').read_text(encoding='utf-8')
source = source.replace('tmp/pickup_second_diag_20261110/project', 'tmp/pickup_driver_fault_read_20261111/project')
source = source.replace('2026-11-10/second-diag', '2026-11-11/driver-fault-read')
source = source.replace("'-Wno-unused-variable',", "'-Wno-unused-variable','-Wno-misleading-indentation',")
source = source.replace('#include "extension_readonly_diag.inc"', 'static uint32_t CAN1_ExtensionReadOnlyBlock(uint8_t c,uint8_t out[64],uint32_t *n){(void)c;(void)out;if(n)*n=0;return 9U;}\n#include "extension_readonly_diag.inc"', 1)
source = source.replace("test=r'''", "prefix+='\\n#define VEHICLE_EXTENSION_HOME_ONLY 1\\n#define VEHICLE_STARTUP_GROUP_ENABLE 0\\nstatic uint32_t eh_stop_pending,eh_started;\\n'\nprefix+=" + repr(helper) + "\n\ntest=r'''", 1)
source = source.replace('g_ext_diag_request=2;it_extension_diag_service();CK(g_ext_diag_state==3 && g_ext_diag_rejected==2', 'g_ext_diag_request=3;it_extension_diag_service();CK(g_ext_diag_state==3 && g_ext_diag_rejected==2', 1)
extra = r'''
 g_startup_axes_state=5;g_ext_diag_request=1;uint32_t first=calls;it_extension_diag_service();
 CK(g_ext_diag_state==1 && calls==first+1 && g_startup_axes_state==5 && g_it_state==9);
 for(uint32_t i=1;i<7;i++)it_extension_diag_service();
 CK(g_ext_diag_state==2 && g_startup_axes_state==5 && g_it_state==9);
 eh_stop_pending=1;g_ext_diag_request=1;first=calls;it_extension_diag_service();CK(g_ext_diag_state==3 && calls==first);eh_stop_pending=0;
 eh_started=1;g_ext_diag_request=1;it_extension_diag_service();CK(g_ext_diag_state==3 && calls==first);eh_started=0;
 g_startup_axes_state=4;before=calls;
'''
marker = ' g_it_state=2;g_it_auto_state=2;g_ext_diag_request=1;io_result=1;it_extension_diag_service();'
assert source.count(marker) == 1
source = source.replace(marker, extra + marker, 1)
exec(compile(source, 'actual-fault-idle-outer-gate', 'exec'), {'__file__': str(__file__), '__name__': '__main__'})

# Reuse actual host branch and exception checks after removing the extension-home
# request, preserving incomplete-diagnostic rejection and no-pickup CLI checks.
source = (R / 'scripts/check_fifo_diagnostic_reference_entry_20261111.py').read_text(encoding='utf-8')
source = source.replace('pickup_fifo_diagnostic_live_20261111.py', 'pickup_driver_fault_read_live_20261111.py')
source = source.replace('2026-11-11/driver-fifo-fix', '2026-11-11/driver-fault-read')
source = source.replace("calls[:3]] == [1, 2, 6]", "calls[:2]] == [1, 6]")
source = source.replace('len(calls) == 4', 'len(calls) == 3')
source = source.replace('--references-and-config-once', '--lift-reference-and-config-once')
exec(compile(source, 'actual-lift-only-entry', 'exec'), {'__file__': str(__file__), '__name__': '__main__'})
source = (R / 'scripts/check_fifo_diagnostic_host_stop_20261111.py').read_text(encoding='utf-8')
source = source.replace('pickup_fifo_diagnostic_live_20261111.py', 'pickup_driver_fault_read_live_20261111.py')
source = source.replace('2026-11-11/driver-fifo-fix', '2026-11-11/driver-fault-read')
exec(compile(source, 'actual-lift-only-stop', 'exec'), {'__file__': str(__file__), '__name__': '__main__'})
