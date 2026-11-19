"""Isolate the validated moving-disc scheduler; never connects hardware."""
from pathlib import Path
import json
import shutil

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_mcu_cycle_20261115/project'
P=R/'tmp/source_mcu_continuous_20261115/project'
E=R/'evidence/integration/2026-11-15/source-continuous'

def main():
    proof=json.loads((E/'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab']
    assert not P.exists(),'Preserve prior candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    (P/'CHASSIS/USER/source_mcu_cycle.inc').write_bytes((R/'output/source_continuous_cycle_20261115.inc').read_text(encoding='utf-8').replace('\n','\r\n').encode('utf-8'))
    path=P/'CHASSIS/USER/pzq_commit_overlap.inc';data=path.read_bytes()
    ending=b'\r\n' if data.count(b'\r\n')==data.count(b'\n') else b'\n'
    def change(old,new):
        nonlocal data
        assert data.count(old)==1,old
        data=data.replace(old,new,1)
    change(b'volatile uint32_t g_low_elapsed_ms,g_low_clear_ms,g_low_clear_valid,g_low_release_ms;',
        b'volatile uint32_t g_low_elapsed_ms,g_low_clear_ms,g_low_clear_valid,g_low_release_ms;'+ending+b'volatile uint32_t g_low_grip_command_ms; /* Diagnostic, not physical contact confirmation. */')
    change(b'g_low_fault=g_low_elapsed_ms=g_low_clear_ms=g_low_clear_valid=g_low_release_ms=0U;',
        b'g_low_grip_command_ms=0U;g_low_fault=g_low_elapsed_ms=g_low_clear_ms=g_low_clear_valid=g_low_release_ms=0U;')
    change(b'if(jiazhua(140.0f)!=1U){Low_Fail(7U);return;}g_it_lead5_grip_done=1U;',
        b'if(jiazhua(140.0f)!=1U){Low_Fail(7U);return;}g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;g_it_lead5_grip_done=1U;')
    change(b'g_low_lift_retract_axes=SPF_LIFT|(low_extension>0.0f?SPF_EXT:0U);',
        b'if(!g_low_grip_command_ms)g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;'+ending+b'  g_low_lift_retract_axes=SPF_LIFT|(low_extension>0.0f?SPF_EXT:0U);')
    path.write_bytes(data)
    print('Prepared separate continuous-disc scheduler; motion trajectory unchanged; no hardware access')

if __name__=='__main__':main()
