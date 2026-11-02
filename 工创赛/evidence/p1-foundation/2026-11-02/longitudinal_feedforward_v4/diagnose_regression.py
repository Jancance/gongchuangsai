"""Reproduce saved micro-reverse regression in the emulator, without hardware."""
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'工创赛/scripts'))
from p1_longitudinal_feedforward_sim import adapter

out=Path(__file__).resolve().parent
diagnostic=out/'fault_reproduction'
diagnostic.mkdir(exist_ok=False)
adapter(diagnostic,10,10,False)
folder=diagnostic/'adapter_regression'
p=folder/'fixture.c'
fixture=p.read_text(encoding='utf8')
fixture=fixture[:fixture.index('uint32_t test_entry(void){')]+'''uint32_t test_entry(void){
 single_turn_case=-1;micro_case=1;
 scenario_run(1,1800,200,.08f,0,0,0);
 CHECK(micro_injected>0);
 return first_failure?(0x80000000U|first_failure):checks;
}
'''
p.write_text(fixture,encoding='utf8')
p=folder/'run_models.py'
runner=p.read_text(encoding='utf8')
marker="    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8')"
extra='''    fields=['reason','state','io_result','address','command_age_ms','status_mask','speed_mask','feedback_age_ms','tick_ms','target_decirpm','measured_decirpm','target_ack_ms','speed_sample_ms','reverse_samples']
    report['fault_diagnostic']=dict(zip(fields,struct.unpack('<9I2i3I',m.mem_read(address('g_chassis_auto_fault_diagnostic'),56))))
    report['injected_samples']=struct.unpack('<I',m.mem_read(address('micro_injected'),4))[0]
    report['progress_mm']=struct.unpack('<f',m.mem_read(address('s_distance_progress_mm'),4))[0]
    report['target_mm']=struct.unpack('<f',m.mem_read(address('s_distance_target_mm'),4))[0]
'''
assert marker in runner
p.write_text(runner.replace(marker,extra+marker),encoding='utf8')
spec=importlib.util.spec_from_file_location('diagnostic_adapter',p)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
try:
    module.run(out/'candidate',diagnostic/'model')
except RuntimeError:
    report=json.loads((diagnostic/'model/report.json').read_text(encoding='utf8'))
    print(json.dumps({k:report[k] for k in ['passed','failure_context','fault_diagnostic','injected_samples','progress_mm','target_mm']},indent=2))
