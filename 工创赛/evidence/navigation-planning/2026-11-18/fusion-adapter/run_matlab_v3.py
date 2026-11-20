from pathlib import Path
import subprocess, hashlib, json, datetime

E = Path(__file__).resolve().parent
R = Path.cwd()
U = R / '3.stm32f407工程/CHASSIS/USER'
m = E / 'verify_fusion_adapter_v3.m'
receipt = E / 'matlab_receipt_v3.json'
log = E / 'matlab_v3.log'
assert not receipt.exists() and not log.exists(), 'Preserve old runs; select a new run suffix.'
cmd = ['E:/Program Files/Polyspace/R2020a/bin/matlab.exe', '-batch',
       "addpath('" + E.as_posix() + "');verify_fusion_adapter_v3"]
d = {'started_at': datetime.datetime.now().astimezone().isoformat(),
     'script_sha256': hashlib.sha256(m.read_bytes()).hexdigest(), 'command': cmd,
     'product_adapter_c_exists_before': (U / 'fusion_adapter.c').exists(),
     'product_adapter_h_exists_before': (U / 'fusion_adapter.h').exists(),
     'hardware_accessed': False}
with log.open('wb') as f:
    p = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
d.update(returncode=p.returncode, finished_at=datetime.datetime.now().astimezone().isoformat(),
         product_adapter_c_exists_after=(U / 'fusion_adapter.c').exists(),
         product_adapter_h_exists_after=(U / 'fusion_adapter.h').exists(),
         log_sha256=hashlib.sha256(log.read_bytes()).hexdigest())
for name in ['matlab_summary_v3.json', 'adapter_vectors_v3.csv', 'quantization_scan_v3.csv']:
    path = E / name
    if path.exists():
        d[name + '_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
receipt.write_text(json.dumps(d, indent=2), encoding='utf-8')
print(json.dumps(d, ensure_ascii=False))
print(log.read_text(encoding='gbk', errors='replace')[-2200:])
raise SystemExit(p.returncode)
