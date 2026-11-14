"""Retain native stdout/stderr bytes, including prefs/cache warnings."""
from pathlib import Path
import datetime
import json
import subprocess

out=Path(__file__).resolve().parent
exe=Path('E:/Program Files/Polyspace/R2020a/bin/matlab.exe')
started=datetime.datetime.now(datetime.timezone.utc).isoformat()
with (out/'matlab_native_stdout.bin').open('wb') as log:
    result=subprocess.run([str(exe),'-batch',"addpath('"+out.as_posix()+"'); stop_distance_study; snapshot_diagnostic_checks"],stdout=log,stderr=subprocess.STDOUT,cwd=out)
data=(out/'matlab_native_stdout.bin').read_bytes()
(out/'matlab_native_stdout_gbk.txt').write_text(data.decode('gbk',errors='replace'),encoding='utf-8')
(out/'matlab_receipt.json').write_text(json.dumps(dict(executable=str(exe),started_utc=started,finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),exit_code=result.returncode,raw_stdout='matlab_native_stdout.bin',decoded_stdout='matlab_native_stdout_gbk.txt',hardware_used=False),indent=2),encoding='utf-8')
print('MATLAB exit',result.returncode)
raise SystemExit(result.returncode)
