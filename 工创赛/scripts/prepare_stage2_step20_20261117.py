"""Only the authorized Stage2 correction distance clamp, after MATLAB."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage2_debug215_20261117/project'
P=R/'tmp/stage2_debug215_step20_20261117/project'
E=R/'evidence/integration/2026-11-17/stage2-debug215-step20'
def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 assert not P.exists(),'Preserve prior candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 path=P/'CHASSIS/USER/source_mcu_cycle.inc';b=path.read_bytes();b.decode('utf-8');assert b.count(b'\n')==b.count(b'\r\n')
 old=b'distance=labs((long)amount)>=30000L?30U:';new=b'distance=labs((long)amount)>=20000L?20U:'
 assert b.count(old)==1;b=b.replace(old,new,1);path.write_bytes(b)
 print('Prepared isolated one-line step30 -> step20 business clamp; no hardware.')
if __name__=='__main__':main()
