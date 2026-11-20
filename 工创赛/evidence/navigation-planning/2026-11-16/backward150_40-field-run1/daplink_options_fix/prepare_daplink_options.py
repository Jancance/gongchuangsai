from pathlib import Path
import hashlib,xml.etree.ElementTree as ET
E=Path(__file__).resolve().parent
R=E.parents[5]
dest=R/'工创赛/tmp/all-directions-short-build-20261116/all_directions_short40.uvoptx'
raw=(E/'daplink_options_template.uvoptx').read_bytes()
if dest.exists():
 assert dest.read_bytes()==raw,'Existing options differ; inspect before replacing'
else:
 dest.write_bytes(raw)
r=ET.parse(dest).getroot()
assert r.findtext('./Target/TargetOption/DebugOpt/pMon')=='BIN\\CMSIS_AGDI.dll'
print('DAPLINK_OPTIONS_READY_NO_HARDWARE',hashlib.sha256(raw).hexdigest())
