"""Compare the flashed candidate with physically accepted 879; no hardware writes."""
from pathlib import Path
import difflib,hashlib,json
from prepare_integration23_sync_20261115 import R,P,E

def main():
    base=R/'tmp/source_extension_lead_pick_20261115/project'
    skip={'Objects','Listings'}
    def files(root):
        return {p.relative_to(root).as_posix():p for p in root.rglob('*')
                if p.is_file() and not any(part in skip for part in p.relative_to(root).parts)}
    a,b=files(base),files(P);changed=[];diffs=[]
    for name in sorted(a.keys()|b.keys()):
        old=a[name].read_bytes() if name in a else b''
        new=b[name].read_bytes() if name in b else b''
        if old==new:continue
        changed.append(name)
        def decode(x):
            try:return x.decode('utf-8')
            except UnicodeDecodeError:return x.decode('gb18030',errors='replace')
        diffs.extend(difflib.unified_diff(decode(old).splitlines(True),decode(new).splitlines(True),
            fromfile='879/'+name,tofile='sync/'+name))
    expected={'CHASSIS/SYSTEM/CAN/can.c','CHASSIS/SYSTEM/CAN/can.h',
              'CHASSIS/USER/pzq_commit_overlap.inc','CHASSIS/USER/vehicle_source_parallel.inc',
              'USER/vehicle_parameters.h'}
    assert set(changed)==expected,changed
    owners=['CHASSIS/HARDWARE/POWER/power.c','CHASSIS/HARDWARE/POWER/power.h',
        'CHASSIS/USER/main.c','CHASSIS/SYSTEM/SAFETY/safety.c','CHASSIS/USER/lift_ruler_adapter.c',
        'CHASSIS/USER/lift_ruler_calibration.c','CHASSIS/USER/source_mcu_cycle.inc',
        'CHASSIS/USER/integration10_complete.inc','gongxun-f407.uvprojx']
    same={}
    for name in owners:
        assert name in a and name in b,name
        assert a[name].read_bytes()==b[name].read_bytes(),name
        same[name]=hashlib.sha256(b[name].read_bytes()).hexdigest()
    (E/'vm-vs879.diff').write_text(''.join(diffs),encoding='utf-8')
    report=dict(compared_to='879f54772af63033d3c11296d877ba6dc834a33de0a6f808ff76b3f40ac385a6',
        changed_files=changed,vm_owners_byte_identical=same,
        no_new_power_request_or_pe2_write=True,initial_query_path_unchanged=True,
        source_sync_branch_not_reached_in_failed_trials=True,hardware_access=False)
    (E/'vm-code-audit.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
