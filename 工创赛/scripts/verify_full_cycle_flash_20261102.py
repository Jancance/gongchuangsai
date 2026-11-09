"""核对Keil从真实目标SAVE得到的Intel HEX与已构建二进制。"""
from pathlib import Path
import hashlib,json
R=Path(__file__).resolve().parents[1]
E=R/'evidence/p2-arm/2026-11-02/full-cycle'
p=Path('D:/Codex/home/visualizations/2026/11/01/01a1974f-9f64-71b0-bba6-341231b0ebee/flash-readback.hex')
image=(E/'firmware.bin').read_bytes();memory={};base=0
for line in p.read_text().splitlines():
    if not line.strip():continue
    assert line.startswith(':'),line[:30]
    row=bytes.fromhex(line[1:]);assert sum(row)%256==0
    n=row[0];address=(row[1]<<8)|row[2];kind=row[3];data=row[4:4+n]
    if kind==0:
        for i,v in enumerate(data):memory[base+address+i]=v
    elif kind==4:base=int.from_bytes(data,'big')<<16
    elif kind==2:base=int.from_bytes(data,'big')<<4
missing=[i for i in range(len(image)) if 0x08000000+i not in memory]
differences=[i for i,b in enumerate(image) if memory.get(0x08000000+i)!=b]
result=dict(flash_base='0x08000000',verified_bytes=len(image),missing_bytes=len(missing),
            differences=len(differences),first_differences= differences[:12],
            expected_sha256=hashlib.sha256(image).hexdigest(),readback_hex_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
            passed=not missing and not differences,scope='target Flash bytes only, not motion acceptance')
(E/'flash-verify.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=2));assert result['passed']
