"""Reuse the reviewed 176B observer only, against the committed half base."""
from pathlib import Path
import ast,hashlib,json

R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER'
OLD=E.parents[1]/'stop-distance-study/snapshot-candidate'
sha=lambda b:hashlib.sha256(b).hexdigest()
assert sha((U/'chassis_auto_test.c').read_bytes())=='4434979a40f940d8ee582378e5b57b45da8caeefb29bdde1558d1b17e85dac19'
assert sha((U/'chassis_auto_test.h').read_bytes())=='e940c64b4a5126f63a6cc6e4d814be2ef3096bd0d59f2e049d6a53257033b764'
assert sha((U/'hwt101.c').read_bytes())=='690b864c1144baf247a93cffb8be585675a0612002333edaf006026b5dbb3c90'
assert sha((U/'hwt101.h').read_bytes())=='eb01b7feb7e18c73495c472b09497514ab14de6941013683315b61f929b56429'
tree=ast.parse((OLD/'implement.py').read_text('utf8'))
header=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='header' for x in n.targets))
before={n:(U/n).read_bytes() for n in ['chassis_auto_test.c','chassis_auto_test.h']};after=before.copy();changes=[]
oldchanges=json.loads((OLD/'source_changes.json').read_text('utf8'))
for change in oldchanges:
 if change['file']=='chassis_auto_test.c':
  old=bytes.fromhex(change['old_hex']);new=bytes.fromhex(change['new_hex']);assert after['chassis_auto_test.c'].count(old)==1
  after['chassis_auto_test.c']=after['chassis_auto_test.c'].replace(old,new);changes.append(change)
old=b'extern volatile uint32_t g_chassis_auto_request;'
new=b'#if APP_BASELINE40_ACTIVE\n'+header.encode('gbk')+b'#endif\n\n'+old
assert after['chassis_auto_test.h'].count(old)==1;after['chassis_auto_test.h']=after['chassis_auto_test.h'].replace(old,new)
changes.append(dict(file='chassis_auto_test.h',old_hex=old.hex(),new_hex=new.hex()))
for n,b in after.items():
 assert b.decode('gbk').encode('gbk')==b
 if n.endswith('.c'):assert b.count(b'\n')==b.count(b'\r\n')
 else:assert b.count(b'\r\n')==before[n].count(b'\r\n')==1
 (E/(n+'.before')).write_bytes(before[n]);(U/n).write_bytes(b)
(E/'source_changes.json').write_text(json.dumps(changes,indent=2),'utf8')
(E/'source_manifest.json').write_text(json.dumps({n:dict(before_sha256=sha(before[n]),after_sha256=sha(after[n])) for n in before},indent=2),'utf8')
print((E/'source_manifest.json').read_text())
