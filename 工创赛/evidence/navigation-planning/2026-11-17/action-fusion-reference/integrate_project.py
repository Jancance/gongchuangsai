from pathlib import Path
import json,hashlib,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).resolve().parent;P=R/'3.stm32f407工程/gongxun-f407.uvprojx';U=P.parent/'CHASSIS/USER'
assert json.loads((E/'matlab_summary.json').read_text('utf8'))['status']=='PASS'
assert (U/'fusion_reference.c').exists() and (U/'fusion_reference.h').exists()
original=P.read_bytes();before=E/'before';before.mkdir(exist_ok=True)
if not (before/P.name).exists():(before/P.name).write_bytes(original)
assert b'<FileName>fusion_reference.c</FileName>' not in original
anchor=b'            <File>\r\n              <FileName>motion_profile.h</FileName>\r\n              <FileType>5</FileType>\r\n              <FilePath>.\\CHASSIS\\USER\\motion_profile.h</FilePath>\r\n            </File>'
newline=b'\r\n' if original.count(b'\r\n')==original.count(b'\n') else b'\n'
anchor=anchor.replace(b'\r\n',newline)
assert original.count(anchor)==2
addition=b''
for name,kind in [('fusion_reference.c','1'),('fusion_reference.h','5')]:
 addition+=('\r\n            <File>\r\n              <FileName>'+name+'</FileName>\r\n              <FileType>'+kind+'</FileType>\r\n              <FilePath>.\\CHASSIS\\USER\\'+name+'</FilePath>\r\n            </File>').encode('ascii')
addition=addition.replace(b'\r\n',newline)
changed=original.replace(anchor,anchor+addition)
assert bytes(x for x in original if x>=128)==bytes(x for x in changed if x>=128)
assert (changed.count(b'\n')==changed.count(b'\r\n'))==(original.count(b'\n')==original.count(b'\r\n'))
xml=ET.fromstring(changed)
for target in xml.findall('./Targets/Target'):
 files=[x.text for x in target.findall('.//File/FileName')]
 assert files.count('fusion_reference.c')==1 and files.count('fusion_reference.h')==1
P.write_bytes(changed)
(E/'project_receipt.json').write_text(json.dumps({'source_project':str(P),'before_sha256':hashlib.sha256(original).hexdigest(),'after_sha256':hashlib.sha256(changed).hexdigest(),'line_ending_preserved':newline.decode('ascii').replace('\r','CR').replace('\n','LF'),'non_ascii_preserved':True,'targets':[t.findtext('TargetName') for t in xml.findall('./Targets/Target')],'runtime_caller_added':False,'hardware_accessed':False},indent=2),'utf8')
print('Added two pure source files to both unified targets; no runtime call added')
