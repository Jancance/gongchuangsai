from pathlib import Path
import json,hashlib,struct,subprocess
R=Path.cwd();E=Path(__file__).parent;U=R/'3.stm32f407工程/CHASSIS/USER'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
receipt=json.loads((E/'actual_matlab_receipt.json').read_text('utf-8-sig'));receipt['script_sha256']=sha(E/'validate_drift_candidate.m');receipt['script']='validate_drift_candidate.m';(E/'actual_matlab_receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf8')
changes=json.loads((E/'source_changes.json').read_text('utf8'));changes['after']={f:sha(U/f) for f in changes['after']};(E/'source_changes.json').write_text(json.dumps(changes,indent=2),encoding='utf8')
protected={}
for name,nextname in [('Chassis_Baseline40Budget','Chassis_Baseline40Commands'),('Chassis_Baseline40Commands','Chassis_LeftTrimCommands'),('Chassis_LeftTrimCommands','Chassis_TranslationHoldCommands')]:
 old=(E/'source_before/chassis_kinematics.c').read_bytes();new=(U/'chassis_kinematics.c').read_bytes()
 def section(b):
  a=b.index(('uint32_t '+name+'(').encode());z=b.index(('uint32_t '+nextname+'(').encode(),a);return b[a:z]
 protected[name]=dict(byte_identical=section(old)==section(new),sha256=hashlib.sha256(section(new)).hexdigest());assert protected[name]['byte_identical']
(E/'protected_functions_check.json').write_text(json.dumps(protected,indent=2),encoding='utf8')
s=(E/'build_candidate.py').read_text('utf8').replace("name='left210_trim4'","name='left210_trim_defaultoff'").replace('APP_LEFT40_DRIFT_TRIM_ENABLE=1,','').replace("E/'build_receipt.json'","E/'defaultoff_build_receipt.json'").replace("E/'build.log'","E/'defaultoff_build.log'").replace("E/'offline_elf_check.json'","E/'defaultoff_offline_elf_check.json'").replace("E/'offline_elf_stderr.txt'","E/'defaultoff_offline_elf_stderr.txt'")
(E/'build_defaultoff.py').write_text(s,encoding='utf8')
subprocess.run(['python',str(E/'build_defaultoff.py')],check=True)
old=json.loads((E.parent/'lateral-field-run1-prep/build_receipt.json').read_text('utf8'));new=json.loads((E/'defaultoff_build_receipt.json').read_text('utf8'))
def loads(path):
 d=Path(path).read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',d);r=[]
 for n in range(h[10]):
  kind,off,vaddr,paddr,fs,ms,flags,align=struct.unpack_from('<8I',d,h[5]+n*h[9])
  if kind==1 and fs:r.append(dict(address=paddr,size=fs,sha256=hashlib.sha256(d[off:off+fs]).hexdigest()))
 return r
a=loads(old['axf']);b=loads(new['axf']);out=dict(defaultoff_pt_load_identical=a==b,baseline=a,defaultoff=b,old_axf=old['axf'],new_axf=new['axf'],hardware_accessed=False)
(E/'defaultoff_load_comparison.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(out,ensure_ascii=False))
