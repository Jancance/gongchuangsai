from pathlib import Path
import datetime,hashlib,json,re,subprocess,sys
R=Path.cwd();E=Path(__file__).parent;P=R/'3.stm32f407工程/gongxun-f407.uvprojx';T=R/'工创赛/tmp/lateral-drift-trim-build-20261116'
T.mkdir(exist_ok=True);name='left210_trim_defaultoff';(T/name).mkdir(exist_ok=True)
defines='APP_BASELINE40_ENABLE=1,VEHICLE_INTEGRATED_ENABLE=0,VEHICLE_STAGE1_ENABLE=0,APP_FUSION_CAPTURE_STATIC_TEST=0,APP_FUSION_CAPTURE_ENABLE=1'
text=P.read_text('utf8').replace('.\\',str(P.parent)+'\\')
for tag in ['OutputDirectory','ListingPath']:
 text=re.sub('<'+tag+'>.*?</'+tag+'>',lambda m:'<'+tag+'>'+name+'\\</'+tag+'>',text)
text=text.replace('<Define>','<Define>'+defines+',');q=T/(name+'.uvprojx');q.write_text(text,'utf8')
cmd=['D:/Keil_v5/UV4/UV4.exe','-r',str(q),'-t','Vehicle_Integrated','-o',str(E/'defaultoff_build.log')]
receipt=dict(started_at=datetime.datetime.now().astimezone().isoformat(),owning_project=str(P),command=cmd,defines=defines,hardware_accessed=False)
def save(): (E/'defaultoff_build_receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf8')
save();cp=subprocess.run(cmd,timeout=300,creationflags=subprocess.CREATE_NO_WINDOW)
log=(E/'defaultoff_build.log').read_bytes().decode('cp936',errors='replace');receipt.update(returncode=cp.returncode,finished_at=datetime.datetime.now().astimezone().isoformat(),success=cp.returncode==0 and '0 Error(s), 0 Warning(s)' in log);save()
assert receipt['success'],log[-4000:]
axf=T/name/'Vehicle_Integrated.axf';h=hashlib.sha256(axf.read_bytes()).hexdigest();receipt.update(axf=str(axf),axf_sha256=h,program_size=re.search('Program Size:.*',log)[0]);save()
reader=R/'工创赛/evidence/navigation-planning/2026-11-15/fusion-b-capture-prep/read_capture_once.py'
cp=subprocess.run([sys.executable,str(reader),'--axf',str(axf),'--expected-sha256',h],capture_output=True,timeout=30)
(E/'defaultoff_offline_elf_check.json').write_bytes(cp.stdout);(E/'defaultoff_offline_elf_stderr.txt').write_bytes(cp.stderr)
assert cp.returncode==0,cp.stderr
print(json.dumps(receipt,indent=2))
