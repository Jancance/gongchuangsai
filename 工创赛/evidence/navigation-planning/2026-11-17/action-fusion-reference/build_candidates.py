from pathlib import Path
import datetime,hashlib,json,re,subprocess,shutil
R=Path.cwd();E=Path(__file__).resolve().parent;P=R/'3.stm32f407工程/gongxun-f407.uvprojx';T=R/'工创赛/tmp/action-fusion-build-20261118';T.mkdir(exist_ok=True)
U=P.parent/'CHASSIS/USER'
assert (U/'fusion_reference.c').is_file() and (U/'fusion_reference.h').is_file()
receipts={}
for name,target in [('integrated','Vehicle_Integrated'),('offline','Vehicle_Offline')]:
 defines='ICM45686_SAMPLE_ONLY=0,APP_CONTINUOUS_MAP_ENABLE=1,APP_MAP_NAV_RESIDUAL_ENABLE=1'
 (T/name).mkdir(exist_ok=True)
 source=P.read_text('utf8').replace('.\\',str(P.parent)+'\\')
 for tag in ['OutputDirectory','ListingPath']:
  source=re.sub('<'+tag+'>.*?</'+tag+'>',lambda m:'<'+tag+'>'+name+'\\</'+tag+'>',source)
 source=source.replace('<Define>','<Define>'+defines+',')
 project=T/(name+'.uvprojx');project.write_text(source,'utf8')
 logp=E/(name+'_build.log');cmd=['D:/Keil_v5/UV4/UV4.exe','-r',str(project),'-t',target,'-o',str(logp)]
 receipt={'started_at':datetime.datetime.now().astimezone().isoformat(),'command':cmd,'defines':defines,'target':target,'hardware_accessed':False,'scope':'Compile pure module in both real targets; no runtime caller is added.'}
 cp=subprocess.run(cmd,timeout=300,creationflags=subprocess.CREATE_NO_WINDOW)
 log=logp.read_bytes().decode('gbk',errors='replace')
 receipt.update(returncode=cp.returncode,finished_at=datetime.datetime.now().astimezone().isoformat(),success=cp.returncode==0 and '0 Error(s), 0 Warning(s)' in log,compiled_reference='compiling fusion_reference.c' in log)
 receipt['source_sha256']={n:hashlib.sha256((U/n).read_bytes()).hexdigest() for n in ['fusion_reference.c','fusion_reference.h']}
 frozen=E/'frozen'/name;frozen.mkdir(parents=True,exist_ok=True)
 for q in [project,*list((T/name).glob('*.axf')),*list((T/name).glob('*.map')),*list((T/name).glob('*.htm')),*list((T/name).glob('*.lnp'))]:shutil.copy2(q,frozen/q.name)
 receipt['frozen_sha256']={q.name:hashlib.sha256(q.read_bytes()).hexdigest() for q in frozen.iterdir() if q.is_file()}
 receipt['size_line']=next((x for x in log.splitlines() if 'Program Size:' in x),'')
 receipts[name]=receipt;(E/'build_receipts.json').write_text(json.dumps(receipts,indent=2),'utf8')
 print(name,receipt['success'],receipt['compiled_reference'],receipt['size_line'],flush=True)
 assert receipt['success'] and receipt['compiled_reference'],log[-2200:]
