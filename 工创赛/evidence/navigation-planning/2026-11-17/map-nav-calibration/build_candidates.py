from pathlib import Path
import datetime,hashlib,json,re,subprocess,shutil
R=Path.cwd();E=Path(__file__).resolve().parent;P=R/'3.stm32f407工程/gongxun-f407.uvprojx';T=R/'工创赛/tmp/map-nav-build-20261117';T.mkdir(exist_ok=True)
baseline=json.loads((E.parent/'control-distance-ratio/build_receipt.json').read_text('utf8'))['defines']
modes=[('integrated_on','APP_CONTINUOUS_MAP_ENABLE=1,APP_MAP_NAV_RESIDUAL_ENABLE=1'),('integrated_off','APP_CONTINUOUS_MAP_ENABLE=1,APP_MAP_NAV_RESIDUAL_ENABLE=0'),('map_off','APP_CONTINUOUS_MAP_ENABLE=0,APP_MAP_NAV_RESIDUAL_ENABLE=0'),('baseline_on','APP_CONTINUOUS_MAP_ENABLE=1,APP_MAP_NAV_RESIDUAL_ENABLE=1,'+baseline)]
all_receipts={}
for name,defines in modes:
 defines="ICM45686_SAMPLE_ONLY=0,"+defines
 (T/name).mkdir(exist_ok=True);text=P.read_text('utf8').replace('.\\',str(P.parent)+'\\')
 for tag in ['OutputDirectory','ListingPath']:text=re.sub('<'+tag+'>.*?</'+tag+'>',lambda m:'<'+tag+'>'+name+'\\</'+tag+'>',text)
 text=text.replace('<Define>','<Define>'+defines+',');project=T/(name+'.uvprojx');project.write_text(text,'utf8')
 logp=E/(name+'_build.log');cmd=['D:/Keil_v5/UV4/UV4.exe','-r',str(project),'-t','Vehicle_Integrated','-o',str(logp)]
 receipt={'started_at':datetime.datetime.now().astimezone().isoformat(),'command':cmd,'defines':defines,'hardware_accessed':False}
 cp=subprocess.run(cmd,timeout=300,creationflags=subprocess.CREATE_NO_WINDOW);log=logp.read_bytes().decode('gbk',errors='replace')
 receipt.update(finished_at=datetime.datetime.now().astimezone().isoformat(),returncode=cp.returncode,success=cp.returncode==0 and '0 Error(s), 0 Warning(s)' in log)
 frozen=E/'frozen'/name;frozen.mkdir(parents=True,exist_ok=True)
 for p in [project,*list((T/name).glob('*.axf')),*list((T/name).glob('*.map')),*list((T/name).glob('*.htm')),*list((T/name).glob('*.lnp'))]:shutil.copy2(p,frozen/p.name)
 receipt['frozen_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen.iterdir() if p.is_file()}
 receipt['size_line']=next((x for x in log.splitlines() if 'Program Size:' in x),'')
 all_receipts[name]=receipt;(E/'build_receipts.json').write_text(json.dumps(all_receipts,indent=2),'utf8');print(name,receipt['success'],receipt['size_line'],flush=True)
 assert receipt['success'],log[-2000:]
