"""Build configurations from the sole daily uvprojx; never download."""
from pathlib import Path
import hashlib,json,re,subprocess,sys
R=Path.cwd();P=R/'3.stm32f407工程/gongxun-f407.uvprojx'
E=Path(__file__).resolve().parent
T=R/'工创赛/tmp/rollback-half-ff-build-20261115';T.mkdir(parents=True,exist_ok=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    report={'owning_project':str(P),'project_sha256':sha(P),'source_sha256':{n:sha(P.parent/'CHASSIS/USER'/n) for n in ['app_odometry.c','app_odometry.h','chassis_kinematics.c','chassis_kinematics.h','chassis_auto_test.c','app_motion_runtime.c','app_serial_runtime.c','chassis_auto_test.h','hwt101.c','hwt101.h']},'hardware_accessed':False,'builds':{}}
    configurations=[('baseline40_integrated','Vehicle_Integrated','APP_BASELINE40_ENABLE=1,VEHICLE_INTEGRATED_ENABLE=0,VEHICLE_STAGE1_ENABLE=0'),
        ('baseline40_offline','Vehicle_Offline','APP_BASELINE40_ENABLE=1,VEHICLE_INTEGRATED_ENABLE=0,VEHICLE_STAGE1_ENABLE=0'),
        ('baseline_off_integrated','Vehicle_Integrated','')]
    for name,target,defines in configurations:
        out=T/name;out.mkdir(exist_ok=True)
        s=P.read_text('utf8').replace('.\\',str(P.parent)+'\\')
        for tag in ['OutputDirectory','ListingPath']:
            s=re.sub('<'+tag+'>.*?</'+tag+'>',lambda m:'<'+tag+'>'+name+'\\</'+tag+'>',s)
        if defines:s=s.replace('<Define>','<Define>'+defines+',')
        q=T/(name+'.uvprojx');q.write_text(s,'utf8')
        log=E/(name+'.log');cmd=['D:/Keil_v5/UV4/UV4.exe','-r',str(q),'-t',target,'-o',str(log)]
        process=subprocess.run(cmd,timeout=300);text=log.read_bytes().decode('cp936')
        result={'returncode':process.returncode,'defines':defines,'command':cmd,'project_copy_sha256':sha(q),'log_sha256':sha(log)}
        report['builds'][name]=result
        (E/'build_summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),'utf8')
        assert process.returncode==0 and '0 Error(s), 0 Warning(s)' in text,(name,text[-4500:])
        result.update(program_size=re.search('Program Size:.*',text)[0],axf=str(out/(target+'.axf')),axf_sha256=sha(out/(target+'.axf')))
        (E/'build_summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),'utf8')
        print(name,result['program_size'],flush=True)
    assert report['project_sha256']==sha(P)
    assert all(sha(P.parent/'CHASSIS/USER'/n)==h for n,h in report['source_sha256'].items())
if __name__=='__main__':main()
