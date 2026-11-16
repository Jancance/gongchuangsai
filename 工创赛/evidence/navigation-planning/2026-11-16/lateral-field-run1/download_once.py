"""One authorized candidate program/verify; no retries and no serial commands."""
from pathlib import Path
import datetime,json,shutil,subprocess,sys
from run_binding import E,R,PREP,EXPECTED,IMAGE,sha,validate
def main():
    binding=validate('download')
    config=json.loads((PREP/'build_receipt.json').read_text('utf8'))
    project=Path(config['command'][2]);candidate=json.loads((E/'candidate_binding.json').read_text('utf8'));assert sha(project)==candidate['project_sha256']
    template=R/'工创赛/tmp/lateral-feedback-build-20261113/baseline40_integrated.uvoptx'
    options_sha='ef26b9526f5e67ece67bd4e7c268f1f57f628aedb1736be2ba357ebc07b420ba'
    assert sha(template)==options_sha
    options=project.with_suffix('.uvoptx')
    if options.exists():assert sha(options)==options_sha
    else:shutil.copyfile(template,options)
    log=E/'download.log'
    record=dict(candidate_sha256=EXPECTED,candidate_sha256_before=sha(IMAGE),attempts=1,
        automatic_retry=False,download_initializes_MCU=True,separate_unified_reset_follows=False,
        flash_read=False,serial_opened=False,ARM_START_written=False,
        command=['D:/Keil_v5/UV4/UV4.exe','-f',str(project),'-t','Vehicle_Integrated','-o',str(log)],
        started_at=datetime.datetime.now().astimezone().isoformat())
    with (E/'download_once_started.json').open('x',encoding='utf8') as f:json.dump(record,f,indent=2)
    try:
        sys.path.insert(0,str(R/'工创赛/tmp/turn_probe_deps'))
        from pyocd.core.helpers import ConnectHelper
        probes=ConnectHelper.get_all_connected_probes(blocking=False)
        record['probe_uids_immediately_before_download']=[p.unique_id for p in probes]
        assert len(probes)==1 and probes[0].unique_id.lower()==binding['probe_uid'].lower(), 'Require exactly this one actual probe; no target attach'
        process=subprocess.run(record['command'],cwd=R,timeout=30,capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
        record['returncode']=process.returncode
        (E/'download_process.log').write_bytes(process.stdout+process.stderr)
    except Exception as exc:
        record['error']=repr(exc)+'; do not retry automatically'
    finally:
        text=log.read_bytes().decode('cp936',errors='replace') if log.exists() else ''
        record['markers']={s:s in text for s in ['Erase Done.','Programming Done.','Verify OK.','Application running ...']}
        record['candidate_sha256_after']=sha(IMAGE);record['candidate_unchanged']=sha(IMAGE)==EXPECTED
        record['success']=record.get('returncode')==0 and all(record['markers'].values()) and record['candidate_unchanged']
        (E/'download_once_receipt.json').write_text(json.dumps(record,indent=2),'utf8')
        print(json.dumps(record),flush=True)
    if not record['success']:raise SystemExit(1)
if __name__=='__main__':main()
