"""One bounded chain; failure saves evidence and never repeats an action."""
from pathlib import Path
import datetime,json,subprocess,sys
from run_binding import E,EXPECTED,validate
def main():
    validate()
    receipt=dict(candidate_sha256=EXPECTED,started_at=datetime.datetime.now().astimezone().isoformat(),
        maximum_downloads=1,maximum_resets=1,maximum_START=1,automatic_retry=False,steps=[],success=False)
    with (E/'campaign_started.json').open('x',encoding='utf8') as f:json.dump(receipt,f,indent=2)
    try:
        for script,timeout in [('download_once.py',40),('reset_once.py',30),('run_and_read_once.py',125)]:
            print('Authorized single stage: '+script,flush=True)
            q=subprocess.run([sys.executable,str(E/script)],timeout=timeout)
            receipt['steps'].append(dict(script=script,returncode=q.returncode))
            if q.returncode:raise RuntimeError(script+' failed; no automatic retry')
        tail=json.loads((E/'tail_read_receipt.json').read_text('utf8'))
        if not tail.get('accepted'):raise RuntimeError('Tail not trusted; saved once without retry')
        receipt['success']=True
    except Exception as exc:
        receipt['error']=repr(exc)
        raise
    finally:
        subprocess.run([sys.executable,str(E/'decode_io1_once.py')],timeout=15)
        receipt['finished_at']=datetime.datetime.now().astimezone().isoformat()
        (E/'campaign_receipt.json').write_text(json.dumps(receipt,indent=2),'utf8')
        print(json.dumps(receipt),flush=True)
if __name__=='__main__':main()
