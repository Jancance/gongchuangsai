"""Decode saved raw bytes only; never opens a serial port or debug probe."""
from pathlib import Path
import hashlib,json,sys
E=Path(__file__).resolve().parent;R=E.parents[5]
sys.path.insert(0,str(R/'工创赛/scripts'))
from vehicle_uart_capture import parse_icm_observer
def main():
    frames=[];errors=[];inputs=[]
    for directory in ['preflight','serial']:
        for path in (E/directory).glob('*.raw'):
            blob=path.read_bytes();inputs.append(dict(path=str(path),sha256=hashlib.sha256(blob).hexdigest()))
            for i,line in enumerate(blob.splitlines()):
                if line.startswith(b'IO1,'):
                    try:frames.append(dict(source=directory,line=i,data=parse_icm_observer(line)))
                    except Exception as exc:errors.append(dict(source=directory,line=i,error=repr(exc),wire=line.decode('ascii',errors='replace')))
    result=dict(inputs=inputs,frames=frames,errors=errors,total=len(frames),valid=sum(bool(x['data']['valid']) for x in frames),
        read_only=True,source='saved raw bytes; IO1 did not participate in control')
    (E/'io1_decoded.json').write_text(json.dumps(result,indent=2),'utf8')
    print(json.dumps({k:result[k] for k in ['total','valid','errors']}),flush=True)
if __name__=='__main__':main()
