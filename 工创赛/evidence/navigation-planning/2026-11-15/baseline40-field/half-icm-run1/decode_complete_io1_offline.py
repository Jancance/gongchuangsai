"""Saved-raw audit: a final unterminated fragment is not a complete CRC frame."""
from pathlib import Path
import hashlib,json,sys
E=Path(__file__).resolve().parent;R=E.parents[5]
sys.path.insert(0,str(R/'工创赛/scripts'))
from vehicle_uart_capture import parse_icm_observer
def main():
    frames=[];errors=[];incomplete=[];inputs=[]
    for source in ['preflight','serial']:
        for path in (E/source).glob('*.raw'):
            blob=path.read_bytes();inputs.append(dict(path=str(path),sha256=hashlib.sha256(blob).hexdigest()))
            for number,line in enumerate(blob.splitlines(keepends=True)):
                if not line.endswith((b'\r',b'\n')):
                    incomplete.append(dict(source=source,line=number,tag=line.split(b',',1)[0].decode('ascii',errors='replace'),bytes=len(line),wire=line.decode('ascii',errors='replace')))
                    continue
                if not line.startswith(b'IO1,'):continue
                try:frames.append(dict(source=source,line=number,data=parse_icm_observer(line)))
                except Exception as exc:errors.append(dict(source=source,line=number,error=repr(exc),wire=line.decode('ascii',errors='replace')))
    report=dict(inputs=inputs,total=len(frames),valid=sum(x['data']['valid'] for x in frames),frames=frames,errors=errors,
        excluded_final_incomplete_fragments=incomplete,read_only=True,hardware_accessed=False,
        note='Original immediate io1_decoded.json remains unchanged. Only complete newline-terminated frames enter CRC/field error counts here.')
    (E/'io1_decoded_complete.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8')
    print(json.dumps({k:report[k] for k in ['total','valid','errors','excluded_final_incomplete_fragments']}))
if __name__=='__main__':main()
