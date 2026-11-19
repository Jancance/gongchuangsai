"""Stage3-only160 preclose after MATLAB; no hardware."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage3_teach_mode_20261116/project'
P=R/'tmp/stage3_preclose160_20261116/project'
E=R/'evidence/integration/2026-11-16/stage3-preclose160'
S=R/'src/stage3_preclose160_20261116'
def edit(name,pairs):
 b=(P/name).read_bytes()
 for old,new in pairs:
  old=old.replace('\n','\r\n').encode('ascii');new=new.replace('\n','\r\n').encode('ascii')
  assert b.count(old)==1,(name,old,b.count(old));b=b.replace(old,new)
 (P/name).write_bytes(b);(S/name).parent.mkdir(parents=True,exist_ok=True);(S/name).write_bytes(b)
def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed'] and len(proof['checks'])==4
 assert not P.exists(),'Preserve candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 edit('CHASSIS/USER/integration10_complete.inc',[
 ('g_it_state==9U || request>15U)', 'g_it_state==9U || (request>15U && !(request==17U && Stage3_Owns())))'),
 ('}else if(request==5U || request==6U){','}else if(request==5U || request==6U || request==17U){'),
 ('jiazhua(request==5U?140.0f:210.0f)','jiazhua(request==5U?140.0f:(request==17U?160.0f:210.0f))'),
 (' case 5U:case 6U:', ' case 5U:case 6U:case 17U:')])
 name='CHASSIS/USER/stage3_cycle.inc';b=(P/name).read_bytes()
 old=b'  if((p==1U || p==6U || p==11U) && it_at_top())';new=b'  if((p==1U || p==7U || p==12U) && it_at_top())';assert b.count(old)==1;b=b.replace(old,new)
 start=b.index(b'  case 4U:S3_Axis(SPF_LIFT,s3_pick);')
 end=b.index(b'  default:g_stage3_state=6U;',start)
 block=b[start:end].decode('ascii');lines=block.splitlines(keepends=True)
 import re
 for i,line in enumerate(lines):
  lines[i]=re.sub(r'case (\d+)U:',lambda m:'case '+str(int(m[1])+1)+'U:',line)
 replacement=b'  case 4U:S3_Command(17U);break; /* Stage3 preclose before pickup descent. */\r\n'+''.join(lines).encode('ascii')
 b=b[:start]+replacement+b[end:];(P/name).write_bytes(b);(S/name).parent.mkdir(parents=True,exist_ok=True);(S/name).write_bytes(b)
 print('Prepared Stage3 preclose160: IT17 plus one pickup phase; source16 preserved')
if __name__=='__main__':main()
