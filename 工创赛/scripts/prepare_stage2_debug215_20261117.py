"""Explicit isolated debug task source; never fabricates QR state."""
from pathlib import Path
import json,shutil,re
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage2_sync_opt_20261117/project'
P=R/'tmp/stage2_debug215_20261117/project'
E=R/'evidence/integration/2026-11-17/stage2-debug215'
def wrap_function(b,signature,body):
 start=b.index(signature.encode());brace=b.index(b'{',start);depth=1;end=brace+1
 while depth:
  if b[end]==123:depth+=1
  elif b[end]==125:depth-=1
  end+=1
 original=b[brace+1:end-1]
 replacement=b'\r\n#if SOURCE_CYCLE_TEST_215\r\n'+body.replace('\n','\r\n').encode()+b'\r\n#else'+original+b'#endif\r\n'
 return b[:brace+1]+replacement+b[end-1:]
def main():
 assert json.loads((E/'matlab-result.json').read_text())['passed']
 assert not P.exists(),'Preserve existing independent debug candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 p=P/'USER/vehicle_parameters.h';b=p.read_bytes();anchor=b'#define VEHICLE_LIFT_RULER_ONLY 0U';assert b.count(anchor)==1
 b=b.replace(anchor,b'/* Explicit stand-alone source debug task; default remains real QR. */\r\n#ifndef SOURCE_CYCLE_TEST_215\r\n#define SOURCE_CYCLE_TEST_215 0U\r\n#endif\r\n\r\n'+anchor,1);p.write_bytes(b)
 p=P/'CHASSIS/USER/source_mcu_cycle.inc';b=p.read_bytes()
 b=wrap_function(b,'void SourceCycle_ScanService(void)', ' /* Independent debug mode does not own the scanner, its mailbox or display. */\n return;')
 b=wrap_function(b,'static uint32_t SC_ScanTask(void)', ' /* User test order only: yellow, red, black; not a four-part scanned task. */\n g_source_cycle_colors[0]=2U;g_source_cycle_colors[1]=1U;g_source_cycle_colors[2]=5U;\n return 1U;')
 p.write_bytes(b)
 p=P/'CHASSIS/USER/region_dispatch.inc';b=p.read_bytes();anchor=b'volatile uint32_t g_region_dispatch_area,g_region_dispatch_rejected;';assert b.count(anchor)==1
 b=b.replace(anchor,b'#if SOURCE_CYCLE_TEST_215\r\nvolatile uint32_t g_region_dispatch_area=1U,g_region_dispatch_rejected;\r\n#else\r\n'+anchor+b'\r\n#endif',1);p.write_bytes(b)
 p=P/'gongxun-f407.uvprojx';b=p.read_bytes();count=0
 def target(m):
  nonlocal count
  block=m.group(0)
  if not any(b'<TargetName>'+name.encode()+b'</TargetName>' in block for name in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']):return block
  match=re.search(rb'<Define>([^<]*)</Define>',block);assert match
  defines=match.group(1);assert b'SOURCE_CYCLE_TEST_215' not in defines
  count+=1;return block[:match.start(1)]+defines+b',SOURCE_CYCLE_TEST_215=1'+block[match.end(1):]
 b=re.sub(rb'<Target>.*?</Target>',target,b,flags=re.S);assert count==2;p.write_bytes(b)
 print('Prepared explicit debug215 after MATLAB; default0 and real QR branch preserved; no hardware.')
if __name__=='__main__':main()
