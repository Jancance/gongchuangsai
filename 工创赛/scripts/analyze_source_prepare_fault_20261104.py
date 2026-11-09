"""Decode SF1 first-fault snapshots from raw UART capture; never accesses hardware."""
import argparse,json
from pathlib import Path
FIELDS='seq tick run_id reason first_tick phase segment command exchange rx_id dlc raw_hi raw_lo position_age origin_mdeg current_mdeg goal_mdeg'.split()
def decode(line):
 payload,tail=line.strip().rsplit(',',1);crc=0xffff
 for v in (payload+',').encode('ascii'):
  crc^=v<<8
  for _ in range(8):crc=((crc<<1)^0x1021)&65535 if crc&0x8000 else (crc<<1)&65535
 if int(tail,16)!=crc:raise ValueError('CRC mismatch')
 cells=payload.split(',');assert cells[0]=='SF1' and len(cells)==18
 f=dict(zip(FIELDS,map(int,cells[1:])))
 raw=f['raw_hi'].to_bytes(4,'big')+f['raw_lo'].to_bytes(4,'big')
 f['response_hex']=raw[:min(f['dlc'],8)].hex(' ')
 return f
def main():
 p=argparse.ArgumentParser();p.add_argument('capture',type=Path);a=p.parse_args()
 lines=a.capture.read_bytes().decode('ascii',errors='replace').splitlines()
 frames=[decode(s[s.index('SF1,'):]) for s in lines if 'SF1,' in s]
 print(json.dumps(frames,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
