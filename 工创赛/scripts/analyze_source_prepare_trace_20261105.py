"""Decode post-stop SX1 diagnostic records without accessing hardware."""
from pathlib import Path
import argparse,json
FIELDS='run_id index tick phase segment command result raw_hi raw_lo options units before_mdeg goal_mdeg driver_target_mdeg dropped'.split()
def decode(line):
 payload,tail=line.strip().rsplit(',',1);crc=0xffff
 for v in (payload+',').encode('ascii'):
  crc^=v<<8
  for _ in range(8):crc=((crc<<1)^0x1021)&65535 if crc&0x8000 else (crc<<1)&65535
 if int(tail,16)!=crc:raise ValueError('CRC mismatch')
 values=payload.split(',');assert values[0]=='SX1' and len(values)==16
 d=dict(zip(FIELDS,map(int,values[1:])));raw=d['raw_hi'].to_bytes(4,'big')+d['raw_lo'].to_bytes(4,'big')
 d['raw_hex']=raw.hex(' ')
 if d['result']==1 and d['command'] in (0x33,0x36) and raw[1]<=1:
  scale=100 if d['options']&0x80 else 10
  d['angle_deg']=(-1 if raw[1] else 1)*int.from_bytes(raw[2:6],'big')/scale
  d['angle_scale_profile']='node6 ruler+independent-encoder verified 2026-11-05'
 if d['result']==1 and d['command']==0x35 and raw[1]<=1:
  d['speed_rpm']=(-1 if raw[1] else 1)*int.from_bytes(raw[2:4],'big')*.1
 d['kind']='tx_part1' if d['command']==0x1fd else 'tx_part2' if d['command']==0x2fd else 'response'
 return d
def main():
 p=argparse.ArgumentParser();p.add_argument('capture',type=Path);p.add_argument('--out',type=Path);a=p.parse_args()
 rows=[decode(s[s.index('SX1,'):]) for s in a.capture.read_bytes().decode('ascii',errors='replace').splitlines() if 'SX1,' in s]
 text=json.dumps(rows,ensure_ascii=False,indent=2)
 if a.out:a.out.write_text(text,encoding='utf-8')
 else:print(text)
if __name__=='__main__':main()
