"""Offline only: decode one coherent, little-endian RAM dump; never open hardware.

Usage: python parse_snapshot.py snapshot.bin
A raw dump alone does not prove coherence: decoded valid means counter validity only.
For future authorized acquisition, require the same session, DONE, backend inactive
and no fault before and after two identical 104-byte reads. Do not halt the MCU,
reset, change VM, mask IRQs or call a getter. Read before the operator resets.
If the state, stopped record updates or byte equality cannot be established, mark
the acquisition unstable and do not accept its decoded numbers as timing evidence.
"""
import argparse,json,struct
from pathlib import Path

def parse(blob):
    if len(blob)!=104: raise ValueError('Expected exactly 104 bytes from the matching ELF symbol')
    enabled,clock=struct.unpack_from('<II',blob)
    result={'enabled':bool(enabled),'SystemCoreClock':clock,'chip_cycles_measured':False,
      'provenance':'offline decode only; hardware provenance not established',
      'irq_included':True,'scopes':[]}
    for i,name in enumerate(['target_plus_feedforward','step_budget_quantize']):
        valid,count,minimum,last,maximum=struct.unpack_from('<5I',blob,8+i*48)
        total,first_ms,last_ms,window_ms=struct.unpack_from('<QIII',blob,8+i*48+24)
        measured=bool(enabled and clock and valid and count)
        s=dict(name=name,valid=measured,count=count,min_cycles=minimum,last_cycles=last,
            max_cycles=maximum,sum_cycles=total,first_ms=first_ms,last_ms=last_ms,window_ms=window_ms,
            count_saturated=count==0xffffffff,
            mean_cycles=total/count if measured else None,
            mean_us=(total/count)*1e6/clock if measured else None,
            min_us=minimum*1e6/clock if measured else None,
            last_us=last*1e6/clock if measured else None,
            max_us=maximum*1e6/clock if measured else None,
            intervals_per_second=(count-1)*1000/window_ms if measured and count>1 and window_ms else None)
        result['scopes'].append(s)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('snapshot',type=Path);a=p.parse_args()
    print(json.dumps(parse(a.snapshot.read_bytes()),indent=2))
