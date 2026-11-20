"""Offline decoder for the matching ELF's 176-byte tail snapshot symbol.

Never opens hardware. Future acquisition needs same-session DONE/backend inactive/
fault0 before and after two identical full 176-byte reads. Do not halt, reset,
change VM, mask IRQs, or call a getter. If quiescence/coherence cannot be shown,
mark acquisition unstable; valid_mask is not provenance. Read before next start.
The getter may be removed by the linker; use the matching ELF global symbol.
"""
from pathlib import Path
import argparse,json,math,struct

FLAG_NAMES=['profile_active','profile_done','run_started','reached_before_event',
 'stop_due_before_event','finish_requested_before_event','alignment_seen','common_age_less_400ms']
FIELDS=['now_ms','run_tick_ms','common_ms','follow_tick_ms','common_mean_mm',
 'reference_position_mm','reference_rpm','follow_rpm','follow_rate_rpm_s',
 'pending_base_rpm','applied_base_rpm','flags']

def parse(blob):
    if len(blob)!=176:raise ValueError('Expected exactly 176 bytes from matching ELF symbol')
    version,session,mask,entry,freeze,arrival,res0,res1=struct.unpack_from('<8I',blob)
    if version not in (0,1):raise ValueError('Unsupported tail snapshot format')
    if mask&~7:raise ValueError('Unknown snapshot slot valid bits')
    slots=[]
    for i,name in enumerate(['first_tail_entry','latest_tail_batch_freeze_NOT_complete_ACK','arrival_BEFORE_reached_assignment']):
        values=struct.unpack_from('<4I5f3I',blob,32+i*48)
        record=dict(zip(FIELDS,values));flags=record['flags']
        present=bool(version==1 and mask&(1<<i))
        finite=all(math.isfinite(x) for x in values[4:9])
        valid=bool(present and finite and not flags&~255)
        record.update(slot=i,name=name,captured=present,record_valid=valid,
          elapsed_ms=((record['now_ms']-record['run_tick_ms'])&0xffffffff) if valid else None,
          common_age_ms=((record['now_ms']-record['common_ms'])&0xffffffff) if valid else None,
          follow_state_age_ms=((record['now_ms']-record['follow_tick_ms'])&0xffffffff) if valid else None,
          flags_decoded={key:bool(flags&(1<<bit)) for bit,key in enumerate(FLAG_NAMES)},
          quantity_kind='reference/request/applied command; NOT measured wheel or ground speed')
        # Retain raw zeros/old values for inspection, never label invalid as a measurement.
        if not valid:record['invalid_reason']='not captured, nonfinite field, unknown flags, or format not initialized'
        slots.append(record)
    return dict(format_version=version,session=session,valid_mask=mask,
      event_counts=dict(first_entry=entry,accepted_tail_batch_freezes=freeze,arrival=arrival),
      reserved=[res0,res1],slots=slots,hardware_provenance_established=False,
      acquisition_status='UNVERIFIED_OFFLINE_BYTES; require external stable double-read receipt',
      B1_enabled=False,ground_distance_accuracy_established=False)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('snapshot',type=Path);args=p.parse_args()
    print(json.dumps(parse(args.snapshot.read_bytes()),indent=2,ensure_ascii=False))
