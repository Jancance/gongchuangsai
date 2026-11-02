"""Zero-only sync protocol check; route cancellation is intentional, not travel success."""
import json,sys
from pathlib import Path
import p1_zero_sync_serial_motion as serial_motion
from p1_straight180_policy import make_controller
Base=make_controller(serial_motion)
class ZeroSyncController(Base):
 def consume(self,r,now):
  if self.action=='run-once' and self.start_sent and self.status is None:
   fresh=self.last_seq is None or 0<((r['seq']-self.last_seq)&0xffffffff)<0x80000000
   if fresh and r['phase']==5 and r['trip']==1 and r.get('sequence_state')==5 and r.get('sequence_completed')==0 and r.get('sequence_count')==1 and r['backend_cause']==9 and not r['backend_active'] and not r['vm_request'] and not r['pe2_active'] and not any(r[k] for k in ['rpm1','rpm2','rpm3','rpm4','can_fault','rtos_fault','safety_fault']):
    self.last_seq=r['seq'];self.last_rx=now;self.status='software_complete';self.stage='finished';return []
  return super().consume(r,now)
def review(s):
 batches=s.get('batch_records',[]);last=s.get('last') or {}
 success=[b for b in batches if b['outcome']==1 and b['sync_result']==1 and b['sync_cache_mask']==15 and b['ack_mask']==15]
 fault=s.get('last_fault')
 expected_cancel_snapshot=(not s.get('fault_frames',0) or bool(fault and fault.get('trip')==1 and fault.get('source_valid')==1 and not fault.get('backend_fault_reason',0)))
 ok=(s.get('status')=='software_complete' and expected_cancel_snapshot and len(batches)==1 and len(success)==1 and
  success[0]['base_rpm']==0 and success[0]['correction_rpm']==0 and not any(s['max_abs_rpm']) and
  last.get('phase')==5 and last.get('sequence_completed')==0 and not last.get('vm_request') and not last.get('pe2_active'))
 return dict(passed=ok,zero_sync_ack_verified=ok,travel_completed=False,expected_route_cancel=True,expected_cancel_snapshot=expected_cancel_snapshot,physical_stationary_confirmation_pending=True)
def main():
 args=sys.argv[1:];action=args[args.index('--action')+1] if '--action' in args else 'monitor'
 if '--output' not in args:raise ValueError('Use an explicit new evidence output directory')
 output=Path(args[args.index('--output')+1])
 if action=='run-once' and list(output.glob('*.summary.json')):raise ValueError('Never reuse a prior zero-check run directory')
 serial_motion.Controller=ZeroSyncController;code=serial_motion.main()
 if action=='run-once':
  p=max(output.glob('*.summary.json'),key=lambda p:p.stat().st_mtime_ns);s=json.loads(p.read_text(encoding='utf8'))
  result=review(s);s['zero_sync_review']=result;s['status']='zero_sync_check_passed' if result['passed'] else 'zero_sync_check_failed'
  p.write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(result));return 0 if result['passed'] else 2
 return code
if __name__=='__main__':raise SystemExit(main())
