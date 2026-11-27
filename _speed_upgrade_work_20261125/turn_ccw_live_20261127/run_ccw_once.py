"""One authorized CCW90 session on COM17; never selects/resets/retries a plan."""
import argparse
import json
from pathlib import Path
import sys
import time
from datetime import datetime

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '工创赛' / 'scripts'))
from p1_serial_motion import WindowsSerial, command, parse_telemetry


class Once:
    def __init__(self):
        self.stage = 'observe'
        self.token = (0, 0)
        self.first = self.previous = None
        self.last_rx = self.stage_at = self.keep_at = None
        self.result = self.first_failure = None
        self.start_sent = False
        self.rejected = None

    def abort(self, reason):
        if self.result is not None:
            return []
        self.first_failure = reason
        self.result = 'failed'
        self.stage = 'finished'
        return ['STOP']

    @staticmethod
    def profile(r):
        return all(r.get(k) == v for k, v in {
            'debug_plan_id': 20, 'motion_mode': 3, 'action_kind': 3,
            'sequence_count': 1, 'base_rpm': 82, 'feedback_limit_rpm': 130,
            'distance_mode': 0, 'selftest': 1, 'protocol_selftest': 1,
        }.items())

    @staticmethod
    def faults(r):
        return any(r.get(k, 0) for k in ('trip', 'can_fault', 'rtos_fault',
            'safety_fault', 'backend_cause', 'can_guard_cause', 'link_lost',
            'distance_fault_reason'))

    @staticmethod
    def idle(r):
        return (all(r.get(k) == 0 for k in ('phase', 'backend_active',
            'backend_result', 'vm_request', 'pe2_active', 'sequence_state',
            'sequence_completed')) and r['source_valid'] == 1 and
            0 <= r['angle_age_ms'] < 100 and 0 <= r['rate_age_ms'] < 100 and
            abs(r['rate_mdps']) <= 3000 and
            all(r.get('rpm' + str(i)) == 0 for i in range(1, 5)))

    def consume(self, r, now):
        if self.result is not None:
            return []
        if self.previous is not None:
            ds = (r['seq'] - self.previous[0]) & 0xffffffff
            dt = (r['tick_ms'] - self.previous[1]) & 0xffffffff
            if ds == 0:
                return []
            if ds >= 0x80000000 or dt == 0 or dt >= 0x80000000:
                return self.abort('telemetry_clock_or_session_changed')
            if abs(dt / 1000 - (now - self.previous[2])) > 0.15:
                if self.stage != 'observe':
                    return self.abort('telemetry_pacing_mismatch')
                self.first = None
        self.previous = (r['seq'], r['tick_ms'], now)
        self.last_rx = now
        if self.rejected is None:
            self.rejected = r['remote_rejected']
        elif r['remote_rejected'] != self.rejected:
            return self.abort('remote_rejection_count_changed')
        if not self.profile(r):
            return self.abort('ccw90_profile_mismatch')
        if self.faults(r):
            return self.abort('firmware_fault')
        if r['remote_result'] >= 4:
            return self.abort('remote_rejected_or_expired')
        if self.start_sent:
            if (r['token_seq'], r['token_tick']) != self.token:
                return self.abort('token_mismatch')
            if (r['sequence_state'] == 3 and r['sequence_completed'] == 1 and
                    r['phase'] == 3 and r['backend_result'] == 1 and
                    r['stop_mask'] == 15 and r['backend_active'] == 0 and r['backend_state'] == 9 and
                    r['remote_state'] == 3 and r['remote_result'] == 2 and
                    all(r.get('rpm' + str(i)) == 0 for i in range(1, 5))):
                self.result = 'software_complete_vm_may_remain_on'
                self.stage = 'finished'
                return []
            if r['sequence_state'] > 3 or r['phase'] > 3:
                return self.abort('sequence_failed')
        if self.stage == 'observe':
            if not self.idle(r) or r['remote_state'] != 0:
                self.first = None
                return []
            if self.first is None:
                self.first = (now, r['tick_ms'], r['seq'])
            hs = now - self.first[0]
            device_span = ((r['tick_ms'] - self.first[1]) & 0xffffffff) / 1000
            if abs(device_span - hs) > 0.15:
                self.first = (now, r['tick_ms'], r['seq'])
            elif hs >= 0.2 and ((r['seq'] - self.first[2]) & 0xffffffff) >= 2:
                self.token = (r['seq'], r['tick_ms'])
                self.stage, self.stage_at = 'arm_wait', now
                return ['ARM']
        elif self.stage == 'arm_wait':
            if r['remote_state'] == 1 and (r['token_seq'], r['token_tick']) == self.token:
                if not self.idle(r):
                    return self.abort('health_changed_before_start')
                self.stage, self.stage_at, self.keep_at = 'start_wait', now, now
                self.start_sent = True
                return ['START']
        elif self.stage == 'start_wait' and r['remote_state'] == 2:
            self.stage = 'running'
        return []

    def tick(self, now):
        if self.result is not None:
            return []
        if self.last_rx is not None and now - self.last_rx >= 0.6:
            return self.abort('telemetry_timeout')
        if self.stage in ('arm_wait', 'start_wait') and now - self.stage_at >= 2:
            return self.abort('ack_timeout')
        if self.start_sent and now - self.keep_at >= 0.2:
            self.keep_at = now
            return ['KEEP']
        return []


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute-once', action='store_true', required=True)
    parser.parse_args()
    prefix = Path(__file__).parent / ('run_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    ctl, serial = Once(), None
    summary = {'port': 'COM17', 'plan': 20, 'authorized_runs': 1,
               'tx': [], 'valid_frames': 0, 'invalid_frames': 0}
    last = None
    with prefix.with_suffix('.raw').open('wb') as raw, prefix.with_suffix('.jsonl').open('w', encoding='utf-8') as log:
        def event(kind, **data):
            log.write(json.dumps({'host_monotonic': time.monotonic(), 'kind': kind, **data}) + '\n')
            log.flush()

        def send(verbs):
            for verb in verbs:
                data = command(verb, ctl.token)
                event('tx_attempt', verb=verb, ascii=data.decode('ascii'))
                summary['tx'].append(verb)
                serial.write(data)
                event('tx_written', verb=verb)
                if verb != 'KEEP':
                    print(verb, ctl.token, flush=True)

        started = time.monotonic()
        drain_until, pending = None, b''
        try:
            serial = WindowsSerial('COM17')
            event('opened', port='COM17')
            while True:
                chunk = serial.read()
                now = time.monotonic()
                if chunk:
                    raw.write(chunk)
                    raw.flush()
                    event('rx_chunk', hex=chunk.hex())
                    pending += chunk
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    event('rx_line', ascii=line.decode('ascii', errors='replace'))
                    try:
                        r = parse_telemetry(line)
                    except (ValueError, UnicodeError) as exc:
                        summary['invalid_frames'] += 1
                        event('parse_error', error=str(exc))
                        continue
                    if r is not None:
                        last = r
                        summary['valid_frames'] += 1
                        event('telemetry', record=r)
                        send(ctl.consume(r, time.monotonic()))
                if len(pending) > 8192:
                    event('unterminated_buffer', hex=pending.hex())
                    pending = b''
                    send(ctl.abort('unterminated_receive_buffer'))
                now = time.monotonic()
                if ctl.result is None and now - started >= 20:
                    send(ctl.abort('maximum_20_seconds'))
                send(ctl.tick(now))
                if ctl.result is not None:
                    if drain_until is None:
                        drain_until = now + 2
                        event('terminal', result=ctl.result, first_failure=ctl.first_failure, record=last)
                    if now >= drain_until:
                        break
                time.sleep(0.005)
        except BaseException as exc:
            event('exception', error=repr(exc))
            if ctl.result is None:
                verbs = ctl.abort('host_exception:' + repr(exc))
                if serial is not None:
                    try:
                        send(verbs)
                    except Exception as stop_exc:
                        event('stop_write_failed', error=repr(stop_exc))
        finally:
            if serial is not None:
                serial.close()
            summary.update(result=ctl.result, first_failure=ctl.first_failure,
                           last=last, pending_hex=pending.hex(), capture_prefix=str(prefix),
                           physical_result='requires user observation')
            prefix.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
            print(json.dumps({'result': ctl.result, 'first_failure': ctl.first_failure,
                              'capture_prefix': str(prefix)}), flush=True)
    return 0 if ctl.result == 'software_complete_vm_may_remain_on' else 1


if __name__ == '__main__':
    sys.exit(main())
