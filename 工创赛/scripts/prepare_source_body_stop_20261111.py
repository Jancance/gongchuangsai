"""Preserve flashed body-step source; isolate only the STOP retry repair."""
from pathlib import Path
import hashlib
import json
import shutil

R = Path(__file__).resolve().parents[1]
B = R / 'tmp/source_body_step_20261111/project'
P = R / 'tmp/source_body_stop_20261111/project'
E = R / 'evidence/integration/2026-11-11/source-body-stop'


def main():
    proof = json.loads((E / 'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed'] and proof['engine'] == 'local MATLAB'
    old_manifest = json.loads((R / 'evidence/integration/2026-11-11/source-body-step/candidate.json').read_text())
    for rel, sha in old_manifest['changes'].items():
        assert hashlib.sha256((B / rel).read_bytes()).hexdigest() == sha, rel
    assert not P.exists(), 'Preserve an existing candidate; never overwrite it.'
    shutil.copytree(B, P, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    rel = 'CHASSIS/SYSTEM/SAFETY/safety.c'
    path = P / rel
    data = path.read_bytes()
    data.decode('gb18030')
    assert data.count(b'\n') == data.count(b'\r\n'), 'Preserve GBK/CRLF.'
    old = '''    /* Never cut VM: keep re-sending stop frames until all four wheels ack. */
    if (s_auto_stop_retries < 20U)
    {
        s_auto_stop_retries++;
        g_motion_auto_stop_state = MOTION_STOP_PENDING;
        g_motion_auto_stop_fail_mask = 0U;
        s_auto_stop_pending = 0x0FU & ~g_motion_auto_stop_ack_mask;
        s_auto_stop_inflight = 0U;
        s_auto_stop_start_tick = xTaskGetTickCount();
        if (s_auto_stop_pending == 0U) g_motion_auto_stop_state = MOTION_STOP_ACKED;
    }
    else
    {
        g_motion_auto_stop_state = MOTION_STOP_ACKED;
        s_auto_stop_pending = 0U;
    }'''
    new = '''    /* VM stays on. Saturate diagnostics, never manufacture an ACK.
     * Retry only missing wheels at the existing 500 ms round deadline. */
    if (s_auto_stop_retries < 20U) s_auto_stop_retries++;
    s_auto_stop_pending = 0x0FU & ~g_motion_auto_stop_ack_mask;
    s_auto_stop_inflight = 0U;
    s_auto_stop_start_tick = xTaskGetTickCount();
    g_motion_auto_stop_state = (s_auto_stop_pending == 0U) ?
        MOTION_STOP_ACKED : MOTION_STOP_PENDING;'''
    old_report = '''            /* Retry this address; VM stays on. */
            Safety_MotionStopCut(1U);'''
    new_report = '''#if APP_FAULT_CUT_VM_ENABLE
            Safety_MotionStopCut(1U);
#else
            /* Finish this round before retrying: a failed low address must
             * not starve STOP for the other wheels. Preserve failure bits. */
            g_motion_auto_stop_cause = 1U;
#endif'''
    for before, after in [(old, new), (old_report, new_report)]:
        before = before.replace('\n', '\r\n').encode('ascii')
        after = after.replace('\n', '\r\n').encode('ascii')
        assert data.count(before) == 1, 'Expected exact original STOP implementation.'
        data = data.replace(before, after, 1)
    path.write_bytes(data)
    changed = {}
    for f in P.rglob('*'):
        if f.is_file() and f.read_bytes() != (B / f.relative_to(P)).read_bytes():
            changed[f.relative_to(P).as_posix()] = hashlib.sha256(f.read_bytes()).hexdigest()
    assert set(changed) == {rel}
    (E / 'candidate.json').write_text(json.dumps(dict(
        base=B.relative_to(R).as_posix(), project=P.relative_to(R).as_posix(),
        base_online_sha256=old_manifest['images']['Vehicle_LiftRuler']['sha256'],
        changes=changed, hardware_access=False, flashed=False, reset=False,
        mechanical_scheduler_unchanged=True, all_other_source_bytes_unchanged=True,
        stop_ack_requires_mask=15, stop_round_ms=500, fault_history_preserved=True,
        matlab_prior_to_implementation=True), indent=2) + '\n', encoding='utf-8')
    print('Prepared isolated STOP repair: only safety.c changed; GBK/CRLF and flashed source preserved.')


if __name__ == '__main__':
    main()
