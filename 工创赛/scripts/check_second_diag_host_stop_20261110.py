"""Fault-inject the actual runner's exception/finally handlers; no devices."""
from pathlib import Path
import ast,json,types

R=Path(__file__).resolve().parents[1]
source=ast.parse((R/'scripts/pickup_second_diag_live_20261110.py').read_text(encoding='utf-8'))
fn=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='run')
original=next(n for n in ast.walk(fn) if isinstance(n,ast.Try) and any(h.name=='exc' for h in n.handlers))
body=ast.parse("raise RuntimeError('original failure')").body
statement=ast.Try(body=body,handlers=original.handlers,orelse=[],finalbody=original.finalbody)
code=compile(ast.fix_missing_locations(ast.Module(body=[statement],type_ignores=[])),'actual-stop-handler','exec')
checks=0
for started,read_fails,write_fails,save_fails in [(True,True,False,False),(True,True,True,True),(True,False,False,True),(False,True,False,True)]:
    calls=[];report={'complete':False,'flash_bytes':0,'sha256':'test'}
    def wr(name,value):
        calls.append(('write',name,value))
        if write_fails:raise OSError('write unavailable')
    def snap():
        calls.append(('read',))
        if read_fails:raise OSError('read unavailable')
        return {'g_it_request':0,'g_source_parallel_stop_mask':0,'g_lift_ruler_stop_pending':0,'g_vehicle_arm_stop_pending':0}
    def save():
        calls.append(('save',))
        if save_fails:raise OSError('disk unavailable')
    ns={'writes_started':started,'report':report,'wr':wr,'snap':snap,'save':save,'json':json,
        'time':types.SimpleNamespace(sleep=lambda _:None),'capture_ext_ring':lambda:{'mocked':True},
        'request_ext_diag':lambda label:calls.append(('diag',label)),'dest':'mock-log','print':lambda *a,**k:None}
    try:exec(code,ns)
    except RuntimeError as exc:assert str(exc)=='original failure';checks+=1
    else:raise AssertionError('Original exception was lost')
    writes=[v for v in calls if v[0]=='write']
    if started:
        assert calls[0]==('write','g_it_request',9) and len(writes)==1
    else:assert not writes
    checks+=1
    if started and not write_fails:assert report['stop_request_sent'];checks+=1
    if write_fails:assert 'stop_request_error' in report;checks+=1
E=R/'evidence/integration/2026-11-10/second-diag';E.mkdir(parents=True,exist_ok=True)
(E/'host-stop-checks.json').write_text(json.dumps({'passed':True,'checks':checks,'actual_handlers_exercised':True,'hardware_access':False},indent=2)+'\n')
print('PASS',checks,'actual host stop-path fault-injection checks; no device access.')
