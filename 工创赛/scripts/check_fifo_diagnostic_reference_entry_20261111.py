"""Exercise the actual reference/config branch with injected I/O; no devices."""
from pathlib import Path
import ast
import copy
import json

R = Path(__file__).resolve().parents[1]
path = R / 'scripts/pickup_fifo_diagnostic_live_20261111.py'
tree = ast.parse(path.read_text(encoding='utf-8'))
fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run')
branch = next(n for n in ast.walk(fn) if isinstance(n, ast.If) and isinstance(n.test, ast.Name) and n.test.id == 'reference_config_only')
test_fn = ast.FunctionDef(name='execute_branch', args=ast.arguments(posonlyargs=[], args=[], kwonlyargs=[], kw_defaults=[], defaults=[]), body=copy.deepcopy(branch.body), decorator_list=[])
code = compile(ast.fix_missing_locations(ast.Module(body=[test_fn], type_ignores=[])), 'actual-reference-branch', 'exec')
checks = 0


def execute(state_change=None, diag_change=None, healthy=True):
    state = dict(pe2=0, g_it_state=0, g_it_auto_state=0, g_lift_ruler_state=0, g_startup_axes_state=0)
    if state_change:
        state.update(state_change)
    diag = dict(state=2, kind=2, commands=[0x1f, 0x21, 0x24, 0x26, 0x27, 0x42, 0x43], results=[1]*7, decode_mask=127, host_wait_timed_out=False)
    if diag_change:
        diag.update(diag_change)
    calls, report = [], {}
    namespace = dict(s=state, health=lambda _: healthy, report=report,
                     cmd=lambda *a, **kw: calls.append((a, kw)),
                     request_ext_diag=lambda *a, **kw: calls.append((a, kw)),
                     request_ext_config=lambda _: diag)
    exec(code, namespace)
    rejected = False
    try:
        namespace['execute_branch']()
    except AssertionError:
        rejected = True
    return rejected, calls, report


rejected, calls, report = execute()
assert not rejected and report['complete'] and not report['three_piece_cycle_started']
assert [call[0][1] for call in calls[:3]] == [1, 2, 6]
assert len(calls) == 4 and calls[-1][1] == {'strict': True}
checks += 4
for field in ('pe2', 'g_it_state', 'g_it_auto_state', 'g_lift_ruler_state', 'g_startup_axes_state'):
    rejected, calls, _ = execute({field: 1})
    assert rejected and not calls
    checks += 1
rejected, calls, _ = execute(healthy=False)
assert rejected and not calls
checks += 1
for change in ({'state': 3}, {'kind': 1}, {'decode_mask': 31}, {'results': [1, 1, 1, 1, 1, 6, 6]}, {'host_wait_timed_out': True}, {'commands': [0]*7}):
    rejected, _, report = execute(diag_change=change)
    assert not rejected and not report['complete'] and not report['three_piece_cycle_started']
    checks += 1
cli = tree.body[-1]
modes = [n.args[0].value for n in ast.walk(cli) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'add_argument' and n.args and isinstance(n.args[0], ast.Constant)]
assert sorted(modes) == sorted(['--check', '--read-only', '--driver-config-only', '--references-and-config-once'])
checks += 1
e = R / 'evidence/integration/2026-11-11/driver-fifo-fix'
(e / 'reference-entry-checks.json').write_text(json.dumps(dict(passed=True, checks=checks, actual_branch_exercised=True, diagnostic_success_not_assumed=True, no_pickup_cli=True, hardware_access=False), indent=2)+'\n', encoding='utf-8')
print('PASS', checks, 'actual reference/config branch checks; no hardware access.')
