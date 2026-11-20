"""Offline preparation only. This script does not import or connect hardware."""
from pathlib import Path
import ast,hashlib,json,shutil

R=Path.cwd();E=Path(__file__).resolve().parent;OLD=E.parent/'half-stop-run7'
S=E.parents[1]/'icm-wheel-lateral-study'
EXPECTED='0ebf9e4e5d89e507cafd9e5cd17ce66570ec861804d38535e61ee89bbad874b3'
PREVIOUS='3966ed37a6c2dd85319884fbe22a2c54a610a9bb9e50f46240e79c006bf1113d'
CLIENT='c8d775e8d91904bb9767a16c44e7b6fae33e3abb6453184f753252d999fbfcb1'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
build=json.loads((S/'build_summary.json').read_text('utf8'))
review=json.loads((S/'independent_review.json').read_text('utf8'))
assert review['candidate_axf_sha256']['baseline40_integrated']==EXPECTED
assert not review['blocking_findings']
assert sha(Path(build['builds']['baseline40_integrated']['axf']))==EXPECTED
assert sha(OLD/'p1_serial_motion_powered_done.py')==CLIENT
assert not (E/'authorization.json').exists(),'Preparation exists; inspect, never overwrite live receipts.'
shutil.copyfile(OLD/'p1_serial_motion_powered_done.py',E/'p1_serial_motion_powered_done.py')
bindings=json.loads((OLD/'authorization.json').read_text('utf8'))['all_changed_source_sha256']
for n,d in build['source_sha256'].items():bindings['3.stm32f407工程/CHASSIS/USER/'+n]=d
bindings['3.stm32f407工程/gongxun-f407.uvprojx']=build['project_sha256']
bindings['工创赛/scripts/vehicle_uart_capture.py']=review['parser_sha256']
for p,d in bindings.items():assert sha(R/p)==d,p
auth=dict(scope='One new-ICM observer candidate download, one unified reset, one PLAN8 40cm/s 210cm START, normal powered20s tail, immediate176B read; failure never auto-retries.',
 user_request='可以实测已经放好',site_reply='可以实测已经放好',site_ready=True,
 site_conditions='Same placement/load/clearance and immediately available power-off just confirmed; no changed conditions reported.',
 local_review_passed=True,review_report=str(S/'independent_review.json'),review_report_sha256=sha(S/'independent_review.json'),
 candidate_sha256=EXPECTED,previous_board_candidate_sha256=PREVIOUS,
 all_changed_source_sha256=bindings,downloads_authorized=1,resets_authorized=1,starts_authorized=1,tail_reads_authorized=1,
 automatic_retry=False,client_sha256=CLIENT,normal_completion_keeps_VM=True,
 reference_policy='Fresh current HWT for this run after unified reset; no reset-crossing yaw reuse.',
 reset_policy='Download once, then explicit reset once; immediate176B read after20s normal powered tail; notify operator only when read complete.',
 tail_address='0x20012f18',tail_size=176,hardware_go_required=True,
 hardware_go_policy='Wait for explicit root HARDWARE_GO and matching hardware_go.json with saved checkpoint commit before any hardware enumeration or connection.')
(E/'authorization.json').write_text(json.dumps(auth,ensure_ascii=False,indent=2),'utf8')
for name in ['reset_once.py','run_once.py','run_and_read_once.py','read_stop_distance_once.py']:
    text=(OLD/name).read_text('utf8')
    text=text.replace(PREVIOUS,EXPECTED).replace('tmp/half-stop-observer-build-20261115','tmp/icm-observer-build-20261115')
    text=text.replace("R / '工创赛/evidence/navigation-planning/2026-11-15/rollback-half-ff-20261115/stop-distance-observer/deployment/download_once_receipt.json'","E / 'download_once_receipt.json'")
    text=text.replace('0x20012d84','0x20012f18')
    if name=='reset_once.py':
        text=text.replace("R = Path(__file__).resolve().parents[6]","R = Path(__file__).resolve().parents[6]\nfrom run_binding import validate\nvalidate()")
        text=text.replace("session = None\ntry:","with (E / 'reset_once_started.json').open('x', encoding='utf8') as stream:\n    json.dump(dict(candidate_sha256=expected, attempts=1, reset_authorized=1, automatic_retry=False),stream)\nsession = None\ntry:")
    elif name=='run_once.py':
        text=text.replace("R = Path(__file__).resolve().parents[6]","R = Path(__file__).resolve().parents[6]\nfrom run_binding import validate\nvalidate()")
    elif name=='read_stop_distance_once.py':
        text=text.replace("def main():\n","def main():\n    from run_binding import validate\n    validate()\n",1)
        text=text.replace("        after = read_state()", "        receipt['icm_statistics'] = {name: target.read32(address) for name,address in stats_symbols.items()}\n        receipt['icm_statistics_note'] = 'One running-core read; stack word count is runtime watermark, not free heap or timing guarantee.'\n        after = read_state()")
        text=text.replace("    wanted = set(STATE_NAMES)","    stats_symbols = {}\n    wanted = set(STATE_NAMES)",1)
        text=text.replace("        table = elf.get_section_by_name('.symtab')","        table = elf.get_section_by_name('.symtab')\n        stats_symbols = {s.name: int(s['st_value']) for s in table.iter_symbols() if s.name in ['g_icm_stack_words','g_icm_publish_sequence'] and int(s['st_size'])==4}")
        text=text.replace("    parsers = {","    require(set(stats_symbols)=={'g_icm_stack_words','g_icm_publish_sequence'}, 'ICM statistics missing')\n    require(all(0x20000000<=a<0x20020000 for a in stats_symbols.values()),'ICM stats outside RAM')\n    parsers = {",1)
    elif name=='run_and_read_once.py':
        text=text.replace("E = Path(__file__).resolve().parent","E = Path(__file__).resolve().parent\nfrom run_binding import validate\nvalidate()")
    assert PREVIOUS not in text,name
    ast.parse(text)
    (E/name).write_text(text,'utf8')
print('Offline preparation completed; hardware_go.json deliberately absent.')
