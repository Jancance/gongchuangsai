"""Offline hash checks; hardware callers additionally require root's GO record."""
from pathlib import Path
import hashlib,json,subprocess
E=Path(__file__).resolve().parent;R=E.parents[5];S=E.parents[1]/'icm-wheel-lateral-study'
EXPECTED='0ebf9e4e5d89e507cafd9e5cd17ce66570ec861804d38535e61ee89bbad874b3'
IMAGE=R/'工创赛/tmp/icm-observer-build-20261115/baseline40_integrated/Vehicle_Integrated.axf'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def validate(require_go=True):
    auth=json.loads((E/'authorization.json').read_text('utf8'))
    assert auth['site_ready'] and auth['local_review_passed'] and auth['candidate_sha256']==EXPECTED
    assert auth['downloads_authorized']==0
    assert auth['resets_authorized']==auth['starts_authorized']==auth['tail_reads_authorized']==1
    download=json.loads(Path(auth['download_receipt']).read_text('utf8'))
    assert download['success'] and download['candidate_unchanged'] and download['candidate_sha256']==EXPECTED
    assert not auth['automatic_retry'] and sha(IMAGE)==EXPECTED
    for path,digest in auth['all_changed_source_sha256'].items():assert sha(R/path)==digest,path
    assert sha(E/'p1_serial_motion_powered_done.py')==auth['client_sha256']
    assert sha(Path(auth['review_report']))==auth['review_report_sha256']
    review=json.loads(Path(auth['review_report']).read_text('utf8'))
    assert review['status'].startswith('PASS_') and not review['blocking_findings']
    assert review['candidate_axf_sha256']['baseline40_integrated']==EXPECTED
    if require_go:
        go=json.loads((E/'hardware_go.json').read_text('utf8'))
        assert go['hardware_go'] is True and go['candidate_sha256']==EXPECTED
        assert go['checkpoint_commit'].startswith('8387f076')
        commit=subprocess.check_output(['git','rev-parse','--verify',go['checkpoint_commit']+'^{commit}'],cwd=R,text=True).strip()
        assert commit.startswith('8387f076')
        subprocess.run(['git','cat-file','-e',commit+':工创赛/evidence/navigation-planning/2026-11-15/icm-wheel-lateral-study/checkpoint_sources.json'],cwd=R,check=True,capture_output=True)
    return auth
