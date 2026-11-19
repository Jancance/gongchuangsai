"""Compile the actual Stage3 recovery tail and IT3 grip completion logic."""
from pathlib import Path
import subprocess
import tempfile


WORK = Path(__file__).resolve().parent
CANDIDATE = WORK / 'stage3_anticipatory_handoff40_20261120'
USER = CANDIDATE / 'project/CHASSIS/USER'


def main():
    stage = (USER / 'stage3_cycle.inc').read_text(encoding='gbk')
    commands = stage[stage.index('static void S3_Command(uint32_t action)'):
                     stage.index('/* Scoped permission for the dedicated')]
    start = stage.index(' if(g_stage3_state==5U){\n  uint32_t p=')
    switch = stage.index('  switch(p){', start)
    prologue = stage[start:switch].replace(
        'p=g_stage3_phase,target=g_stage3_item<3U?s3_positions[g_stage3_item]-1U:0U;',
        'p=g_stage3_phase;')
    recovery = stage[stage.index('  case 22U:', switch):stage.rfind('\n}')]
    placement = stage[stage.index('  case S3_SYNC_RETURN_PHASE:', switch):stage.index('  case 13U:', switch)]
    header = (USER / 'stage3_cycle.h').read_text(encoding='latin1')
    offset = next(line for line in header.splitlines() if line.startswith('#define S3_FAST_HANDOFF_OFFSET_PULSES '))
    it = (USER / 'integration10_complete.inc').read_text(encoding='latin1')
    start = it.index(' case 5U:case 6U:case 17U:\n  /* User-selected')
    grip = it[start:it.index(' case 7U:', start)]
    assert 'if(jiazhua(request==5U?140.0f:(request==17U?160.0f:210.0f))' in it
    assert 'g_it_lift_pulses!=s3_depth[s3_positions[g_stage3_item]-1U]-S3_FAST_HANDOFF_OFFSET_PULSES' in it
    template = (WORK / 'stage3_handoff40_upper_fixture.c').read_text(encoding='ascii')
    fixture = template.replace('/* EXTRACT_COMMANDS */', commands).replace(
        '/* EXTRACT_RECOVERY */', prologue + '  switch(p){\n' + recovery).replace(
        '/* EXTRACT_GRIP */', grip).replace('/* EXTRACT_OFFSET */', offset).replace(
        '/* EXTRACT_PLACEMENT */', placement)
    with tempfile.TemporaryDirectory(dir=WORK) as scratch:
        source = Path(scratch) / 'stage3_handoff40.c'
        exe = source.with_suffix('.exe')
        source.write_text(fixture, encoding='gbk')
        subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', str(source), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
