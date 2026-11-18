from pathlib import Path

here = Path(__file__).resolve().parent
runner = here / 'run_zone2_front_replay.py'
source = runner.read_text(encoding='utf-8')
start = source.index('ns.update(UNIT=')
end = source.index('sys.argv =', start)
source = source[:start] + (
    "fixture = here / 'independent_vectors.c'\n"
    "ns.update(UNIT=fixture.read_text(encoding='utf-8'), OUT=output, FW=fw)\n"
) + source[end:]
exec(compile(source, str(runner), 'exec'), {'__file__': str(runner)})
