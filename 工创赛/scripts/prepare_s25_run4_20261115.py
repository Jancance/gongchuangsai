"""Reuse the previous one-shot package preparation for the explicit fourth request."""
from pathlib import Path

original = Path.cwd() / '工创赛/scripts/prepare_s25_run3_20261115.py'
text = original.read_text(encoding='utf8')
before = "['s25-run1', 's25-run2', 's25-run3']"
after = "['s25-run1', 's25-run3', 's25-run4']"
assert text.count(before) == 1
text = text.replace(before, after).replace('third', 'fourth').replace('Third', 'Fourth')
exec(compile(text, str(original), 'exec'), {'__name__': '__main__', '__file__': __file__})
