import os, sys
CODE = '/home/orangepi/p2-source-observer-20261106/vision_app/code'
os.chdir(CODE)
sys.path.insert(0, CODE)
sys.argv = ['source_observe.py', '--mode', 'M', '--frames', '2', '--session-dir', '/tmp/obs_m']
import main as app
from project_camera import DisabledSerialLink
app.TurntableSerialLink = DisabledSerialLink
import source_observe
raise SystemExit(source_observe.main())
