"""Final route+startup+source-prep chain; device/DM/PWM/scheduling remain modeled."""
from pathlib import Path
import os
R=Path(__file__).resolve().parents[1]
(R/'evidence/integration/2026-11-04/source-parallel/whole-stubs').mkdir(parents=True,exist_ok=True)
(R/'evidence/integration/2026-11-04/source-parallel/whole-stubs/FreeRTOS.h').write_text('#include <stdint.h>\n')
os.environ.setdefault('STAGE12_SOURCE_ROOT',str(R/'tmp/source_parallel_20261104'))
os.environ.setdefault('STAGE12_WHOLE_OUT',str(R/'evidence/integration/2026-11-04/source-parallel/whole'))
source=(R/'scripts/check_stage12_chain_20261104.py').read_text(encoding='utf-8')
source=source.replace("unit=prefix+r'''", "from stage12_admission_fixture import inject,extend\nprefix=inject(prefix,S)\nunit=prefix+r'''")
source=source.replace("p=E/'whole-unit.c';", "from source_parallel_whole_fixture import adapt\nunit=adapt(extend(unit))\np=E/'whole-unit.c';")
source=source.replace('cases=40,','cases=74,')
source=source.replace('f=subprocess.run(cmd,capture_output=True',"cmd[1:1]=['-I'+str(S/'HARDWARE/DM_CAN2'),'-I'+str(S/'HARDWARE/GRIPPER'),'-I'+str(S/'HARDWARE/CARRIER'),'-I'+str(R/'evidence/integration/2026-11-04/source-parallel/whole-stubs')]\nf=subprocess.run(cmd,capture_output=True",1)
source=source.replace('910','860').replace('2110','2060').replace('GOAL_OFFSET_X_MM==-50','GOAL_OFFSET_X_MM==-100')
source=source.replace("(E/'result.json').write_text", "report['actual_source_preparation_and_arm']=True\nreport['source_sha256'].update({n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in ['CHASSIS/USER/source_prepare.inc','CHASSIS/USER/source_prepare_math.h','CHASSIS/USER/vehicle_arm.c','HARDWARE/DM_CAN2/dm_can2_motion.c']})\n(E/'result.json').write_text")
exec(compile(source,str(R/'scripts/check_stage12_chain_20261104.py'),'exec'))
