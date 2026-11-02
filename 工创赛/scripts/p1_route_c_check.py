"""Run actual firmware route conversion/validation/dispatch in ARM emulation, no IO."""
import argparse
import importlib
import json
from pathlib import Path
import random
import shutil
import struct
import subprocess
import sys

from p1_route_preview import USER, KINDS, f32, validate


def extract(text, name):
    prefix = 'static uint32_t ' if 'static uint32_t ' + name + '(' in text else 'uint32_t '
    start = text.index(prefix + name + '(')
    opening = text.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def run(out, arm_bin, unicorn_path=None):
    if unicorn_path:
        sys.path.insert(0, str(unicorn_path))
    uc = importlib.import_module('unicorn')
    reg = importlib.import_module('unicorn.arm_const')
    out.mkdir(parents=True, exist_ok=False)
    random.seed(2810)
    cases=[]
    for kind in KINDS:
        for amount in [-100, -90, -5, 0, 5, 10, 30, 90, 100, 101, float('nan')]:
            for speed in [0, 1.2, 3, 5, 20, float('inf'), float('nan')]:
                cases.append((kind,f32(amount),f32(speed)))
        for _ in range(40):
            cases.append((kind,f32(random.uniform(-100,110)),f32(random.uniform(-1,21))))
    source=(Path(__file__).parent/'p1_offline_tests/route_config.c').read_text()
    main=(USER/'main.c').read_bytes().decode('gbk')
    functions='\n'.join(extract(main,name) for name in ['App_MotionPlanValid','App_CombinedLoadAction'])
    source=source.replace('/* LIVE_FUNCTIONS */', '#if APP_TEST_MODE==3U\n'+functions+'\n#endif')
    reports=[]
    for name,custom,entry,mode,invalid in [('default',0,1,3,0),('custom',1,1,3,0),
                                        ('invalid',1,1,3,1),('competition',1,2,3,0),('single',1,1,2,0)]:
        directory=out/name;directory.mkdir();src=directory/'src';src.mkdir()
        matrix=[]
        for kind, amount, speed in cases:
            expected=validate([(kind,amount,speed)],(77,212,206),30000 if mode==3 else 20000)
            matrix.append([KINDS[kind],struct.unpack('<I',struct.pack('<f',amount))[0],
                           struct.unpack('<I',struct.pack('<f',speed))[0],expected['error'],
                           expected['rows'][0]['rpm'] if expected['ok'] else 0])
        variant_source=source.replace('/* PARITY_CASES */','static const uint32_t parity[][5]={\n'+
                              ',\n'.join('{'+','.join(str(v)+'U' for v in row)+'}' for row in matrix)+'\n};')
        for filename in ['route_config.h','route_plan.h','motion_action_plan.h','chassis_kinematics.h',
                         'chassis_kinematics.c','app_config.h','app_entry_config.h','app_entry.h',
                         'app_entry.c','motion_trial20.h','motion_distance_limits.h','motion_control.h','motion_route_trial.h',
                         'motion_profile.h','motion_profile.c']:
            shutil.copyfile(USER/filename,src/filename)
        config=src/'route_config.h'
        blob=config.read_bytes().replace(b'APP_ROUTE_USE_CUSTOM 0',f'APP_ROUTE_USE_CUSTOM {custom}'.encode())
        if invalid:blob=blob.replace(b'30.0f, 5.0f',b'30.0f, 20.0f')
        config.write_bytes(blob)
        (directory/'unit.c').write_bytes(variant_source.encode('gbk'))
        (directory/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
        commands=[]
        for c,obj in [('unit.c','unit.o'),('src/chassis_kinematics.c','kinematics.o'),('src/motion_profile.c','profile.o')]:
            commands.append([str(arm_bin/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','src',
                             f'-DAPP_ENTRY={entry}',f'-DAPP_DEBUG_MODE={mode}',f'-DEXPECT_INVALID={invalid}','-c',c,'-o',obj])
        commands.append([str(arm_bin/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','kinematics.o','profile.o','--library_type=microlib','-o','unit.axf'])
        logs=[]
        for command in commands:
            p=subprocess.run(command,cwd=directory,capture_output=True,text=True,errors='replace',timeout=60)
            logs.append({'command':command,'returncode':p.returncode,'stdout':p.stdout,'stderr':p.stderr})
            (directory/'build.json').write_text(json.dumps(logs,indent=2))
            if p.returncode:raise RuntimeError(p.stdout+p.stderr)
        b=(directory/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
        machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
        machine.mem_map(0x08000000,0x100000);machine.mem_map(0x20000000,0x20000)
        for i in range(h[10]):
            kind,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
            if kind==1 and size:machine.mem_write(address,b[offset:offset+size])
        stop=0x080FF000
        machine.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
        machine.emu_start(h[4]|1,stop,count=10000000)
        result=machine.reg_read(reg.UC_ARM_REG_R0)
        ok=machine.reg_read(reg.UC_ARM_REG_PC)==stop and not result&0x80000000
        report={'variant':name,'ok':ok,'assertions':result if ok else None,'failure_line':result&0x7fffffff if not ok else None,
                'parity_cases':len(matrix),'hardware_accessed':False}
        reports.append(report);(out/'checks.json').write_text(json.dumps(reports,indent=2))
        print(report,flush=True)
        if not ok:raise RuntimeError(f'{name} actual C failed: {report}')
    return reports


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--arm-bin',type=Path,default=Path(r'D:\Keil_v5\ARM\arm5_compiler\bin'))
    p.add_argument('--unicorn-path',type=Path)
    a=p.parse_args();run(a.out,a.arm_bin,a.unicorn_path)
