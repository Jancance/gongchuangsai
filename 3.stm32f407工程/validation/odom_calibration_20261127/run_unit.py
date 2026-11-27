import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'_speed_upgrade_work_20261125/odom_calibration_replay_20261127'
ARM=Path('D:/Keil_v5/ARM/arm5_compiler/bin')
DEFINES=['-DAPP_CONTINUOUS_MAP_ENABLE=1','-DAPP_MAP_NAV_RESIDUAL_ENABLE=1',
         '-DAPP_ZONE2_NAV_ENABLE=1','-DAPP_ZONE2_NAV_MOTION_ENABLE=1',
         '-DAPP_ZONE2_MISSION_ENABLE=1','-DAPP_ZONE2_POSE_TRACE_ENABLE=1']

def run():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }',encoding='ascii')
    odom=ROOT/'3.stm32f407工程/CHASSIS/USER/chassis_odometry.c'
    kinematics=ROOT/'3.stm32f407工程/CHASSIS/USER/chassis_kinematics.c'
    include=ROOT/'3.stm32f407工程/CHASSIS/USER'
    commands=[
        [str(ARM/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I',str(include),*DEFINES,'-c',str(HERE/'unit.c'),'-o',str(OUT/'unit.o')],
        [str(ARM/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I',str(include),*DEFINES,'-c',str(odom),'-o',str(OUT/'chassis_odometry.o')],
        [str(ARM/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I',str(include),*DEFINES,'-c',str(kinematics),'-o',str(OUT/'chassis_kinematics.o')],
        [str(ARM/'armlink.exe'),'--entry','test_entry','--scatter',str(OUT/'unit.sct'),str(OUT/'unit.o'),str(OUT/'chassis_odometry.o'),str(OUT/'chassis_kinematics.o'),'--library_type=microlib','-o',str(OUT/'unit.axf')],
    ]
    logs=[]
    for cmd in commands:
        p=subprocess.run(cmd,cwd=HERE,capture_output=True,text=True,errors='replace')
        logs.append({'command':cmd,'returncode':p.returncode,'output':p.stdout+p.stderr})
        if p.returncode:
            (OUT/'build.json').write_text(json.dumps(logs,indent=2)+'\n',encoding='utf-8')
            raise RuntimeError(p.stdout+p.stderr)
    (OUT/'build.json').write_text(json.dumps(logs,indent=2)+'\n',encoding='utf-8')
    sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    blob=(OUT/'unit.axf').read_bytes();header=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
    machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
    machine.mem_map(0x08000000,0x100000);machine.mem_map(0x20000000,0x20000)
    machine.mem_write(0x20000000,b'\xa5'*0x20000)
    for i in range(header[12]):
        _,kind,flags,address,offset,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',blob,header[6]+i*header[11])
        if flags&2 and kind==1 and size:machine.mem_write(address,blob[offset:offset+size])
        elif flags&2 and kind==8 and size:machine.mem_write(address,b'\0'*size)
    symbols=subprocess.check_output([str(ARM/'fromelf.exe'),'--text','-s',str(OUT/'unit.axf')],cwd=OUT,text=True,errors='replace')
    def read(name):
        address=int(re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
        return struct.unpack('<I',machine.mem_read(address,4))[0]
    stop=0x080ff000
    machine.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
    machine.emu_start(header[4]|1,stop,count=300000000)
    result={'passed':machine.reg_read(reg.UC_ARM_REG_PC)==stop and machine.reg_read(reg.UC_ARM_REG_R0)==0,
            'distance_selftest_passed':read('distance_selftest_passed')==1,
            'cases':read('odom_cases'),'assertions':read('odom_assertions'),
            'source_sha256':hashlib.sha256(odom.read_bytes()).hexdigest(),
            'kinematics_sha256':hashlib.sha256(kinematics.read_bytes()).hexdigest(),
            'unit_sha256':hashlib.sha256((HERE/'unit.c').read_bytes()).hexdigest(),
            'axf_sha256':hashlib.sha256((OUT/'unit.axf').read_bytes()).hexdigest(),
            'hardware_accessed':False}
    (OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))
    if not result['passed']:raise RuntimeError('actual C odometry replay failed')

if __name__=='__main__':run()
