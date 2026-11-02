"""Offline 90-degree speed comparison. No serial, flashing or vehicle commands."""
import argparse
import csv
import hashlib
import importlib
import json
from pathlib import Path
import re
import shutil
import statistics
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
USER = ROOT / '3.stm32f407工程/USER'
LABELS = ['current_5rpm', 'direct_27rpm', 'profile30_prototype']
FIELDS = ['variant', 'lag_ms', 'tau_ms', 'gain', 'direction', 'state', 'done_s',
          'final_error_deg', 'overshoot_deg', 'peak_rate_dps', 'peak_accel_dps2',
          'peak_jerk_dps3', 'max_command_jump_rpm', 'reversals', 'steps']


def run(out, dependencies, plots, prototype_speed=30, noise_deg=0.0, sensor_lag_ms=20, session_trial=False):
    out.mkdir(parents=True, exist_ok=False)
    labels = ['current_5rpm','direct_27rpm',f'brake_{prototype_speed}dps_prototype']
    src = out / 'src'; src.mkdir()
    hashes = {}
    for name in ['heading_controller.c', 'heading_controller.h', 'motion_profile.c',
                 'motion_profile.h', 'chassis_kinematics.h', 'chassis_kinematics.c',
                 'app_config.h', 'app_entry_config.h', 'route_config.h',
                 'motion_trial20.h', 'motion_route_trial.h'] + (['heading_session.c','heading_session.h','turn_brake.h','route_plan.h','motion_action_plan.h'] if session_trial else []):
        blob = (USER / name).read_bytes()
        hashes[name] = hashlib.sha256(blob).hexdigest()
        if name == 'heading_controller.c':
            assert b'#define HEADING_CONTROLLER_MAX_RPM 10U' in blob
            # Isolated what-if only. Live firmware's max remains untouched.
            blob = blob.replace(b'#define HEADING_CONTROLLER_MAX_RPM 10U',
                                b'#define HEADING_CONTROLLER_MAX_RPM 30U')
        if session_trial and name=='route_config.h':
            blob=blob.replace(b'APP_ROUTE_USE_CUSTOM 0',b'APP_ROUTE_USE_CUSTOM 1')
        (src / name).write_bytes(blob)
    shutil.copyfile(Path(__file__).parent / ('p1_offline_tests/turn_session.c' if session_trial else 'p1_offline_tests/turn_speed.c'), out / 'unit.c')
    if session_trial: shutil.copyfile(Path(__file__).parent/'p1_offline_tests/turn_session_guards.h',src/'turn_session_guards.h')
    (out / 'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
    arm = Path('D:/Keil_v5/ARM/arm5_compiler/bin')
    commands = [[str(arm/'armcc.exe'), '--cpu', 'Cortex-M3', '--c99', '--split_sections',
                 '-O1', '-I', 'src', f'-DTURN_SIM_SPEED_DPS={prototype_speed}.0f', f'-DTURN_SIM_NOISE_DEG={noise_deg}f', f'-DTURN_SIM_SENSOR_TICKS={sensor_lag_ms//10}U', '-c', source, '-o', obj] for source,obj in
                [('unit.c','unit.o'),('src/heading_controller.c','heading.o'),
                 ('src/motion_profile.c','profile.o'), ('src/chassis_kinematics.c','kinematics.o')]]
    if session_trial:
        for cmd in commands: cmd.insert(1,'-DAPP_TURN_BRAKE_TRIAL_ENABLE=1')
        for cmd in commands: cmd.insert(1,f'-DAPP_TURN_BRAKE_TRIAL_DPS={prototype_speed}U')
        commands.append([str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','src','-DAPP_TURN_BRAKE_TRIAL_ENABLE=1',f'-DAPP_TURN_BRAKE_TRIAL_DPS={prototype_speed}U','-c','src/heading_session.c','-o','session.o'])
    commands.append([str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',
                     'unit.o','heading.o','profile.o','kinematics.o','--library_type=microlib','-o','unit.axf'])
    if session_trial: commands[-1].insert(-2,'session.o')
    logs = []
    for cmd in commands:
        p = subprocess.run(cmd, cwd=out, capture_output=True, text=True, errors='replace', timeout=60)
        logs.append(dict(command=cmd, returncode=p.returncode, stdout=p.stdout, stderr=p.stderr))
        (out/'build.json').write_text(json.dumps(logs,indent=2))
        if p.returncode: raise RuntimeError(p.stdout+p.stderr)
    sys.path.insert(0,str(dependencies.resolve()))
    uc = importlib.import_module('unicorn'); reg = importlib.import_module('unicorn.arm_const')
    machine = uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
    machine.mem_map(0x08000000,0x100000); machine.mem_map(0x20000000,0x20000)
    blob=(out/'unit.axf').read_bytes(); h=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
    for i in range(h[10]):
        kind,offset,address,_,size,_,_,_=struct.unpack_from('<IIIIIIII',blob,h[5]+i*h[9])
        if kind==1 and size:machine.mem_write(address,blob[offset:offset+size])
    symbols=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True)
    def address(name):
        found=re.search(r'\b'+name+r'\s+(0x[0-9a-fA-F]+)',symbols)
        if not found: raise RuntimeError('missing symbol '+name)
        return int(found[1],16)
    traces=[]; trace_address=address('trace_values'); hook_address=address('TurnTrace')&~1
    def sample(m,pc,size,data):
        traces.append(struct.unpack('<12f',m.mem_read(trace_address,48)))
    machine.hook_add(uc.UC_HOOK_CODE,sample,begin=hook_address,end=hook_address)
    stop=0x080ff000;machine.reg_write(reg.UC_ARM_REG_SP,0x2001fff0)
    machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
    machine.emu_start(h[4]|1,stop,count=3000000000,timeout=180000000)
    if machine.reg_read(reg.UC_ARM_REG_PC)!=stop:raise RuntimeError('simulation budget exhausted')
    result=machine.reg_read(reg.UC_ARM_REG_R0)
    if result:raise RuntimeError('simulation harness failed '+hex(result))
    rows=[dict(zip(FIELDS,row)) for row in struct.iter_unpack('<15f',machine.mem_read(address('results'),162*15*4))]
    for row in rows:row['name']=labels[int(row['variant'])]
    for filename,fields,values in [('cases.csv',list(rows[0]),rows),
         ('traces.csv',['case_id','variant','time_s','angle_deg','rate_dps','command_rpm',
                        'error_deg','state','reference_deg','planned_speed_dps',
                        'peak_accel_dps2','peak_overshoot_deg'],traces)]:
        with (out/filename).open('w',newline='',encoding='utf8') as f:
            if isinstance(values[0],dict):
                writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(values)
            else:
                writer=csv.writer(f);writer.writerow(fields);writer.writerows(values)
    summaries=[]
    for variant,name in enumerate(labels):
        group=[row for row in rows if int(row['variant'])==variant]
        summaries.append(dict(name=name,cases=len(group),completed=sum(row['state']==1 for row in group),
             median_done_s=statistics.median(row['done_s'] for row in group),
             worst_overshoot_deg=max(row['overshoot_deg'] for row in group),
             worst_final_error_deg=max(abs(row['final_error_deg']) for row in group),
             max_command_jump_rpm=max(row['max_command_jump_rpm'] for row in group),
             worst_accel_dps2=max(row['peak_accel_dps2'] for row in group),
             max_reversals=max(row['reversals'] for row in group),
             peak_rate_range_dps=[min(row['peak_rate_dps'] for row in group),max(row['peak_rate_dps'] for row in group)]))
    unchanged=all(hashlib.sha256((USER/name).read_bytes()).hexdigest()==sha for name,sha in hashes.items())
    report=dict(hardware_accessed=False,session_trial=session_trial,live_source_unchanged=unchanged,source_sha256=hashes,
                summaries=summaries,case_count=len(rows),trace_count=len(traces),
                assumptions={'dt_ms':10,'extra_command_lag_ms':[0,80,160],
                             'per_wheel_delivery_ms':[0,20,40,60],'batch_period_ms':70,
                             'motor_tau_ms':[80,200,400],'speed_gain':[0.9,1,1.1],
                             'sensor_lag_ms':sensor_lag_ms,'sensor_yaw_noise_deg':noise_deg,'sensor_rate_noise_dps':0.5 if noise_deg else 0.0,'directions':['CCW','CW'],
                             'timeout_ms':30000,'prototype_speed_ceiling':prototype_speed,
                             'prototype_brake_accel_dps2':15,'prototype_jerk_slew_dps3':60,
                             'prototype_lookahead_s':0.7,'prototype_delay_allowance_s':0.8,
                             'prototype_feedback_filter_ms':80,'settle_entry_deg':0.8,'settle_exit_deg':0.9},
                limits=['Controller C is live math; direct27 raises only an isolated validation ceiling.',
                        ('Integrated HeadingSession and TurnBrake with simulated backend; not full RTOS/CAN runtime.' if session_trial else 'Test-only remaining-angle braking prototype.'),
                        'Speed clipping/settling and integer RPM quantization can introduce reference-jerk steps; no strict physical jerk proof.',
                        'No RTOS/CAN/VM/heading-session guards or load/slip/electrical limits are simulated.',
                        'Acceleration/jerk depend on the assumed first-order plant and10ms sampling.',
                        'Models do not prove physical safety, smoothness, actual angular speed or stop accuracy.'])
    (out/'report.json').write_text(json.dumps(report,indent=2))
    if not unchanged:raise RuntimeError('live source changed during simulation')
    sys.path.insert(0,str(plots.resolve()))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,3,figsize=(14,7),layout='constrained')
    case_ids=sorted({int(t[0]) for t in traces})
    for case_id in case_ids:
        data=[t for t in traces if int(t[0])==case_id];variant=int(data[0][1]);stress=rows[case_id]['tau_ms']==400
        for col,value in enumerate([3,4,5]):
            ax=axes[int(stress)][col];ax.plot([t[2] for t in data],[t[value] for t in data],label=labels[variant],lw=1.4)
    for index,row in enumerate(axes):
        for col,ax in enumerate(row):
            ax.set(xlabel='Time (s)',ylabel=['Body angle (deg)','Body angular speed (deg/s)','Wheel target (RPM)'][col],
                   title=('Higher inertia /160ms added lag' if index else 'Nominal /80ms added lag'))
            ax.grid(alpha=.25);ax.legend(fontsize=8)
            if col==0:ax.axhline(90,color='black',ls='--',lw=.8)
    fig.suptitle('Offline 90-degree turn comparison: assumed plant; no vehicle validation')
    fig.savefig(out/'comparison.png',dpi=160);fig.savefig(out/'comparison.pdf');plt.close(fig)
    print(json.dumps(report['summaries'],indent=2),flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--unicorn-path',type=Path,required=True)
    p.add_argument('--plot-path',type=Path,required=True)
    p.add_argument('--session-trial',action='store_true')
    p.add_argument('--prototype-speed',type=int,choices=[15,30],default=30)
    p.add_argument('--noise-deg',type=float,default=0.0)
    p.add_argument('--sensor-lag-ms',type=int,choices=[20,40,60],default=20)
    a=p.parse_args();run(a.out.resolve(),a.unicorn_path,a.plot_path,a.prototype_speed,a.noise_deg,a.sensor_lag_ms,a.session_trial)
