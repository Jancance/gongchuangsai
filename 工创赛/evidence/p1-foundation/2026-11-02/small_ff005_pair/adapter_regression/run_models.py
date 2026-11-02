"""Actual-C reverse/perimeter plant checks; no serial/probe or download code."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

sys.path.insert(0,'C:\\Users\\lenovo\\Desktop\\工创赛\\工创赛\\scripts')
from p1_turn_fault_audit import function
ROOT=Path('C:\\Users\\lenovo\\Desktop\\工创赛')


def run(candidate,out):
    candidate=Path(candidate).resolve();out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
    src=out/'src';shutil.copytree(candidate/'USER',src)
    shutil.copytree(Path('C:\\Users\\lenovo\\Desktop\\工创赛\\工创赛\\scripts\\p1_offline_tests\\stubs'),out/'stubs')
    stub=out/'stubs/can.h'
    st=stub.read_text().replace('CAN1_ReadSpeed(uint8_t,CanRxMsg*,int32_t*)','CAN1_ReadSpeed(uint8_t,CanRxMsg*,float*)').replace('CAN1_SetEmmSpeedRaw','CAN1_SetXSpeedRaw').replace('uint16_t,uint8_t,CanRxMsg*','uint16_t,uint16_t,CanRxMsg*')
    st += '\nuint32_t CAN1_ReadXSystemSnapshot(uint8_t,CanRxMsg*,float*);\n'
    stub.write_text(st)
    main=(src/'main.c').read_bytes().decode('gbk').replace('\r\n','\n')
    begin=main.index('typedef struct {uint32_t captured,tick_ms;ChassisOdometryStatus pose;} PerimeterRecord;')
    end=main.index('\n};',main.index('static float s_perimeter_targets',begin))+3
    declarations='''
typedef struct {uint32_t sequence,sample_ms,angle_ms,rate_ms,healthy,valid_mask;float yaw_deg,rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static ChassisOdometry s_odometry;
volatile ChassisOdometryStatus g_chassis_odometry;
static struct {uint32_t state,completed;}g_combined_test;
volatile uint32_t g_odometry_pose_request,g_odometry_pose_result,g_vm_test_request;
volatile float g_odometry_pose_x_mm,g_odometry_pose_y_mm,g_odometry_pose_heading_deg;
static uint32_t s_can_jog_preparing;
static uint32_t App_HeadingFrameAge(const AppHeadingSource *s,uint32_t now,uint32_t bit){return (s->valid_mask&bit)?now-s->angle_ms:0xFFFFFFFFU;}
'''+main[begin:end]
    if (src/'route_heading_reference.h').exists():
        declarations += '\n#include \"src/route_heading_reference.h\"\nstatic RouteHeadingReference s_route_heading_reference;\n'
    source=Path(__file__).with_name('fixture.c').read_text(encoding='utf8')
    if (src/'route_heading_reference.h').exists():
        source=source.replace('/* REFERENCE_MODE */','#define REFERENCE_MODE 1')
    if 'APP_FINE_RPM_OFFLINE_ONLY' in (src/'app_config.h').read_text(encoding='gbk'):
        source=source.replace('/* FINE_RPM_MODE */','#define FINE_RPM_MODE 1')
    source=source.replace('/* MAIN_GLOBALS */',declarations).replace('/* MAIN_ADAPTER */',function(main,'App_OdometryStep'))
    (out/'unit.c').write_bytes(source.encode('gbk'))
    (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
    arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin');logs=[]
    objects=[('unit.c','unit.o'),('src/runtime_route.c','route.o'),('src/chassis_kinematics.c','kinematics.o'),('src/motion_profile.c','profile.o'),('src/heading_controller.c','heading.o'),('src/heading_session.c','session.o')]
    commands=[[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-DAPP_X_OFFLINE_ONLY=1','-DAPP_FF_FORWARD_ENABLE=1','-DAPP_FF_BACKWARD_ENABLE=0','-DAPP_FF_FORWARD_PERMILLE=5','-DAPP_FF_BACKWARD_PERMILLE=0','-I','src','-I','stubs','-c',s,'-o',o] for s,o in objects]
    commands.append([str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct']+[o for s,o in objects]+['--library_type=microlib','-o','unit.axf'])
    for cmd in commands:
        p=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace',timeout=60)
        logs.append(dict(command=cmd,returncode=p.returncode,output=p.stdout+p.stderr));(out/'build.json').write_text(json.dumps(logs,indent=2))
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'))
    import unicorn as uc
    from unicorn import arm_const as reg
    m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
    b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
    for i in range(h[10]):
        kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
        if kind==1 and size:m.mem_write(addr,b[off:off+size])
    symbols=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True)
    def address(name):
        return int(re.search(r'\b'+re.escape(name)+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
    traces=[];trace_addr=address('trace_data');hook=address('PerimeterTrace')&~1
    def capture(machine,*args):traces.append(struct.unpack('<12f',machine.mem_read(trace_addr,48)))
    m.hook_add(uc.UC_HOOK_CODE,capture,begin=hook,end=hook)
    fine_traces=[]
    if out.name.endswith('_trace') and 'APP_FINE_RPM_OFFLINE_ONLY' in (src/'app_config.h').read_text(encoding='gbk'):
        fine_addr=address('fine_trace');fine_hook=address('FineTrace')&~1
        def fine_capture(machine,*args):fine_traces.append(struct.unpack('<16f',machine.mem_read(fine_addr,64)))
        m.hook_add(uc.UC_HOOK_CODE,fine_capture,begin=fine_hook,end=fine_hook)
    stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1)
    m.emu_start(h[4]|1,stop,count=4000000000,timeout=180000000)
    (out/'fine_traces.json').write_text(json.dumps(fine_traces))
    value=m.reg_read(reg.UC_ARM_REG_R0);completed=m.reg_read(reg.UC_ARM_REG_PC)==stop
    count=struct.unpack('<I',m.mem_read(address('scenarios_run'),4))[0]
    rows=[];fields=['full_perimeter','tau_s','sensor_delay_ticks','wheel_bias','fault_kind','completed_actions','valid_checkpoint_mask','odom_x_mm','odom_y_mm','odom_heading_deg','ground_x_mm','ground_y_mm','backend_cause','distance_fault_reason','heading_cause','elapsed_ms','stress_model','backend_result']
    data=struct.unpack('<1008f',m.mem_read(address('results'),1008*4))
    for i in range(count):rows.append(dict(scenario=i,**dict(zip(fields,data[i*18:(i+1)*18]))))
    requested=struct.unpack('<56I',m.mem_read(address('model_requested'),56*4))
    for i,row in enumerate(rows):row['requested_actions']=requested[i]
    context=list(struct.unpack('<10I',m.mem_read(address('failure_context'),40)))
    report=dict(hardware_accessed=False,emulator_completed=completed,passed=completed and not value&0x80000000,
        assertions=value if value<0x80000000 else None,failure_line=value&0x7fffffff if value&0x80000000 else None,failure_context=context,
        models=rows,performance_matrix_all_completed=all(r['completed_actions']==r['requested_actions'] for r in rows if not r['fault_kind']),vehicle_ready=False,
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in src.glob('*') if p.is_file()},
        limits='Actual backend, HeadingSession, odometry and adapter; sequence driver, CAN/VM/RTOS and physical plant are models. No vehicle proof.')
    report['fault_detail_fields']=['action_index','tick_ms','heading_cause','sample_age_ms','healthy','yaw_deg','rate_dps','start_yaw','direction','progress_deg','target_angle','brake_dt_ms','session_elapsed_ms','filtered_progress','filtered_rate','backend_running']
    detail=struct.unpack('<896f',m.mem_read(address('fault_details'),896*4))
    report['fault_details']=[dict(scenario=i,**dict(zip(report['fault_detail_fields'],detail[i*16:(i+1)*16]))) for i in range(count) if detail[i*16+2]]
    data=struct.unpack('<560f',m.mem_read(address('endpoint_injections'),2240))
    keys=['ack_target_rpm','injected_rpm','base_rpm','progress_mm','remaining_mm','ack_age_ms','reference_mm','reference_acceleration','micro_case','injection_count']
    report['endpoint_injections']=[dict(scenario=i,**dict(zip(keys,data[i*10:(i+1)*10]))) for i in range(count) if data[i*10+9]]
    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    with (out/'traces.csv').open('w',newline='',encoding='utf8') as f:
        writer=csv.writer(f);writer.writerow(['scenario','tick_ms','action_index','ground_x_mm','ground_y_mm','ground_heading_deg','odom_x_mm','odom_y_mm','odom_heading_deg','odom_valid','backend_state','backend_cause']);writer.writerows(traces)
    print(json.dumps({k:report[k] for k in ['passed','assertions','failure_line','failure_context']}),flush=True)
    print('models',count,'traces',len(traces),flush=True)
    if not report['passed']:raise RuntimeError('Actual C model check failed; inspect report and unit.c')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();run(a.candidate,a.out)
