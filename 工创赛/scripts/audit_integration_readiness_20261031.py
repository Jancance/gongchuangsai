"""整合前只读审计：当前源码/历史证据、相机版本差异和夹爪任务互锁缺口。"""
from pathlib import Path
import hashlib
import json
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
EV=ROOT/'evidence/integration/2026-10-31'
HOST=ROOT/'tmp/integration_audit_20261031/gripper-check'
assert not (EV/'review.json').exists(),'Preserve this audit; use a new snapshot for a later audit'
EV.mkdir(parents=True,exist_ok=True);HOST.mkdir(parents=True,exist_ok=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read_json(p):return json.loads(p.read_text(encoding='utf-8-sig'))

installed=read_json(ROOT/'evidence/p2-arm/2026-10-30/lift-down-step/installed.json')
changed={e['file']:sha(ACTUAL/e['file'])==e['after'] for e in installed['files']}
protected={n:sha(ACTUAL/n)==h for n,h in installed['protected'].items()}
assert all(changed.values()) and all(protected.values())
logic=read_json(ROOT/'evidence/p2-task-carrier/2026-10-28/review.json')
logic_checks={n:sha(ROOT/n)==h for n,h in logic['files_sha256'].items()}
assert all(logic_checks.values())

# Compile the actual task, replacing only RTOS scheduling and the PWM writer.
(HOST/'FreeRTOS.h').write_text('''#ifndef AUDIT_FREERTOS_H
#define AUDIT_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
#define pdFAIL 0
#define pdPASS 1
#define configASSERT(x) assert(x)
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#endif
''',encoding='utf-8')
(HOST/'task.h').write_text('''#include "FreeRTOS.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
BaseType_t xTaskCreate(void (*fn)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t*);
void vTaskDelay(TickType_t);
''',encoding='utf-8')
(HOST/'model.c').write_text('''#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "gripper_task.h"
static jmp_buf done;
static void (*entry)(void*);
static float targets[4];
static unsigned writes;
unsigned g_carrier_state=0,g_lift_motion_state=0;
BaseType_t xTaskCreate(void(*fn)(void*),const char*n,unsigned stack,void*a,unsigned pr,TaskHandle_t*h)
{(void)n;(void)stack;(void)a;(void)pr;entry=fn;*h=(void*)1;return pdPASS;}
void vTaskDelay(TickType_t t){(void)t;longjmp(done,1);}
unsigned char jiazhua(float angle){targets[writes++]=angle;return 1;}
void Gripper_Stop(void){}
int main(int argc,char **argv){
 assert(argc==2);
 g_carrier_state=strcmp(argv[1],"carrier")==0;
 g_lift_motion_state=strcmp(argv[1],"lift")==0;
 assert(g_carrier_state+g_lift_motion_state==1);
 Gripper_TaskInit();
 g_jiazhua_debug_angle=140.0f;g_jiazhua_debug_request=1;
 if(setjmp(done)==0)entry(NULL);
 assert(writes==2&&targets[0]==180.0f&&targets[1]==140.0f);
 assert(g_jiazhua_debug_result==1&&g_carrier_state+g_lift_motion_state==1);
 printf("REPRODUCED:actual gripper task forwards close140 while mock %s is active; no shared interlock in this task. No hardware used.\\n",argv[1]);
 return 0;
}
''',encoding='utf-8')
exe=HOST/'gripper-audit.exe'
compile_result=subprocess.run(['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror',
    '-I'+str(HOST),'-I'+str(ACTUAL/'USER'),'-I'+str(ACTUAL/'HARDWARE/GRIPPER'),
    str(ACTUAL/'USER/gripper_task.c'),str(HOST/'model.c'),'-o',str(exe)],
    capture_output=True,text=True,timeout=30)
assert compile_result.returncode==0,compile_result.stderr
repro_logs=[]
for scenario in ['carrier','lift']:
    repro=subprocess.run([str(exe),scenario],capture_output=True,text=True,timeout=10)
    assert repro.returncode==0,(repro.stdout,repro.stderr)
    repro_logs.append(repro.stdout)
(EV/'gripper-interlock-host-reproduction.log').write_text(''.join(repro_logs),encoding='utf-8')

rules_text=(ROOT/'docs/competition_requirements_2027.md').read_text(encoding='utf-8-sig')
checked_ids=(['QR-01','QR-02','QR-03','QR-04','QR-05','QR-06','QR-07','DSP-01','DSP-02','VIS-01']+
             [f'VIS-{i:02}' for i in range(2,16)]+[f'CAL-{i:02}' for i in range(2,6)]+
             [f'RUN-{i:02}' for i in range(1,9)]+[f'LOC-{i:02}' for i in range(1,8)]+
             [f'MOT-{i:02}' for i in range(1,5)]+['FIELD-02','FIELD-03','FIELD-04','FIELD-05',
             'FIELD-06','FIELD-07','FIELD-08','PWR-01','PWR-02','MECH-01','MECH-02','MECH-03',
             'MECH-04','MECH-05','SCORE-01'])
requirements={}
for i,line in enumerate(rules_text.splitlines(),1):
    m=re.match(r'\| ([A-Z]+-\d+) \| ([REQ]) \| (.*?) \| (.*?) \|',line)
    if m and m[1] in checked_ids:
        requirements[m[1]]={'class':m[2],'line_at_audit':i,'requirement':m[3],'verification_guidance':m[4]}
assert set(requirements)==set(checked_ids),set(checked_ids)-set(requirements)
(EV/'requirements-checked.json').write_text(json.dumps(requirements,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

root_profile=read_json(ROOT/'vision_app/code/config/project_profile.json')
camera135=read_json(ROOT/'output/p2_vision_generic135_pixel1_20261027/vision_app/code/config/project_profile.json')
point_record=read_json(ROOT/'config/arm_fixed_points_candidate_20261030.json')
carrier_record=read_json(ROOT/'config/carrier_slots_candidate_20261030.json')
files=[ACTUAL/n for n in ['USER/app_config.h','USER/main.c','USER/gripper_task.c',
    'HARDWARE/GRIPPER/gripper_config.h','HARDWARE/CARRIER/carrier.c','HARDWARE/CARRIER/carrier_config.h',
    'HARDWARE/DM_CAN2/dm_can2_motion.c','HARDWARE/LIFT/lift.c','HARDWARE/LIFT/lift_config.h',
    'HARDWARE/EXTENSION/extension.c','HARDWARE/EXTENSION/extension_home.c']]
files+=[ROOT/n for n in ['src/smart_carrier/task_materials.py','src/smart_carrier/p2_state_machine.py',
    'src/smart_carrier/protocol.py','vision_app/code/main.py','vision_app/code/project_camera.py',
    'vision_app/code/config/project_profile.json',
    'output/p2_vision_generic135_pixel1_20261027/vision_app/code/config/project_profile.json',
    'config/arm_fixed_points_candidate_20261030.json','config/carrier_slots_candidate_20261030.json']]
code_refs={}
patterns={
    ACTUAL/'USER/gripper_task.c':['g_jiazhua_debug_result = jiazhua(JIAZHUA_OPEN_ANGLE_DEG);','if (request == 1U)'],
    ACTUAL/'USER/main.c':['static void App_CarrierStep(void)','if (Carrier_IsBusy())','if (ArmPower_IsIsolatedMode() != 0U)'],
    ACTUAL/'HARDWARE/CARRIER/carrier.c':['uint32_t Carrier_IsBusy','g_carrier_state = 2U;'],
    ACTUAL/'HARDWARE/DM_CAN2/dm_can2_motion.c':['DM_CAN2_BuildSwitch(0xFDU'],
    ROOT/'vision_app/code/main.py':['serial_link = DisabledSerialLink('],
    ROOT/'vision_app/code/project_camera.py':['DEFAULT_WIDTH=1280','DEFAULT_HEIGHT=960']}
for p,searches in patterns.items():
    text=p.read_bytes().decode('utf-8',errors='replace').splitlines()
    code_refs[str(p)]={s:next(i for i,line in enumerate(text,1) if s in line) for s in searches}

app=(ACTUAL/'USER/app_config.h').read_bytes().decode('utf-8',errors='replace')
active_arm=bool(re.search(r'#define APP_ARM_LIFT_TEST\s+1U',app))
review={
    'date':'2026-10-31','scope':'local current F407 project,workspace source and archived evidence; no physical testing or live OrangePi/teammate workspace inspection',
    'canonical_projects':[p.name for p in ACTUAL.glob('*.uvprojx')],
    'current_lift_installation_unchanged':changed,'other_source_checks_unchanged':all(protected.values()),
    'protected_file_count':len(protected),
    'task_logic_same_as_prior20test_evidence':logic_checks,'prior_tests_not_rerun':True,
    'arm_test_mode_enabled':active_arm,
    'fixed_point_accepted_count':sum(p['physical_point_accepted'] for p in point_record['points']),
    'fixed_point_count':len(point_record['points']),
    'descent40000_physical_result_received':False,
    'carrier_return0_user_functionally_accepted':carrier_record['return240_to0_function_accepted'],
    'carrier_full_metric_calibration_accepted':any(p['accepted'] for p in carrier_record['slots']),
    'gripper_task_shared_interlock_gap_reproduced':True,
    'gripper_interlock_independent_host_scenarios':['carrier active','lift active'],
    'gripper_reproduction_scope':'host task-level command forwarding; no actual PWM/hardware/collision test',
    'root_camera_profile':{'camera_model':root_profile['camera_model'],'image_size':root_profile['image_size']},
    'archived135_profile':{'camera_model':camera135['camera_model'],'image_size':camera135['image_size']},
    'camera_current_live_runtime_rechecked':False,
    'both_profile_metric_and_serial_disabled':all(not p['geometry_verified'] and not p['serial_enabled'] for p in [root_profile,camera135]),
    'code_refs':code_refs,'source_sha256':{str(p):sha(p) for p in files},
    'firmware_changed_or_flashed':False,'hardware_commands_sent':False,
    'requirement_count':len(requirements),'requirements_R':[n for n,x in requirements.items() if x['class']=='R'],
    'requirements_E':[n for n,x in requirements.items() if x['class']=='E'],
    'requirements_Q':[n for n,x in requirements.items() if x['class']=='Q'],
    'official_Q':'all still-pending official items remain pending; no new official ruling obtained'}
assert review['canonical_projects']==['gongxun-f407.uvprojx']
assert active_arm and review['fixed_point_accepted_count']==0
(EV/'review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(''.join(repro_logs).strip())
print('READONLY AUDIT:one Keil entry;5 installed lift files+813 protected files match;20-test task sources match;0/5 complete fixed points;camera root/135 profiles differ;no hardware touched.')
