"""Offline-only latest-baseline Stage3 modes. Never connects or flashes hardware."""
import argparse
import json
from pathlib import Path
import shutil
import struct
import subprocess

from prepare_stage3_ring2_touchdown_20261119 import edit
from prepare_stage3_speed_offline_20261119 import digest, extract_function, TARGETS

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/stage3_integrated_align15_yaw365_20261119/stage3-integrated-align15-compatible.json'
OUT = ROOT / 'tmp/stage3_offline_cycle_20261119'


def patch(project):
    names = []
    names.append(edit(project, 'stage3_cycle.inc', (
        'volatile float g_stage3_recovery_depth[3]={59400.0f,61400.0f,60600.0f};',
        '''/* 0 three-place baseline; 1 rough round-trip; 2 storage stack. */
volatile uint32_t g_stage3_cycle_mode;
volatile uint32_t g_stage3_locked_run;
volatile uint32_t g_stage3_first_colors[3],g_stage3_first_positions[3],g_stage3_first_mapping_valid;
/* Independent real calibration required; historical recovery Z is not restored. */
volatile float g_stage3_recovery_depth[3]={-1.0f,-1.0f,-1.0f};
static uint32_t s3_cycle_mode,s3_locked_top_scale;
static int64_t s3_locked_top_raw;
static float s3_locked_extension_home;'''), (
        'static uint32_t S3_Config(void)',
        '''static uint32_t S3_TargetPosition(uint32_t item)
{
 uint32_t i;
 if(g_stage3_cycle_mode!=2U)return g_stage3_positions[item];
 if(!g_stage3_first_mapping_valid)return 0U;
 for(i=0U;i<3U;i++)if(g_stage3_first_colors[i]==g_stage3_colors[item])
  return g_stage3_first_positions[i];
 return 0U;
}
static uint32_t S3_Config(void)'''), (
        ' uint32_t i,j,found,first=g_stage3_test_first_item,count=g_stage3_test_count;',
        ''' uint32_t i,j,found,first=g_stage3_test_first_item,count=g_stage3_test_count;
 if(g_stage3_cycle_mode>2U)return 0U;'''), (
        '  for(j=0U;j<i;j++)if(g_stage3_colors[j]==g_stage3_colors[i] || g_stage3_positions[j]==g_stage3_positions[i])return 0U;',
        '''  for(j=0U;j<i;j++)if(g_stage3_colors[j]==g_stage3_colors[i] || g_stage3_positions[j]==g_stage3_positions[i])return 0U;
  if(g_stage3_cycle_mode==2U){
   uint32_t mapped=S3_TargetPosition(i);
   if(mapped<1U || mapped>3U)return 0U;
   for(j=0U;j<i;j++)if(S3_TargetPosition(j)==mapped)return 0U;
  }'''), (
        '  j=g_stage3_positions[i]-1U;\n  /* Placement-only mission has no ring recovery targets. */',
        '''  j=S3_TargetPosition(i);
  if(j<1U || j>3U)return 0U;
  j--;
  if(g_stage3_cycle_mode==1U && !(g_stage3_recovery_depth[j]>=3200.0f &&
    g_stage3_recovery_depth[j]<=(float)SPF_STAGE3_MAX_LIFT_PULSES))return 0U;
  if(g_stage3_cycle_mode==2U && !(g_stage3_target_depth[j]-60.0f*SOURCE_LIFT_PULSES_PER_MM>=3200.0f &&
    g_stage3_target_depth[j]-60.0f*SOURCE_LIFT_PULSES_PER_MM<=(float)SPF_STAGE3_MAX_LIFT_PULSES))return 0U;
  /* Reuse the existing referenced geometry bounds below. */'''), (
        ' s3_round_trip=g_stage3_task_run!=0U;',
        ''' s3_round_trip=g_stage3_task_run!=0U;
 s3_cycle_mode=g_stage3_cycle_mode;
 g_stage3_locked_run=g_stage3_run;
 s3_locked_top_raw=g_lift_ruler_controller.top_position_raw;
 s3_locked_top_scale=g_lift_ruler_controller.top_scale;
 s3_locked_extension_home=g_extension_home_reference_deg;'''), (
        's3_positions[i]=g_stage3_positions[i];',
        's3_positions[i]=S3_TargetPosition(i);'), (
        '  s3_recovery_depth[i]=g_stage3_recovery_depth[i];',
        '''  if(s3_cycle_mode==2U)s3_depth[i]-=60.0f*SOURCE_LIFT_PULSES_PER_MM;
  s3_recovery_depth[i]=g_stage3_recovery_depth[i];'''), (
        '  case 30U:\n   g_stage3_placed_count++;\n   if(++g_stage3_item<s3_first+s3_count){g_stage3_phase=0U;return;}\n   g_stage3_state=6U;S3_Pose();return;',
        '''  case 30U:
   g_stage3_placed_count++;
   if(++g_stage3_item<s3_first+s3_count){g_stage3_phase=0U;return;}
   if(s3_count==3U && s3_first==0U && s3_cycle_mode!=2U && g_stage3_task_batch==1U){
    uint32_t i;
    for(i=0U;i<3U;i++){
     g_stage3_first_colors[i]=s3_colors[i];g_stage3_first_positions[i]=s3_positions[i];
    }
    g_stage3_first_mapping_valid=1U;
   }
   if(s3_cycle_mode==1U){g_stage3_item=s3_first;g_stage3_phase=S3_RECOVER_FIRST_PHASE;return;}
   g_stage3_state=6U;S3_Pose();return;'''), (
        '  case 25U:S3_Axis(SPF_LIFT,s3_pick);break;\n  case 26U:S3_Command(6U);break;',
        '''  case 25U:
   /* User choice: release directly at real top, after DM/tray completion. */
   if(it_at_top())g_stage3_phase++;else S3_Command(4U);
   break;
  case 26U:S3_Command(17U);break;'''), (
        '   if(++g_stage3_item<3U)g_stage3_phase=S3_RECOVER_FIRST_PHASE;',
        '   if(++g_stage3_item<s3_first+s3_count)g_stage3_phase=S3_RECOVER_FIRST_PHASE;'), (
        'static uint32_t S3_Fresh(uint32_t now)',
        '''static uint32_t S3_BeginRecoverPlaced(void)
{
 uint32_t i,j;
 if(g_stage3_state!=6U || g_stage3_fault || g_stage3_placed_count!=3U ||
    g_stage3_recovered_count || !g_stage3_locked_run || g_stage3_locked_run!=g_stage3_run ||
    s3_first!=0U || s3_count!=3U || s3_cycle_mode==2U ||
    !S3_Platform() || !SingleTeach_ReferencesReady() || !g_low_wheel_hold_ready ||
    !it_at_top() || s3_pending || g_it_request || s3_teach_pending ||
    g_stage3_teach_request || g_source_body_step_request ||
    s3_locked_top_raw!=g_lift_ruler_controller.top_position_raw ||
    s3_locked_top_scale!=g_lift_ruler_controller.top_scale ||
    s3_locked_extension_home!=g_extension_home_reference_deg)return 0U;
 for(i=0U;i<3U;i++){
  j=s3_positions[i]-1U;
  if(!(g_stage3_recovery_depth[j]>=3200.0f &&
    g_stage3_recovery_depth[j]<=(float)SPF_STAGE3_MAX_LIFT_PULSES))return 0U;
 }
 for(i=0U;i<3U;i++)s3_recovery_depth[i]=g_stage3_recovery_depth[i];
 s3_cycle_mode=1U;s3_round_trip=1U;
 g_stage3_item=s3_first;g_stage3_phase=S3_RECOVER_FIRST_PHASE;g_stage3_state=5U;
 S3_Pose();return 1U;
}
static uint32_t S3_Fresh(uint32_t now)'''), (
        ' if(!g_stage3_state || g_stage3_state==6U || g_stage3_state==9U || g_stage3_state==11U)return;',
        ''' if(request==32U){
  if(!S3_BeginRecoverPlaced())g_stage3_rejected++;
  return;
 }
 if(!g_stage3_state || g_stage3_state==6U || g_stage3_state==9U || g_stage3_state==11U)return;''')))
    names.append(edit(project, 'stage3_cycle.h', ('uint32_t Stage3_Owns(void);',
        '''extern volatile uint32_t g_stage3_cycle_mode,g_stage3_locked_run;
extern volatile uint32_t g_stage3_first_colors[3],g_stage3_first_positions[3],g_stage3_first_mapping_valid;
extern volatile float g_stage3_recovery_depth[3];
uint32_t Stage3_Owns(void);''')))
    return names


def run_fixture(project):
    source=(project/'CHASSIS/USER/stage3_cycle.inc').read_text(encoding='gbk')
    functions='\n'.join(extract_function(source, name) for name in (
        'static uint32_t S3_TargetPosition', 'static uint32_t S3_Config', 'static void S3_LockConfig',
        'static uint32_t S3_BeginRecoverPlaced'))
    start=source.index(' if(g_stage3_state==5U){\n  uint32_t p=')
    block=source[start:source.rfind('\n}')]
    template=(ROOT/'scripts/stage3_offline_cycle_fixture_20261119.c').read_text()
    cfile=OUT/'actual-offline-cycle.c'
    begin=source.index(' if(request==32U){')
    end=source.index('\n if(!g_stage3_state',begin)
    dispatch='\nstatic void request32(uint32_t request){\n'+source[begin:end]+'\n}'
    cfile.write_text(template.replace('// INSERT_EXTRACTED_FIRMWARE',functions+'\nstatic void service(uint32_t now) {\n'+block+'\n}'+dispatch),encoding='ascii')
    exe=OUT/'actual-offline-cycle.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Werror',str(cfile),'-o',str(exe)],check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15)
    if result.returncode:
        raise RuntimeError(result.stdout+result.stderr)
    log=OUT/'actual-offline-cycle.log';log.write_text(result.stdout,encoding='ascii')
    print(result.stdout,end='')
    return {'log':str(log),'sha256':digest(log),'source_sha256':digest(cfile)}


def symbol_sizes(axf):
    """Read ELF32 symbol sizes without importing any debug/hardware library."""
    b=axf.read_bytes()
    assert b[:6]==b'\x7fELF\x01\x01'
    offset=struct.unpack_from('<I',b,32)[0]
    stride,count=struct.unpack_from('<HH',b,46)
    sections=[struct.unpack_from('<IIIIIIIIII',b,offset+i*stride) for i in range(count)]
    table=next(s for s in sections if s[1]==2)
    strings=sections[table[6]]
    names=b[strings[4]:strings[4]+strings[5]]
    expected={'g_stage3_cycle_mode':4,'g_stage3_locked_run':4,'g_stage3_first_mapping_valid':4,
              'g_stage3_first_colors':12,'g_stage3_first_positions':12,'g_stage3_recovery_depth':12}
    actual={}
    for offset in range(table[4],table[4]+table[5],table[9]):
        entry=struct.unpack_from('<IIIBBH',b,offset)
        name=names[entry[0]:].split(b'\0',1)[0].decode(errors='replace')
        if name in expected: actual[name]=entry[2]
    assert actual==expected,actual
    return actual


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',action='store_true')
    parser.add_argument('--fixture',action='store_true')
    parser.add_argument('--refresh-source',action='store_true')
    args=parser.parse_args()
    matlab=OUT/'matlab-validation.json'
    assert json.loads(matlab.read_text())['passed'], 'MATLAB prerequisite'
    manifest=json.loads(BASE.read_text(encoding='utf-8'))
    base=Path(manifest['project']); project=OUT/'project'
    for name, expected in (manifest['unchanged_sha256']|manifest['changes']).items():
        assert digest(base/name)==expected, name
    candidate=OUT/'stage3-offline-cycle-compatible.json'
    if not project.exists() or args.refresh_source:
        if project.exists():
            previous=json.loads(candidate.read_text())
            for name,expected in (previous['unchanged_sha256']|previous['changes']).items():
                assert digest(project/name)==expected,name
            for name in ('stage3_cycle.inc','stage3_cycle.h'):
                shutil.copyfile(base/'CHASSIS/USER'/name,project/'CHASSIS/USER'/name)
        else:
            shutil.copytree(base,project,ignore=shutil.ignore_patterns('Objects','Listings'))
        for name in patch(project):
            manifest['unchanged_sha256'].pop(name,None)
            manifest['changes'][name]=digest(project/name)
        manifest.update(project=str(project),images={},build_passed=False,flashed=False,
                        baseline_manifest=str(BASE),baseline_manifest_sha256=digest(BASE),
                        offline_cycle_matlab_sha256=digest(matlab),
                        stage3_cycle_modes={'three-place':0,'rough-round-trip':1,'storage-stack':2},
                        pending_calibration=['recovery_depth_by_ring','top release physical stability','storage work-plane'],
                        carrier_return='actual top release160; no descent',
                        stage3_recover_placed_request=32,
                        physical_acceptance=False)
        manifest['capabilities'].update(stage3_cycle_modes=True,stage3_rough_round_trip=True,
                                       stage3_storage_stack=True,stage3_first_batch_mapping=True,
                                       stage3_recover_placed=True)
        candidate.write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    else:
        manifest=json.loads(candidate.read_text())
        for name, expected in (manifest['unchanged_sha256']|manifest['changes']).items():
            assert digest(project/name)==expected,name
    if args.fixture: manifest['offline_cycle_c_fixture']=run_fixture(project)
    if args.build:
        for target in TARGETS:
            for folder in ('Objects','Listings'): (project/folder/target).mkdir(parents=True,exist_ok=True)
            log=OUT/f'{target}.log'
            result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(project/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=240)
            assert result.returncode==0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes(),log
            axf=project/'Objects'/target/f'{target}.axf'
            manifest['images'][target]={'axf':str(axf),'sha256':digest(axf),'cycle_symbol_sizes':symbol_sizes(axf)}
            print(target,'0 errors, 0 warnings',flush=True)
        manifest['build_passed']=True
    candidate.write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(candidate)


if __name__=='__main__': main()
