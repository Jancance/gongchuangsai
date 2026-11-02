"""Prepare an isolated forward-only heading-output-off trial. No hardware access."""
from pathlib import Path
import hashlib, json, shutil

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'工创赛/evidence/p1-foundation/2026-11-02/small_ff005_pair/firmware'
OUT=ROOT/'工创赛/evidence/p1-foundation/2026-11-02/no_heading_once'
OUT.mkdir(exist_ok=True)
DEST=OUT/'firmware'
if not DEST.exists():
    shutil.copytree(BASE,DEST,ignore=shutil.ignore_patterns('Objects*','Listings*','*.uvguix.*'))
header='''#ifndef NO_HEADING_TRIAL_H
#define NO_HEADING_TRIAL_H
#include <stdint.h>
#include "app_config.h"
#include "longitudinal_feedforward.h"
#ifndef APP_NO_HEADING_FORWARD_TRIAL
#define APP_NO_HEADING_FORWARD_TRIAL 0
#endif
#if APP_NO_HEADING_FORWARD_TRIAL != 0 && APP_NO_HEADING_FORWARD_TRIAL != 1
#error Invalid_no_heading_trial
#endif
#if APP_NO_HEADING_FORWARD_TRIAL && (!APP_IS_DEBUG || APP_TEST_MODE != 3U)
#error No_heading_trial_requires_debug_mode3
#endif
#if APP_NO_HEADING_FORWARD_TRIAL && (APP_FF_FORWARD_ENABLE || APP_FF_BACKWARD_ENABLE)
#error No_heading_trial_requires_feedforward_off
#endif
/* 仅独立前进距离对照：保留姿态校验、PD计算和故障保护，只将下发修正归零。 */
static __inline float NoHeadingTrialCorrection(uint32_t mode, float distance_mm, float *correction)
{
#if APP_NO_HEADING_FORWARD_TRIAL
    if (mode == 0U && distance_mm > 0.0f) *correction = 0.0f;
#endif
    return *correction;
}
#endif
'''
(DEST/'USER/no_heading_trial.h').write_bytes(header.replace('\n','\r\n').encode('gbk'))
p=DEST/'USER/main.c';raw=(BASE/'USER/main.c').read_bytes()
assert p.read_bytes()==raw,'Candidate already edited; do not overwrite'
source=raw.decode('gbk')
anchor='ChassisAuto_UpdateForwardHold(correction)'
assert source.count(anchor)==1
source=source.replace('#include "app_config.h"','#include "app_config.h"\r\n#include "no_heading_trial.h"')
source=source.replace(anchor,'ChassisAuto_UpdateForwardHold(NoHeadingTrialCorrection(\r\n                        App_TestTranslationMode(), App_StageDistanceMm(), &correction))')
p.write_bytes(source.encode('gbk'))
template=(BASE/'gongxun-f407-baseline_vehicle.uvprojx').read_bytes()
projects=[]
for kind in ('vehicle','offline'):
    data=template.replace(b'APP_X_OFFLINE_ONLY=0',('APP_X_OFFLINE_ONLY='+str(int(kind=='offline'))+',APP_NO_HEADING_FORWARD_TRIAL=1').encode())
    data=data.replace(b'Objects_baseline_vehicle',('Objects_no_heading_'+kind).encode()).replace(b'Listings_baseline_vehicle',('Listings_no_heading_'+kind).encode())
    name='gongxun-f407-no_heading_'+kind+'.uvprojx'
    (DEST/name).write_bytes(data);projects.append(name)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
changed=[]
for p in BASE.rglob('*'):
    if p.is_file() and p.suffix in ('.c','.h','.s'):
        q=DEST/p.relative_to(BASE)
        if q.exists() and sha(q)!=sha(p):changed.append(p.relative_to(BASE).as_posix())
assert changed==['USER/main.c'],changed
manifest={'baseline':str(BASE),'changed_existing_sources':changed,'new_source':'USER/no_heading_trial.h','projects':projects,'feedforward_enabled':False,'forward_distance_heading_output_enabled':False,'PD_computation_and_sensor_checks_preserved':True,'all_other_source_files_identical':True,'main_encoding':'GBK; original mixed line endings preserved; inserted lines CRLF','hardware_accessed':False,'reference_A_sha256':'f8462206ca1fcdb22b5e18bd9980d3dc8dba11622fb3ae70b082fa816a0e3630'}
(OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf8')
shutil.copyfile(BASE.parent/'forward210.json',OUT/'forward210.json')
print(json.dumps(manifest,ensure_ascii=False,indent=2))
