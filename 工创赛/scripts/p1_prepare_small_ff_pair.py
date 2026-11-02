"""Prepare isolated baseline/0.5% forward FF builds. Never flash or drive."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from p1_longitudinal_feedforward_sim import ROOT, hashes


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();out=a.out.resolve()
    if not out.is_relative_to(ROOT):raise ValueError('Output outside workspace')
    out.mkdir(parents=True,exist_ok=False)
    live=ROOT/'3.stm32f407工程';before=hashes(live)
    old=ROOT/'工创赛/evidence/p1-foundation/2026-11-02/longitudinal_feedforward_v4'
    assert (live/'USER/chassis_kinematics.c').read_bytes()==(old/'baseline/USER/chassis_kinematics.c').read_bytes()
    project=out/'firmware'
    shutil.copytree(live,project,ignore=shutil.ignore_patterns('Objects*','Listings*','*.uvguix.*','.vscode'))
    # Reuse the evaluated FF math; the only new gate is explicit vehicle opt-in.
    for name in ['chassis_kinematics.c','longitudinal_feedforward.h']:
        data=(old/'candidate/USER'/name).read_bytes()
        assert b'\r\n' in data and b'\n' not in data.replace(b'\r\n',b'')
        if name.endswith('.h'):
            text=data.decode('gbk')
            text=text.replace('#include "zdt_x_config.h"','#include "zdt_x_config.h"\r\n#include "app_entry_config.h"\r\n'+
                '/* 独立对照工程才可显式启用实车；源码默认仍禁止。 */\r\n'+
                '#ifndef APP_FF_VEHICLE_OPT_IN\r\n#define APP_FF_VEHICLE_OPT_IN 0\r\n#endif\r\n'+
                '#if APP_FF_VEHICLE_OPT_IN != 0 && APP_FF_VEHICLE_OPT_IN != 1\r\n#error Invalid_feedforward_vehicle_opt_in\r\n#endif')
            text=text.replace('#if (APP_FF_FORWARD_ENABLE || APP_FF_BACKWARD_ENABLE) && !APP_X_OFFLINE_ONLY',
                '#if (APP_FF_FORWARD_ENABLE || APP_FF_BACKWARD_ENABLE) && !APP_X_OFFLINE_ONLY && !APP_FF_VEHICLE_OPT_IN')
            text=text.replace('#error Feedforward_trial_requires_offline_build','#error Feedforward_requires_offline_or_explicit_vehicle_opt_in')
            text=text.replace('/* 只用于S曲线距离入口。',
                '#if (APP_FF_FORWARD_ENABLE || APP_FF_BACKWARD_ENABLE) && (!APP_IS_DEBUG || APP_TEST_MODE != 3U)\r\n'+
                '#error Feedforward_trial_requires_debug_mode3\r\n#endif\r\n'+
                '/* 只用于S曲线距离入口。')
            data=text.encode('gbk')
        (project/'USER'/name).write_bytes(data)
    template=(live/'gongxun-f407.uvprojx').read_bytes()
    manifest=[]
    for variant,enable,ratio in [('baseline',0,0),('ff005',1,5)]:
        for offline in [True,False]:
            kind='offline' if offline else 'vehicle'
            label=variant+'_'+kind
            defines=f'APP_X_OFFLINE_ONLY={int(offline)},APP_FF_FORWARD_ENABLE={enable},APP_FF_BACKWARD_ENABLE=0,APP_FF_FORWARD_PERMILLE={ratio},APP_FF_BACKWARD_PERMILLE=0,APP_FF_VEHICLE_OPT_IN={int(not offline and enable)}'
            data=template.replace(b'APP_X_OFFLINE_ONLY=0',defines.encode('ascii')).replace(
                b'.\\Objects\\',('.\\Objects_'+label+'\\').encode('ascii')).replace(
                b'.\\Listings\\',('.\\Listings_'+label+'\\').encode('ascii'))
            name='gongxun-f407-'+label+'.uvprojx';(project/name).write_bytes(data)
            manifest.append(dict(variant=variant,kind=kind,project=name,defines=defines,
                                 output_directory='Objects_'+label,build_only=True))
    (out/'live_before.json').write_text(json.dumps(before,indent=2),encoding='utf8')
    (out/'manifest.json').write_text(json.dumps(dict(hardware_accessed=False,github_pushed=False,
        current_main_unchanged=before==hashes(live),projects=manifest,forward_difference_permille=5,
        backward_enabled=False,protected_sources_unchanged=True,logs_note='Existing diagnostic sampling unchanged; no high-rate acquisition claim'),indent=2),encoding='utf8')
    # A straight 210cm route, without the historical +0.8% route-distance trial.
    route=dict(schema='p1-runtime-route-v1',frame='body-relative',purpose='Prepared A/B 210cm straight trial; no +0.8% route-distance addition',
               actions=[dict(kind='FORWARD',distance_cm=210,speed_cm_s=20,timeout_ms=35000)])
    (out/'forward210.json').write_text(json.dumps(route,indent=2),encoding='utf8')
    print(json.dumps(dict(out=str(out),projects=manifest,live_unchanged=before==hashes(live)),indent=2))


if __name__=='__main__':main()
