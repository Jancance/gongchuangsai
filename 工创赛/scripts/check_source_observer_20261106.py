"""Cross-language observer checks; synthetic geometry and archived diagnostics."""
from pathlib import Path
import copy
import hashlib
import json
import subprocess
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'evidence/integration/2026-11-06/source-observer'
STAGE = ROOT / 'tmp/source_observer_20261106/project'
sys.path.insert(0, str(ROOT / 'vision_app/code'))
from source_metric_preview import preview_rim
from metric_alignment_protocol import MetricObservation, encode_observation, decode_request, decode_observation
from source_metric_bridge import observation_packet
from finish_lift_ruler_candidate_20261105 import function_end


def cstr(wire):
    return '"' + ''.join(f'\\{b:03o}' for b in wire) + '"'


UNIT = r'''
#include <stdio.h>
#include <string.h>
#include "source_observer.h"
#include "vehicle_mission.h"
static uint32_t ready=1;static unsigned checks,failures;
static VehicleAlignmentRequest current={2,2,1,2,14,2060,1200,0};
uint32_t VehicleMission_ReadAlignmentRequest(VehicleAlignmentRequest*out){if(!ready)return 0;*out=current;return 1;}
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line%u\n",__LINE__);}}while(0)
static void feed(const char*p,uint32_t now){while(*p)SourceObserver_Byte((uint8_t)*p++,now);}
int main(void){
 MetricAlignmentProfile p={1,10,20,30,40,50,200,900,100000,10000};char request[256];unsigned i;
 SourceObserver_Init(0);SourceObserver_Service(100);
 CK(g_source_observer_reason==3 && !g_source_observer_accepted && !SourceObserver_Request(request,sizeof(request)));
 feed(WIRE,110);SourceObserver_Service(120);CK(!g_source_observer_accepted);
 SourceObserver_Init(&p);SourceObserver_Service(100);
 CK(SourceObserver_Request(request,sizeof(request))>0);printf("REQUEST:%s",request);
 CK(!SourceObserver_Request(request,sizeof(request)));
 feed(WIRE,125);SourceObserver_Service(140);
 CK(g_source_observer_accepted && g_source_observer_dx_milli_mm==-12500 && g_source_observer_dy_milli_mm==2500);
 CK(g_source_observer_frames==1);
 SourceObserver_Service(321);CK(!g_source_observer_accepted && !g_source_observer_dx_milli_mm);
 SourceObserver_Service(1100);CK(g_source_observer_requests==2);
 feed(WIRE,1110);SourceObserver_Service(1120);CK(!g_source_observer_accepted && g_source_observer_reason==6);
 SourceObserver_Init(&p);SourceObserver_Service(100);
 feed(WIRE,125);feed(WIRE,125);CK(g_source_observer_dropped==1);SourceObserver_Service(140);CK(g_source_observer_accepted);
 ready=0;SourceObserver_Service(150);CK(!g_source_observer_accepted);
 ready=1;SourceObserver_Init(&p);SourceObserver_Service(100);
 for(i=0;i<700;i++)SourceObserver_Byte('A',101);SourceObserver_Byte('\n',101);
 CK(g_source_observer_dropped>0);feed(WIRE,125);SourceObserver_Service(140);CK(g_source_observer_accepted);
 SourceObserver_Init(&p);SourceObserver_Service(100);
 feed("MA1,1,2,",125);SourceObserver_Byte('X',400);SourceObserver_Byte('\n',401);SourceObserver_Service(402);
 CK(!g_source_observer_accepted);
 SourceObserver_Init(&p);SourceObserver_Service(0xfffffff0U);
 feed(WIRE,5);SourceObserver_Service(24);CK(g_source_observer_accepted);
 printf("SOURCE_STREAM checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''


def main():
    matlab = json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))
    assert matlab['passed']
    context = {'camera_id': 'SYNTHETIC_ONLY', 'mount_id': 'test', 'arm_pose_id': 'test',
               'plane_id': 'test', 'image_size': [1920, 1080], 'pixel_space': 'raw'}
    profile = {'context': context, 'homography_image_to_plane': None,
               'fit_hull_image_px': [[0,0],[1919,0],[1919,1079],[0,1079]],
               'independent_check_passed': True, 'field_calibration_accepted': True,
               'pose_binding_verified': True, 'plane_to_chassis_verified': True,
               'operational_extrapolation_enabled': False,
               'plane_to_chassis': [[0,-1,250],[1,0,30],[0,0,1]],
               'desired_disk_center_chassis_mm': matlab['synthetic_desired_center_chassis_mm']}
    checks = 0
    for case in matlab['cases']:
        profile['homography_image_to_plane'] = np.linalg.inv(case['plane_to_image']).tolist()
        q = preview_rim(case['boundary_px'], profile, frame_context=context, stable=True, confidence=.99, frame_age_ms=20, timing_verified=True)
        assert q['metric_observation_ready'] and not q['motion_allowed'], q
        assert np.linalg.norm(np.asarray(q['center_plane_mm'])-case['center_mm']) < 1e-8
        assert np.linalg.norm(np.asarray(q['delta_chassis_mm'])-matlab['synthetic_delta_chassis_mm']) < 1e-8
        checks += 3
    points = matlab['cases'][-1]['boundary_px']
    for change, kwargs, expected in [
        ({}, {'timing_verified': False}, False),
        ({'field_calibration_accepted': False}, {}, False),
        ({'pose_binding_verified': False}, {}, False),
        ({'plane_to_chassis_verified': False}, {}, False),
        ({}, {'frame_age_ms': 200}, False),
        ({}, {'stable': False}, False),
        ({}, {'confidence': .5}, False),
        ({'fit_hull_image_px': [[900,500],[920,500],[920,520],[900,520]]}, {}, False),
    ]:
        p = copy.deepcopy(profile);p.update(change)
        k = dict(frame_context=context, stable=True, confidence=.99, frame_age_ms=20, timing_verified=True);k.update(kwargs)
        q = preview_rim(points, p, **k)
        assert q['metric_observation_ready'] == expected and not q['motion_allowed'], q
        checks += 1
    bad_context = copy.deepcopy(context);bad_context['arm_pose_id'] = 'changed'
    assert preview_rim(points, profile, frame_context=bad_context, stable=True, confidence=.99)['status'] == 'CONTEXT_MISMATCH';checks += 1
    assert preview_rim(points, None, frame_context=context, stable=True, confidence=.99)['status'] == 'UNCONFIGURED';checks += 1
    for bad in [[], [[3,3]]*12, [[float('nan'),2]]*12, [[-2,1]]*12]:
        assert not preview_rim(bad, profile, frame_context=context, stable=True, confidence=.99)['metric_observation_ready'];checks += 1
    current = ROOT / 'evidence/integration/2026-11-05/source-markers/current-221545-a3467c481720'
    data = json.loads((current / 'measurement-input.json').read_text(encoding='utf-8'))
    archived = json.loads((ROOT / 'calibration/source_paper_current220715_20261105.json').read_text(encoding='utf-8'))
    q = preview_rim(data['rim_boundary_image_px'], archived, frame_context=archived['context'], stable=True, confidence=.99)
    assert q['status'] == 'PLANE_PREVIEW' and not q['metric_observation_ready'] and not q['motion_allowed'], q
    assert np.linalg.norm(np.asarray(q['center_plane_mm'])-matlab['archive_rim_center_plane_mm']) < 1e-6
    assert abs(q['radius_mm']-matlab['archive_rim_radius_mm']) < 1e-6;checks += 3
    (OUT / 'archive-rim-preview.json').write_text(json.dumps(q, ensure_ascii=False, indent=2), encoding='utf-8')
    # The CPU receiver receives the actual Python-encoded packet, and its MR1
    # is decoded back by the actual Python protocol implementation.
    wire = encode_observation(MetricObservation(1,2,2,1,2,14,10,20,30,40,50,1,10,20,950,1,-12500,2500,0))
    source, exe = OUT / 'observer-stream-host.c', OUT / 'observer-stream-host.exe'
    source.write_text('#define WIRE '+cstr(wire)+'\n'+UNIT, encoding='utf-8')
    cmd = ['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function',
           '-Wno-misleading-indentation','-DSOURCE_OBSERVER_HOST=1',
           '-I'+str(ROOT/'software_interfaces/source_observer'),'-I'+str(ROOT/'software_interfaces/stage2_metric_alignment'),
           '-I'+str(STAGE/'CHASSIS/USER'),str(source),str(ROOT/'software_interfaces/source_observer/source_observer.c'),'-o',str(exe)]
    built = subprocess.run(cmd,capture_output=True,text=True)
    (OUT/'stream-host-build.log').write_text(built.stdout+built.stderr,encoding='utf-8')
    assert built.returncode==0,built.stderr
    ran=subprocess.run([str(exe)],capture_output=True,text=True)
    (OUT/'stream-host-check.log').write_text(ran.stdout+ran.stderr,encoding='utf-8')
    print(ran.stdout);assert ran.returncode==0,ran.stdout+ran.stderr
    request_line=next(line[len('REQUEST:'):] for line in ran.stdout.splitlines() if line.startswith('REQUEST:'))
    req=decode_request((request_line+'\r\n').encode());assert req.request_id==1 and req.run_id==2 and req.stage==2;checks+=1
    valid_preview = preview_rim(points, profile, frame_context=context, stable=True, confidence=.99,
                                frame_age_ms=20, timing_verified=True)
    ids = {k:getattr(req,k) for k in ['camera_id','calibration_id','mount_id','height_id','target_id']}
    options = dict(profile_ids=ids, frame_sequence=2, capture_after_request_ms=10,
                   frame_age_ms=20, confidence=.99, timing_verified=True, translation_only_verified=True)
    packet = observation_packet(valid_preview, req, **options)
    o = decode_observation(packet);assert (o.dx_milli_mm,o.dy_milli_mm,o.yaw_mdeg)==(8000,-4000,0);checks+=1
    for change in [{'timing_verified':False},{'translation_only_verified':False},
                   {'frame_age_ms':200},{'confidence':.5},
                   {'profile_ids':dict(ids,camera_id=999)}]:
        bad=dict(options);bad.update(change)
        try:observation_packet(valid_preview,req,**bad)
        except ValueError:checks+=1
        else:raise AssertionError('unverified metric packet emitted')
    e2e_unit=UNIT.replace('==-12500','==8000').replace('==2500','==-4000')
    e2e_source,e2e_exe=OUT/'geometry-to-C-host.c',OUT/'geometry-to-C-host.exe'
    e2e_source.write_text('#define WIRE '+cstr(packet)+'\n'+e2e_unit,encoding='utf-8')
    command=cmd.copy();command[command.index(str(source))]=str(e2e_source);command[command.index(str(exe))]=str(e2e_exe)
    built=subprocess.run(command,capture_output=True,text=True);assert built.returncode==0,built.stderr
    e2e=subprocess.run([str(e2e_exe)],capture_output=True,text=True)
    (OUT/'geometry-to-C-check.log').write_text(e2e.stdout+e2e.stderr,encoding='utf-8');assert e2e.returncode==0,e2e.stdout+e2e.stderr
    for bad in [request_line.encode(),(request_line+'\r\n').encode().replace(b'MR1',b'MR2')]:
        try:decode_request(bad)
        except ValueError:checks+=1
        else:raise AssertionError('bad request accepted')
    # Exercise the real formal vehicle_arm position() implementation after
    # its node5 Scale fix; no invented driver or actuator completion flags.
    text=(STAGE/'CHASSIS/USER/vehicle_arm.c').read_text(encoding='utf-8')
    begin=text.index('static uint32_t position(void)')
    position=text[begin:function_end(text,'static uint32_t position(void)')+1]
    unit=r'''
#include <stdint.h>
#include <stdio.h>
#include <math.h>
typedef struct {uint8_t Data[8];} Reply;
static Reply s_rx;static float s_current;static uint8_t s_address,s_options;
static float g_vehicle_lift_current_deg;static uint32_t g_vehicle_lift_feedback_valid;
static uint32_t query(uint8_t cmd,uint8_t n){return cmd==0x36 && n==7;}
POSITION
int main(void){unsigned checks=0;uint32_t raw[]={0,27000,144017,1219509};
 for(unsigned addr=5;addr<=6;addr++)for(unsigned scale=10;scale<=100;scale+=90)
 for(unsigned sign=0;sign<2;sign++)for(unsigned i=0;i<4;i++){
  s_address=addr;s_options=scale==100?0x80:0;s_rx.Data[1]=sign;
  s_rx.Data[2]=raw[i]>>24;s_rx.Data[3]=raw[i]>>16;s_rx.Data[4]=raw[i]>>8;s_rx.Data[5]=raw[i];
  if(!position() || fabs(s_current-(sign?-1.:1.)*raw[i]/scale)>.01)return 1;checks++;
 }
 puts("NODE5_SCALE checks=32 failures=0");return 0;
}
'''.replace('POSITION',position)
    scale_src,scale_exe=OUT/'node5-scale-host.c',OUT/'node5-scale-host.exe';scale_src.write_text(unit,encoding='utf-8')
    built=subprocess.run(['C:/mingw/mingw64/bin/gcc.exe','-std=c99',str(scale_src),'-lm','-o',str(scale_exe)],capture_output=True,text=True);assert built.returncode==0,built.stderr
    ran2=subprocess.run([str(scale_exe)],capture_output=True,text=True);assert ran2.returncode==0,ran2.stdout+ran2.stderr;print(ran2.stdout)
    report={'passed':True,'python_geometry_and_protocol_checks':checks,'MATLAB_checks':matlab['check_count'],
            'C_stream_output':ran.stdout.strip(),'geometry_to_C_output':e2e.stdout.strip(),'formal_position_scale_output':ran2.stdout.strip(),
            'actual_camera_or_robot_operated':False,'motion_commands':0,'field_profile_calibrated':False,
            'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
              [ROOT/'vision_app/code/source_metric_preview.py',ROOT/'vision_app/code/source_metric_bridge.py',ROOT/'vision_app/code/metric_alignment_protocol.py',ROOT/'software_interfaces/source_observer/source_observer.c']}}
    (OUT/'checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'passed':True,'python_checks':checks,'MATLAB_checks':matlab['check_count']}))


if __name__=='__main__':
    main()
