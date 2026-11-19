"""Synthetic geometric/protocol tests, no camera/UART/hardware connection."""
from pathlib import Path
import binascii,json,sys
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'vision_app/code'))
from source_alignment_session import SourceAlignmentSession
from metric_alignment_protocol import MetricRequest,decode_observation
E=R/'evidence/integration/2026-11-09/source-station-offline'
context={'camera_id':'test','mount_id':'test','arm_pose_id':'test','plane_id':'test',
 'pixel_space':'raw','image_size':[1000,1000]}
cal={'context':context,'homography_image_to_plane':[[1,0,0],[0,1,0],[0,0,1]],
 'plane_to_chassis':[[1,0,0],[0,1,0],[0,0,1]],'desired_disk_center_chassis_mm':[400,400],
 'independent_check_passed':True,'field_calibration_accepted':True,'pose_binding_verified':True,
 'plane_to_chassis_verified':True,'operational_extrapolation_enabled':True,
 'rim_limits':{'minimum_points':12,'minimum_arc_deg':90,'maximum_rms_mm':3,'radius_range_mm':[130,170]}}
ids=dict(camera_id=1,calibration_id=2,mount_id=3,height_id=4,target_id=5)
req=MetricRequest(1,9,2,1,2,14,1000,1,2,3,4,5,200,900,20000,1000,2060000,1200000)
from dataclasses import asdict
body=('MR1,'+','.join(map(str,asdict(req).values()))+',').encode()
wire=body+f'{binascii.crc_hqx(body,0xffff):04X}\r\n'.encode()
t=np.linspace(0,2*np.pi,120,endpoint=False);points=np.c_[412+150*np.cos(t),395+150*np.sin(t)]
session=SourceAlignmentSession(cal,context,ids)
def observe(now=150,capture=125,timing=True):
 return session.observe(points,frame_sequence=1,captured_ms=capture,now_ms=now,stable=True,
  confidence=.99,exposure_time_verified=timing,translation_only_verified=True)
n=0
assert observe()[0] is None;n+=1
session.receive(wire[:30],100);session.receive(wire[30:],100)
packet,preview=observe();o=decode_observation(packet)
assert o.dx_milli_mm==12000 and o.dy_milli_mm==-5000;n+=1
assert observe(timing=False)[0] is None;n+=1
assert observe(capture=99)[0] is None;n+=1
assert observe(capture=160)[0] is None;n+=1
session.receive(wire,400);assert session.received_ms==100;n+=1
assert observe(now=1100,capture=1090)[0] is None;n+=1
session.receive(wire,1200);session.calibration=dict(cal,plane_to_chassis_verified=False)
assert observe(now=1250,capture=1225)[0] is None;n+=1
session.calibration=cal;bad=bytearray(wire);bad[-4]^=1;session.receive(bytes(bad),1300)
assert session.request is None;n+=1
session.receive(b'MR1,'+b'1'*300+b'\n',1400);assert session.request is None;n+=1
session.receive(b'\xaa\x05\xbb'+wire,1500);assert session.request is not None;n+=1
out={'passed':True,'checks':n,'actual_python_session_and_existing_geometry_and_codec':True,
 'synthetic_points_and_calibration':True,'hardware_access':False,'sample_delta_mm':[12,-5]}
(E/'python-alignment-checks.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out))
