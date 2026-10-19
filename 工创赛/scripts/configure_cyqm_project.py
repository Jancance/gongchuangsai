"""One-time initial camera/config migration; never run after user calibration."""
from pathlib import Path
import json

root=Path(__file__).resolve().parents[1]/'vision_app'
config=root/'code/config';reference=root/'reference_config'
marker=config/'project_profile.json'
if marker.exists():
    # An initial stub profile may already exist, but never replace calibrated
    # user configs. Migration is allowed only while the six source configs
    # still match their exact imported reference bytes.
    untouched=('material_color_thresholds.json','material_pickup_detector.json',
               'turntable_material_seed.json','turntable_detector.json',
               'ring_station_detector.json','stacked_ring_detector.json')
    if not all((config/n).read_bytes()==(reference/n).read_bytes() for n in untouched):
        raise SystemExit('已适配或用户已标定；拒绝覆盖当前配置')

def old(name):return json.loads((reference/name).read_text(encoding='utf-8'))
def write(name,data):(config/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

write('camera_processing.json',{'undistort_enabled':False,'reason':'DECXIN residual distortion not verified; no upstream calibration'})
colors=old('material_color_thresholds.json')
colors.update(camera_device='/dev/v4l/by-id/usb-DECXIN_DECXIN_CAMERA_01.00.00-video-index0',
              image_width=1280,image_height=960,calibration_file='config/decxin_camera_unconfigured.yaml',
              calibrated_for_this_camera=False,source='project candidate HSV ranges; recalibrate with own samples')
ranges={1:[([0,90,35],[12,255,255]),([165,90,35],[179,255,255])],
        2:[([18,90,35],[34,255,255])],3:[([100,140,35],[135,255,255])],
        4:[([35,60,35],[85,255,255])],5:[([0,0,0],[179,255,44])],
        6:[([90,25,100],[115,139,255])]}
for c in colors['colors']:c['ranges']=[{'lower':a,'upper':b} for a,b in ranges[c['color_id']]]
write('material_color_thresholds.json',colors)
material=old('material_pickup_detector.json')
material.update(image_width=1280,image_height=960,calibrated=False,
                regions={n:None for n in ['workspace','left','right','down']})
material['detection'].update(maximum_area_px=150000.,maximum_dimension_px=600.)
write('material_pickup_detector.json',material)
seed=old('turntable_material_seed.json');seed['enabled']=False
seed['reference']={'center_px':[640.,480.],'radius_px':300.}
seed['reference_confirmed']=False
write('turntable_material_seed.json',seed)
turntable=old('turntable_detector.json');turntable['parameters_verified_for_this_camera']=False
write('turntable_detector.json',turntable)
ring=old('ring_station_detector.json');ring['profile_name']='decxin_unconfigured_station'
ring['reference']['ring_centers_px']=[[360.,480.],[640.,480.],[920.,480.]]
ring['reference']['image_to_station_affine']=[[1.,0.],[0.,1.]]
ring['correction_calibration']['chassis_image_to_motion']=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
ring.update(calibration_verified=False,placeholder_note='Structure-only parser defaults; mm corrections must never be treated as valid')
write('ring_station_detector.json',ring)
stack=old('stacked_ring_detector.json');stack['search_calibration']=None;stack['calibration_verified']=False
write('stacked_ring_detector.json',stack)
write('project_profile.json',{'schema_version':1,'camera_model':'DECXIN user 77deg/960P variant',
    'image_size':[1280,960],'camera_verified':False,'geometry_verified':False,
    'ring_geometry_verified':False,'stack_geometry_verified':False,
    'serial_enabled':False,'tracking_units':'pixels','follow_grasp_enabled':False,
    'capture_timestamps':'host_read_completion','raw_turntable_height_range_mm':[80,100],
    'raw_turntable_height_mm':None,'height_version':None,
    'note':'80-100mm is user engineering range CAL-05; actual height must be measured'})
(config/'decxin_camera_unconfigured.yaml').write_text(
    '%YAML:1.0\n---\ncamera_name: decxin_main_unconfigured\nimage_width: 1280\nimage_height: 960\ncalibrated: 0\n',encoding='utf-8')
print('DECXIN initial configs written; references retained, geometry and serial unverified.')
