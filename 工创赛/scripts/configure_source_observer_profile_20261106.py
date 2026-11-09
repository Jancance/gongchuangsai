"""Generate the candidate C profile only from explicitly accepted field data."""
from pathlib import Path
import argparse
import json


def render(profile, geometry):
    if (profile.get('calibrated') is not True or
            any(geometry.get(k) is not True for k in ['independent_check_passed', 'field_calibration_accepted',
                                                     'pose_binding_verified', 'plane_to_chassis_verified'])):
        raise ValueError('field calibration/pose/body transform must be explicitly accepted')
    names=['camera_id','calibration_id','mount_id','height_id','target_id']
    if any(type(profile.get(k)) is not int or not 1<=profile[k]<2**32 for k in names):
        raise ValueError('nonzero actual version identifiers required')
    age,confidence,delta,yaw=[profile.get(k) for k in ['max_frame_age_ms','min_confidence_permille','max_delta_milli_mm','max_yaw_mdeg']]
    if (any(type(v) is not int for v in [age,confidence,delta,yaw]) or not 1<=age<=200 or
            not 1<=confidence<=1000 or not 1<=delta<=100000 or not 1<=yaw<=10000):
        raise ValueError('invalid observation limits')
    values=[1]+[profile[k] for k in names]+[age,confidence,delta,yaw]
    return '#ifndef SOURCE_OBSERVER_CONFIG_H\n#define SOURCE_OBSERVER_CONFIG_H\n#define SOURCE_OBSERVER_PROFILE_INITIALIZER {'+','.join(str(v)+'U' for v in values)+'}\n#endif\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    text=render(json.loads(args.profile.read_text(encoding='utf-8')),json.loads(args.geometry.read_text(encoding='utf-8')))
    if args.output.exists():raise ValueError('preserve existing profile header; choose a new output file')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(text,encoding='utf-8')
    print('Profile header prepared only; firmware not built, installed or flashed.')


if __name__=='__main__':main()
