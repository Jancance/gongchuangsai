"""Archive MATLAB's current view models as inactive diagnostics only."""
from pathlib import Path
import hashlib,json
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-11/source-view-entry/paper50-geometry'
def main():
    measurement=json.loads((E/'measurement-input.json').read_text(encoding='utf-8'))
    result=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    models=json.loads((E/'model-comparison.json').read_text(encoding='utf-8'))
    assert result['passed'] and result['paper_width_mm']==50 and result['paper_height_mm']==50
    assert not models['model_selected_for_motion'] and not result['motion_allowed']
    src=E.parent/'source-observe-20261111-033718/last-raw.png'
    assert hashlib.sha256(src.read_bytes()).hexdigest()==result['image_sha256']
    output=R/'calibration/source_paper50_view65_20261111.diagnostic.json'
    profile=dict(schema_version=1,kind='inactive_model_comparison_not_runtime_calibration',
        calibration_id='source-view65-paper50-'+result['image_sha256'][:12]+'-diagnostic',
        paper_width_mm=50,paper_height_mm=50,dimensions_basis=measurement['size_basis'],
        context=dict(image_size=[1920,1080],pixel_space='raw',camera_id='Generic_USB_CAMERA_EP.20CC54K01',
            yaw_observed_deg=measurement['yaw_feedback_deg'],extension_command_mm=65,plane_z_mm=None,
            plane_axes=result['plane_axes']),
        image_sha256=result['image_sha256'],image_path=src.relative_to(R).as_posix(),
        configs=measurement['configs'],models=models['models'],
        matlab_result=(E/'matlab-result.json').relative_to(R).as_posix(),
        model_comparison=(E/'model-comparison.json').relative_to(R).as_posix(),
        homography_image_to_plane=None,plane_to_chassis=None,desired_disk_center_chassis_mm=None,
        independent_check_passed=False,field_calibration_accepted=False,
        plane_to_chassis_verified=False,pose_binding_verified=False,
        full_plane_metric_accuracy_accepted=False,material_top_mapping_verified=False,
        sensor_exposure_age_verified=False,operational_extrapolation_enabled=False,motion_allowed=False,
        next='Independent ruler length outside50mm paper; keep scene fixed; select model only after evidence.')
    ruler_path=E.parent/'ruler-review/matlab-result.json'
    if ruler_path.exists():
        ruler=json.loads(ruler_path.read_text(encoding='utf-8'))
        assert ruler['passed'] and not ruler['motion_allowed']
        simple=next(x for x in ruler['models'] if x['model']=='similarity')
        assert simple['ruler_strip_within3mm']
        profile.update(preferred_diagnostic_model='similarity',single_ruler_strip_validated=True,
            ruler_validation=ruler_path.relative_to(R).as_posix(),
            ruler_span_200mm_measured_mm=simple['span_200mm_measured_mm'],
            ruler_pair_max_abs_error_mm=simple['pair_max_abs_error_mm'],
            ruler_scope=ruler['criterion_scope'],
            next='Clean empty-disk target circles and actual chassis-axis/parking reference binding; no motion yet.')
    output.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(output)
if __name__=='__main__':main()
