"""Offline candidate from four measured encoder/HWT responses; MATLAB prevalidation required."""
from pathlib import Path
import hashlib
import json
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'evidence/integration/2026-11-16/stage3-encoder-vision'


def fit():
    data=json.loads((FOLDER/'input.json').read_text(encoding='utf-8'))
    matlab=json.loads((FOLDER/'matlab-result.json').read_text(encoding='utf-8'))
    assert matlab['mathematics_passed'], 'MATLAB mathematical checks required first'
    geometry=data['geometry'];direction=np.asarray(geometry['forward_direction'])
    transform=np.array([[1,1,-1],[1,-1,1],[1,-1,-1],[1,1,1]],dtype=float)
    inputs=[];outputs=[]
    for row in data['rows']:
        source=ROOT/row['analysis_file']
        assert hashlib.sha256(source.read_bytes()).hexdigest()==row['analysis_sha256']
        signed=lambda name:np.asarray(row[name+'_raw'],dtype=float)*(1-2*np.asarray(row[name+'_sign']))
        delta=signed('end')-signed('start')
        wheel=delta*np.pi*geometry['wheel_diameter_mm']/65536*(1-2*direction)
        body=transform.T@wheel/4
        inputs.append([-body[1],body[0],row['hwt_delta_deg']])
        outputs.append([*row['delta_center_px'],row['delta_line_deg']])
    x=np.array(inputs);y=np.array(outputs)
    assert np.allclose(x,matlab['actual_X_right_forward_encoder_mm_HWT_deg'],atol=1e-12)
    assert np.allclose(y,matlab['actual_Y_du_dv_line_deg'],atol=1e-12)
    coefficients=np.linalg.lstsq(x,y,rcond=None)[0]
    assert np.allclose(coefficients,matlab['response_coefficients_input_by_output'],atol=1e-10)
    loo=[]
    for held,row in enumerate(data['rows']):
        keep=np.arange(len(x))!=held
        fit_without=np.linalg.lstsq(x[keep],y[keep],rcond=None)[0]
        residual=y[held]-x[held]@fit_without
        assert np.allclose(residual,matlab['leave_one_out'][held]['residual'],atol=1e-10)
        loo.append(dict(held_out=row['label'],residual_du_dv_line_deg=residual.tolist(),
                        pixel_xy_norm=float(np.linalg.norm(residual[:2]))))
    result=dict(matlab_prevalidated=True,hardware_access=False,
        input_sha256=hashlib.sha256((FOLDER/'input.json').read_bytes()).hexdigest(),
        matlab_result_sha256=hashlib.sha256((FOLDER/'matlab-result.json').read_bytes()).hexdigest(),
        row_labels=[r['label'] for r in data['rows']],
        inputs_right_forward_encoder_mm_hwt_deg=x.tolist(),outputs_du_dv_line_deg=y.tolist(),
        response_input_rows_by_output_columns=coefficients.tolist(),
        negative_response_inverse_encoder_candidate=-np.linalg.inv(coefficients.T),
        rank=int(np.linalg.matrix_rank(x)),column_normalized_condition=float(np.linalg.cond(x/np.linalg.norm(x,axis=0))),
        training_residual=(y-x@coefficients).tolist(),leave_one_out=loo,
        yaw_sensitivity=matlab['yaw_sensitivity'],calibration_verified=False,
        tcp_reference_confirmed=False,actual_precision_accepted=False,effective_config_changed=False,
        limitations=['Four observations give one residual degree of freedom.',
                     'Encoder distance uses nominal wheel geometry and is not independently measured body travel.',
                     'The inverse targets encoder displacement, not a validated command-to-physical movement map.',
                     'Pixel residuals and image-line angle residuals do not certify 2mm/1degree physical precision.',
                     'No TCP/drop acceptance; active Pi and MCU mappings remain unchanged.'])
    result['negative_response_inverse_encoder_candidate']=result['negative_response_inverse_encoder_candidate'].tolist()
    return result


if __name__=='__main__':
    output=FOLDER/'candidate.json';result=fit()
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(dict(output=str(output),rank=result['rank'],condition=result['column_normalized_condition'],
                          leave_one_out=result['leave_one_out'],calibration_verified=False)))
