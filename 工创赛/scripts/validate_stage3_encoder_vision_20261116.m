% Data-only encoder/vision identification prevalidation. No motion or new gates.
root=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(root,'evidence','integration','2026-11-16','stage3-encoder-vision');
data=jsondecode(fileread(fullfile(folder,'input.json')));
diameter=data.geometry.wheel_diameter_mm;
reversal=data.geometry.forward_direction(:).';
% Actual C order LR/RR/LF/RF: [f+l-t, f-l+t, f-l-t, f+l+t].
M=[1 1 -1;1 -1 1;1 -1 -1;1 1 1];
assert(isequal(M.'*M,4*eye(3)));
maximumRoundtrip=0;
for body=[10 0 0;0 10 0;0 0 1;-4 7 -2;5 -3 .6].'
    wheel=(M*body).';
    rawSigned=wheel.*(1-2*reversal)*65536/(pi*diameter);
    signValue=rawSigned<0;rawValue=abs(rawSigned);
    recoveredWheel=(rawValue.*(1-2*signValue))*(pi*diameter)/65536.*(1-2*reversal);
    recoveredBody=M.'*recoveredWheel.'/4;
    maximumRoundtrip=max(maximumRoundtrip,max(abs(recoveredBody-body)));
end
assert(maximumRoundtrip<1e-10);
% Synthetic independent response with four rows and exact holdout recovery.
syntheticX=[10 0 .7;10 0 -.05;0 10 -.7;0 10 .17];
truth=[.1 2.6 .0;2.8 .1 .0;-12 13 1];
syntheticY=syntheticX*truth;
assert(norm(syntheticX\syntheticY-truth,'fro')<1e-10);
for held=1:4
    training=setdiff(1:4,held);fit=syntheticX(training,:)\syntheticY(training,:);
    assert(norm(syntheticX(held,:)*fit-syntheticY(held,:))<1e-10);
end

X=zeros(4,3);Y=zeros(4,3);wheelSigned=zeros(4,4);
for i=1:4
    r=data.rows(i);
    delta=r.end_raw(:).'.*(1-2*r.end_sign(:).')-r.start_raw(:).'.*(1-2*r.start_sign(:).');
    wheelSigned(i,:)=delta*pi*diameter/65536.*(1-2*reversal);
    body=M.'*wheelSigned(i,:).'/4;
    X(i,:)=[-body(2),body(1),r.hwt_delta_deg];
    Y(i,:)=[r.delta_center_px(:).',r.delta_line_deg];
end
assert(all(isfinite(X(:))) && all(isfinite(Y(:))));
report=struct;
report.mathematics_passed=true;
report.synthetic_roundtrip_max_error=maximumRoundtrip;
report.actual_X_right_forward_encoder_mm_HWT_deg=X;
report.actual_Y_du_dv_line_deg=Y;
report.actual_wheel_forward_mm=wheelSigned;
report.rank=rank(X);
report.column_normalized_condition=cond(X./vecnorm(X));
if rank(X)==3
    coefficients=X\Y;
    report.response_coefficients_input_by_output=coefficients;
    report.training_residual=Y-X*coefficients;
    loo=repmat(struct('held',0,'rank',0,'condition',0,'residual',[]),4,1);
    for held=1:4
        training=setdiff(1:4,held);train=X(training,:);
        loo(held).held=held;loo(held).rank=rank(train);
        loo(held).condition=cond(train./vecnorm(train));
        if rank(train)==3
            local=train\Y(training,:);
            loo(held).residual=Y(held,:)-X(held,:)*local;
        end
    end
    report.leave_one_out=loo;
    sensitivity=struct('assumed_yaw_perturbation_deg',{},'max_coefficient_change_fro',{}, ...
        'max_predicted_observation_change_at_original_X',{});
    for amplitude=[.05 .1]
        coefficientChange=0;predictionChange=zeros(1,3);
        for bits=0:15
            changed=X;changed(:,3)=changed(:,3)+amplitude*(2*bitget(bits,1:4).'-1);
            if rank(changed)<3,continue;end
            perturbed=changed\Y;
            coefficientChange=max(coefficientChange,norm(perturbed-coefficients,'fro'));
            predictionChange=max(predictionChange,max(abs(X*(perturbed-coefficients)),[],1));
        end
        sensitivity(end+1)=struct('assumed_yaw_perturbation_deg',amplitude, ...
            'max_coefficient_change_fro',coefficientChange, ...
            'max_predicted_observation_change_at_original_X',predictionChange);
    end
    report.yaw_sensitivity=sensitivity;
end
report.actual_precision_accepted=false;
report.calibration_verified=false;
report.notes={'Mathematical numerical assertions only; no added operational thresholds.', ...
    'Encoder projections are nominal wheel geometry, not independently measured body travel.', ...
    'HWT and camera line sign are both observed positive in right-A and negative in forward-A.', ...
    'Yaw perturbations are declared diagnostic scenarios, not measured uncertainty.', ...
    'Four observations fit three inputs: one residual degree of freedom; no physical accuracy certification.', ...
    'Only actual four-wheel C kinematics; no command10 substitution, no forward-scale compensation in raw projections.'};
fid=fopen(fullfile(folder,'matlab-result.json'),'w','n','UTF-8');
fwrite(fid,jsonencode(report),'char');fclose(fid);
disp(jsonencode(report));
