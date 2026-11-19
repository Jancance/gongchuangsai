% Pure synthetic prevalidation of A2/B3 calibration mathematics. No hardware.
% R nominal geometry: official task PDF p5/p6 => diameter95 from phi50;
% E spacing150 by symmetric drawing; affine approximation is not calibration proof.
% Q independent baseline definition/manufacturing tolerance remains unresolved.
root=fileparts(fileparts(mfilename('fullpath')));
checks=struct('name',{},'passed',{},'details',{});
r=95/2; spacing=150;
caseCount=0; maxAError=0; maxAngleError=0;
for deg=[-60 -20 0 25 60]
    rot=[cosd(deg) -sind(deg);sind(deg) cosd(deg)];
    for sx=[2 4]
        for sy=[1.5 3]
            for shear=[-0.3 0 0.4]
                J=rot*[sx shear;0 sy];
                % Ambiguous second column fixed by explicit image-y-positive convention.
                if J(2,2)<0, J(:,2)=-J(:,2); end
                c0=[320;240]; c1=c0-J(:,1)*spacing; c3=c0+J(:,1)*spacing;
                Q=r^2*(J*J.');
                [A,rankResidual]=fitA2(c1,c3,Q,r,spacing);
                maxAError=max(maxAError,norm(A*J-eye(2),'fro'));
                assert(norm(A*J-eye(2),'fro')<1e-10);
                assert(rankResidual<1e-10);
                for v={[8;-5],[-12;7],[0;0]}
                    physical=v{1}; pixel=J*physical;
                    assert(norm(A*pixel-physical)<1e-9);
                end
                for yaw=[-8 -1.001 -1 0 1 1.001 8]
                    imageLine=J*[cosd(yaw);sind(yaw)];
                    theta=atan2d(imageLine(2),imageLine(1));
                    converted=metricYaw(A,theta);
                    e=wrapLine(converted-yaw); maxAngleError=max(maxAngleError,abs(e));
                    assert(abs(e)<1e-9);
                    assert(abs(wrapLine(metricYaw(A,theta+180)-converted))<1e-9);
                end
                caseCount=caseCount+1;
            end
        end
    end
end
addCheck('A2_60_affine_scale_rotation_anisotropy_shear',true, ...
    'Q/r2-vxvxT recovers second column up to sign; image-y-positive convention; independent synthetic XY recoveries and transformed line yaw');

% Recover from an explicit ellipse axis/rotation representation, not only Q.
J=[3.1 0.4;0.6 2.2]; Q=r^2*J*J.';
[V,L]=eig(Q); ellipseAxes=2*sqrt(diag(L));
QfromEllipse=V*diag((ellipseAxes/2).^2)*V.';
[A,~]=fitA2(-spacing*J(:,1),spacing*J(:,1),QfromEllipse,r,spacing);
assert(norm(A*J-eye(2),'fro')<1e-10);
% Outer contour diameter95 differs from black-line-center diameter93.5.
[wrongA,wrongResidual]=fitA2(-spacing*J(:,1),spacing*J(:,1),QfromEllipse,93.5/2,spacing);
assert(norm(wrongA-A,'fro')>1e-4 && wrongResidual>0);
addCheck('ellipse_representation_and_contour_definition',true, ...
    'ellipse axes are full diameters;95 outer edge versus93.5 line center mismatch yields metric bias and nonzero rank residual');

% Mathematically unidentifiable A2 must not be represented as a measured inverse.
assert(fails(@()fitA2([0;0],[0;0],zeros(2),r,spacing)));
Jflat=[3 0;0 0];
assert(fails(@()fitA2(-spacing*Jflat(:,1),spacing*Jflat(:,1),r^2*(Jflat*Jflat.'),r,spacing)));
addCheck('A2_degeneracy',true,'zero ellipse and rank-deficient projection cannot produce an invertible local map');

for a=[-8 -1.001 -1 -0.999 0 0.999 1 1.001 8 179 -179]
    assert(abs(wrapLine(a)-wrapLine(a+180))<1e-10);
    assert(abs(wrapLine(a)-wrapLine(a-180))<1e-10);
end
assert(wrapLine(179)==-1 && wrapLine(-179)==1);
assert(wrapLine(180)==0 && wrapLine(-180)==0);
assert(abs(wrapLine(1))<=1 && abs(wrapLine(1.001))>1);
addCheck('undirected_baseline_180_equivalence',true, ...
    'metric yaw difference uses modulo180 [-90,90), not modulo360; flipped line endpoints never imply chassis180deg; exact90 remains ambiguous');

% B3 units: commands right_mm/forward_mm/CCW_deg; observations station_mm/mm/deg.
C=[10 0 0;0 10 0;0 0 2;-10 0 0;0 -10 0;0 0 -2;5 -4 1];
responses=cat(3,-eye(3),[-1 .02 .3;.01 -1 -.4;.002 -.003 -1], ...
    [1.2 -.1 .5;.07 -.8 -.2;.01 .02 .9]);
validationC=[3 2 .4;-6 7 -.7;4 -8 1.2];
maxFitError=0; maxValidationError=0;
for k=1:size(responses,3)
    R=responses(:,:,k); D=C*R.';
    [B,Rfit,fitResidual]=fitB3(C,D);
    assert(norm(Rfit-R,'fro')<1e-10);
    assert(norm(B+inv(R),'fro')<1e-10);
    maxFitError=max(maxFitError,norm(fitResidual,'fro'));
    independentD=validationC*R.';
    independentResidual=independentD-validationC*Rfit.';
    maxValidationError=max(maxValidationError,norm(independentResidual,'fro'));
    assert(norm(independentResidual,'fro')<1e-9);
    for error={[4;-5;1],[-2;3;-1],[0;0;0]}
        e=error{1}; command=B*e;
        assert(norm(e+R*command)<1e-9);
    end
end
[B,~,~]=fitB3(C,-C);
assert(norm(B-eye(3),'fro')<1e-10); % positive target error => positive right/forward/CCW.
assert(fails(@()fitB3([10 0 0;20 0 0;30 0 0],zeros(3))));
assert(fails(@()fitB3(eye(3),diag([1 1 0]))));
addCheck('B3_units_signs_cross_coupling_and_rank',true, ...
    'D=C*Rtranspose; B=-invR maps error to right_mm/forward_mm/CCWdeg; full-rankcommands and response required; +/-axes and cross coupling close ideal error');

% Three interpolation pairs can fit exactly while new independent data fails.
Cmin=diag([10 10 2]); R=-eye(3); Dmin=Cmin*R.';
[~,Rmin,fitMin]=fitB3(Cmin,Dmin);
assert(norm(fitMin,'fro')<1e-12);
heldOutD=validationC*R.'+[1.2 -.8 .3;-.5 1.1 -.2;.7 -.6 .4];
heldOutResidual=heldOutD-validationC*Rmin.';
assert(norm(heldOutResidual,'fro')>1);
heldOutPlane=sqrt(sum(heldOutResidual(:,1:2).^2,2));
heldOutYaw=heldOutResidual(:,3);
addCheck('fit_residual_is_not_independent_validation',true, ...
    'three exact fit pairs yield zero training residual but noisy independent pairs remain nonzero; XYnorm mm and yaw deg reported separately; no automatic certification');

result=struct('status','PASS_SYNTHETIC_CALIBRATION_MATH_ONLY', ...
    'generated_at',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')), ...
    'matlab_version',version,'checks',checks,'affine_cases',caseCount, ...
    'max_A_identity_error',maxAError,'max_transformed_yaw_error_deg',maxAngleError, ...
    'max_exact_training_residual',maxFitError,'max_exact_independent_residual',maxValidationError, ...
    'zero_training_but_bad_validation_plane_mm',heldOutPlane, ...
    'zero_training_but_bad_validation_yaw_deg',heldOutYaw, ...
    'physical_calibration_verified',false);
result.requirement_ids={'VIS-04','VIS-05','VIS-07','VIS-12','VIS-15','CAL-02','CAL-03','CAL-05','RUN-06'};
result.assumptions={'local affine plane and true outer ellipse, not projective calibration'; ...
    'A2 station frame sign chosen by image-y-positive convention; B3 measures physical command signs'; ...
    '95mm nominal outer contour and150mm symmetric-drawing spacing are not field measurements'; ...
    'known command values only model response per command, not independent proof of actual chassis travel'; ...
    'B3 applies locally for small turns and fixed camera/TCP, not arbitrary90/180 chassis turns'; ...
    'synthetic post-turn frames represent re-observation, not real camera capture'};
result.not_verified={'real frame extraction and ellipse/line detection'; ...
    'lens distortion, perspective variation, camera flex, TCP reference'; ...
    'true right/forward/CCW directions and units on this vehicle'; ...
    'real independent validation residuals, physical2mm/1deg precision'; ...
    'firmware, hardware motion, reachability or placement success'};
out=fullfile(root,'evidence','integration','2026-11-16','stage3','calibration-matlab-result.json');
fid=fopen(out,'w','n','UTF-8'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fwrite(fid,jsonencode(result),'char'); clear cleanup;
fprintf('STAGE3 CALIBRATION MATLAB PASS: %d groups, %d affine cases; physical verified=false.\n',numel(checks),caseCount);

function [A,residual]=fitA2(c1,c3,Q,r,s)
    vx=(c3-c1)/(2*s); q=Q/r^2-vx*vx.'; q=(q+q.')/2;
    [v,l]=eig(q); [values,order]=sort(diag(l));
    assert(values(end)>0,'Unidentifiable projection');
    vy=v(:,order(end))*sqrt(values(end)); if vy(2)<0, vy=-vy; end
    J=[vx vy]; assert(rank(J)==2,'Singular local map');
    A=inv(J); residual=abs(values(1))/values(end);
end
function y=metricYaw(A,t)
    v=A*[cosd(t);sind(t)]; y=atan2d(v(2),v(1));
end
function y=wrapLine(x), y=mod(x+90,180)-90; end
function [B,R,residual]=fitB3(C,D)
    assert(rank(C)==3,'Need independent three-axis commands');
    R=(C\D).'; assert(rank(R)==3,'Singular response');
    B=-inv(R); residual=D-C*R.';
end
function yes=fails(f)
    yes=false; try, f(); catch, yes=true; end
end
function addCheck(name,passed,details)
    checks=evalin('base','checks'); checks(end+1)=struct('name',name,'passed',passed,'details',details);
    assignin('base','checks',checks);
end
