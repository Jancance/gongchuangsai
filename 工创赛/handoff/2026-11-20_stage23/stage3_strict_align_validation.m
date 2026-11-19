function report=stage3_strict_align_validation(minMoveUm,minTurnMdeg)
% Offline geometry and admission model. Motion limits must be supplied after review.
assert(nargin==2,'Supply the reviewed translation and turn command floors.');
validateattributes(minMoveUm,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(minTurnMdeg,{'numeric'},{'scalar','real','finite','positive'});
work=fileparts(mfilename('fullpath'));
cfg=jsondecode(fileread(fullfile(work,'stage3_adjust_recover_20261119','stage3-vision-13-5.json')));
row=cfg.ring_row_alignment;
assert(~cfg.calibration_verified && ~row.physical_placement_verified);
J=row.response_pixel_per_right_forward_ccw;
A=row.nominal_image_to_station_affine;
assert(all(size(J)==[3 3]) && all(size(A)==[2 2]) && rcond(J)>0);

limits=[500 500]; floor=[minMoveUm minTurnMdeg];
ctx=struct('state',2,'run',12,'pose',7,'seq',9,'tick',1000);
f=observation(ctx,[300 400],500);
assert(decision(f,ctx,limits,floor)=="accept");
assert(decision(observation(ctx,[300 400],-500),ctx,limits,floor)=="accept");
assert(decision(observation(ctx,[300 401],500),ctx,limits,floor)~="accept");
assert(decision(observation(ctx,[300 400],501),ctx,limits,floor)~="accept");
assert(decision(observation(ctx,[400 0],400),ctx,limits,floor)=="accept");
for key=["run","pose","seq","crcOK","valid","state"]
    g=f; c=ctx;
    switch key
        case "seq", g.seq=ctx.seq;
        case "crcOK", g.crcOK=false;
        case "valid", g.valid=false;
        case "state", c.state=3;
        otherwise, g.(key)=f.(key)+1;
    end
    assert(decision(g,c,limits,floor)=="observe");
end
g=f; g.seq=0; assert(decision(g,ctx,limits,floor)=="observe");
g=f; g.seq=ctx.seq-1; assert(decision(g,ctx,limits,floor)=="observe");
g=f; g.age=150; g.tick=ctx.tick-50;
assert(decision(g,ctx,limits,floor)=="accept");
g.tick=g.tick-1; assert(decision(g,ctx,limits,floor)=="observe");
g=f; g.age=201; assert(decision(g,ctx,limits,floor)=="observe");
g=f; g.tick=ctx.tick-201; assert(decision(g,ctx,limits,floor)=="observe");
g=f; c=ctx; c.tick=2; g.tick=2^32-2; g.age=196;
assert(decision(g,c,limits,floor)=="accept");
c.seq=2^32-2; g.seq=2;
assert(decision(g,c,limits,floor)=="accept");

diag=observation(ctx,[400 400],0);
[diagAction,axis,amount]=decision(diag,ctx,limits,floor);
assert(diag.normUm==566 && diagAction~="accept");
if minMoveUm<=400, assert(diagAction=="translate" && axis==1 && amount==400); end
if minMoveUm>=1000, assert(diagAction=="observe"); end
% Continuous Euclidean lower bound; rounded action and verifier norm are separate wire fields.
dominantLowerUm=500/sqrt(2);
for theta=linspace(0,2*pi,721)
    v=500.001*[cos(theta) sin(theta)];
    assert(max(abs(v))>=dominantLowerUm);
end
[yawOnly,~,yawAmount]=decision(observation(ctx,[0 0],501),ctx,limits,floor);
if minTurnMdeg<=501, assert(yawOnly=="yaw" && yawAmount==501); end
g=observation(ctx,[0 0],600); g.turnMdeg=300;
if minTurnMdeg>300, assert(decision(g,ctx,limits,floor)=="observe"); end

legacyDistanceToleranceUm=2000; candidateDistanceToleranceUm=100;
moveErrorUm=abs(amount); % 400 um requested on the dominant axis of the diagonal example
assert(moveErrorUm==400 && moveErrorUm<=legacyDistanceToleranceUm && moveErrorUm>candidateDistanceToleranceUm);
legacyRoundedRpm=round(0.49); candidateMinRpm=1;
assert(legacyRoundedRpm==0 && signedRpm(-0.49,candidateMinRpm,false)==-1);
assert(signedRpm(0.49,candidateMinRpm,false)==1 && signedRpm(0,candidateMinRpm,false)==0);
assert(signedRpm(0.49,candidateMinRpm,true)==0);
% An invented plant that advances 0.25 mm per update at 1 RPM can overshoot
% a 0.12 mm remaining target; nonzero output is not proof of finer travel.
toyOvershootUm=250-120; assert(toyOvershootUm>candidateDistanceToleranceUm);
candidateHeadingToleranceMdeg=250;
assert(501>limits(2) && 501<=1000 && 501>candidateHeadingToleranceMdeg);
assert(251>candidateHeadingToleranceMdeg && 250<=candidateHeadingToleranceMdeg);

% Explicit counterexamples, not a noise estimate or a physical plant fit.
rounded=observation(ctx,[500.49 0],500.49);
assert(decision(rounded,ctx,limits,floor)=="accept");
noisy=observation(ctx,[550 0]+[-60 0],540-60);
assert(decision(noisy,ctx,limits,floor)=="accept");
% Toy command/plant quantum with no sensor noise: +0.55 -> -0.55 -> +0.55.
state=[550 0]; history=zeros(1,5); history(1)=state(1);
for k=1:4
    state(1)=state(1)-sign(state(1))*1100;
    history(k+1)=state(1);
end
assert(isequal(history,[550 -550 550 -550 550]) && all(abs(history)>limits(1)));

report=struct('offline_model',true,'calibration_verified',false,'physical_precision_verified',false, ...
    'convergence_verified',false,'accept_norm_um',limits(1),'accept_abs_yaw_mdeg',limits(2), ...
    'tested_translation_floor_um',minMoveUm,'tested_turn_floor_mdeg',minTurnMdeg, ...
    'continuous_dominant_axis_lower_um',dominantLowerUm,'diagonal_action',char(diagAction), ...
    'yaw_501mdeg_action',char(yawOnly),'nominal_response_J',J,'nominal_image_to_station',A, ...
    'legacy_distance_tolerance_um',legacyDistanceToleranceUm,'legacy_zero_travel_done_example_um',moveErrorUm, ...
    'candidate_distance_tolerance_um',candidateDistanceToleranceUm,'candidate_min_nonzero_rpm',candidateMinRpm, ...
    'candidate_heading_fine_tolerance_mdeg',candidateHeadingToleranceMdeg, ...
    'toy_1rpm_overshoot_um',toyOvershootUm, ...
    'rounded_example_true', [500.49 500.49], 'rounded_example_wire', [rounded.normUm rounded.errorYawMdeg], ...
    'noise_example_true',[550 540],'noise_example_observed',[noisy.normUm noisy.errorYawMdeg], ...
    'synthetic_quantized_motion_trace_um',history, ...
    'scope','Geometry, modeled O31 admission and synthetic motor/heading counterexamples only; no firmware, target I/O, or measured noise model.');
fid=fopen(fullfile(work,'stage3_strict_align_validation.json'),'w'); assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(report)); fclose(fid);
fprintf('PASS Stage3 strict alignment model: 500 um / 500 mdeg, 250 um motion floor, 250 mdeg fine heading window.\n');
fprintf('Stage3 candidates only: 100 um distance tolerance, 1 RPM minimum. No measured precision or convergence.\n');
end

function rpm=signedRpm(command,minRpm,stop)
if stop || command==0, rpm=0; else, rpm=sign(command)*max(abs(round(command)),minRpm); end
end

function f=observation(c,errorUm,yawMdeg)
f=struct('valid',true,'crcOK',true,'run',c.run,'pose',c.pose,'seq',c.seq+1,'tick',c.tick, ...
    'age',0,'stepUm',round(errorUm),'normUm',round(norm(errorUm)), ...
    'errorYawMdeg',round(yawMdeg),'turnMdeg',round(yawMdeg));
end

function [action,axis,amount]=decision(f,c,limits,floor)
action="observe"; axis=0; amount=0;
elapsed=mod(c.tick-f.tick,2^32);
delta=mod(f.seq-c.seq,2^32);
if c.state~=2 || ~f.valid || ~f.crcOK || f.run~=c.run || f.pose~=c.pose || ...
        f.seq<=0 || f.seq>=2^32 || (c.seq~=0 && (delta==0 || delta>=2^31)) || ...
        f.age<0 || f.age>200 || elapsed>200 || f.age+elapsed>200
    return;
end
if f.normUm<=limits(1) && abs(f.errorYawMdeg)<=limits(2)
    action="accept";
elseif abs(f.errorYawMdeg)>limits(2)
    if abs(f.turnMdeg)>=floor(2), action="yaw"; amount=f.turnMdeg; end
else
    [~,k]=max(abs(f.stepUm));
    if abs(f.stepUm(k))>=floor(1), action="translate"; axis=k; amount=f.stepUm(axis); end
end
end
