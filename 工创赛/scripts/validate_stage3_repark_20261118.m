% Prevalidate the existing step executor for user-confirmed pixel target.
% Uses actual ring centers/row angle from the preserved physical training set.
% No live configuration write and no physical precision certification.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-18','stage3-resume');
f=jsondecode(fileread(fullfile(base,'matlab-response','response-training-frozen.json')));
m=jsondecode(fileread(fullfile(base,'repark-base-measurement.json')));
o=f.input_metadata.observations;features=zeros(4,3);
for k=1:4
    centers=double(o(k).ring_centers_px);
    features(k,:)=[centers(2,:) atan2d(centers(3,2)-centers(1,2),centers(3,1)-centers(1,1))];
end
C=diag([10 10 3]);D=diff(features,1,1);J=(C\D).';
assert(rank(J)==3 && all(isfinite(J(:))));
% Same observed commands, alternate measured feature; not a new CV algorithm.
assert(norm(C*J.'-D,'fro')<1e-10);
A=double(f.A2);target=[960;540;0];current=[double(m.center_px(:));double(m.row_yaw_deg)];
cases=0;unfinished=0;maxSteps=0;
for x=[-60 -20 0 20 60]
 for y=[-80 -30 0 30 80]
  for yaw=[-5 -1 0 1 5]
   e=[x;y;yaw];steps=0;
   for n=1:20
    if norm(A*e(1:2))<=2 && abs(e(3))<=1,break;end
    if abs(e(3))>1
     axis=3;turn=-e(3)/J(3,3);command=sign(turn)*min(max(abs(turn),1.001),10);
    else
     planar=-(J(1:2,1:2)\e(1:2));
     [~,axis]=max(abs(planar));command=sign(planar(axis))*min(abs(planar(axis)),30);
     if abs(command)<1,break;end
    end
    e=e+J(:,axis)*command;steps=steps+1;
   end
   cases=cases+1;maxSteps=max(maxSteps,steps);
   if norm(A*e(1:2))>2 || abs(e(3))>1,unfinished=unfinished+1;end
  end
 end
end
assert(unfinished==0,'Existing per-axis strategy did not close every synthetic case');
% Quantized existing executor can leave sub-step residuals: report them.
% Request faults/STOP/preparation loss never become completed in host evidence.
failureExits={'existing MCU fault: preserve first error, no next command'; ...
 'lost fresh pose/status: no image binding and no next command'; ...
 'capture failure: do not retry motion';'STOP: existing firmware stop path remains'; ...
 'sub-step residual: report it, do not invent smaller supported travel'};
commands=[5 0 0;0 5 0;0 0 2];predictions=commands*J.';
result=struct('status','MATLAB_STEP_MATH_PREVALIDATED_ONLY','matlab_version',version, ...
 'features','actual middle ring center u/v and actual ring1-to3 row yaw', ...
 'source_training_candidate_preserved',true,'J_pixel_per_right_forward_ccw',J, ...
 'A2_nominal',A,'target_user_confirmed',target.','initial_measurement',current.', ...
 'initial_nominal_translation_error_mm',norm(A*(current(1:2)-target(1:2))), ...
 'diagnostic_commands',commands,'predicted_feature_changes',predictions, ...
 'synthetic_cases',cases,'synthetic_unfinished_with_existing_min_steps',unfinished, ...
 'synthetic_max_steps',maxSteps,'failure_exit_review',{failureExits}, ...
 'calibration_verified',false,'physical_verified',false,'running_config_written',false);
result.rules_R={'RUN-01','RUN-03','FIELD-06','PWR-01'};
result.engineering_E={'CAL-02','CAL-03','CAL-05','VIS-15','RUN-06'};
result.limitations={'User confirms pixel target; actual physical TCP/placement remains separately unverified'; ...
 'Transferred local model still needs new independent physical-response observations'; ...
 '95/150 nominal geometry does not certify independently measured mm precision'; ...
 'No new stop/fault thresholds; existing 2mm/1deg and command bounds used in simulation'};
path=fullfile(base,'repark-matlab-prevalidation-v2.json');assert(~isfile(path));
fid=fopen(path,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result));clear cleanup;
fprintf('MATLAB prevalidation: cases=%d unfinished=%d maxsteps=%d; hardware verified=false\n',cases,unfinished,maxSteps);
disp(J);disp('Independent probe predictions:');disp(predictions);
