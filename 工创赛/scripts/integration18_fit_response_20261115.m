function integration18_fit_response_20261115(useLong)
% Fit command-to-pixel response from this run's right6/back3 probes only.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','integration18');
if nargin<1,useLong=false;end
names={'integration18-current-20261115-b','integration18-after-right6-20261115-c','after-back3-e'};
commands=[6 0;0 -3];suffix='';
if useLong
    names{4}='after-back10-g';commands(3,:)=[0 -10];suffix='-v2';
end
centers=zeros(numel(names),2);
for k=1:numel(names)
    s=fullfile(out,names{k});
    if ~isfile(fullfile(s,'rim-matlab.json')),integration18_check_rim_20261115(s);end
    d=jsondecode(fileread(fullfile(s,'rim-matlab.json')));
    assert(d.passed && d.local_matlab);
    centers(k,:)=d.centers_px(2,:);
end
J=(commands\diff(centers))';
assert(cond(J)<10); % Same diagnostic fit criterion as the existing calibration.
p=jsondecode(fileread(fullfile(root,'output','source_body_registration_20261114.json')));
delta=centers(end,:)-p.reference_center_px(:)';
u=J\(-delta');
[~,axis]=max(abs(u));distance=max(1,min(p.max_step_mm,round(abs(u(axis)))));
if axis==1
    if u(axis)>0,direction=2;else,direction=1;end
else
    if u(axis)>0,direction=0;else,direction=3;end
end
report=struct('passed',true,'local_matlab',true,'sessions',{names},...
    'centers_px',centers,'command_right_forward_mm',commands,...
    'response_px_per_command_mm',J,'condition_number',cond(J),...
    'reference_center_px',p.reference_center_px,'current_error_mm',delta*p.scale_mm_per_px,...
    'continuous_right_forward_mm',u','candidate_direction',direction,'candidate_distance_mm',distance,...
    'predicted_after_step_mm',(delta+J(:,axis)'*sign(u(axis))*distance)*p.scale_mm_per_px,...
    'current_two_axis_probe_used',true,'physical_displacement_measured',false,...
    'physical_accuracy_verified',false,'motion_allowed',false);
f=fopen(fullfile(out,['current-response-matlab' suffix '.json']),'w','n','UTF-8');
fprintf(f,'%s\n',jsonencode(report));fclose(f);
p.profile_id=['source-rim-top180-ext65-z600-20261115-current' suffix];
p.response_px_per_command_mm=J;
p.calibration_evidence=['current-response-matlab' suffix '.json'];
p.physical_auto_correction_passed=false;
p.current_session_probes={'right6','back3'};
if useLong,p.current_session_probes{3}='back10';end
f=fopen(fullfile(out,['source_body_registration_current' suffix '.json']),'w','n','UTF-8');
fprintf(f,'%s\n',jsonencode(p));fclose(f);
fprintf('CURRENT RESPONSE condition %.3f, error %.3fmm; candidate dir%d %.0fmm; predicted %.3fmm\n',...
    cond(J),norm(report.current_error_mm),direction,distance,norm(report.predicted_after_step_mm));
disp(J);
end
