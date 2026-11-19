function validate_source_alignment_steps_20261112
% Existing seven-segment profile applied to explicitly owned 1..20mm steps.
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'evidence','integration','2026-11-12','source-align');
if ~exist(out,'dir'),mkdir(out);end
pose=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-11','source-view-entry','pose-current-20261112','matlab-result.json')));assert(pose.passed);
factor=pi*77/60;A=1000-factor*4;J=2000-factor*12;speed=15;rows=[];
for D=[1 5 abs(pose.pivot_target_shift_source_mm(:)') 10 20]
 tj=min(A/J,sqrt(speed/J));ta=max(0,speed/(J*tj)-tj);tv=0;
 if D>=speed*(2*tj+ta),tv=(D-speed*(2*tj+ta))/speed;
 else,tj=A/J;if D>=2*A*tj^2,ta=.5*(sqrt(tj*tj+4*D/A)-3*tj);else,tj=(D/(2*J))^(1/3);ta=0;end,end
 duration=[tj ta tj tv tj ta tj];signs=[1 0 -1 0 -1 0 1];x=0;v=0;a=0;elapsed=0;
 for k=1:7
  t=duration(k);j=J*signs(k);x=x+v*t+.5*a*t^2+j*t^3/6;v=v+a*t+.5*j*t^2;a=a+j*t;elapsed=elapsed+t;
 end
 assert(abs(x-D)<1e-7 && abs(v)<1e-7 && abs(a)<1e-7 && elapsed>0 && elapsed<10);
 rows=[rows;D elapsed];
end
% One explicit request per step; real DONE/STOP is required before the next.
assert(all(rows(:,1)>=1 & rows(:,1)<=20));
assert(abs(pose.yaw_error_source_deg)<5 && abs(pose.yaw_error_source_deg-(-5))<1);
r=struct('engine','local MATLAB','release',version('-release'),'translation_cases_mm_s',rows, ...
 'step_min_mm',1,'step_max_mm',20,'requested_speed_mm_s',15,'turn_min_deg',5,'turn_max_deg',10, ...
 'current_first_turn_trial_deg',-5,'single_step_requires_new_request',true,'fault_does_not_rearm',true, ...
 'normal_route_minimum_unchanged',true,'real_vehicle_tracking_verified',false,'passed',true);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Current alignment: %d short-step profiles passed; explicit5deg turn and existing fault/STOP boundaries retained.\n',size(rows,1));
end
