function validate_original_speed_overlap_20261115
% Diagnose feasibility from EXISTING actual samples. No motor access.
% The existing hover clearance is a path condition, not a new stop threshold.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','original-speed-overlap');
if ~exist(out,'dir'),mkdir(out);end
files={fullfile(root,'evidence','integration','2026-11-15','integration20-yaw165-window10','committed-20261115-154633.json'),...
 fullfile(root,'evidence','integration','2026-11-15','source-continuous','pc13-watch-20261115-173446.json')};
rows={};checks=0;n=0;
for f=1:numel(files)
 d=jsondecode(fileread(files{f}));trace=d.trace;
 for seq=1:3
  accepted=[trace.g_low_accepted_sequence];phase=[trace.g_low_phase];
  approach=trace(accepted==seq & phase==2);final=trace(accepted==seq & phase==4);
  assert(~isempty(approach) && ~isempty(final));checks=checks+1;
  % Use the following observed settled X as the historical target location.
  % This is feedback, not a new physical ruler calibration.
  target=median([final.g_startup_extension_current_deg]);
  x=abs([approach.g_startup_extension_current_deg]-target)/(315.5*360/3200);
  z=[approach.lift_depth_pulses];time=[approach.g_low_elapsed_ms];
  firstX=find(x<=5,1);firstZ=find(z>=15600-400,1);
  assert(~isempty(firstX) && ~isempty(firstZ));checks=checks+1;
  % At a FRESH reported sample with Z at the old clearance, X still has
  % measured travel remaining. Keeping Z at that target is the observed wait.
  observedConflict=x(firstZ)>5;
  checks=checks+1;n=n+1;
  previousZ=max(1,firstZ-1);previousX=max(1,firstX-1);
  rows{n}=struct('source',files{f},'sequence',seq,...
   'first_clearance_sample_elapsed_ms',time(firstZ),...
   'clearance_sample_bracket_ms',[time(previousZ) time(firstZ)],...
   'extension_remaining_at_clearance_sample_mm',x(firstZ),...
   'extension_within_existing5mm_sample_elapsed_ms',time(firstX),...
   'extension_arrival_sample_bracket_ms',[time(previousX) time(firstX)],...
   'observed_clearance_before_extension_arrival',observedConflict,...
   'observed_gap_ms',time(firstX)-time(firstZ)); %#ok<AGROW>
  assert(observedConflict);checks=checks+1;
 end
end
% The prior ideal unit model failed to represent these observed trajectories.
% Do not invent fitted driver constants or label six old traces as proof of
% a new uninterrupted FULL descent trajectory that has never been executed.
report=struct('diagnostic_completed',true,'local_matlab',true,'checks',checks,...
 'existing_actual_runs',n,'results',[rows{:}],...
 'descent_rpm_unchanged',1846.4,'extension_rpm_unchanged',150,...
 'simultaneous_start_already_exists_in_current_firmware',true,...
 'continuous_full_descent_at_original_speed_validated',false,...
 'firmware_implementation_gate_passed',false,...
 'failure','Existing traces reach old Z clearance while X is still outside existing5mm arrival. Removing the Z hold is not validated.',...
 'no_new_fault_or_runtime_threshold',true,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('MATLAB diagnostic complete: %u checks/%u actual traces. Original-speed simultaneous continuous-descent implementation gate NOT passed.\n',checks,n);
for k=1:numel(rows)
 r=rows{k};fprintf('trace%u seq%u: X remaining %.2fmm at Z clearance, sampled gap %ums\n',k,r.sequence,r.extension_remaining_at_clearance_sample_mm,r.observed_gap_ms);
end
end
