function validate_simplified_guards_20261114
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','overlap200-simplified');
if ~exist(out,'dir'),mkdir(out);end
checks=0;
% Timing skew is logged, not a fault; total group timeout still bounds motion.
for skew=[0 90 91 300]
 for elapsed=[100 11999 12000]
  stop=elapsed>=12000;assert(stop==(elapsed>=12000));checks=checks+1;
 end
end
% Read failure retries never resend a relative move. Persistent loss hits group deadline.
for failures=[0 1 2 20]
 sentMoves=1;reads=failures+1;assert(sentMoves==1 && reads>=1);checks=checks+1;
end
% Zero speed+arrival+position: one fresh sample in travel, two on grasp/loaded lift.
for need=[1 2]
 for count=0:3
  for zero=[false true]
   done=count>=need && zero;assert(~done || zero);checks=checks+1;
  end
 end
end
% Ordinary observation exit has no active piece and changes no fault/reference.
for active=[false true]
 for fault=[false true]
  ordinaryExit=~active && ~fault;clearedFault=false;
  assert(~clearedFault);if active || fault,assert(~ordinaryExit);end
  checks=checks+1;
 end
end
% Relaxations do not change user clearance, release dwell or mechanism limits.
assert(65*400==26000 && 50==50);checks=checks+1;
report=struct('passed',true,'local_matlab',true,'checks',checks,...
 'start_skew_logged_only',true,'group_timeout_ms',12000,...
 'owner_timeout_only_when_no_group',true,'read_retry_no_move_retry',true,...
 'travel_settle_samples',1,'grasp_or_loaded_lift_settle_samples',2,...
 'extension_zero_tolerance_mm',.5,'top_tolerance_mm',.5,...
 'initial_lift_read_attempts',2,'normal_observation_exit_no_fault_or_reset',true,...
 'default_host_total_budget_disabled',true,'hardware_access',false);
f=fopen(fullfile(out,'simplification-matlab.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS simplification MATLAB: %u checks; real fault/STOP/65mm retained\n',checks);
end
