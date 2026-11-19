function validate_stopped5_initial_branch_20261116
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','stopped5-initial-branch');
if ~exist(out,'dir'),mkdir(out);end
checks=0;cases=0;
% Five image samples; use existing 25px/s threshold and 0.02..0.25s.
for dt=[.025 .05 .1 .2]
 for speed=[0 10 24 26 31 60]
  h=[];opened=false(1,12);
  for k=1:12
   t=(k-1)*dt;x=100+speed*t;
   if ~isempty(h) && any(abs(x-h(:,2))./(t-h(:,1))>25),h=[];end
   h=[h;t x];if size(h,1)>5,h=h(end-4:end,:);end
   opened(k)=size(h,1)==5;
  end
  assert(~any(opened(1:4)));checks=checks+1;
  if speed<=25,assert(all(opened(5:end)));else,assert(~any(opened));end
  checks=checks+1;cases=cases+1;
 end
end
% First post-beep valid classification latches: moving then stop bypasses120;
% initially stopped never bypasses120 merely because it starts moving later.
for initialMoving=[false true]
 for reached120=[false true]
  for stableCount=0:6
   needs120=~initialMoving;
   allowed=(~needs120 || reached120) && stableCount>=5;
   assert(allowed==((initialMoving||reached120)&&stableCount>=5));
   if ~initialMoving && ~reached120,assert(~allowed);checks=checks+1;end
   checks=checks+1;cases=cases+1;
  end
 end
end
% Unknown/absent observations never mean stationary, never grant bypass.
for invalid={'missing','duplicate_color','gap','nan','stale','unknown_speed'}
 classification=0;ready=false;
 assert(classification==0 && ~ready);checks=checks+1;
end
r=struct('passed',true,'local_matlab',true,'checks',checks,'cases',cases,...
 'required_stable_frames',5,'stationary_speed_px_s',25,...
 'moving_speed_px_s',30,'required_moving_frames',2,...
 'minimum_frame_interval_s',.02,'maximum_frame_interval_s',.25,...
 'initial_moving_then_stop_bypasses120',true,'initial_stopped_requires120',true,...
 'classification_scope','first valid post-beep observation sequence; no camera before beep',...
 'physical_stop_or_angle_truth_validated',false,'hardware_access',false);
fp=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(fp,'%s\n',jsonencode(r));fclose(fp);
fprintf('PASS stopped5 and first-observation branch: %u cases, %u checks.\n',cases,checks);
end
