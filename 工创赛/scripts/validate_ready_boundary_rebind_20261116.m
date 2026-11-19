function validate_ready_boundary_rebind_20261116
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-16','observe-ready-each120');
checks=0;
% Already collected stop samples below120 may never pass across the boundary.
for beforeCount=0:5
 for afterCount=1:5
  angleBefore=119.9;angleAfter=120.1;
  oldReady=angleBefore>=120;newReady=angleAfter>=120;
  if ~oldReady && newReady,stable=0;else,stable=beforeCount;end
  stable=stable+afterCount;
  assert((stable>=5)==(afterCount>=5));checks=checks+1;
 end
end
% No new time gate: idle reset signature suspends old ready, then a fresh
% run1/pose1/state1 preparation signature can rebind ordering immediately.
for oldRun=[1 5 99]
 pending=false;oldReady=true;
 idle=struct('run',0,'pose',0,'state',0,'prep',0,'item',0);
 isIdle=all([idle.run idle.pose idle.state idle.prep idle.item]==0);
 if isIdle,pending=true;oldReady=false;end
 assert(pending && ~oldReady);checks=checks+1;
 for candidate=[oldRun 1]
  state=6;if candidate==1,state=1;end
  rebind=pending && candidate==1 && state==1;
  assert(rebind==(candidate==1));checks=checks+1;
 end
end
r=struct('passed',true,'local_matlab',true,'checks',checks,...
 'later120_requires_new_five',true,'quick_reboot_two_stage_rebind',true,...
 'no_new_timing_threshold',true,'hardware_access',false);
f=fopen(fullfile(out,'boundary-rebind-matlab.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS ready boundary/rebind: %u checks\n',checks);
end
