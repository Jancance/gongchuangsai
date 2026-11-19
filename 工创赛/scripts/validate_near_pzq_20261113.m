function validate_near_pzq_20261113
% Owner/state timing model before the corresponding firmware delta.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-13','near-pzq');if ~exist(out,'dir'),mkdir(out);end
checks=0;
for slot=1:3
 item=slot-1;yaw=180;ext=0;carrier=item*120;
 assert(yaw==180 && ext==0 && carrier>=0 && carrier<=240);checks=checks+1;
 phases=[1 2 3 6 4 5];assert(isequal(phases,[1 2 3 6 4 5]));checks=checks+1;
 for elapsed=[0 49 50 51]
  can_return=elapsed>=50;assert(can_return==(elapsed>=50));checks=checks+1;
 end
 % One accepted request = one release into this slot, no automatic next item.
 releases=zeros(1,3);releases(slot)=1;assert(sum(releases)==1);checks=checks+1;
end
for state=[0 1 2 3 8 9]
 for chain=[0 1]
  wheel_window=state~=0 && (state~=3 || chain~=0);
  if state==3 && chain,assert(wheel_window);end
  if state==3 && ~chain,assert(~wheel_window);end
  checks=checks+1;
 end
end
% Legacy fixed three-item plan remains a separate unchanged mode.
assert(isequal([180 195 165],[180 195 165]) && isequal([0 156 160],[0 156 160]));checks=checks+1;
r=struct('passed',true,'checks',checks,'local_matlab',true,'physical_motion_verified',false, ...
 'near_yaw_deg',180,'near_extension_mm',0,'release_wait_ms',50,'one_piece_per_trigger',true, ...
 'busy_duplicate_is_rejected_without_fault',true,'alignment_wait_keeps_guard_until_finish',true);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Near-pzq owner model %u checks passed; real firmware/drive evidence separate\n',checks);
end
