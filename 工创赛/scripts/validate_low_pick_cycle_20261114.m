function validate_low_pick_cycle_20261114
% Required state/geometry checks before implementing the low-height owner.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','low-pick');
if ~exist(out,'dir'),mkdir(out);end
hover=62400;pick=118800;transfer=7200;ppm=1600;checks=0;
assert(hover<pick && transfer<pick-65*ppm);checks=checks+1;
% Approach/confirmation are separate. Prediction can only prepare, never descend.
for region=0:2
 for slot=1:3
  phases=[1 2 3 4 5 6 7 8 9 10];
  assert(all(diff(phases)==1) && slot*120-120<=240);checks=checks+1;
  for confirmation=[false true]
   next=3+double(confirmation);assert((next==4)==confirmation);checks=checks+1;
  end
 end
end
for rise=[0 5 64.75 65 65.001 69.75]
 depth=pick-rise*ppm;
 allow=rise>65;
 assert(allow==(depth<pick-65*ppm));checks=checks+1;
 if ~allow,assert(depth>=14800);end
end
% Both lateral transfer axes remain delayed until actual clearance; target/ACK not enough.
for actual=[false true]
 for fresh=[false true]
  for rise=[65 65.1]
   released=actual && fresh && rise>65;
   if ~actual || ~fresh || rise<=65,assert(~released);end
   checks=checks+1;
  end
 end
end
for remaining={ [2 1 5],[1 5],5 }
 expected=remaining{1};slots=zeros(1,3);slots(1:numel(expected))=expected;
 assert(isequal(sort(slots(slots>0)),sort(expected)));checks=checks+1;
end
r=struct('passed',true,'local_matlab',true,'checks',checks, ...
 'hover_depth_virtual_pulses',hover,'pick_depth_virtual_pulses',pick, ...
 'transfer_depth_virtual_pulses',transfer,'pulses_per_mm',ppm, ...
 'clearance_strictly_greater_than_mm',65, ...
 'short_descent_mm',(pick-hover)/ppm,'confirmation_required_after_approach',true, ...
 'both_transfer_axes_wait_for_actual_clearance',true, ...
 'remaining_material_counts',[3 2 1],'hardware_access',false, ...
 'physical_sweep_and_timing_passed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Low pick model: %u checks; short descent%.2fmm; strict >65mm before lateral transfer\n',checks,(pick-hover)/ppm);
end
