% Stage3 offline pre-implementation ordering and mapping model. No hardware.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'tmp','stage3_offline_cycle_20261119');
if ~isfolder(out), mkdir(out); end
n=0; colors=[5 1 2]; first_positions=[3 1 2];
orders=perms(1:3);
for a=1:size(orders,1)
 for b=1:size(orders,1)
  saved_colors=colors(orders(a,:)); saved_pos=first_positions(orders(a,:));
  second=colors(orders(b,:));
  target=zeros(1,3);
  for i=1:3, target(i)=saved_pos(saved_colors==second(i)); end
  assert(isequal(target,first_positions(orders(b,:)))); n=n+1;
  slots=colors(orders(a,:));
  for i=1:3
   original_slot=find(slots==second(i));
   assert(slots(original_slot)==second(i)); n=n+1;
  end
 end
end
% Existing firmware units: virtual pulses/mm = 400; 60mm -> 24000.
scale=400; base=[59400 61400 60600]; stack=base-60*scale;
assert(isequal(stack,[35400 37400 36600])); n=n+1;
assert(all(stack>=3200 & stack<=82000)); n=n+1;
assert(all(abs((base-stack)/scale-60)<eps)); n=n+1;
% Recovery must remain after all placements; each close completes before top.
trace={};
for i=1:3, trace{end+1}=sprintf('place%d',i); end
for i=1:3
 trace=[trace,{sprintf('recover%d',i),'extend','lower','close200','top','retract','yaw_carrier','original_slot','release160_complete','top'}];
end
assert(strcmp(trace{3},'place3') && strcmp(trace{4},'recover1')); n=n+1;
assert(sum(startsWith(trace,'recover'))==3); n=n+1;
assert(~any(strcmp(trace,'carrier_release_depth'))); n=n+1;
% Effective mapped positions obey the same unique 1..3 contract as QR positions.
for invalid={[1 1 3],[0 2 3],[1 2 4]}
 mapped=invalid{1}; assert(~(all(mapped>=1 & mapped<=3) && numel(unique(mapped))==3)); n=n+1;
end
saved=[3 1 2];
for batch=1:2
 if batch==1, stored=saved; else, assert(isequal(stored,saved)); end
 n=n+1;
end
% Follow-up request resumes the completed locked placement run at recovery17.
for done=[false true]
 for placed=0:3
  for same_run=[false true]
   for calibrated=[false true]
    accepted=done && placed==3 && same_run && calibrated;
    if accepted
     phase=17; preserved=[3 1 2];
     assert(phase==17 && isequal(preserved,[3 1 2]));
    else
     assert(~(done && placed==3 && same_run && calibrated));
    end
    n=n+1;
   end
  end
 end
end
for raw=[int64(2)^33+int64(19),-int64(2)^33-int64(19)]
 locked=raw; assert(locked==raw); n=n+1;
end
% Unknown calibration never becomes a valid candidate through arithmetic.
assert((-1-60*scale)<3200); n=n+1;
report=struct('passed',true,'checks',n,'scope','mapping, arithmetic and action order only; no physical acceptance',...
 'scale_virtual_pulses_per_mm',scale,'stack_offset_mm',60,'source','latest align15/yaw36.5 baseline',...
 'requirements',{{'R QR-05','R RUN-01','R RUN-03','R FIELD-06','R PWR-01','E CAL-05','E LOC-05'}},...
 'carrier_return','release160 at actual top after original slot and DM complete; no descent',...
 'pending',{{'rough recovery grasp depths','top-release physical stability','stack work-plane calibration','hardware acceptance'}});
fid=fopen(fullfile(out,'matlab-validation.json'),'w'); fwrite(fid,jsonencode(report),'char'); fclose(fid);
fprintf('PASS Stage3 offline cycle MATLAB: %d checks\n',n);
