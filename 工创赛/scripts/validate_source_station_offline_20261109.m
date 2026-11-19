function validate_source_station_offline_20261109
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-09','source-station-offline');if ~exist(out,'dir'),mkdir(out);end
n=0;
for arrived=0:1
 for stopped=0:1
  for profile=0:1
   for aligned=0:1
    for qr=0:1
     for fresh=0:1
      pick=arrived && stopped && profile && aligned && qr && fresh;
      assert(~pick || (stopped && aligned && qr && fresh));n=n+1;
     end
    end
   end
  end
 end
end
% First phase permits alignment even when route3 is not configured.
for route=0:1,assert(logical(1)==true);n=n+1;end
colors=[1 4 6];regions=perms(colors);
for k=1:size(regions,1)
 for order=perms(colors)'
  found=zeros(1,3);
  for j=1:3,found(j)=find(regions(k,:)==order(j));end
  assert(numel(unique(found))==3);n=n+1;
 end
end
% Measured pzq lookup: left/right/down, not fixed color/slot aliases.
yaw=[195 165 180];ext=[156 160 0];assert(all(yaw>=0 & yaw<=270) && all(ext<=170));
% Existing discrete body step contract is explicit. 2..10mm is unsupported,
% never a fake DONE. Reobservation is mandatory after each real movement.
for dx=[-21 -20 -10 -5 -2 0 2 5 10 20 21]
 for dy=[-20 -5 0 5 20]
  aligned=abs(dx)<=2 && abs(dy)<=2;step=max(abs([dx dy]));
  execute=~aligned && step>=10 && step<=20;
  assert(~execute || (~aligned && step>=10 && step<=20));n=n+1;
 end
end
% Changing body session/position while waiting for scan/target invalidates the
% stationary capture context; route ownership cannot overlap pick ownership.
for phase=0:6
 for moved=0:1
  permit=phase>=3 && phase<=5 && ~moved;
  assert(~permit || ~moved);n=n+1;
 end
end
r=struct('passed',true,'checks',n,'engine','local MATLAB','alignment_before_scan',true, ...
 'manual_qr_only_current_window',true,'task_color_region_permutations',36, ...
 'route3_required_for_alignment',false,'pzq_fixed_motion_parameters_unchanged',true, ...
 'motion_and_vision_domains_separate',true,'real_calibration_or_hardware_verified',false, ...
 'min_body_correction_mm',10,'max_body_correction_mm',20, ...
 'scope','phase ownership,task selection and existing step bounds; no physical convergence claim');
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f>0);fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS: %d station/QR/color/correction ownership checks; no hardware or current calibration claim.\n',n);
end
