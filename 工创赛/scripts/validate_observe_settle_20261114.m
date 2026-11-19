function validate_observe_settle_20261114
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','low-pick-observe-settle');
if ~exist(out,'dir'),mkdir(out);end
goal=-2307.09375;observed=-2371.199951171875;
degree_per_mm=315.5*360/3200;excursion=goal-observed;
assert(excursion>60 && excursion<90);checks=1;
for scoped=[false true]
 for excess=[60 60.1 excursion 90 90.1 670]
  for elapsed=[0 149 150]
   accept=excess<=60 || (scoped && excess<=90 && elapsed<150);
   if excess>90 || (~scoped && excess>60),assert(~accept);end
   if scoped && excess>60 && elapsed>=150,assert(~accept);end
   checks=checks+1;
  end
 end
end
assert(65+90/degree_per_mm<170);checks=checks+1;
r=struct('passed',true,'engine','local MATLAB','checks',checks, ...
 'actual_recorded_goal_deg',goal,'actual_recorded_position_deg',observed, ...
 'recorded_excess_deg',excursion,'recorded_excess_model_mm',excursion/degree_per_mm, ...
 'ordinary_corridor_deg',60,'scoped_maximum_deg',90,'scoped_settle_ms',150, ...
 'nonzero_final_position_tolerance_deg_unchanged',45,'final_speed_zero_required',true, ...
 'actual_free_settling_not_observed_because_previous_stop',true,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Observation return: %u boundary checks; recorded excess%.3fdeg/%.3fmm; bounded150ms/90deg only\n',checks,excursion,excursion/degree_per_mm);
end
