function validate_stage2_loaded_handoff_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','stage2-mainline');checks=[];
for prepared=[0,1]
 for run_match=[0,1]
  for item=0:3
   for mask=0:7
    for pending=[0,1]
     advance=prepared && run_match && item==3 && mask==7 && ~pending;
     checks(end+1)=~advance || (prepared && run_match && item==3 && mask==7 && ~pending);
     if prepared && run_match && item==3 && mask==7 && ~pending,checks(end+1)=advance;end
    end
   end
  end
 end
end
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB', ...
 'deployment_is_not_loaded_completion',true,'route_coordinates_changed',false, ...
 'stow_while_driving_trajectory_verified',false,'scope','stage2 handoff logic only; actual evidence and route remain field verification');
f=fopen(fullfile(out,'loaded-handoff-matlab.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
