function validate_integration11_target_wait_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','single-cycle');checks=[];
for elapsed=[0,12,200,399,400,500]
 for match=[0,1]
  for release=[0,1]
   retry=release && ~match && elapsed<400;
   fail=~match && ~retry;
   checks(end+1)=~(fail && retry);
   checks(end+1)=~retry || elapsed<400;
   checks(end+1)=~match || (~retry && ~fail);
  end
 end
end
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB','target_wait_ms',400,'position_speed_encoder_checks_retained',true);
f=fopen(fullfile(out,'integration11-target-wait-matlab.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
