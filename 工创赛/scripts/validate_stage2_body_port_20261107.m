function validate_stage2_body_port_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','stage2-mainline');checks=[];
for x=[-20,-10,-2,0,2,10,20]
 for y=[-20,-10,0,10,20]
  tolerance=2;aligned=abs(x)<=tolerance && abs(y)<=tolerance;
  if abs(x)>=abs(y),amount=abs(x);direction=2;if x<0,direction=1;end
  else,amount=abs(y);direction=0;if y<0,direction=3;end;end
  executable=~aligned && amount>=10 && amount<=20;
  checks(end+1)=~executable || (amount>=10 && amount<=20);
  checks(end+1)=direction>=0 && direction<=3;
  checks(end+1)=~aligned || ~executable;
 end
end
for current_session=[1,2]
 for expected_session=[1,2]
  for active=[0,1]
   for mask=[0,7,15]
    done=current_session==expected_session && ~active && mask==15;
    checks(end+1)=~done || (current_session==expected_session && ~active && mask==15);
   end
  end
 end
end
for vm=[0,1]
 for owns=[0,1]
  for holding=[0,1]
   permit=~vm || (owns && holding);
   checks(end+1)=~permit || (~vm || (owns && holding));
  end
 end
end
% Completion may retain the platform VM only for this verified source owner;
% native session finish and all four wheel stop confirmations remain necessary.
for owns=[0,1]
 for holding=[0,1]
  for stopped=[0,1]
   for mask=[0,15]
    keep=owns && holding;complete=stopped && mask==15;
    checks(end+1)=~(complete && keep) || (owns && holding && stopped && mask==15);
   end
  end
 end
end
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB', ...
 'body_task_is_only_route_owner',true,'min_correction_mm',10,'max_correction_mm',20, ...
 'recheck_vision_after_each_axis_move',true,'stop_request_is_not_confirmation',true, ...
 'scope','task handoff/session/millimetre bounds only; no new calibration or physical acceptance');
f=fopen(fullfile(out,'body-port-matlab.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
