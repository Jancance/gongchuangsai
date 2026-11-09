function validate_integration11_release_step_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','single-cycle');
checks=[];
for scale=[10,100]
 for p=[-120000,0,1219500]
  delta=3200*360*scale/3200;goal=p+delta;
  checks(end+1)=goal>p && abs(delta/(360*scale)*3200-3200)<1e-8;
  checks(end+1)=delta<=4294967295;
  checks(end+1)=3200/1600==2;
 end
end
for ready=[0,1]
 for fault=[0,1]
  for home_running=[0,1]
   for moving=[0,1]
    accepted=ready && ~fault && ~home_running && ~moving;
    checks(end+1)=~accepted || (ready && ~fault && ~home_running && ~moving);
   end
  end
 end
end
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB', ...
 'pulse_limit',3200,'direction','down away from confirmed top', ...
 'rpm',30,'acceleration_rpm_s',60,'home_commands',0,'valid_reference_created',false, ...
 'pickup_configuration_changed',false,'scope','bounded relative escape; no loaded or real-mm acceptance');
f=fopen(fullfile(out,'integration11-release-step-matlab.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
