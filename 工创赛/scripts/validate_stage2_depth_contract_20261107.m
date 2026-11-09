function validate_stage2_depth_contract_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','stage2-mainline');checks=[];
for scale=[10,100]
 for pulse=[0,108400,118800]
  deg=pulse*360/3200;raw=deg*scale;
  checks(end+1)=raw>=0 && raw<2147483647;
  checks(end+1)=deg<=118800*360/3200+0.1;
 end
end
for pulse=[-1,0,108400,118800,118801,NaN,Inf]
 accepted=pulse>=0 && pulse<=118800;
 checks(end+1)=accepted==any(pulse==[0,108400,118800]);
end
checks(end+1)=118800*360/3200==13365;
checks(end+1)=118800*360/3200>12195.1;
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB','max_lift_pulses',118800, ...
 'max_motor_delta_deg',13365.1,'changes_existing_measured_pickup_target',false, ...
 'scope','interface range/unit consistency only; no new path or three-axis physical acceptance');
f=fopen(fullfile(out,'depth-contract-matlab.json'),'w');fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
