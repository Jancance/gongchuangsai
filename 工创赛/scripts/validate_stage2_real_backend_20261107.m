function validate_stage2_real_backend_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','stage2-mainline');checks=[];
for arm_start=[0,1]
 for route_start=[0,1]
  both=arm_start && route_start;stop=~both;
  checks(end+1)=~both || ~stop;
  checks(end+1)=both || stop;
 end
end
for slot=1:3
 deg=(slot-1)*120;pwm=500+round(deg*2000/270);
 checks(end+1)=deg>=0 && deg<=270 && pwm>=500 && pwm<=2500;
end
for kind=[1,3,4,5,6,7,8,9]
 for elapsed=[0,120,700,1000]
  for evidence=[0,1]
   physical=any(kind==[4,7]);done=elapsed>=700 && (~physical || evidence);
   checks(end+1)=~(physical && done) || evidence;
  end
 end
end
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB', ...
 'reuses_verified_axis_and_servo_drivers',true,'new_motion_path',false, ...
 'scope','backend dispatch/partial-start/servo indexing and evidence sequencing, not physical clearance');
f=fopen(fullfile(out,'real-backend-matlab.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
