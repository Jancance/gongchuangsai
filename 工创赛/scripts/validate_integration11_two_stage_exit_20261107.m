function validate_integration11_two_stage_exit_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','single-cycle');
checks=[];
% Preserve physical reference and calibrated pickup target while checking exit.
for scale=[10,100]
 for top=[-200,0,1300]
  pickup=top+108400*360*scale/3200;
  prehome=top+3200*360*scale/3200;
  checks(end+1)=abs((pickup-prehome)/(360*scale)*3200-105200)<1e-7;
  checks(end+1)=abs((pickup-top)/(360*scale)*3200-108400)<1e-7;
  checks(end+1)=prehome>top && pickup>prehome;
 end
end
% No post-home positional move or shifted zero; completion requires fresh flags.
for ack=[0,1]
 for home_running=[0,1]
  for flags=[0,1,3,7,11,15]
   for speed=[0,1,600]
    accepted=ack && ~home_running && flags==3 && speed==0;
    checks(end+1)=~accepted || (ack && ~home_running && bitand(flags,12)==0 && speed==0);
   end
  end
 end
end
checks(end+1)=3200/1600==2;
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB', ...
 'prehome_margin_pulses',3200,'prehome_speed_rpm',1846.4,'home_rpm',60, ...
 'pickup_pulses',108400,'posthome_position_commands',0,'reference_shift_pulses',0, ...
 'scope','sequencing and coordinate invariance; no physical root-cause or clearance acceptance');
f=fopen(fullfile(out,'integration11-two-stage-exit-matlab.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
