function validate_home_exit_bound_20261114
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','low-pick-homeexit');
if ~exist(out,'dir'),mkdir(out);end
ppm=1600;bound_mm=.02;pulses=bound_mm*ppm;assert(pulses==32);
checks=0;rows=[];
for scale=[10 100]
 raw_bound=pulses*360*scale/3200;
 assert(abs(raw_bound/scale*(3200/360)/ppm-bound_mm)<1e-12);checks=checks+1;
 for delta=[-2*raw_bound,-raw_bound-1,-raw_bound,0,raw_bound,raw_bound+1,2*raw_bound]
  accepted=abs(delta)<=raw_bound;
  assert(accepted==(abs(delta/scale*(3200/360)/ppm)<=bound_mm));checks=checks+1;
  rows=[rows;scale delta raw_bound accepted]; %#ok<AGROW>
 end
end
assert(bound_mm<20/100 && bound_mm<4.75/100);checks=checks+1;
r=struct('passed',true,'engine','local MATLAB','checks',checks,'exit_position_bound_mm',bound_mm, ...
 'exit_position_bound_virtual_pulses',pulses,'case_rows',rows, ...
 'old_bound_mm',.3*(3200/360)/ppm,'keep_measured_homing_reference',true, ...
 'real_failed_second_position_unknown',true,'physical_drift_cause_proven',false,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Home-exit bounded check %u passed; 0.020mm/32virtual pulses; original reference retained\n',checks);
end
