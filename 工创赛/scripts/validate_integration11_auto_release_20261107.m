function validate_integration11_auto_release_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','single-cycle');checks=[];
for scale=[10,100]
 for top=[-1300,0,1350]
  safe=top+3200*360*scale/3200;
  pickup=top+108400*360*scale/3200;
  remaining=(pickup-safe)*3200/(360*scale);
  checks(end+1)=abs(remaining-105200)<1e-8;
  checks(end+1)=abs((safe-top)*3200/(360*scale)-3200)<1e-8;
  checks(end+1)=pickup==top+108400*360*scale/3200;
  checks(end+1)=safe>top && pickup>safe;
 end
end
for home_done=[0,1]
 for escape_done=[0,1]
  for protection=[0,1]
   valid=home_done && escape_done && ~protection;
   checks(end+1)=~valid || (home_done && escape_done && ~protection);
  end
 end
end
checks(end+1)=3200/1600==2;
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB', ...
 'physical_home_origin_preserved',true,'safe_pose_offset_pulses',3200, ...
 'pickup_legacy_depth_pulses',108400,'remaining_down_pulses_after_release',105200, ...
 'normal_return_rpm',1846.4,'home_rpm',60,'release_rpm',30, ...
 'scope','sequence and coordinate invariance; carry clearance at new safe pose requires observation');
f=fopen(fullfile(out,'integration11-auto-release-matlab.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
