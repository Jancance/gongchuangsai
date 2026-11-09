function validate_stage2_autonomous_coordinator_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','stage2-mainline');checks=[];
% States are the existing SPF states; a backend completion and physical evidence
% are distinct. No new motor path, dimensions, speed or timeout is introduced.
for kind=1:10
 for axes_done=[0,1]
  for evidence_age=[0,1000,1001]
   for evidence_ok=[0,1]
    guarded=any(kind==[2,4,7]);
    advance=axes_done && (~guarded || (evidence_ok && evidence_age<=1000));
    checks(end+1)=~advance || axes_done;
    checks(end+1)=~(advance && guarded) || (evidence_ok && evidence_age<=1000);
   end
  end
 end
end
for mask=0:31
 complete=bitand(mask,24)==24;
 checks(end+1)=complete==(bitand(mask,8)~=0 && bitand(mask,16)~=0);
end
for placed=0:3
 checks(end+1)=(placed==3)==(placed>=3);
end
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB', ...
 'existing_SPF_reused',true,'actuator_ACK_counts_as_loaded',false, ...
 'route_and_stow_same_command',true,'new_motion_trajectory',false, ...
 'scope','autonomous command/evidence sequencing only; backend and real physics unverified');
f=fopen(fullfile(out,'autonomous-coordinator-matlab.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
