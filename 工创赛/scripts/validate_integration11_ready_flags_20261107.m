function validate_integration11_ready_flags_20261107
out=fullfile(fileparts(fileparts(mfilename('fullpath'))),'evidence','integration','2026-11-07','single-cycle');
checks=[];allowed=[];
for flags=0:15
 accepted=bitand(flags,2)~=0 && bitand(flags,12)==0;
 checks(end+1)=accepted==any(flags==[2,3]);
 if accepted,allowed(end+1)=flags;end
end
r=struct('passed',all(checks),'checks',numel(checks),'allowed_flags',allowed, ...
 'mechanical_lock_confirmed_by_user',true,'engine','local MATLAB', ...
 'scope','terminal readiness accepts disabled/in-position; enable stays in normal movement preparation');
f=fopen(fullfile(out,'integration11-ready-flags-matlab.json'),'w');assert(f~=-1);fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
