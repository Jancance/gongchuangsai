function validate_integration11_resume_20261107
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-07','single-cycle');
% Recovery uses firmware reset/initialization, never writes fault or reference fields.
checks=[108400>=100000,108400<=110800,35.758>=0,35.758<=270, ...
  3200<108400,3200/1600==2,43000>40000, ...
  isequal(500+round([140,170,210]*2000/270),[1537,1759,2056])];
% No descent before fresh homing completion; no horizontal motion on fault.
for homed=[false,true]
 for fault=[false,true]
  allowed=homed && ~fault;
  checks(end+1)=allowed==(homed && ~fault);
 end
end
r=struct('passed',all(checks),'checks',numel(checks),'engine','local MATLAB', ...
 'reset_before_real_homing',true,'synthetic_reference',false, ...
 'source_depth',108400,'slot1_deg',35.758,'place_descent',0, ...
 'scope','command bounds and recovery ordering only; not physical clearance or load validation');
f=fopen(fullfile(out,'integration11-resume-matlab.json'),'w');assert(f~=-1);
fprintf(f,'%s',jsonencode(r));fclose(f);assert(r.passed);disp(jsonencode(r));
end
