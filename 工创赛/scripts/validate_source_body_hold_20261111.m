function validate_source_body_hold_20261111
% 仅验证显式小步的保持权限；不模拟机械、不放宽普通机械动作准入。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-11','source-body-hold');
if ~exist(out,'dir'),mkdir(out);end
% columns: ready,CAN result,safety fault,RTOS fault,CAN fault,body active,owner window,holding.
rows=zeros(257,8);
for n=0:255,rows(n+1,:)=bitget(n,1:8);end
rows(end,:)=[1 2 0 0 0 1 1 1];
healthy=rows(:,1)==1 & rows(:,2)==1 & ~rows(:,3) & ~rows(:,4) & ~rows(:,5);
ordinary=healthy & ~rows(:,6);
held=ordinary | (healthy & rows(:,7)==1 & rows(:,8)==1);
checks=struct('name',{},'passed',{});
check('all257 rows retain original ordinary motion condition',all(ordinary==(healthy & ~rows(:,6))));
check('no failed health combination is accepted',~any(held & ~healthy));
check('active body requires explicit owner window',~any(held & rows(:,6)==1 & rows(:,7)==0));
check('active body requires existing real hold',~any(held & rows(:,6)==1 & rows(:,8)==0));
case_row=find(all(rows==repmat([1 1 0 0 0 1 1 1],size(rows,1),1),2));
check('old active-body predicate reproduces false rejection',~ordinary(case_row));
check('dedicated hold predicate admits same healthy owner',held(case_row));
check('non-success CAN selftest2 remains rejected',~held(end));
check('idle healthy hold remains unchanged',all(held(healthy & ~rows(:,6))));
r=struct('engine','local MATLAB','release',version('-release'),'passed',all([checks.passed]), ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'checks',checks, ...
 'inputs',rows,'ordinary_expected',ordinary,'held_expected',held,'hardware_access',false, ...
 'physical_motion_verified',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(r));fclose(f);
save(fullfile(out,'matlab-result.mat'),'r');
fprintf('MATLAB held-platform: %d checks / %d failures;257 health/ownership combinations.\n',numel(checks),r.failure_count);
assert(r.passed);
 function check(name,passed),checks(end+1)=struct('name',name,'passed',logical(passed));end
end
