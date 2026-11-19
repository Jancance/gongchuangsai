work=fileparts(mfilename('fullpath'));
header=fileread(fullfile(work,'stage3_anticipatory_settlefix_20261120','project','CHASSIS','USER','stage3_cycle.h'));
normLimit=defineValue(header,'S3_STRICT_NORM_UM');
yawLimit=defineValue(header,'S3_STRICT_YAW_MDEG');
assert(normLimit==2000 && yawLimit==1000);
floor=defineValue(header,'S3_STRICT_TRANSLATE_MIN_MM');
assert(floor==0.25);
n=[0;1501;1999;2000;2001]; yaw=[-1001 -1000 -999 0 999 1000 1001];
observed=n<=normLimit & abs(yaw)<=yawLimit;
expected=logical([1;1;1;1;0]*[0 1 1 1 1 1 0]);
assert(isequal(observed,expected));
assert(hypot(1200,1600)==normLimit && round(hypot(1200,1601))==normLimit+1);
report=struct('passed',true,'cases',numel(observed),'accept_norm_um',normLimit, ...
    'accept_abs_yaw_mdeg',yawLimit,'translate_floor_mm',floor,'physical_acceptance',false);
f=fopen(fullfile(work,'stage3_align20_yaw1_validation.json'),'w');assert(f>=0);
fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS alignment thresholds: %d signed yaw/Euclidean position boundary combinations; 2000 um, 1000 mdeg.\n',numel(observed));

function value=defineValue(header,name)
m=regexp(header,['(?m)^#define\s+' name '\s+([\d.]+)'],'tokens','once');
assert(~isempty(m),name);value=str2double(m{1});
end
