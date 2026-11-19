function validate_black_short_extension_20261113
% Planning proxy only: speed/latency/deceleration budget, not a drive model.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-13','source-resume','pose');
p=jsondecode(fileread(fullfile(out,'matlab-result.json')));
distance=p.extensions_mm(3);assert(distance>0 && distance<10);
degree_per_mm=315.5*360/3200;accel=600*6;latency=.080;
speeds=[106 20];budget=zeros(1,2);duration=zeros(1,2);
for k=1:2
 v=speeds(k)*6;goal=distance*degree_per_mm;
 budget(k)=(v*latency+v^2/(2*accel))/degree_per_mm;
 if goal>=v^2/accel,duration(k)=goal/v+v/accel;else,duration(k)=2*sqrt(goal/accel);end
end
assert(budget(2)<45/degree_per_mm && budget(2)<budget(1));
d=struct('passed',true,'local_matlab',true,'distance_mm',distance,'selected_rpm',20, ...
 'comparison_rpm',speeds,'assumed_sample_latency_ms',80,'accel_rpm_s',600, ...
 'latency_and_braking_proxy_mm',budget,'ideal_duration_s',duration, ...
 'physical_performance_verified',false,'firmware_position_limits_unchanged',true,'motion_allowed',false);
f=fopen(fullfile(out,'black-short-matlab.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(d));fclose(f);
fprintf('SHORT5mm 20RPM proxy %.4fmm vs106RPM %.4fmm; no real-drive accuracy claim\n',budget(2),budget(1));
end
