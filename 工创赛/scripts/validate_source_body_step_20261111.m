function validate_source_body_step_20261111
% 当前后端同公式七段S曲线；仅10mm左移模型，不替代电机/滑移验收。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-11','source-body-step');
if ~exist(out,'dir'),mkdir(out);end
factor=pi*77/60;speed=15;distance=10;
acceleration=1000-factor*2*2*1;jerk=2000-factor*6*2*1^2;
tj=min(acceleration/jerk,sqrt(speed/jerk));ta=max(0,speed/(jerk*tj)-tj);
minimum=speed*(2*tj+ta);tv=0;
if distance>=minimum,tv=(distance-minimum)/speed;
else
 tj=acceleration/jerk;
 if distance>=2*acceleration*tj^2,ta=.5*(sqrt(tj*tj+4*distance/acceleration)-3*tj);
 else,tj=(distance/(2*jerk))^(1/3);ta=0;end
end
duration=[tj ta tj tv tj ta tj];signs=[1 0 -1 0 -1 0 1];
x=0;v=0;a=0;elapsed=0;values=[];
for k=1:7
 dt=linspace(0,duration(k),max(2,ceil(duration(k)/.001)));j=jerk*signs(k);
 values=[values;elapsed+dt',x+v*dt'+.5*a*dt'.^2+j*dt'.^3/6, ...
  v+a*dt'+.5*j*dt'.^2,a+j*dt',ones(size(dt'))*j];
 t=duration(k);x=x+v*t+.5*a*t*t+j*t^3/6;v=v+a*t+.5*j*t*t;a=a+j*t;elapsed=elapsed+t;
end
checks=struct('name',{},'passed',{});
check('10mm within existing route lower bound',distance==10);
check('native requested15mm per second',speed==15 && max(values(:,3))<=speed+1e-7);
check('nonnegative monotonic travel to10mm',min(values(:,3))>=-1e-7 && abs(x-10)<1e-7);
check('zero speed and acceleration at completion',abs(v)<1e-7 && abs(a)<1e-7);
check('acceleration and jerk within reserved backend values',max(abs(values(:,4)))<=acceleration+1e-7 && max(abs(values(:,5)))<=jerk+1e-7);
check('single action stays inside user10cm free space',max(values(:,2))<100);
% Ruler model predicts the scene, not an actuator result. Camera travels left
% (away from disk). A frozen disk should move upward in the declared view.
cameraScale=norm([.2410467291222243 .036166778783864782]);pixels=distance/cameraScale;
check('expected image signal measurably exceeds prior1px jitter',pixels>30 && pixels<60);
% A real guard must acquire post-settle feedback. This models four轮status
% staggered80ms, not real RTOS timing or guaranteed bus utilisation.
ticks=[1080 1160 1240 1320];now=1340;
check('post1second settle actual wheel status can establish freshness',all(now-ticks<=1000) && all(ticks>=1000));
for bad=1:4
 test=ticks;test(bad)=0;
 check(sprintf('stale wheel%d cannot establish postsettle guard',bad),~all(test>=1000));
end
r=struct('engine','local MATLAB','release',version('-release'),'distance_mm',distance, ...
 'direction','vehicle-left; away from disk per user180deg arm points vehicle-right', ...
 'reference_speed_mm_s',speed,'profile_duration_s',elapsed,'profile_peak_speed_mm_s',max(values(:,3)), ...
 'reserved_acceleration_mm_s2',acceleration,'reserved_jerk_mm_s3',jerk, ...
 'nominal_expected_scene_upward_px',pixels,'actual_can_or_motor_verified',false, ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'checks',checks,'passed',all([checks.passed]), ...
 'motion_allowed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(r));fclose(f);
save(fullfile(out,'matlab-result.mat'),'values','r');
fprintf('MATLAB body step %d/%d failures: left10mm speed15mm/s duration%.6fs; nominal scene up%.3fpx. No physical claim.\n', ...
 numel(checks),r.failure_count,elapsed,pixels);assert(r.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
end
