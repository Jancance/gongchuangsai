function rotation_200_precheck()
% Analytical and 5 ms checks for a 90 degree quintic mecanum turn.
root=fileparts(mfilename('fullpath'));
repo=fileparts(fileparts(fileparts(root)));
parameters=fileread(fullfile(repo,'3.stm32f407工程','USER','vehicle_parameters.h'));
diameter=readDefine(parameters,'VEHICLE_WHEEL_DIAMETER_MM');
track=readDefine(parameters,'VEHICLE_TRACK_WIDTH_MM');
wheelbase=readDefine(parameters,'VEHICLE_WHEELBASE_MM');
k=(track+wheelbase)/2;
wheelPerDegSec=k*pi/180/(diameter*pi/60);
A=90; nominalT=2.080; desiredWheelRPM=200;
yawV=90; yawA=180; yawJ=1440;
wheelV=640; wheelA=1000; wheelJ=4500;
q1=1.875; q2=10/sqrt(3); q3=60;
targetYawV=desiredWheelRPM/wheelPerDegSec;
targetT=q1*A/targetYawV;
Tmin=[q1*A/yawV, sqrt(q2*A/yawA), (q3*A/yawJ)^(1/3), ...
    wheelPerDegSec*q1*A/wheelV, sqrt(wheelPerDegSec*q2*A/wheelA), ...
    (wheelPerDegSec*q3*A/wheelJ)^(1/3)];
allowedT=max(Tmin);
durationNames={'yaw_v90','yaw_a180','yaw_j1440','wheel_rpm640','wheel_a1000','wheel_j4500'};
cases=[nominalT,targetT,allowedT];
rows=cell(1,3);
fprintf('Geometry D=%.3f mm track=%.3f wheelbase=%.3f R=%.3f mm wheelRPM/degps=%.9f\n', ...
    diameter,track,wheelbase,k,wheelPerDegSec);
fprintf('Fixed 90 deg quintic 10s^3-15s^4+6s^5; endpoint velocity and acceleration zero, jerk nonzero.\n');
for ci=1:numel(cases)
    T=cases(ci); t=(0:0.005:T)'; if t(end)<T, t=[t;T]; end
    s=t/T;
    angle=A*(10*s.^3-15*s.^4+6*s.^5);
    rate=A/T*(30*s.^2-60*s.^3+30*s.^4);
    accel=A/T^2*(60*s-180*s.^2+120*s.^3);
    jerk=A/T^3*(60-360*s+360*s.^2);
    wheelRPM=wheelPerDegSec*rate;
    wheelAccel=wheelPerDegSec*accel;
    wheelJerk=wheelPerDegSec*jerk;
    observed=[max(abs(rate)),max(abs(accel)),max(abs(jerk)), ...
        max(abs(wheelRPM)),max(abs(wheelAccel)),max(abs(wheelJerk))];
    limits=[yawV,yawA,yawJ,wheelV,wheelA,wheelJ];
    pass=observed<=limits+1e-8;
    row=struct('duration_s',T,'angle_deg',angle(end), ...
        'yaw_v_peak_deg_s',observed(1),'yaw_a_peak_deg_s2',observed(2), ...
        'yaw_j_peak_deg_s3',observed(3),'wheel_rpm_peak',observed(4), ...
        'wheel_accel_peak_rpm_s',observed(5),'wheel_jerk_peak_rpm_s2',observed(6), ...
        'limits',limits,'pass',pass,'violations',{durationNames(~pass)}, ...
        'start_end_yaw_rate_deg_s',[rate(1),rate(end)], ...
        'start_end_yaw_accel_deg_s2',[accel(1),accel(end)], ...
        'start_end_yaw_jerk_deg_s3',[jerk(1),jerk(end)], ...
        'samples',numel(t),'step_s',0.005);
    rows{ci}=row;
    fprintf('T=%.6f angle=%.1f yawV=%.3f yawA=%.3f yawJ=%.3f wheelRPM=%.3f wheelA=%.3f wheelJ=%.3f pass=%u violations=%s\n', ...
        T,A,observed(1),observed(2),observed(3),observed(4),observed(5),observed(6),all(pass),strjoin(durationNames(~pass),','));
end
assert(abs(rows{1}.yaw_v_peak_deg_s-81.1298076923)<0.01);
assert(all(rows{1}.pass));
assert(abs(rows{2}.wheel_rpm_peak-desiredWheelRPM)<0.05);
assert(~all(rows{2}.pass));
assert(abs(rows{3}.duration_s-1.875)<1e-9 && all(rows{3}.pass));
assert(abs(rows{1}.start_end_yaw_jerk_deg_s3(1)-q3*A/nominalT^3)<1e-8);
expectedEndpointJerk=q3*A/nominalT^3;
assert(abs(rows{1}.start_end_yaw_jerk_deg_s3(2)-expectedEndpointJerk)<1e-6*max(1,expectedEndpointJerk));
maxWheelAtAllowed=wheelPerDegSec*yawV;
result=struct('rotation_deg',A,'profile','quintic 10s^3-15s^4+6s^5', ...
    'nominal_duration_s',nominalT,'desired_motor_peak_rpm',desiredWheelRPM, ...
    'wheel_rpm_per_yaw_deg_s',wheelPerDegSec,'desired_equivalent_yaw_peak_deg_s',targetYawV, ...
    'duration_to_reach_target_wheel_rpm_s',targetT,'duration_min_by_limits_s',Tmin, ...
    'minimum_duration_all_limits_s',allowedT,'max_wheel_rpm_at_all_yaw_limits',maxWheelAtAllowed, ...
    'cases',[rows{:}],'measured_on_vehicle',false,'endpoint_jerk_is_nonzero',true,'notes', ...
    '200 rpm is a requested target, not an observed speed. The firmware yaw_v90 planner cap maps to 81.43 wheel rpm.');
fid=fopen(fullfile(root,'rotation_200_precheck_result.json'),'w'); c=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(result));
fprintf('PASS: duration-specific constraints, discrete 5 ms profile, zero endpoint rate/acceleration, and nonzero endpoint jerk checks. MATLAB %s\n',version);
end

function x=readDefine(source,name)
token=regexp(source,['#define\s+' name '\s+([0-9.]+)f?'],'tokens','once');
assert(~isempty(token),'Missing geometry define %s',name); x=str2double(token{1});
end
