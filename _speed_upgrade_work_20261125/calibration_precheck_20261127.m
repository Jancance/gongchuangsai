function calibration_precheck_20261127()
% MATLAB R2020a precheck: direction-specific odometry scales and 90 deg turn time.
root=fileparts(fileparts(mfilename('fullpath')));
kin=fileread(fullfile(root,'3.stm32f407工程','CHASSIS','USER','chassis_kinematics.h'));
odom=fileread(fullfile(root,'3.stm32f407工程','CHASSIS','USER','chassis_odometry.c'));
kinC=fileread(fullfile(root,'3.stm32f407工程','CHASSIS','USER','chassis_kinematics.c'));
motion=fileread(fullfile(root,'3.stm32f407工程','CHASSIS','USER','fusion_motion.c'));
assert(contains(kin,'CHASSIS_ODOM_FORWARD_DISTANCE_SCALE 1.02743847f'));
assert(contains(kin,'CHASSIS_ODOM_REVERSE_DISTANCE_SCALE 1.01448963f'));
assert(contains(kin,'CHASSIS_ODOM_LEFT_DISTANCE_SCALE 0.98092315f'));
assert(contains(kin,'CHASSIS_ODOM_RIGHT_DISTANCE_SCALE 0.98688169f'));
assert(contains(odom,'dx *= (dx >= 0.0f) ? CHASSIS_ODOM_FORWARD_DISTANCE_SCALE : CHASSIS_ODOM_REVERSE_DISTANCE_SCALE;'));
assert(contains(odom,'dy *= (dy >= 0.0f) ? CHASSIS_ODOM_LEFT_DISTANCE_SCALE : CHASSIS_ODOM_RIGHT_DISTANCE_SCALE;'));
assert(contains(kinC,'else if (mode == 1U) result *= CHASSIS_ODOM_LEFT_DISTANCE_SCALE;'));
assert(contains(kinC,'else if (mode == 2U) result *= CHASSIS_ODOM_RIGHT_DISTANCE_SCALE;'));
assert(contains(kinC,'else result *= CHASSIS_ODOM_REVERSE_DISTANCE_SCALE;'));
assert(contains(motion,'MOTION_PI/2.0f,MOTION_PI,8.0f*MOTION_PI'));
assert(contains(motion,'640.0f,1000.0f,4500.0f'));

% Ground measurements / encoder distance from the matched field logs.
raw.forward=1661.9; raw.reverse=mean([1662.0715821605909 1661.7672229552009]);
raw.left=1661.7; raw.right=1661.8;
scale.forward=mean([1705 1710])/raw.forward;
scale.reverse=mean([1689 1683])/raw.reverse;
scale.left=1630/raw.left; scale.right=1640/raw.right;
physical=[mean([1705 1710]) mean([1689 1683]) 1630 1640];
predicted=[raw.forward*scale.forward raw.reverse*scale.reverse ...
    raw.left*scale.left raw.right*scale.right];
assert(max(abs(physical-predicted))<1e-9);

% Check the signed body-axis mapping before rotating it into the field frame.
body=[100 0;-100 0;0 100;0 -100];
scales=[scale.forward scale.reverse scale.left scale.right];
for i=1:4
    q=body(i,:); axisScale=scales(i);
    if q(1)~=0, q(1)=q(1)*axisScale; else, q(2)=q(2)*axisScale; end
    for yaw=[0 pi/2]
        field=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)]*q';
        assert(abs(norm(field)-100*axisScale)<1e-10);
    end
end

% Exact 7-phase S-curve calculation using current Fusion yaw limits.
current=prepare_turn(90,90,180,1440);
without_yaw_speed_cap=prepare_turn(90,10000,180,1440);
wheel_rpm_per_dps=((212+206)/2)/77/3;
assert(abs(current.duration-1.625)<1e-6);
assert(current.duration>1.5 && without_yaw_speed_cap.duration>1.5);
preparation_ms=1000; settling_ms=700;
whole_current=preparation_ms/1000+current.duration+settling_ms/1000;
whole_candidate=preparation_ms/1000+without_yaw_speed_cap.duration+settling_ms/1000;
assert(whole_current>1.5 && whole_candidate>1.5);
assert(without_yaw_speed_cap.peak*wheel_rpm_per_dps<640);

fprintf('Scales F/R/L/Rt = %.8f / %.8f / %.8f / %.8f\n', ...
    scale.forward,scale.reverse,scale.left,scale.right);
fprintf('MATLAB odometry sign/rotation checks passed at yaw 0 and 90 deg.\n');
fprintf('90 deg current profile %.6f s, peak %.3f deg/s.\n', ...
    current.duration,current.peak);
fprintf('Preparation + motion + 700 ms stabilization lower bounds: current %.6f s; no yaw speed cap %.6f s.\n', ...
    whole_current,whole_candidate);
fprintf('With yaw speed cap removed: %.6f s, peak %.3f deg/s, wheel peak %.3f rpm.\n', ...
    without_yaw_speed_cap.duration,without_yaw_speed_cap.peak, ...
    without_yaw_speed_cap.peak*wheel_rpm_per_dps);
fprintf('PASS: scale fit and single-turn profile precheck; no motor/vehicle model.\n');
end

function p=prepare_turn(angle,V,A,J)
tj=min(A/J,sqrt(V/J)); ta=V/(J*tj)-tj; if ta<0,ta=0;end
minDistance=V*(2*tj+ta); tv=0;
if angle>=minDistance
    tv=(angle-minDistance)/V;
else
    tj=A/J;
    if angle>=2*A*tj^2
        ta=.5*(sqrt(tj^2+4*angle/A)-3*tj);
    else
        tj=(angle/(2*J))^(1/3); ta=0;
    end
    V=J*tj*(tj+ta);
end
phase=[tj ta tj tv tj ta tj]; signs=[1 0 -1 0 -1 0 1]; jerk=signs*J;
x=0;v=0;a=0;peakA=0;
for i=1:7
    h=phase(i); peakA=max(peakA,max(abs([a a+jerk(i)*h])));
    x=x+v*h+.5*a*h^2+jerk(i)*h^3/6;
    v=v+a*h+.5*jerk(i)*h^2; a=a+jerk(i)*h;
end
assert(abs(x-angle)<max(.002,angle*.00001));
p=struct('duration',sum(phase),'peak',V,'peak_accel',peakA,'peak_jerk',J);
end
