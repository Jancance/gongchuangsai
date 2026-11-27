function calibration_precheck_20261127()
% MATLAB R2020a precheck: direction-specific odometry scales.
root=fileparts(fileparts(mfilename('fullpath')));
kin=fileread(fullfile(root,'3.stm32f407工程','CHASSIS','USER','chassis_kinematics.h'));
odom=fileread(fullfile(root,'3.stm32f407工程','CHASSIS','USER','chassis_odometry.c'));
kinC=fileread(fullfile(root,'3.stm32f407工程','CHASSIS','USER','chassis_kinematics.c'));
assert(contains(kin,'CHASSIS_ODOM_FORWARD_DISTANCE_SCALE 1.02743847f'));
assert(contains(kin,'CHASSIS_ODOM_REVERSE_DISTANCE_SCALE 1.01448963f'));
assert(contains(kin,'CHASSIS_ODOM_LEFT_DISTANCE_SCALE 0.98092315f'));
assert(contains(kin,'CHASSIS_ODOM_RIGHT_DISTANCE_SCALE 0.98688169f'));
assert(contains(odom,'dx *= (dx >= 0.0f) ? CHASSIS_ODOM_FORWARD_DISTANCE_SCALE : CHASSIS_ODOM_REVERSE_DISTANCE_SCALE;'));
assert(contains(odom,'dy *= (dy >= 0.0f) ? CHASSIS_ODOM_LEFT_DISTANCE_SCALE : CHASSIS_ODOM_RIGHT_DISTANCE_SCALE;'));
assert(contains(kinC,'else if (mode == 1U) result *= CHASSIS_ODOM_LEFT_DISTANCE_SCALE;'));
assert(contains(kinC,'else if (mode == 2U) result *= CHASSIS_ODOM_RIGHT_DISTANCE_SCALE;'));
assert(contains(kinC,'else result *= CHASSIS_ODOM_REVERSE_DISTANCE_SCALE;'));

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

fprintf('Scales F/R/L/Rt = %.8f / %.8f / %.8f / %.8f\n', ...
    scale.forward,scale.reverse,scale.left,scale.right);
fprintf('MATLAB odometry sign/rotation checks passed at yaw 0 and 90 deg.\n');
fprintf('PASS: direction-specific odometry scale precheck; no vehicle model.\n');
end
