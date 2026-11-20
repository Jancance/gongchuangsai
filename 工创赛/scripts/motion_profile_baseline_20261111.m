function motion_profile_baseline_20261111
root = fileparts(fileparts(mfilename('fullpath')));
outDir = fullfile(root, 'evidence', 'navigation-planning', '2026-11-11', 'matlab-baseline');
if ~exist(outDir, 'dir'), mkdir(outDir); end

wheelDiameter = 77.0;
trackWidth = 212.0;
wheelbase = 206.0;
turnLever = 0.5 * (trackWidth + wheelbase);
wheelMmPerRpm = pi * wheelDiameter / 60.0;
distanceScale = 1.02182540;
followRpm = 2.0;
followOmega = 1.0;
wheelRpmCap = 52.0;
requestedAccel = 40.0;
requestedJerk = 80.0;
directions = [1 0; 0 1; 0 -1; -1 0];
directionNames = {'forward','left','right','backward'};
distances = [100 1000 2100];
speeds = [120 200];
dt = 0.002;

profileRows = zeros(numel(directions(:,1))*numel(distances)*numel(speeds), 8);
row = 0;
maxEndpointError = 0;
maxSpeedObserved = 0;
for di = 1:size(directions, 1)
    wheelFactor = wheelMmPerRpm;
    if di == 1 || di == 4, wheelFactor = wheelFactor * distanceScale; end
    accel = requestedAccel - wheelFactor * 2 * followRpm * followOmega;
    jerk = requestedJerk - wheelFactor * 6 * followRpm * followOmega^2;
    assert(accel > 0 && jerk > 0, 'Route follow reserve consumes the configured acceleration or jerk.');
    for vi = 1:numel(speeds)
        speed = speeds(vi);
        rpm = ceil(speed / wheelFactor) + 2;
        assert(rpm <= wheelRpmCap, 'Requested route speed exceeds the candidate wheel RPM budget.');
        limits = [speed accel jerk];
        for si = 1:numel(distances)
            distance = distances(si);
            [phase, duration, peak] = prepareProfile(distance, limits);
            t = (0:dt:duration)';
            if t(end) < duration, t(end+1,1) = duration; end
            [position, velocity] = sampleProfile(phase, t, distance, duration);
            err = abs(position(end) - distance);
            assert(err < max(0.002, distance * 1e-5) + 1e-8, 'Profile endpoint error exceeds the C planner tolerance.');
            assert(all(diff(position) >= -1e-7), 'Profile position must be monotonic.');
            assert(abs(velocity(1)) < 1e-9 && abs(velocity(end)) < 1e-9, 'Profile must start and end at rest.');
            assert(max(velocity) <= speed + 1e-8, 'Profile exceeds its requested speed.');
            row = row + 1;
            profileRows(row,:) = [di vi distance speed accel jerk duration peak];
            maxEndpointError = max(maxEndpointError, err);
            maxSpeedObserved = max(maxSpeedObserved, max(velocity));
        end
    end
end

profileTable = array2table(profileRows, 'VariableNames', ...
    {'direction_index','speed_index','distance_mm','speed_limit_mm_s','accel_limit_mm_s2','jerk_limit_mm_s3','duration_s','peak_speed_mm_s'});
profileTable.direction = directionNames(profileTable.direction_index)';
writetable(profileTable, fullfile(outDir, 'profile_cases.csv'));

turnAngleDeg = 90.0;
turnDistance = turnLever * deg2rad(turnAngleDeg);
turnSpeed = 5.0 * wheelMmPerRpm;
turnAccel = requestedAccel - wheelMmPerRpm * 2 * followRpm * followOmega;
turnJerk = requestedJerk - wheelMmPerRpm * 6 * followRpm * followOmega^2;
[turnPhase, turnDuration, turnPeak] = prepareProfile(turnDistance, [turnSpeed turnAccel turnJerk]);

mixes = [120 0 0; 0 120 0; 0 0 5*wheelMmPerRpm/turnLever; ...
    120 80 5*wheelMmPerRpm/turnLever; 200 0 0; 0 200 0; ...
    120 120 5*wheelMmPerRpm/turnLever];
mixRows = zeros(size(mixes,1), 6);
for i = 1:size(mixes,1)
    vx = mixes(i,1); vy = mixes(i,2); wz = mixes(i,3);
    turn = turnLever * wz;
    wheelRpm = [vx+vy-turn, vx-vy+turn, vx-vy-turn, vx+vy+turn] / wheelMmPerRpm;
    mixRows(i,:) = [vx vy wz max(abs(wheelRpm)) wheelRpmCap max(abs(wheelRpm)) <= wheelRpmCap];
end
mixTable = array2table(mixRows, 'VariableNames', ...
    {'vx_mm_s','vy_mm_s','wz_rad_s','peak_wheel_rpm','candidate_limit_rpm','within_candidate_limit'});
writetable(mixTable, fullfile(outDir, 'wheel_demand_cases.csv'));

latencyMs = [0 10 20 40];
linearError = speeds(end) * latencyMs / 1000;
yawRate = 5.0 * wheelMmPerRpm / turnLever;
yawErrorDeg = rad2deg(yawRate * latencyMs / 1000);
latencyTable = table(latencyMs', linearError', yawErrorDeg', ...
    'VariableNames', {'assumed_delay_ms','translation_error_at_200mm_s_mm','yaw_error_at_5rpm_deg'});
writetable(latencyTable, fullfile(outDir, 'latency_sensitivity.csv'));

validMasks = uint8([15 14 13 11 7]);
accepted = validMasks == 15;
assert(accepted(1) && all(~accepted(2:end)), 'The source feedback contract requires all four wheel samples.');

fig = figure('Visible','off','Color','w');
hold on;
for di = 1:4
    wheelFactor = wheelMmPerRpm;
    if di == 1 || di == 4, wheelFactor = wheelFactor * distanceScale; end
    accel = requestedAccel - wheelFactor * 2 * followRpm * followOmega;
    jerk = requestedJerk - wheelFactor * 6 * followRpm * followOmega^2;
    [phase, duration] = prepareProfile(1000, [200 accel jerk]);
    t = linspace(0, duration, 1200)';
    [p, ~] = sampleProfile(phase, t, 1000, duration);
    plot(t, p, 'LineWidth', 1.4, 'DisplayName', directionNames{di});
end
grid on; xlabel('Time (s)'); ylabel('Progress (mm)'); legend('Location','southeast');
title('Seven-phase profile, 1000 mm, 200 mm/s candidate');
print(fig, fullfile(outDir, 'profile_reference.png'), '-dpng', '-r120');
close(fig);

report = struct();
report.matlab_version = version;
report.geometry_mm = struct('wheel_diameter',wheelDiameter,'track_width',trackWidth,'wheelbase',wheelbase);
report.planner_limits = struct('requested_accel_mm_s2',requestedAccel,'requested_jerk_mm_s3',requestedJerk, ...
    'follow_rpm',followRpm,'follow_omega',followOmega,'wheel_rpm_candidate_cap',wheelRpmCap);
report.profile_cases = height(profileTable);
report.max_endpoint_error_mm = maxEndpointError;
report.max_sampled_speed_mm_s = maxSpeedObserved;
report.turn90_proxy = struct('angle_deg',turnAngleDeg,'arc_mm',turnDistance,'speed_limit_mm_s',turnSpeed, ...
    'duration_s',turnDuration,'peak_speed_mm_s',turnPeak,'interpretation','kinematic proxy only; turn controller is not this profile');
report.mixed_case_count = height(mixTable);
report.mixed_cases_within_candidate_limit = sum(mixTable.within_candidate_limit);
report.assumed_latency_ms = latencyMs;
report.latency_is_measured = false;
report.feedback_valid_masks = validMasks;
report.feedback_accepted = accepted;
report.vehicle_or_hardware_tested = false;
report.firmware_changed = false;
fid = fopen(fullfile(outDir, 'summary.json'), 'w');
assert(fid >= 0, 'Cannot write MATLAB summary JSON.');
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(report), 'char');
clear cleanup;
fprintf('MATLAB_BASELINE_OK profiles=%d max_endpoint_error_mm=%.9g mixed_within=%d/%d turn90_proxy_s=%.6f\n', ...
    height(profileTable), maxEndpointError, report.mixed_cases_within_candidate_limit, height(mixTable), turnDuration);
end

function [phase, duration, peak] = prepareProfile(distance, limits)
speed = limits(1); accelLimit = limits(2); jerkLimit = limits(3);
tj = min(accelLimit / jerkLimit, sqrt(speed / jerkLimit));
ta = max(0, speed / (jerkLimit * tj) - tj);
minimumDistance = speed * (2 * tj + ta);
tv = 0;
peak = speed;
if distance < minimumDistance
    tj = accelLimit / jerkLimit;
    if distance >= 2 * accelLimit * tj^2
        ta = 0.5 * (sqrt(tj^2 + 4 * distance / accelLimit) - 3 * tj);
    else
        tj = (distance / (2 * jerkLimit))^(1/3);
        ta = 0;
    end
    peak = jerkLimit * tj * (tj + ta);
else
    tv = (distance - minimumDistance) / speed;
end
assert(tj > 0 && ta >= 0 && tv >= 0 && peak <= speed * 1.00001);
phaseTime = [tj ta tj tv tj ta tj];
phaseJerk = [jerkLimit 0 -jerkLimit 0 -jerkLimit 0 jerkLimit];
phase = zeros(7,5);
position = 0; velocity = 0; acceleration = 0; time = 0;
for k = 1:7
    d = phaseTime(k); j = phaseJerk(k);
    phase(k,:) = [time position velocity acceleration j];
    position = position + velocity*d + 0.5*acceleration*d^2 + j*d^3/6;
    velocity = velocity + acceleration*d + 0.5*j*d^2;
    acceleration = acceleration + j*d;
    time = time + d;
end
duration = time;
assert(abs(position - distance) <= 0.002 + distance*1e-5);
end

function [position, velocity] = sampleProfile(phase, time, distance, duration)
phaseTime = [phase(:,1); duration];
position = zeros(size(time)); velocity = zeros(size(time));
for n = 1:numel(time)
    if time(n) >= duration
        position(n) = distance;
        continue;
    end
    k = find(time(n) < phaseTime(2:end), 1, 'first');
    d = time(n) - phase(k,1);
    position(n) = phase(k,2) + phase(k,3)*d + 0.5*phase(k,4)*d^2 + phase(k,5)*d^3/6;
    velocity(n) = phase(k,3) + phase(k,4)*d + 0.5*phase(k,5)*d^2;
end
end
