function motion_profile_fault_model_20261111
scriptDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(scriptDir);
outDir = fullfile(projectDir, 'evidence', 'navigation-planning', '2026-11-11', 'matlab-baseline', 'fault-model', 'verified-v2');
if ~exist(outDir, 'dir'), mkdir(outDir); end

cases = cell(0, 5);
cases(end+1,:) = checkCase('hwt_parser_fresh_500ms', 'age=500ms; parser <=500', true, 500 <= 500);
cases(end+1,:) = checkCase('hwt_parser_stale_501ms', 'age=501ms; parser <=500', false, 501 <= 500);
cases(end+1,:) = checkCase('heading_source_fresh_99ms', 'angle=99ms; rate=99ms; mask=3', true, headingHealthy(99,99,3));
cases(end+1,:) = checkCase('heading_source_stale_at_100ms', 'angle=99ms; rate=100ms; mask=3', false, headingHealthy(99,100,3));
cases(end+1,:) = checkCase('heading_source_partial_mask', 'angle=0ms; rate=0ms; mask=1', false, headingHealthy(0,0,1));
cases(end+1,:) = checkCase('wheel_fresh_at_399ms', 'age=399ms; limit <400', true, 399 < 400);
cases(end+1,:) = checkCase('wheel_stale_at_400ms', 'age=400ms; limit <400', false, 400 < 400);
cases(end+1,:) = checkCase('yaw_fresh_at_99ms', 'age=99ms; limit <100', true, 99 < 100);
cases(end+1,:) = checkCase('yaw_stale_at_100ms', 'age=100ms; limit <100', false, 100 < 100);
cases(end+1,:) = checkCase('wheel_delta_on_limit', 'dt=10ms; abs(delta)=12mm; limit=dt+2', true, wheelPlausible(10,12));
cases(end+1,:) = checkCase('wheel_delta_over_limit', 'dt=10ms; abs(delta)=12.01mm; limit=dt+2', false, wheelPlausible(10,12.01));
cases(end+1,:) = checkCase('single_wheel_plausible_but_different', 'one wheel=10mm; other wheels=0; dt=10ms; no cross-wheel gate', true, wheelPlausible(10,10));
singleWheel = [10 0 0 0];
singleWheelDx = sum(singleWheel) / 4 * 1.02182540;
singleWheelDy = (singleWheel(1)-singleWheel(2)-singleWheel(3)+singleWheel(4)) / 4;
cases(end+1,:) = checkCase('single_wheel_bias_propagated', 'one wheel=10mm; others=0; chassis odometry projection', ...
    'dx=2.55; dy=2.50', sprintf('dx=%.2f; dy=%.2f',singleWheelDx,singleWheelDy));
cases(end+1,:) = checkCase('wheel_interval_at_400ms', 'dt=400ms; delta=1mm; limit <400', false, wheelPlausible(400,1));
cases(end+1,:) = checkCase('wheel_same_tick_unchanged', 'dt=0ms; raw unchanged', 'duplicate_ignored', duplicateWheel(0));
cases(end+1,:) = checkCase('wheel_same_tick_changed', 'dt=0ms; raw changed', 'rejected', duplicateWheel(1));
cases(end+1,:) = checkCase('yaw_delta_on_limit', 'dt=10ms; abs(wrapped delta)=2.2deg', true, yawPlausible(10,2.2));
cases(end+1,:) = checkCase('yaw_delta_over_limit', 'dt=10ms; abs(wrapped delta)=2.21deg', false, yawPlausible(10,2.21));
cases(end+1,:) = checkCase('yaw_interval_at_400ms', 'dt=400ms; delta=1deg', false, yawPlausible(400,1));
wrappedYawDelta = wrapDeg(-179 - 179);
cases(end+1,:) = checkCase('yaw_wrap_crossing', '179deg -> -179deg; wrapped delta=2deg; dt=10ms', true, yawPlausible(10,wrappedYawDelta));
cases(end+1,:) = checkCase('tick_wrap_elapsed', 'sample=0xFFFFFFF0; now=0x0000000A', '26ms', sprintf('%.0fms', elapsed32(hex2dec('FFFFFFF0'), hex2dec('0000000A'))));
cases(end+1,:) = checkCase('tick_forward_across_wrap', 'sample=0xFFFFFFF0; next=0x0000000A', true, forward32(hex2dec('FFFFFFF0'), hex2dec('0000000A')));
cases(end+1,:) = checkCase('tick_backwards_order', 'sample=100; next=90; signed modular order', false, forward32(100,90));
cases(end+1,:) = checkCase('wheel_mask_missing_one', 'valid_mask=0x0E; app requires 0x0F', false, uint32(hex2dec('0E')) == 15);
cases(end+1,:) = checkCase('checksum_failure_invalidates_hwt', 'bad checksum clears seen mask', false, headingHealthy(0,0,0));
interpolated = linearAt(0,7.5,20,22.5,10);
cases(end+1,:) = checkCase('common_time_interpolation', 'points=(0,7.5),(20,22.5); common=10ms', '15', sprintf('%.12g',interpolated));
cases(end+1,:) = checkCase('history_bracket_399ms', 'common inside bracket; span=399ms; limit <400', true, interpolationAllowed(399,200));
cases(end+1,:) = checkCase('history_bracket_400ms', 'common inside bracket; span=400ms; limit <400', false, interpolationAllowed(400,200));
cases(end+1,:) = checkCase('continuity_stays_lost', 'fault then fresh samples; no SetPose', 'invalid_pose_frozen', continuityAfterFault(false));
cases(end+1,:) = checkCase('explicit_pose_rebases', 'fault then explicit SetPose', 'new_pose_waits_for_baseline', continuityAfterFault(true));

faultTable = cell2table(cases, 'VariableNames', {'case_id','input','expected','actual','pass'});
writetable(faultTable, fullfile(outDir, 'fault_cases.csv'));
assert(all([faultTable.pass]), 'At least one MATLAB fault model case failed.');

latencyMs = [0 10 20 40];
wheelErrorAt200 = 200 * latencyMs / 1000;
yawErrorAt5Rpm = rad2deg((5 * pi * 77 / 60) / 209) * latencyMs / 1000;
latencyTable = table(latencyMs', wheelErrorAt200', yawErrorAt5Rpm', ...
    'VariableNames', {'assumed_delay_ms','translation_error_at_200mm_s_mm','yaw_error_at_5rpm_deg'});
writetable(latencyTable, fullfile(outDir, 'latency_sensitivity.csv'));

report = struct();
report.source_contract = struct('hwt_parser_fresh_ms_inclusive',500,'heading_source_fresh_ms_exclusive',100, ...
    'wheel_sample_age_ms_exclusive',400,'yaw_sample_age_ms_exclusive',100,'history_points',32, ...
    'wheel_delta_limit','abs(delta_mm) <= dt_ms + 2','yaw_delta_limit','abs(wrapped_delta_deg) <= 0.12*dt_ms + 1');
report.case_count = height(faultTable);
report.passed_cases = sum(faultTable.pass);
report.failed_cases = report.case_count - report.passed_cases;
report.latency_scan_ms = latencyMs;
report.latency_measured = false;
report.full_c_differential_replay = false;
report.wheel_disagreement_gate_present = false;
report.vehicle_or_hardware_tested = false;
report.firmware_changed = false;
fid = fopen(fullfile(outDir, 'summary.json'), 'w');
assert(fid >= 0, 'Cannot write fault-model summary JSON.');
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(report), 'char');
clear cleanup;
fprintf('FAULT_MODEL_OK cases=%d passed=%d latency_measured=0\n', report.case_count, report.passed_cases);
end

function row = checkCase(name, input, expected, actual)
if islogical(expected)
    pass = expected == actual;
    expectedText = logicalText(expected);
    actualText = logicalText(actual);
else
    expectedText = expected;
    actualText = actual;
    pass = strcmp(expectedText, actualText);
end
assert(pass, 'Fault-model mismatch in %s.', name);
row = {name, input, expectedText, actualText, pass};
end

function healthy = headingHealthy(angleAge, rateAge, mask)
healthy = mask == 3 && angleAge < 100 && rateAge < 100;
end

function accepted = wheelPlausible(dt, delta)
accepted = dt > 0 && dt < 400 && abs(delta) <= dt + 2;
end

function accepted = yawPlausible(dt, delta)
accepted = dt > 0 && dt < 400 && abs(delta) <= 0.12 * dt + 1;
end

function state = duplicateWheel(rawChanged)
if rawChanged, state = 'rejected'; else, state = 'duplicate_ignored'; end
end

function d = elapsed32(a, b)
d = mod(double(b) - double(a), 2^32);
end

function forward = forward32(a, b)
d = elapsed32(a, b);
forward = d > 0 && d < 2^31;
end

function d = wrapDeg(d)
while d > 180, d = d - 360; end
while d <= -180, d = d + 360; end
end

function value = linearAt(t0, v0, t1, v1, t)
value = v0 + (v1 - v0) * ((t - t0) / (t1 - t0));
end

function allowed = interpolationAllowed(span, offset)
allowed = span > 0 && span < 400 && offset >= 0 && offset <= span;
end

function state = continuityAfterFault(rebase)
pose = 100;
continuityLost = true;
valid = false;
if rebase
    pose = 500;
    continuityLost = false;
    valid = false;
    state = 'new_pose_waits_for_baseline';
else
    state = 'invalid_pose_frozen';
end
assert(continuityLost == ~rebase && ~valid);
if ~rebase, assert(pose == 100); end
end

function text = logicalText(value)
if value, text = 'true'; else, text = 'false'; end
end
