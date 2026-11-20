function check_position_lag(vectors)
% Recheck the frozen corner certificates with an additional 40 ms allowance.
data = jsondecode(fileread(vectors));
assert(data.passed && data.limits_model.batch == 0.08);
assert(data.accepted_windows == 25);
corners = struct([]);
legs = [];
for k = 1:numel(data.routes)
    selected = data.routes(k).corners;
    if isempty(selected), continue; end
    selected = reshape(selected(strcmp({selected.reason}, 'ok')), [], 1);
    corners = [corners; selected]; %#ok<AGROW>
    legs = [legs; repmat(k, numel(selected), 1)]; %#ok<AGROW>
end
assert(numel(corners) == 9);
extra = 0.12 - data.limits_model.batch;
margin = zeros(numel(corners), 1);
fprintf('leg center_x center_y original_mm added_guard_mm remaining_mm exit_mm\n');
for k = 1:numel(corners)
    corner = corners(k);
    assert(corner.clearance_mm > 0 && corner.max_speed_bound_mm_s > 0);
    added = corner.max_speed_bound_mm_s * extra;
    margin(k) = corner.clearance_mm - added;
    exit = corner.available_post_mm - sum(corner.stop_distance_and_guard_mm) ...
        - added - corner.join_speed_mm_s * extra;
    fprintf('%d %.0f %.0f %.4f %.4f %.4f %.4f\n', legs(k), corner.center_mm, ...
        corner.clearance_mm, added, margin(k), exit);
end
fprintf('P4_LAG_SCREEN %d frozen route corners, %d lose the original sufficient clearance bound\n', ...
    numel(corners), nnz(margin <= 0));

% Preserve the certified shape. Slower time parameterization reduces all
% derivative bounds; round the braking duration up to a full control batch.
batch = data.limits_model.batch;
delay = 0.12 + batch;
% Worst combined command and feedback rounding, decoded along both axes.
speedError = sqrt(2) * 0.55 * (77 * pi / 60);
accepted = 0;
fprintf('leg center_x center_y batches stop_batches observed_speed_low_mm_s clearance_low_mm exit_low_mm\n');
for k = 1:numel(corners)
    c = corners(k);
    original = c.nominal_batch_count;
    last = floor(original * c.min_speed_mm_s / (data.limits_model.minSpeed + speedError));
    oldEntry = c.join_speed_mm_s * batch;
    found = false;
    for n = original:last
        scale = n / original;
        joins = c.join_speed_mm_s / scale;
        stopBatches = ceil(c.stop_s * scale / batch);
        stopReach = joins * (delay + stopBatches * batch / 2);
        guard = data.limits_model.tracking + (c.max_speed_bound_mm_s / scale) * delay;
        oldStop = c.stop_distance_and_guard_mm(1);
        oldGuard = c.stop_distance_and_guard_mm(2);
        rise = max([0, joins * delay - oldEntry, stopReach - oldStop]);
        clear = c.clearance_mm - max(0, guard - oldGuard) - rise;
        exit = c.available_post_mm - stopReach - guard;
        if min(clear, exit) <= 0, continue; end
        speedLow = c.min_speed_mm_s / scale - speedError;
        assert(speedLow >= data.limits_model.minSpeed - 1e-9);
        found = true;
        accepted = accepted + 1;
        fprintf('%d %.0f %.0f %d %d %.4f %.4f %.4f\n', ...
            legs(k), c.center_mm, n, stopBatches, speedLow, clear, exit);
        break;
    end
    if ~found
        fprintf('%d %.0f %.0f no_certificate\n', legs(k), c.center_mm);
    end
end
fprintf('P4_LAG_RETIME %d of %d frozen route corners retain a conservative static certificate\n', ...
    accepted, numel(corners));
end
