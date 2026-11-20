function validate_gripper_tim4()
    pclk1 = 42000000;
    timerHz = 2 * pclk1;
    prescaler = timerHz / 1000000 - 1;
    period = 20000;
    assert(timerHz / (prescaler + 1) == 1000000);

    encode = @(angle) 500 + floor(angle * 2000 / 270 + 0.5);
    assert(isequal(encode([0 140 180 270]), [500 1537 1833 2500]));
    valid = @(angle) (angle >= 0 && angle <= 270);
    assert(all(~arrayfun(valid, [-1 271 NaN Inf -Inf])));

    radar = [500 2500];
    idle = struct('ready', false, 'enabled', false, 'active', 0, ...
        'pending', 0, 'update', false, 'radar', radar, 'count', 0);
    assert(~apply(init(idle, false), 180).enabled);
    phase = 0:(period - 1);
    for tick = phase
        s = idle;
        s.count = tick;
        s.update = true; % stale UIF from radar or an earlier request
        s = apply(init(s, true), 180);
        assert(s.enabled && ~s.update && s.active == 0 && s.count == tick);
        assert(isequal(s.radar, radar));
        s = nextFrame(s);
        assert(s.enabled && s.update && s.active == 1833 && s.count == 0);
        assert(nnz(phase < s.active) == 1833);

        s.count = tick;
        rejected = apply(s, NaN);
        assert(isequal(rejected, s));
        s = apply(s, 140);
        assert(s.count == tick && s.active == 1833 && ~s.update);
        s = nextFrame(s);
        assert(s.enabled && s.update && s.active == 1537);

        s.count = tick;
        s = stop(s);
        assert(~s.enabled && s.active == 0 && s.pending == 0);
        s = apply(s, 180); % restart in the same frame
        assert(s.enabled && s.active == 0 && ~s.update && s.count == tick);
        s = nextFrame(s);
        assert(s.enabled && s.update && s.active == 1833);
        assert(isequal(s.radar, radar));
    end
    % UIF may clear just after the CCR4 write is transferred. That can add a
    % frame before success is reported, but cannot report a transfer early.
    s = nextFrame(apply(init(idle, true), 180));
    s.update = false;
    assert(s.active == 1833 && ~s.update);
    s = nextFrame(s);
    assert(s.update);
    fprintf('%s PASS: PSC=%d, ARR=%d, %d phases; CH4 start, update, stop/restart; radar CH1/2 fixed\n', ...
        version, prescaler, period - 1, numel(phase));
end

function s = init(s, radarReady)
    s = stop(s);
    s.ready = radarReady;
end

function s = apply(s, angle)
    if s.ready && (angle >= 0 && angle <= 270)
        s.enabled = true;
        s.pending = 500 + floor(angle * 2000 / 270 + 0.5);
        s.update = false;
    end
end

function s = nextFrame(s)
    s.count = 0;
    s.active = s.pending;
    s.update = true;
end

function s = stop(s)
    s.enabled = false;
    s.active = 0;
    s.pending = 0;
end
