% Absolute slot timing for wheel feedback and independent status reads.
% Times represent FreeRTOS tick observations, not device sampling times.
checks = 0;
for slot = [0, 17, 2^32-4, 2^32-1]
    for arrival = -1:4
        for tx_done = 0:8
            for last = 0:8
                for decode_delay = 0:1
                    status_frame = [last,1,0,3,1];
                    f43 = [min(last,3)*ones(4,1),ones(4,1),(0:3)',8*ones(4,1),ones(4,1);
                           last,1,4,2,1];
                    [result,tx_ms,rx_ms,valid] = replay(slot, arrival, tx_done, ...
                                                       status_frame, decode_delay, false);
                    window_ok = arrival >= 0 && arrival <= 2 && ...
                                max([arrival,tx_done,last])+decode_delay <= 6;
                    assert(valid == window_ok && (result == 1) == window_ok);
                    if valid
                        assert(tx_ms == mod(slot+arrival,2^32));
                        assert(rx_ms == mod(slot+max([arrival,tx_done,last]),2^32));
                    else
                        assert(tx_ms == 0 && rx_ms == 0);
                    end
                    [result,tx_ms,rx_ms,valid] = replay(slot, arrival, tx_done, ...
                                                       f43, decode_delay, true);
                    assert(valid == window_ok && (result == 1) == window_ok);
                    if valid
                        assert(tx_ms == mod(slot+arrival,2^32));
                        assert(rx_ms == mod(slot+max([arrival,tx_done,last]),2^32));
                    else
                        assert(tx_ms == 0 && rx_ms == 0);
                    end
                    checks = checks + 2;
                end
            end
        end
    end

    % Frame columns are arrival, matching address/opcode, packet index,
    % frame DLC and payload/terminator validity.
    prefix = [3,1,0,8,1; 4,1,1,8,1; 4,1,2,8,1; 5,1,3,8,1];
    last = [6,1,4,2,1];
    foreign = [6,0,0,8,1];
    cases = {
        [prefix; foreign; foreign; last], true, 1;
        prefix, true, 10;
        [prefix; 6,1,3,2,1], true, 6;
        [prefix; 6,1,5,2,1], true, 6;
        [prefix; 6,1,4,3,1], true, 6;
        [prefix; 6,1,4,2,0], true, 6;
        [prefix; 7,1,4,2,1], true, 10;
        [foreign; foreign; 6,1,0,3,1], false, 1;
        [foreign; foreign; 6,1,0,4,1], false, 6;
        [foreign; foreign; 6,1,0,3,0], false, 6;
        [foreign; foreign; 7,1,0,3,1], false, 10;
    };
    for n = 1:size(cases,1)
        [result,~,rx_ms,valid] = replay(slot, 2, 3, cases{n,1}, 0, cases{n,2});
        assert(result == cases{n,3} && valid == (result == 1));
        assert(rx_ms == mod(slot+6,2^32)*valid);
        checks = checks + 1;
    end
end
fprintf('Navigation CAN 43/3A timing and assembly: %d checks passed\n', checks);

function [result,tx_ms,rx_ms,valid] = replay(slot,admission,tx_done,frames,decode_delay,is43)
result = 10;
tx_ms = 0;
rx_ms = 0;
valid = false;
observed_admission = mod(mod(slot+admission,2^32)-slot,2^32);
if observed_admission > 2 || tx_done > 6
    return;
end
bytes = 0;
packet = 0;
for i = 1:size(frames,1)
    observed = max([admission,tx_done,frames(i,1)]);
    if observed > 6
        return;
    end
    if ~frames(i,2)
        continue;
    end
    dlc = frames(i,4);
    if is43
        if dlc < 2 || dlc > 8
            continue;
        end
        bytes = bytes + dlc - (packet > 0);
        if frames(i,3) ~= packet || bytes > 30
            result = 6;
            return;
        end
        packet = packet + 1;
        if bytes < 30
            continue;
        end
    end
    if observed + decode_delay > 6
        return;
    end
    if (~is43 && dlc ~= 3) || ~frames(i,5)
        result = 6;
        return;
    end
    result = 1;
    valid = true;
    tx_ms = mod(slot+admission,2^32);
    rx_ms = mod(slot+observed,2^32);
    return;
end
end
