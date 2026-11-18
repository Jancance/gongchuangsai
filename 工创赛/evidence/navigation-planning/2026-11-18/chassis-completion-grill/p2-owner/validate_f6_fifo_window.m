% Replay FIFO0 capacity (three) and inclusive +6 ms completion.
checks = 0;
for slot = [0, 1000, 2^32-4, 2^32-1]
    for age = 0:8
        now = mod(slot + age, 2^32);
        for foreign = 0:2
            % 0 is a foreign frame, 1 the relevant, validated ACK.
            fifo = [zeros(1,foreign), 1];
            [elapsed,reads] = replay(slot,now,fifo,3,inf);
            if age <= 6
                assert(elapsed == age && reads == foreign+1);
            else
                assert(isnan(elapsed) && reads == 0);
            end
            checks = checks + 1;
        end
    end
    for age = 5:6
        [elapsed,reads] = replay(slot,mod(slot+age,2^32),[0 0 1],1,inf);
        assert(isnan(elapsed) && reads == 7-age);
        [elapsed,reads] = replay(slot,mod(slot+age,2^32),[0 0 1],3,inf);
        assert(elapsed == age && reads == 3);
        checks = checks + 2;
    end
    [elapsed,reads] = replay(slot,mod(slot+6,2^32),[0 1],3,1);
    assert(isnan(elapsed) && reads == 1);
    [elapsed,reads] = replay(slot,mod(slot+6,2^32),[1],3,1);
    assert(isnan(elapsed) && reads == 1);
    [elapsed,reads] = replay(slot,mod(slot+6,2^32),[0 0 0 1],3,inf);
    assert(isnan(elapsed) && reads == 3);
    checks = checks + 3;
end
fprintf('F6 FIFO window model: %d checks passed\n', checks);

function [elapsed,reads] = replay(slot,now,fifo,budget,preempt_after)
elapsed = NaN;
reads = 0;
for age = mod(now-slot,2^32):6
    cursor = age;
    for frame = 1:budget
        if isempty(fifo) || cursor > 6
            break;
        end
        current = fifo(1);
        fifo(1) = [];
        reads = reads + 1;
        if reads == preempt_after
            cursor = cursor + 1;
        end
        if current && cursor <= 6
            elapsed = cursor;
            return;
        end
    end
end
end
