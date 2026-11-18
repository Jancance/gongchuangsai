% Only the bounded wheel reader requires the canonical five CAN fragments.
% Delivery is decided after decode/diagnostics at a single FreeRTOS tick.
checks = 0;
for n = 4:6
    vectors = dec2base(0:7^n-1,7,n)-'0'+2;
    candidates = vectors(sum(vectors,2)==29+n,:);
    accepted = 0;
    for j = 1:size(candidates,1)
        frames = candidates(j,:);
        accepted = accepted + canonical(frames,0:n-1);
        checks = checks + 1;
    end
    assert(accepted == (n==5));
    checks = checks + 1;
end
assert(~canonical([8 8 8 7 2 2],0:5));
assert(~canonical([8 8 8 8 2],[0 1 2 2 4]));
assert(~canonical([8 8 8 8 3],0:4));
assert(canonical([8 8 8 8 2],0:4));
checks = checks + 4;

for slot = [0, 100, 2^32-6, 2^32-1]
    for rx = 0:7
        for decode = 0:2
            for diagnostics = 0:2
                for preempt = 0:2
                    for well_formed = [false,true]
                        receive = mod(slot+rx,2^32);
                        tested = mod(slot+rx+decode+diagnostics+preempt,2^32);
                        [ok, reported] = deliver(slot, receive, tested, well_formed);
                        expected = well_formed && rx+decode+diagnostics+preempt <= 6;
                        assert(ok == expected);
                        assert(reported == receive*expected);
                        checks = checks+2;
                    end
                end
            end
        end
    end
end
fprintf('Bounded CAN canonical fragments and final delivery: %d checks passed\n', checks);

function valid = canonical(dlc,sequence)
valid = numel(dlc) == 5 && all(sequence == 0:4) && ...
        all(dlc(1:4) == 8) && dlc(5) == 2;
end

function [valid,reported] = deliver(slot,rx,delivery,well_formed)
valid = well_formed && mod(rx-slot,2^32) <= 6 && mod(delivery-slot,2^32) <= 6;
reported = rx*valid;
end
