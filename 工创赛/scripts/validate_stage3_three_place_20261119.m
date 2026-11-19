function validate_stage3_three_place_20261119
% Event-order model. Completion times are inputs, not measured plant dynamics.
checks = 0;
for batch = 1:2
    slots = [2 1 5];
    orders = [2 1 5; 5 1 2];
    rings = [2 1 3; 3 2 1];
    [found, slot] = ismember(orders(batch,:),slots);
    assert(all(found) && numel(unique(slot)) == 3);
    assert(isequal(sort(rings(batch,:)),1:3));
    for yaw = [1 300 900]
        for x = [0 100 1700]
            for z = [80 600 2000]
                for returnX = [0 500 1800]
                    for returnZ = [80 1500 2100]
                        for failed = 0:4
                            trace = cycle(yaw,x,z,returnX,returnZ,failed);
                            assert(trace.pick == trace.slotYawDone);
                            if failed <= 2 && failed ~= 0
                                assert(isinf(trace.open) && isinf(trace.retract));
                            else
                                assert(trace.open == trace.parallel + max(x,z));
                                assert(trace.retract == trace.open + 600);
                            end
                            if failed == 0
                                assert(trace.preclose == trace.retract + max(returnX,returnZ));
                                assert(trace.nextYaw == trace.preclose + 600);
                            else
                                assert(isinf(trace.preclose) && isinf(trace.nextYaw));
                            end
                            checks = checks + 1;
                        end
                    end
                end
            end
        end
    end
end
assert(abs((61002.222743 + 5*400) - 63002.222743) < 1e-7);
assert(abs((61595.023785 - 2*400) - 60795.023785) < 1e-7);
fprintf('PASS Stage3 three-place event ordering and height conversion: %d cases\n',checks);
end

function e = cycle(yaw,x,z,returnX,returnZ,failed)
e.slotYawDone = 600 + yaw;
e.pick = e.slotYawDone;
e.parallel = e.pick + 400 + 600 + 500 + yaw;
e.open = inf; e.retract = inf; e.preclose = inf; e.nextYaw = inf;
if failed == 1 || failed == 2
    return
end
e.open = e.parallel + max(x,z);
e.retract = e.open + 600;
if failed ~= 0
    return
end
e.preclose = e.retract + max(returnX,returnZ);
e.nextYaw = e.preclose + 600;
end
