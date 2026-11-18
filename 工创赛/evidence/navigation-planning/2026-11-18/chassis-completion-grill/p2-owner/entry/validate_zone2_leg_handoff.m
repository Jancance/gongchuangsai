function validate_zone2_leg_handoff()
% Same-map next-leg eligibility and heading wrap with ordinary pose errors.
nominal = [90 90;90 90;90 -90;-90 180;180 90;90 -90;-90 180;180 90];
checked = 0;
for leg = 1:8
    start = nominal(leg,1);
    goal = nominal(leg,2);
    planned = angle(goal-start);
    for delta = [-1.01 -1 -0.7 -0.01 0 0.01 0.7 1 1.01]
        for offset = [-360 0 360]
            measured = start+delta+offset;
            error = angle(measured-start);
            allowed = abs(error)<=1;
            assert(allowed==(abs(delta)<=1));
            assert(ismember(planned,[-180 -90 0 90]));
            if allowed
                assert(abs(angle(measured-goal) - angle(delta-planned)) < 1e-8);
            end
            checked = checked+1;
        end
    end
end
for base = [100 2^32-2]
    for request = [0 1]
        for stopped = [0 1]
            for priorSession = [2 3]
                for power = [45 46]
                    proof = struct('run',7,'session',priorSession,'power',power, ...
                        'confirmed',stopped,'at',mod(base+3,2^32));
                    accepted = request && proof.confirmed && proof.run==7 && ...
                        proof.session==2 && proof.power==45;
                    assert(accepted==(request && stopped && priorSession==2 && power==45));
                    if accepted
                        nextSession=priorSession+1;
                        assert(nextSession~=proof.session);
                        assert(mod(proof.at-base,2^32)==3);
                    end
                    checked = checked+1;
                end
            end
        end
    end
end
fprintf('ZONE2 next-leg nominal heading/session %d checks PASS MATLAB %s\n',checked,version('-release'));
end

function d = angle(d)
d = mod(d+180,360)-180;
end
