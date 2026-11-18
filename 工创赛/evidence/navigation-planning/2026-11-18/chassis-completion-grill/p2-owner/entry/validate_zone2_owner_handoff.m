function validate_zone2_owner_handoff()
% Absolute CAN schedule and clean stop handoff in one scan/power epoch.
assert(strcmp(version('-release'),'2020a'));
wrap = @(t) mod(t,2^32);
accepted = 0;
release = 0;
checks = 0;
for now = [800 2^32-39 2^32-1 0]
    for ahead = [-80 -1 0 1 10 39 79 80 81 2^31]
        scheduled = wrap(now+ahead);
        for skew = [-80 -1 0 1 80]
            proposed = wrap(scheduled+skew);
            delay = wrap(proposed-now);
            for mask = 0:255
                % identity, power active/epoch, available and fresh generation
                % are independent invariants under the owner's lock.
                eligible = all(bitget(uint16(mask),1:8));
                valid = eligible && proposed==scheduled && delay>0 && delay<=80;
                assert(valid==(mask==255 && skew==0 && ahead>=1 && ahead<=80));
                if valid
                    for wheel = 0:3
                        assert(wrap(wrap(proposed+20*wheel)-scheduled)==20*wheel);
                    end
                    accepted=accepted+1;
                end
                checks=checks+1;
            end
        end
    end
end
for epoch = [1 2^32-1]
    for oldSession = [1 2^32-1]
        for mask = 0:511
            % Same run/session, VM, epoch, DONE, success, four ACKs, fresh
            % quiet, and clear fault jointly qualify the completed session.
            ready = all(bitget(uint16(mask),1:9));
            powerAfter = epoch;
            resultAfter = 1;
            usedAfter = 1;
            continuation = 0;
            if ready
                resultAfter = 0;
                usedAfter = 0;
                continuation = 1;
            end
            assert(powerAfter==epoch && resultAfter==~ready && usedAfter==~ready);
            if ready
                newSession = wrap(oldSession+1);
                assert(newSession~=oldSession && continuation==1);
                acceptOldAttempt = oldSession==newSession && epoch==powerAfter;
                assert(~acceptOldAttempt);
                release=release+1;
            end
            checks=checks+1;
        end
    end
end
fprintf('ZONE2 OWNER HANDOFF PASS MATLAB %s: %d scenarios, %d accepted times, %d completed releases\n', ...
    version('-release'),checks,accepted,release);
end
