function validate_zone2_external_stop
% State and timing precondition for the zero-only front end.
% Front: 0 ZERO, 1 STOP, 2 SHADOW_DONE, 3 FAILED.
% Backend: 0 RUN, 1 STOP, 2 SETTLE, 3 DONE, 4 FAULT.
checks = 0;
for anchor = [1000 2^32-80]
    for firstBackend = 1:3
        for recoverAt = [0 240 1280 2490 2500 inf]
            for missing = 2:3
                phase = 0;
                stopStart = NaN;
                actions = [];
                for offset = 0:10:2600
                    now = mod(anchor+offset,2^32);
                    backend = max(firstBackend,1+(offset>=100)+(offset>=230));
                    evidence = offset>=recoverAt | (1:3)~=missing;
                    [phase,stopStart,action] = step(phase,stopStart,now,backend,true,...
                        true,evidence(1),evidence(2),evidence(3));
                    actions(end+1) = action; %#ok<AGROW>
                    assert(phase~=0 && mod(now-stopStart,2^32)==offset);
                    checks = checks+1;
                end
                assert(~any(actions==1));
                assert((phase==2 && sum(actions==2)==1) == (recoverAt<2500));
                assert((phase==3 && sum(actions==3)==1) == (recoverAt>=2500));
                checks = checks+3;
            end
        end
    end
end
for backend = 1:4
    for sameIdentity = [true false]
        for sameMap = [true false]
            for committedZero = [true false]
                [phase,~,action] = step(0,NaN,250,backend,sameIdentity,...
                    committedZero,sameMap,true,true);
                assert((phase==3)==(~sameIdentity || ~sameMap || backend==4));
                assert((action==2)==(sameIdentity && sameMap && committedZero && backend==3));
                checks = checks+2;
            end
        end
    end
end
fprintf('ZONE2 externally requested zero STOP %d checks PASS MATLAB %s\n',checks,version('-release'));
end

function [phase,start,action] = step(phase,start,now,backend,sameIdentity,committedZero,map,wheels,sources)
action = 0;
if phase>=2, return; end
if ~sameIdentity || ~map || backend==4
    phase=3; action=3; return;
end
if phase==0 && backend>=1
    phase=1; start=now;
end
if phase==1
    if mod(now-start,2^32)>=2500
        phase=3; action=3;
    elseif backend==3 && committedZero && map && wheels && sources
        phase=2; action=2;
    end
elseif backend==0
    action=1;
end
end
