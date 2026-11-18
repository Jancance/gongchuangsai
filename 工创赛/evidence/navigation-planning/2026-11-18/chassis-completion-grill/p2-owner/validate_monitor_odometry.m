function validate_monitor_odometry
checks = 0;
wrap = 2^32;
% before: session, power generation, VM active, real backend
before = [11,21,1,1];
after = [11,21,1,1,1,1]; % plus current session active and healthy
for startMs = [200,wrap-1]
    for span = [0,1,30]
        replyMs = mod(startMs+span,wrap);
        for lag = [0,1,30]
            now = mod(replyMs+lag,wrap);
            fresh = [2,1,1,startMs,replyMs,1,16842754]; % address, valid, result, request/reply, sign, counts
            check(eligible(before,after,2,fresh,now,false,0));
            check(eligible(before,after,2,fresh,now,true,mod(replyMs-1,wrap)));
            check(~eligible(before,after,2,fresh,now,true,replyMs));
            check(~eligible(before,after,2,fresh,now,true,mod(replyMs+1,wrap)));
            check(~eligible(before,after,2,fresh,mod(replyMs-1,wrap),false,0));
            bad = fresh; bad(4) = mod(replyMs+1,wrap);
            check(~eligible(before,after,2,bad,now,false,0));
            for i = 1:numel(after)
                changed = after; changed(i) = changed(i)-1;
                check(~eligible(before,changed,2,fresh,now,false,0));
            end
            for i = 3:numel(before)
                blocked = before; blocked(i) = 0;
                check(~eligible(blocked,after,2,fresh,now,false,0));
            end
            for i = 1:3
                malformed = fresh; malformed(i) = malformed(i)-1;
                check(~eligible(before,after,2,malformed,now,false,0));
            end
        end
    end
end
fprintf('validate_monitor_odometry checks=%u PASS MATLAB %s\n',checks,version('-release'));
    function check(ok)
        assert(ok);
        checks = checks+1;
    end
end

function allowed = eligible(before,after,address,reply,now,hasSample,sampleMs)
allowed = all(before(1:2)==after(1:2)) && all(before(3:4)) && all(after(3:6)) && ...
    reply(1)==address && reply(2)==1 && reply(3)==1 && ...
    delta(reply(5),reply(4))>=0 && delta(now,reply(5))>=0 && ...
    (~hasSample || delta(reply(5),sampleMs)>0);
end

function elapsed = delta(a,b)
elapsed = mod(a-b+2^31,2^32)-2^31;
end
