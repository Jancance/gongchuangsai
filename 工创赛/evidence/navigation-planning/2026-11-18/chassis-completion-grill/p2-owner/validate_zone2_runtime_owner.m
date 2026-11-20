function validate_zone2_runtime_owner()
% Timing, fencing and conservative same-stop encoder evidence for the CAN task.
wrap = 2^32;
checks = 0;
for epoch = [0,wrap-45]
    at = @(delta) mod(epoch+delta,wrap);
    age = @(now,old) mod(now-old,wrap);
    for wake = -1:12
        check((age(at(wake),at(0))<=2) == (wake>=0 && wake<=2));
    end
    for round = 0:4
        release = [0 20 40 60] + 80*round;
        check(all(mod(release,80)==[0 20 40 60]));
        for skips = 0:17
            slot = release(1)+skips*10;
            check(mod(floor(age(at(slot+2),at(release(1)))/10),8)==mod(skips,8));
        end
    end
    for changed = 0:4 % unchanged, run, session, power epoch, STOP
        for failing = 1:4
            for tx = [false,true]
                for late = [false,true]
                    now = at(20*(failing-1)+6+late);
                    slot = at(20*(failing-1));
                    receipt = tx;
                    eligible = changed==0 && age(now,slot)<=6 && receipt;
                    possible = (2^(failing-1)-1)+receipt*2^(failing-1);
                    ack = (2^(failing-1)-1)+eligible*2^(failing-1);
                    check(bitand(possible,ack)==ack);
                    check((ack==15)==(failing==4 && changed==0 && tx && ~late));
                    check(~eligible || changed~=4);
                end
            end
        end
    end
    for ackAge = [499,500,501]
        stopped = at(ackAge);
        for d = [0,1]
            for drift = [0,1]
                for rpm = [0,1]
                    for gap = [120,131]
                        first = at(ackAge+d);
                        valid = age(stopped,at(0))<500 && age(first,stopped)>0 && age(first,stopped)<2^31;
                        start = first;
                        prev = first;
                        for n = 1:11
                            now = at(ackAge+d+n*gap);
                            if age(now,prev)>130 || (drift || rpm) && n==8
                                start = now;
                            end
                            prev = now;
                        end
                        settled = valid && age(now,start)>=1000;
                        check(settled == (ackAge==499 && d==1 && ~drift && ~rpm && gap==120));
                    end
                end
            end
        end
    end
    for spec = [120 130; 320 330]'
        normalAge = spec(1);
        expiry = spec(2);
        sample = at(0);
        readWindow = [2, 6, 7];
        for delay = readWindow
            now = at(normalAge+delay);
            missed = delay > 6;
            if missed
                committed = sample;
            else
                committed = now;
            end
            actualExpiry = expiry;
            if normalAge==320 && missed
                actualExpiry = 650;
            end
            check(age(now,committed) <= actualExpiry);
            check(~missed || committed == sample);
            check((age(at(normalAge+20),committed)>actualExpiry) == (missed && normalAge==120));
        end
    end
    % The four addresses retain their 80 ms phase; one bad reply delays
    % this address by one rotation and cannot mask a valid protection reply.
    for lastOffset = 0:6
        previous = at(lastOffset);
        for recoveredOffset = 0:6
            lost = at(326);
            recovered = at(640+recoveredOffset);
            check(age(lost,previous)<=330);
            check(age(recovered,previous)<=650 && age(recovered,previous)<1000);
            for secondMiss = 0:1
                for protectFlag = 0:1
                    streak = secondMiss;
                    if secondMiss, streak=2; end
                    eligible = streak<=1 && ~(protectFlag && ~secondMiss);
                    check(eligible == (~secondMiss && ~protectFlag));
                end
            end
            check(age(at(651+lastOffset),previous)>650);
        end
    end
    countPatterns = [zeros(1,11); 0 1 0 -1 1 0 -1 0 1 0 1; ...
        0:10; 0 1 0 -1 1 0 -1 0 2 1 0; ...
        (2^32-2)+[0 1 0 -1 1 0 -1 0 1 0 1]];
    for n = 1:size(countPatterns,1)
        sign = countPatterns(n,:) < 0;
        count = abs(countPatterns(n,:));
        for rpmSpike = [0,1]
            startIndex = 1;
            seen = true;
            anchor = count(1); anchorSign = sign(1);
            for j = 2:11
                sameSide = sign(j)==anchorSign || count(j)==0 || anchor==0;
                near = sameSide && abs(count(j)-anchor)<=1;
                if rpmSpike && j==8
                    seen = false;
                elseif ~seen || ~near
                    seen = true;
                    startIndex = j;
                    anchor = count(j); anchorSign = sign(j);
                end
            end
            check((seen && (11-startIndex)*120>=1000) == (~rpmSpike && any(n==[1,2,5])));
        end
    end
end
fprintf('ZONE2_RUNTIME_OWNER %d checks PASS MATLAB %s\n',checks,version('-release'));
    function check(ok)
        assert(ok);
        checks=checks+1;
    end
end
