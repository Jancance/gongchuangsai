function validate_zone2_wheel_retry()
% 80 ms wheel schedule, with no extra transaction on a failed read.
assert(strcmp(version('-release'),'2020a'));
normalMax=0; recoveryMax=0; poseMax=0; cleanMax=0; cases=0; rejected=0;
for phase=12:23
    for delays=0:15
        for second=0:5
            [n,r,p,c,stop]=replay(phase,delays,second);
            if second==0
                assert(~stop);
                normalMax=max(normalMax,n);
                recoveryMax=max(recoveryMax,r);
                poseMax=max(poseMax,p);
                cleanMax=max(cleanMax,c);
            elseif second<=4
                assert(stop);
                rejected=rejected+1;
            end
            cases=cases+1;
        end
    end
end
assert(normalMax<=130 && recoveryMax<=150 && poseMax<=160 && cleanMax<=130);
fprintf('ZONE2 WHEEL RETRY PASS MATLAB %s: %d scenarios, %d sustained fault cases, owner normal/recovery %d/%d ms, common %d ms, post-recovery %d ms\n', ...
    version('-release'),cases,rejected,normalMax,recoveryMax,poseMax,cleanMax);
end

function [normalMax,recoveryMax,poseMax,cleanMax,stop]=replay(first,delays,second)
delays=6*double(bitget(uint8(delays),1:4));
latest=zeros(1,4);
previous=zeros(1,4);
schedule=[10 30 50];
cursor=1; refreshed=false(1,4); recovery=false;
normalMax=0; recoveryMax=0; poseMax=0; cleanMax=0; stop=false;
trial=0;
for slot=0:10:960
    if ismember(mod(slot,80),schedule)
        trial=trial+1;
        if trial==first || (second>0 && trial==first+second)
            recovery=all(refreshed);
            refreshed(:)=false;
            if ~recovery
                cursor=mod(cursor,4)+1;
            end
        else
            % Consumer can run before commit at maximum transaction delay.
            if all(previous) && trial>=first && recovery
                recoveryMax=max(recoveryMax,slot+6-min(latest));
            end
            previous(cursor)=latest(cursor);
            latest(cursor)=slot+delays(cursor);
            refreshed(cursor)=true;
            if all(refreshed)
                recovery=false;
            end
            cursor=mod(cursor,4)+1;
        end
    end
    if trial<8, continue, end
    now=slot+6; % first later 10-ms operation leaves at most 6 ms transaction delay
    age=now-min(latest);
    common=min(latest); % latest interpolable timestamp if all histories bracket it
    if second==0
        assert(all(previous<=common)); % no new oldest instant can be extrapolated
    end
    poseMax=max(poseMax,now-common);
    if recovery
        recoveryMax=max(recoveryMax,age);
        stop=stop || age>150;
    else
        normalMax=max(normalMax,age);
        stop=stop || age>130;
        if trial>=first+4 && second==0
            cleanMax=max(cleanMax,age);
        end
    end
end
end
