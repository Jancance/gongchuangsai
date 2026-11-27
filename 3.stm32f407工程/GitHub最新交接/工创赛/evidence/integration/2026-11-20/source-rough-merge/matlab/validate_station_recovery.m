function validate_station_recovery
% Candidate position window and feedback-watchdog witnesses.
half = [254 283]/2;
support = half*cosd(2) + fliplr(half)*sind(2) + 5;
center = [1200 300];
deviation = [55 5];
lo = center-deviation-support;
hi = center+deviation+support;
cell2 = [960 0;1440 480];
margin = min([lo-cell2(1,:) cell2(2,:)-hi]);
assert(margin>0);
assert(all(abs([1254 303]-center)<=deviation));
assert(~all(abs([1256 303]-center)<=deviation));
assert(~all(abs([1200 240]-center)<=deviation));
assert(~all(abs([1200 306]-center)<=deviation));

t = 0:.1:65;
% A 0.2 mm/s approach exceeds the 0.5 mm threshold within 3 s.
[attempts,fault,finished] = supervise(t,13.2-.2*t);
assert(attempts==0 && isnan(fault) && finished==65);
[attempts,fault,finished] = supervise(t,5+.08*sin(t*20));
assert(attempts==1 && fault>6 && isnan(finished));
[attempts,fault,finished] = supervise(t,5+.2*t);
assert(attempts==1 && fault>6 && isnan(finished));
afterRecovery = max(.2,1.1-(t-3.5)*.2);
afterRecovery(t<3.5) = 1.1;
[attempts,fault,finished] = supervise(t,afterRecovery);
assert(attempts==1 && isnan(fault) && finished>3.5);
assert(wait_tick(100,0,false)==0);
assert(wait_tick(100,0,true)==1);
assert(wait_tick(329,9,false)==2);
fprintf('STATION_RECOVERY_PASS cell-contained margin %.3f mm, 0.2 mm/s to endpoint at 65 s, one zero-and-quiet retry\n',margin);
end

function state=wait_tick(now,quietMs,valid)
if valid
    state = 1;
elseif now-quietMs<320
    state = 0;
else
    state = 2;
end
end

function [attempts,fault,finished]=supervise(t,error)
attempts=0;
fault=NaN;
finished=NaN;
anchor=error(1);
last=0;
resumeAt=-1;
for i=1:numel(t)
    if resumeAt>=0
        if t(i)<resumeAt
            continue;
        end
        last=t(i);
        anchor=error(i);
        resumeAt=-1;
    end
    if error(i)<=.2+1e-7
        finished=t(i);
        return
    end
    if anchor-error(i)>=.5
        anchor=error(i);
        last=t(i);
    elseif t(i)-last>=3
        if attempts
            fault=t(i);
            return
        end
        attempts=1;
        resumeAt=t(i)+.5;
    end
end
end
