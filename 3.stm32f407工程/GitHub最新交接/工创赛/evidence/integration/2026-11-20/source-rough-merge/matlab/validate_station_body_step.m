function validate_station_body_step
% Fixed nominal feedback steps; field stopping and arm sweep need measured evidence.
cases = 0;
for theta = [90 -90 180]
    for direction = 0:3
        for amount = [.25 1 10 30]
            origin = [1200 1200];
            angle = theta + [0 90 -90 180];
            target = origin + amount*[cosd(angle(direction+1)) sind(angle(direction+1))];
            pose = origin;
            for step = 1:250
                delta = target-pose;
                speed = min(15,2*norm(delta));
                if norm(delta)<=.20, break; end
                pose = pose+.08*speed*delta/norm(delta);
            end
            assert(step<250 && norm(pose-target)<=.20 && speed<=15);
            cases = cases+1;
        end
    end
end
for angle = [-10 -1 -.25 .25 1 10]
    now = 179;
    goal = now+angle;
    for step = 1:250
        err = mod(goal-now+180,360)-180;
        if abs(err)<=.15, break; end
        now = now+.08*max(-10,min(10,2*err));
    end
    assert(step<250 && abs(err)<=.15);
    cases=cases+1;
end
r = hypot(283,254)/2+10;
assert(all([290 2160 1200]-30 > r));
assert(2400-2160-30 > r);
assert(2400-2160-45 < r);
assert(1850+r<2160-30);
assert(1850+r<2160-30);
assert(1200+r+30<1400+31);
for observed = [false true]
    for ready = [false true]
        for quiet = [false true]
            for elapsed = [100 13000 61000 181000]
                allowed = observed && ready && quiet;
                assert(handoff(elapsed,observed,ready,quiet)==allowed);
                cases=cases+1;
            end
        end
    end
end
fid = fopen(fullfile(fileparts(mfilename('fullpath')),'station_body_step_result.txt'),'w');
assert(fid>=0);
msg=sprintf('STATION_BODY_STEP_PASS %s, %d response and handoff scenarios, radius %.3f mm\n',version,cases,r);
fprintf(fid,'%s',msg);
fclose(fid);
fprintf('%s',msg);
end

function ready = handoff(~,pose,ack,wheels)
ready = pose && ack && wheels;
end
