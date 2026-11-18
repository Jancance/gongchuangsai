function validate_zone2_runtime_contract()
% E: clock and source contract for the PC13/radar/CAN/HWT join.
% Integer clock operations emulate 32-bit ms across tick rollover.
checks = 0;
T = 2^32;
for base = [1000 T-90]
    tick = @(delta) mod(base+delta,T);
    for button = 0:1
        for zone = 1:2
            for radar = 0:1
                check((button && zone==2 && radar) == (button*radar*double(zone==2)==1));
            end
        end
    end
    for map = 0:1
        for stowed = 0:1
            for active = 0:1
                for power = 0:1
                    canPrime = map && stowed && ~active;
                    check(canPrime == (map+stowed+(~active)==3));
                    canBase = canPrime && power;
                    check(canBase == (map+stowed+(~active)+power==4));
                end
            end
        end
    end
    for delay = [-1 0 130 131 160 161]
        check(fresh(tick(0),tick(-delay),130) == (delay>=0 && delay<=130));
        check(fresh(tick(0),tick(-delay),160) == (delay>=0 && delay<=160));
    end
    for delay = [-1 0 99 100]
        check(fresh(tick(0),tick(-delay),99) == (delay>=0 && delay<=99));
    end
    % A common map identity is checked independently of the radar scan generation.
    for mapVersion = [1 2 41]
        for radarGeneration = [0 2 41]
            world = mapVersion==2 && radarGeneration~=0;
            check(world == ismember([mapVersion radarGeneration],[2 2;2 41],'rows'));
            for sameRun = 0:1
                for samePower = 0:1
                    for sameLeg = 0:1
                        for allAck = 0:1
                            for cancelled = 0:1
                                allowed = world && sameRun && samePower && sameLeg && allAck && ~cancelled;
                                check(allowed == (world+sameRun+samePower+sameLeg+allAck+(~cancelled)==6));
                            end
                        end
                    end
                end
            end
        end
    end
    % Feedback fusion preserves the common sample timestamp. Successive wheel
    % observations are raw RPM samples, the interpolation is a model estimate.
    samples = tick([-227 -113 -12]);
    rates = [4.0 8.0 16.0];
    for dt = [-228 -227 -150 -113 -65 -12 -11]
        [value,ok] = aligned(samples,rates,tick(dt));
        check(ok == (dt>=-227 && dt<=-12));
        if ok
            check(abs(value-interp1([-227 -113 -12],rates,dt))<1e-12);
        end
    end
    ack = tick(25);
    for wall = [999 1000 1001]
        for poseDelta = [0 1 50]
            for movingCounts = 0:1
                for rate = [0 1 1.001]
                    now = tick(25+wall);
                    common = tick(25+wall-poseDelta);
                    wheels = tick(25+wall-[125 80 50 6]);
                    ok = confirm(now,ack,common,wheels,tick(25+wall-10), ...
                        tick(25+wall-15),movingCounts,rate);
                    check(ok == (wall>=1000 && ~movingCounts && rate<=1));
                end
            end
        end
    end
    check(~confirm(tick(1025),ack,ack,tick([899 901 930 999]),tick(1021),tick(1022),0,0));
    check(~confirm(tick(1025),ack,tick(924),tick([899 901 930 999]),ack,tick(1022),0,0));
    check(~confirm(tick(1025),ack,tick(924),tick([899 901 930 999]),tick(1021),ack,0,0));
    check(~confirm(tick(1025),ack,tick(924),tick([894 901 930 999]),tick(1021),tick(1022),0,0));
end
% Preserve the stricter footprint condition on the eighth arrival. Adjacent
% turns can be complete at the modeled stationary point without ending a leg.
for x = [145 150 155 166]
    for y = [145 150 155]
        for yaw = [89 89.75 90 90.25 91]
            r = deg2rad(abs(yaw-90)+1);
            extent = 0.5*[254*cos(r)+283*sin(r),283*cos(r)+254*sin(r)]+5;
            inside = all([x y]>=extent & [x y]<=300-extent);
            check(inside == (x<=155 && y==150 && abs(yaw-90)<=0.25));
        end
    end
end
fprintf('ZONE2 PC13/CAN/HWT contract %d checks PASS MATLAB %s\n',checks,version('-release'));
    function check(ok)
        assert(ok);
        checks = checks + 1;
    end
end

function ok = fresh(now,stamp,limit)
ok = mod(now-stamp,2^32)<=limit;
end

function [value,ok] = aligned(stamps,rates,sample)
offset = mod(stamps-stamps(1),2^32);
selected = mod(sample-stamps(1),2^32);
ok = selected <= offset(end);
value = 0;
if ok
    value = interp1(offset,rates,selected);
end
end

function yes = confirm(now,ack,pose,wheels,angle,rateStamp,movingCounts,rate)
samples = [pose,wheels,angle,rateStamp];
delta = mod(samples-ack,2^32);
yes = all(delta>0 & delta<2^31) && fresh(now,ack+1000,2000) && ...
    fresh(now,pose,160) && all(fresh(now,wheels,130)) && ...
    fresh(now,angle,99) && fresh(now,rateStamp,99) && ...
    ~movingCounts && abs(rate)<=1;
end
