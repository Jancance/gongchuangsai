function validate_speed_trial160
vPeak = 1600;
dt = 0.080;
jerkTime = 3*dt;
accelTime = 5*dt;
holdTime = 2*dt;
aPeak = vPeak/(jerkTime+accelTime);
jerk = aPeak/jerkTime;
phases = [3 5 3 2 3 5 3];
phaseJerk = jerk*[1 0 -1 0 -1 0 1];
state = [0 0 0];
samples = state;
for n = 1:numel(phases)
    for k = 1:phases(n)
        j = phaseJerk(n);
        x = state(1); v = state(2); a = state(3);
        state = [x+v*dt+a*dt^2/2+j*dt^3/6, v+a*dt+j*dt^2/2, a+j*dt];
        samples(end+1,:) = state; %#ok<AGROW>
    end
end
assert(all(diff(samples(:,1)) >= -1e-8));
assert(all(samples(:,2) >= -1e-8 & samples(:,2) <= vPeak+1e-8));
assert(sum(abs(samples(:,2)-vPeak)<1e-8)==3);
assert(norm(samples(end,2:3))<1e-8 && abs(state(1)-1664)<1e-8);
axis = [1 0; -1 0; 0 1; 0 -1];
wheel = [1 1; 1 -1; 1 -1; 1 1];
factor = pi*77/60;
names = {'forward','reverse','left','right'};
for d = 1:4
    rpm = samples(:,2)*(wheel*axis(d,:)')'/factor;
    peak = max(abs(rpm(:)));
    assert(peak>380 && round(peak)==397 && round(10*peak)>3000);
    assert(all(abs(rpm(1,:))+abs(rpm(end,:))<1e-8));
    actual = rpm*wheel*factor/4;
    desired = samples(:,2)*axis(d,:);
    assert(max(abs(actual(:)-desired(:)))<1e-8);
    fprintf('%s: %.3f rpm, raw integer %d, raw deci %d (reject)\n',names{d},peak,round(peak),round(peak*10));
end
for d = 1:4
    t = (0:sum(phases)*4+7)'*0.020;
    x = zeros(size(t));
    for k = 1:numel(t)
        n = min(floor((k-1)/4)+1,size(samples,1));
        u = t(k)-(n-1)*dt;
        if n==size(samples,1)
            x(k) = samples(n,1);
        else
            phase = find(n<=cumsum(phases),1);
            x(k) = samples(n,1)+samples(n,2)*u+samples(n,3)*u^2/2+phaseJerk(phase)*u^3/6;
        end
    end
    r = zeros(1,4);
    command = zeros(numel(t)-4,4);
    sent = zeros(size(command,1),1);
    for k = 1:size(command,1)
        wheelIdx = mod(k-1,4)+1;
        signs = wheel(wheelIdx,:)*axis(d,:)';
        r(wheelIdx) = round(signs*(x(k+4)-x(k))/dt/factor);
        command(k,:) = r;
        sent(k) = r(wheelIdx);
    end
    assert(all(command(end,:)==0));
    if d == 1
        fprintf('Forward integer command matrix, rows by batch:\n');
        disp(reshape(sent,4,[])');
    end
    estimate = command*wheel*factor/4;
    along = estimate*axis(d,:)';
    across = estimate*[-axis(d,2);axis(d,1)];
    assert(max(abs(command(:)))==397);
    assert(abs(sum(along)*0.020-1664)<4);
    assert(max(abs(cumsum(across)*0.020))<10);
    yawRate = command*[-1;1;-1;1]*factor/(4*209);
    yawEdge = cumsum(yawRate)*0.020;
    yaw = yawEdge-yawRate*0.010;
    world = [cos(yaw).*estimate(:,1)-sin(yaw).*estimate(:,2), ...
             sin(yaw).*estimate(:,1)+cos(yaw).*estimate(:,2)];
    worldAlong = world*axis(d,:)';
    worldAcross = world*[-axis(d,2);axis(d,1)];
    fprintf('%s staggered free kinematics: peak yaw %.3f deg stop %.1f mm max cross %.1f mm\n', ...
        names{d},max(abs(yawEdge))*180/pi,sum(worldAlong)*0.020,max(abs(cumsum(worldAcross)*0.020)));
    fprintf('%s staggered integer model: peak %.1f mm/s nominal stop %.1f mm peak open-loop cross %.1f mm\n', ...
        names{d},max(along),sum(along)*0.020,max(abs(cumsum(across)*0.020)));
end
reserve = 1920-state(1);
assert(reserve>vPeak*dt);
fprintf('PASS: peak %.0f mm/s, A %.0f mm/s^2, J %.1f mm/s^3, nominal stop %.0f mm, reserve %.0f mm, duration %.2f s\n', ...
vPeak,aPeak,jerk,state(1),reserve,sum(phases)*dt);
end
