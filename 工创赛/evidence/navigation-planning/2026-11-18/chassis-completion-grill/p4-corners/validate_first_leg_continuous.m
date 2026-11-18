function result=validate_first_leg_continuous(p4file, p3file, certificate, output)
% Replay the whole frozen first leg using a single clock and wheel history.
p4 = jsondecode(fileread(p4file));
p3 = jsondecode(fileread(p3file));
cert = jsondecode(fileread(certificate));
L = cert.assumptions;
H = L.H;
corners = p4.routes(1).corners;
points = p4.routes(1).points_mm;
assert(numel(corners)==2 && all([cert.rows(1:2).accepted]));
assert(isequal(points,[150 150;240 240;1200 240;1200 300]));
assert(isequal(points,p3.legs(1).points_mm) && isequal(p3.body_mm(:),2*L.half(:)));
P = {corners(1).control_points_mm, corners(2).control_points_mm};
turnTime = [cert.rows(1:2).batches]*H;
join = arrayfun(@(i) norm(5*(P{i}(2,:)-P{i}(1,:)))/turnTime(i),1:2);
a = [points(1,:); P{1}(end,:); P{2}(end,:)];
b = [P{1}(1,:); P{2}(1,:); points(end,:)];
lengths = vecnorm(b-a,2,2);
ramps = [1.6 .8; 2.4 2.4; .8 1.6];
v = [0 join 0];
profiles = zeros(3,6);
for i=1:3
    direction = (b(i,:)-a(i,:))/lengths(i);
    if i>1
        assert(norm(5*(P{i-1}(end,:)-P{i-1}(end-1,:))/turnTime(i-1)-v(i)*direction)<1e-8);
    end
    if i<3
        assert(norm(5*(P{i}(2,:)-P{i}(1,:))/turnTime(i)-v(i+1)*direction)<1e-8);
        assert(norm(diff(P{i}(1:3,:),2))<1e-8 && norm(diff(P{i}(end-2:end,:),2))<1e-8);
    end
    profiles(i,:) = straight(lengths(i),v(i:i+1),ramps(i,:),H);
end
edges = [0 cumsum([sum(profiles(1,1:3)),turnTime(1),sum(profiles(2,1:3)),...
    turnTime(2),sum(profiles(3,1:3))])];
[slot,wheel] = ndgrid((-2:round(edges(end)/H)+2)*H,0:3);
slot = slot(:)+wheel(:)*.020;
wheel = wheel(:)+1;
R = [0 -1; 1 0];
W = [1 1;1 -1;1 -1;1 1];
command = round(10*sum(((position(slot+H)-position(slot))*R).*W(wheel,:),2)/H/L.K)/10;
assert(max(abs(command))<=300);
for i=1:4
    q=command(wheel==i);
    rate=(abs(diff(q))/(H-L.late));
    jerk=(abs(diff(q,2))/(H-L.late)+abs(diff(q(1:end-1)))*2*L.late/(H-L.late)^2)/(H-L.late);
    assert(max(rate)<=250 && max(jerk)<=1400);
end
result = struct('version',version,'join_mm_s',join,'straight_profiles',profiles,...
    'edges_s',edges,'slot_ms',round(slot*1000),'wheel',wheel,'command_rpm',command,...
    'startup_tracking_mm',1,'startup_tracking_until_s',.64,'normal_patterns',[]);
for pattern=0:1
    applied=slot+pattern*L.late*mod(round((slot-(wheel-1)*.020)/H)+wheel,2);
    [applied,order]=sort(applied);
    held=zeros(1,4); pose=[points(1,:) pi/2]; minGap=inf; minCorner=[inf inf];
    worstSweepTime=0; uniformGap=inf; independentGap=inf;
    latencyPosition=0; latencyYaw=0;
    maxOffset=0; maxYaw=0;
    for k=1:numel(order)-1
        at=order(k);
        jump=abs(held(wheel(at))-command(at)*L.K);
        held(wheel(at))=command(at)*L.K;
        dt=applied(k+1)-applied(k);
        body=held*W/4;
        dYaw=(held*[-1;1;-1;1]/(4*L.lever))*dt;
        t=applied(k);
        speed=norm(body);
        for c=1:2
            if t+dt>edges(2*c) && t<edges(2*c+1), minCorner(c)=min(minCorner(c),speed); end
        end
        yaw=pose(3)+dYaw/2;
        factor=1;
        if abs(dYaw)>1e-12, factor=sin(dYaw/2)/(dYaw/2); end
        endPose=pose+[body*[cos(yaw) sin(yaw);-sin(yaw) cos(yaw)]*dt*factor dYaw];
        % Bound every intermediate pose by the endpoint chord and the
        % interpolation error of a constant body twist, |p''|*dt^2/8.
        angle=max(abs([pose(3) endPose(3)]-pi/2))+L.yaw;
        assert(angle<atan(min(L.half)/max(L.half)));
        half=[L.half(2) L.half(1)]*cos(angle)+[L.half(1) L.half(2)]*sin(angle);
        half=half+L.pos+speed*abs(dYaw)*dt/8;
        lo=min(pose(1:2),endPose(1:2))-half;
        hi=max(pose(1:2),endPose(1:2))+half;
        boxes=p3.yellow_boxes_mm;
        separate=max([boxes(:,1)-hi(1),boxes(:,2)-hi(2),lo(1)-boxes(:,3),lo(2)-boxes(:,4)],[],2);
        available=min([lo 2400-hi separate']);
        uniformGap=min(uniformGap,available-L.track);
        track=L.track;
        if t+dt<=result.startup_tracking_until_s+1e-9, track=result.startup_tracking_mm; end
        gap=available-track;
        if pattern==0
            % Each independently late command differs from the zero-delay
            % one only over the first L.late seconds of this slot.
            assert(L.late<dt);
            latencyYaw=latencyYaw+jump*L.late/(4*L.lever);
            latencyPosition=latencyPosition+jump*L.late*sqrt(2)/4+speed*dt*latencyYaw;
            independentGap=min(independentGap,gap-latencyPosition-norm(L.half)*latencyYaw);
        end
        if gap<minGap
            minGap=gap;
            worstSweepTime=t;
        end
        pose=endPose;
        maxOffset=max(maxOffset,norm(pose(1:2)-position(t+dt)));
        maxYaw=max(maxYaw,abs(pose(3)-pi/2)*180/pi);
    end
    assert(all(minCorner>=12) && all(held==0));
    finalError=norm(pose(1:2)-points(end,:));
    uncorrected=zeros(1,2);
    for sign=[-1 1]
        ang=sign*L.yaw;
        drifted=points(1,:)+(pose(1:2)-points(1,:))*[cos(ang) sin(ang);-sin(ang) cos(ang)];
        uncorrected((sign+3)/2)=norm(drifted-points(end,:));
    end
    fprintf('pattern %d: uniform sweep %.3f mm, startup-conditioned sweep %.3f mm at %.3f s; endpoint %.3f mm\n',...
        pattern,uniformGap,minGap,worstSweepTime,finalError);
    result.normal_patterns=[result.normal_patterns; struct('pattern',pattern,...
        'uniform_sweep_mm',uniformGap,'sweep_mm',minGap,'min_corner_speed_mm_s',minCorner,...
        'worst_sweep_time_s',worstSweepTime,'max_reference_offset_mm',maxOffset,...
        'max_heading_error_deg',maxYaw,'final_position_error_mm',finalError,'final_pose',pose,...
        'uncorrected_initial_heading_endpoint_errors_mm',uncorrected)];
    if pattern==0
        result.independent_late_sweep_mm=independentGap;
        result.independent_late_endpoint_error_bound_mm=finalError+latencyPosition;
    end
end
result.uniform_budget_passed=all([result.normal_patterns.uniform_sweep_mm]>0);
result.startup_conditioned_model_passed=all([result.normal_patterns.sweep_mm]>0 & ...
    [result.normal_patterns.final_position_error_mm]<=5 & [result.normal_patterns.max_heading_error_deg]<1);
if nargin>=4
    f=fopen(output,'w'); assert(f>=0); fprintf(f,'%s\n',jsonencode(result)); fclose(f);
end
assert(~result.uniform_budget_passed,'Retained startup regression did not reproduce');
assert(result.startup_conditioned_model_passed,'First leg did not meet the conditional sweep and arrival budgets');
assert(all([result.normal_patterns.uncorrected_initial_heading_endpoint_errors_mm]>5));
fprintf('Independent lateness sweep %.3f mm; endpoint bound %.3f mm with exact initial heading\n',...
    result.independent_late_sweep_mm,result.independent_late_endpoint_error_bound_mm);
fprintf('FIRST_LEG_CONTINUOUS_CONDITIONAL %.3fs, min sweep %.3fmm\n',edges(end),min([result.normal_patterns.sweep_mm]));

    function p = position(t)
        t=min(edges(end),max(0,t(:)));
        p=zeros(numel(t),2);
        for j=1:5
            idx=t>=edges(j) & t<=edges(j+1);
            if ~any(idx), continue; end
            local=t(idx)-edges(j);
            shape=ceil(j/2);
            if mod(j,2)
                u=distance(local,profiles(shape,:))/lengths(shape);
                p(idx,:)=a(shape,:)+u*(b(shape,:)-a(shape,:));
            else
                u=local/turnTime(shape);
                value=zeros(numel(u),2);
                for n=0:5
                    value=value+nchoosek(5,n)*u.^n.*(1-u).^(5-n)*P{shape}(1+n,:);
                end
                p(idx,:)=value;
            end
        end
    end
end

function profile = straight(len,v,b,H)
for n=0:200
    flat=n*H;
    cruise=(len-dot(b,v)/2)/(flat+sum(b)/2);
    change=abs(cruise-v);
    if cruise>0 && cruise<=140 && all(1.5*change./b<=90) && all(6*change./b.^2<=240)
        profile=[b(1) flat b(2) v(1) cruise v(2)];
        return;
    end
end
error('No straight profile for the selected joins');
end

function s = distance(t,p)
a=p(1); f=p(2); b=p(3); v0=p(4); v=p(5); v1=p(6);
x=min(1,t/a); y=max(0,min(1,(t-a-f)/b));
s=v0*a*x+(v-v0)*a*(x.^3-.5*x.^4)+v*max(0,min(f,t-a))...
    +v*b*y+(v1-v)*b*(y.^3-.5*y.^4);
end
