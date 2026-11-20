function [accepted,out]=velocity_candidate(q0,a0,q1,dtMs,limits)
% B?zier control hull checks, with four equal subdivisions and bounded 20-ms search.
% Body q for xy in mm/s and yaw in rad/s; W uses wheel RPM.
out=struct('p',zeros(4,3),'duration_ms',0,'distance_mm',0,'rotation_rad',0);
accepted=false;
if any(~isfinite([q0 a0 q1 dtMs])) || dtMs<1 || dtMs>200 || dtMs~=floor(dtMs)
    return
end
W=limits.W;
if ~within([q0;q1],limits.componentV,limits.wV,limits.rpmV,W,false) || ...
   ~within(a0,limits.xyA,limits.wA,limits.rpmA,W,true)
    return
end
% Any valid curve must satisfy displacement-in-velocity-space / acceleration.
delta=q1-q0;
lowerS=max([norm(delta(1:2))/limits.xyA abs(delta(3))/limits.wA ...
            max(abs(W*delta'))/limits.rpmA]);
startMs=20*max(1,ceil(max(dtMs,lowerS*1000-1e-8)/20));
for durationMs=startMs:20:8000
    t=durationMs*0.001;
    p=[q0;q0+t/3*a0;q1;q1];
    [first,second]=split(p);
    [a,b]=split(first); [c,d]=split(second);
    pieces={a b c d};
    h=t/4; feasible=true; bound=0; rotation=0;
    for i=1:4
        piece=pieces{i};
        D=3*diff(piece,1,1)/h;
        J=2*diff(D,1,1)/h;
        if ~within(piece,limits.componentV,limits.wV,limits.rpmV,W,false) || ...
           ~within(D,limits.xyA,limits.wA,limits.rpmA,W,true) || ...
           ~within(J,limits.xyJ,limits.wJ,limits.rpmJ,W,true)
            feasible=false;
            break
        end
        % Norm of the weighted sum is bounded by weighted norm sum.
        % Each Bernstein basis function integrates to exactly h/4.
        bound=bound+h/4*sum(hypot(piece(:,1),piece(:,2)));
        rotation=rotation+h/4*sum(abs(piece(:,3)));
    end
    if feasible
        out=struct('p',p,'duration_ms',durationMs,'distance_mm',bound,'rotation_rad',rotation);
        accepted=true;
        return
    end
end
end

function ok=within(x,xy,w,rpm,W,useNorm)
if useNorm
    translation=hypot(x(:,1),x(:,2));
else
    translation=max(abs(x(:,1:2)),[],2);
end
% Explicit model roundoff tolerance. Wheel encoding is checked independently.
ok=all(translation<=xy+1e-7) && all(abs(x(:,3))<=w+1e-9) && ...
    all(all(abs(x*W')<=rpm+1e-7));
end

function [left,right]=split(p)
a=(p(1:3,:)+p(2:4,:))*0.5;
b=(a(1:2,:)+a(2:3,:))*0.5;
m=(b(1,:)+b(2,:))*0.5;
left=[p(1,:);a(1,:);b(1,:);m];
right=[m;b(2,:);a(3,:);p(4,:)];
end
