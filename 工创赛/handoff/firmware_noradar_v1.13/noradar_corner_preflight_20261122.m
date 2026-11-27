function noradar_corner_preflight_20261122
% Nominal QR-to-raw path; centerline kinematics, not a vehicle test.
root=fileparts(mfilename('fullpath'));
p=fileread(fullfile(root,'CHASSIS','USER','zone2_fixed_routes_20261121.h'));
row=regexp(p,'(?m)^[ \t]*\{3,[^\r\n]*leg 2,[^\r\n]*','match','once');
xy=reshape(str2double(regexp(row,'-?\d+\.\d+(?=f)','match')),2,[])';
assert(isequal(size(xy),[3 2]));
d=diff(xy); L=vecnorm(d,2,2); a=d(1,:)/L(1); b=d(2,:)/L(2);
r=min(160,.4*min(L));
ctrl=r*[-a;-2*a/3;-a/3;b/3;2*b/3;b];
v=5*diff(ctrl); ac=4*diff(v); j=3*diff(ac);
peak=@(q) max(vecnorm(q,2,2));
for limit=[90 120 150]
    req=max([peak(v)/250 sqrt(peak(ac)/600) (peak(j)/1200)^(1/3) norm(v(1,:))/limit]);
    for ticks=ceil(req/.08):100
        T=ticks*.08; join=norm(v(1,:))/T;
        in=line_time(L(1)-r,0,join); out=line_time(L(2)-r,join,0);
        if isfinite(in+out), break; end
    end
    assert(isfinite(in+out));
    assert(join<=limit+1e-9 && peak(v)/T<=250+1e-9 && peak(ac)/T^2<=600+1e-9 && peak(j)/T^3<=1200+1e-9);
    fprintf('join_limit=%.0f join=%.3f blend_s=%.2f short_line_plus_blend_s=%.2f\n',limit,join,T,in+T+out);
end
end
function best=line_time(L,vi,vf)
best=inf;
for ramp=[5 8 10 15 20 25 30 40]
    b=ramp*.08;
    for n=0:200
        t=(n+2*ramp)*.08;
        v=(L-b*(vi+vf)*.5)/(n*.08+b);
        if v+.0001<max(vi,vf) || t>=best, break; end
        delta=v-min(vi,vf);
        if v<=250 && 1.5*delta<=600*b && 6*delta<=1200*b*b, best=t; break; end
    end
end
end
