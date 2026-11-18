function report=validate_eight_leg_reference(output,speed,occupiedDetour)
% Conditional offline model of time-indexed reference from copied P3 points.
folder=fileparts(mfilename('fullpath'));
data=jsondecode(fileread(fullfile(folder,'..','p3-path','p3_path_vectors.json')));
c=jsondecode(fileread(fullfile(folder,'joint_model_1.json')));
L=c.assumptions;
L.field=data.field_size_mm;
L.box=data.yellow_boxes_mm;
L.trackEdge=1;
L.edgeDistance=5;
if nargin<2, speed=250; end
assert(isscalar(speed) && isfinite(speed) && speed>0 && speed<=250);
L.speed=speed;
L.minExit=12*(L.H+36*L.H/2)+L.track;
assert(data.passed && isequal(data.body_mm(:),2*L.half(:)));
assert(data.position_error_mm==L.pos && data.heading_error_deg==rad2deg(L.yaw));
if nargin>=3 && occupiedDetour
    assert(ismember(double(occupiedDetour),[1 5 7]));
    occupied=max(5,double(occupiedDetour));
    report=verify_occupied_detour(data,L,occupied);
    write_report(output,report);
    return;
end
for i=1:numel(data.legs)
    leg=data.legs(i);
    rows(i)=simulate_leg(leg,L,true); %#ok<AGROW>
    stopped(i)=simulate_leg(leg,L,false); %#ok<AGROW>
    fprintf('Leg %d: %.2f s, all-stop %.2f s, %d blends, %d interior stops, clearance %.3f mm\n',...
        i,rows(i).time_s,stopped(i).time_s,numel(rows(i).blend),numel(rows(i).stops),rows(i).clearance_mm);
end
assert(numel(rows)==8 && ismember(2,rows(1).blend));
assert(sum([rows.time_s])<180);
leg=data.legs(1);
for i=1:4
    q=leg;
    q.points_mm(1,:)=q.points_mm(1,:)+.2*[(-1)^i (-1)^floor(i/2)];
    q.points_mm(2,1)=q.points_mm(2,1)+2*(-1)^i;
    q.headings.hold=q.headings.hold+.015*(-1)^i;
    q.headings.start=q.headings.hold;
    q.headings.terminal=q.headings.hold;
    perturbed(i)=simulate_leg(q,L,true); %#ok<AGROW>
    assert(ismember(2,perturbed(i).blend));
end
blocked=L;
blocked.box=[L.box;1032 1339 1040 1347];
base=simulate_leg(data.legs(4),L,true);
fallback=simulate_leg(data.legs(4),blocked,true);
assert(ismember(2,base.blend) && isempty(fallback.blend) && isequal(fallback.stops,2));
strict=L;
strict.trackEdge=L.track;
unavailable=simulate_leg(leg,strict,true);
assert(~unavailable.available);
report=struct('version',version,'passed',true,'model_only',true,'deployment_allowed',false,...
    'legs',rows,'stopped',stopped,'perturbed',perturbed,'occupied_fallback',fallback,...
    'uniform_tracking_refused',unavailable,'navigation_time_s',sum([rows.time_s]),...
    'stop_only_time_s',sum([stopped.time_s]),'assumptions',L);
if nargin>0
    write_report(output,report);
end
fprintf('EIGHT_LEG_REFERENCE_PASS %.2f s vs %.2f s all-stop, excluding scan/handling/physical settle\n',...
    report.navigation_time_s,report.stop_only_time_s);
end

function write_report(output,report)
fid=fopen(output,'w');
assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(report));
fclose(fid);
end

function report=verify_occupied_detour(data,L,occupied)
cellSize=data.cell_size_mm;
lo=grid_xy(occupied)*cellSize;
occupiedBox=[lo lo+cellSize];
if occupied==5
    changedLeg=2;
    waypoints=[12 14];
    before=[2 3 4 5 14];
    after=[2 7 12 13 14];
    assert(isequal(occupiedBox,[1920 480 2400 960]));
else
    changedLeg=8;
    waypoints=[20 0];
    before=[22 17 12 7 2 1 0];
    after=[22 21 20 19 10 9 0];
    assert(isequal(occupiedBox,[960 480 1440 960]));
end
L.box=[L.box;occupiedBox];
original=data.legs(changedLeg);
oldCells=expand_grid(original.from,original.waypoints);
assert(isequal(oldCells,before));
assert(any(oldCells==occupied));
assert(~simulate_leg(original,L,true).available);
replies={data.legs.waypoints};
replies{changedLeg}=waypoints;
detour=expand_grid(original.from,waypoints);
assert(isequal(detour,after));
for i=1:numel(data.legs)
    leg=data.legs(i);
    cells{i}=expand_grid(leg.from,replies{i}); %#ok<AGROW>
    assert(cells{i}(end)==leg.to && ~any(cells{i}==occupied));
    if i==changedLeg
        nodes=grid_xy([leg.from replies{i}])*cellSize+cellSize/2;
        leg.points_mm=unique([leg.points_mm(1,:);nodes;leg.points_mm(end,:)],'rows','stable');
        assert(isequal(leg.points_mm(end,:),original.points_mm(end,:)));
    else
        assert(isequal(cells{i},leg.cells(:)'));
    end
    if i>1
        assert(leg.from==data.legs(i-1).to);
        assert(abs(wrap(leg.headings.start-data.legs(i-1).headings.terminal))<1e-8);
    end
    rows(i)=simulate_leg(leg,L,true); %#ok<AGROW>
    assert(rows(i).available && rows(i).clearance_mm>0);
    fprintf('Occupied leg %d: %.2f s, clearance %.3f mm\n',i,rows(i).time_s,rows(i).clearance_mm);
end
assert(numel(rows)==8 && data.legs(end).to==data.legs(1).from);
assert(isequal([data.legs.cache_hit],logical([0 0 0 0 0 1 1 0])));
% The inflated body is in contact with the occupied box but the center is outside it.
ex=body_extent(original.headings.hold,L)+L.track;
touch=[mean(occupiedBox([1 3])) occupiedBox(2)-ex(2)];
on=sweep(touch,ex,L,0);
clear=sweep(touch-[0 .1],ex,L,0);
assert(on<=1e-9 && clear>.09);
report=struct('version',version,'passed',true,'model_only',true,'deployment_allowed',false,...
    'occupied_grid_id',occupied,'occupied_box_mm',occupiedBox,'old_cells',oldCells,...
    'old_route_rejected',true,'modified_leg',changedLeg,'new_waypoints',waypoints,'new_cells',detour,...
    'all_wire_waypoints',{replies},'all_expanded_cells',{cells},...
    'contact_margin_mm',on,'just_clear_margin_mm',clear,...
    'navigation_time_s',sum([rows.time_s]),'legs',rows,'assumptions',L);
fprintf('ZONE2_OCCUPIED_DETOUR_PASS 8 legs, %.2f s, original path rejected\n',report.navigation_time_s);
end

function xy=grid_xy(ids)
y=floor(ids(:)/5);
x=mod(ids(:),5);
odd=mod(y,2)==1;
x(odd)=4-x(odd);
xy=[x y];
end

function cells=expand_grid(start,waypoints)
cells=start;
from=grid_xy(start);
for id=waypoints(:)'
    to=grid_xy(id);
    d=to-from;
    assert(nnz(d)==1);
    for k=1:sum(abs(d))
        from=from+sign(d);
        col=from(1);
        if mod(from(2),2), col=4-col; end
        cells(end+1)=5*from(2)+col; %#ok<AGROW>
    end
end
end

function row=simulate_leg(leg,L,blendEnabled)
p=leg.points_mm;
h=leg.headings.hold;
bad=struct('available',false,'time_s',nan,'blend',[],'stops',[],'clearance_mm',nan,...
    'wheel_peak_rpm',[],'wheel_rate_rpm_s',[],'wheel_jerk_rpm_s2',[],...
    'min_join_mm_s',[],'edges_s',[],'spans',[]);
row=bad;
len=vecnorm(diff(p),2,2);
u=diff(p)./len;
n=size(p,1);
ext=body_extent(h,L);
clearance=inf;
for i=1:n-1
    split=min(L.edgeDistance,len(i));
    if i==1
        edge=p(i,:)+split*u(i,:);
        g=min(sweep([p(i,:);edge],ext+L.trackEdge,L,7),sweep([edge;p(i+1,:)],ext+L.track,L,7));
    elseif i==n-1
        edge=p(i+1,:)-split*u(i,:);
        g=min(sweep([p(i,:);edge],ext+L.track,L,7),sweep([edge;p(i+1,:)],ext+L.trackEdge,L,7));
    else
        g=sweep(p(i:i+1,:),ext+L.track,L,7);
    end
    if g<=0, return; end
    clearance=min(clearance,g);
end
corners=cell(1,n);
spans=struct('kind',{},'sec',{},'at',{},'p',{},'q',{},'profile',{});
for i=2:n-1
    a=u(i-1,:);
    b=u(i,:);
    angle=abs(atan2d(det([a;b]),dot(a,b)));
    if ~blendEnabled || angle<1e-6 || angle>150, continue; end
    trim=min(160,.4*min(len(i-1),len(i)));
    exit=len(i)-trim-.4*len(i)*(i+1<n);
    if trim<8 || exit<L.minExit, continue; end
    P=[p(i,:)-trim*a;p(i,:)-2*trim*a/3;p(i,:)-trim*a/3;...
       p(i,:)+trim*b/3;p(i,:)+2*trim*b/3;p(i,:)+trim*b];
    D=5*diff(P);
    A=4*diff(D);
    J=3*diff(A);
    sec=L.H*ceil(max([max(vecnorm(D,2,2))/L.speed,sqrt(max(vecnorm(A,2,2))/120),...
            nthroot(max(vecnorm(J,2,2))/240,3),norm(D(1,:))/90])/L.H);
    g=sweep(P,ext+L.track,L,7);
    if g<=0, continue; end
    while min(D*((a+b)/norm(a+b))')/sec>=12
        v=norm(D(1,:))/sec;
        if ~isempty(profile(len(i-1)-trim,0,v,L)) && ~isempty(profile(len(i)-trim,v,0,L))
            assert(min(D*((a+b)/norm(a+b))')>0);
            corners{i}=struct('p',P,'sec',sec,'v',v,'g',g);
            break;
        end
        sec=sec+L.H;
    end
end
while true
    poses=spans;
    stops=[];
    blends=[];
    joined=[];
    g=clearance;
    badLine=0;
    yaw=[leg.headings.first,leg.headings.second];
    assert(abs(wrap(h-leg.headings.start-yaw(1)))<1e-8);
    assert(abs(wrap(leg.headings.terminal-h-yaw(2)))<1e-8);
    [poses,g]=add_turn(poses,g,p(1,:),leg.headings.start,yaw(1),L);
    for i=1:n-1
        a=p(i,:); b=p(i+1,:); v0=0; v1=0;
        if ~isempty(corners{i})
            a=corners{i}.p(end,:);
            v0=corners{i}.v;
        end
        if ~isempty(corners{i+1})
            b=corners{i+1}.p(1,:);
            v1=corners{i+1}.v;
        end
        line=profile(norm(b-a),v0,v1,L);
        if isempty(line)
            badLine=i+~isempty(corners{i+1});
            break;
        end
        poses(end+1)=span('line',sum(line(1:3)),a,b,line);
        if v1>0
            k=corners{i+1};
            poses(end+1)=span('blend',k.sec,k.p,[],[]);
            g=min(g,k.g);
            blends(end+1)=i+1;
            joined(end+1)=k.v;
        elseif i+1<n
            poses(end+1)=span('stop',2*L.H,b,b,[]);
            stops(end+1)=i+1;
        end
    end
    if ~badLine, break; end
    assert(~isempty(corners{badLine}));
    corners{badLine}=[];
end
[poses,g]=add_turn(poses,g,p(end,:),h,yaw(2),L);
if g<=0, return; end
edges=[0 cumsum([poses.sec])];
for i=1:numel(poses)
    poses(i).at=edges(i);
end
start=leg.headings.start;
finish=leg.headings.terminal;
[p0,theta0]=sample([-L.H 0],poses,start,finish);
[p1,theta1]=sample([edges(end) edges(end)+L.H],poses,start,finish);
assert(all(vecnorm(p0-p(1,:),2,2)<1e-8) && all(vecnorm(p1-p(end,:),2,2)<1e-8));
assert(all(abs(wrap(theta0-start))<1e-8) && all(abs(wrap(theta1-finish))<1e-8));
[slot,wheel]=ndgrid((-2:round(edges(end)/L.H)+2)'*L.H,0:3);
slot=slot(:)+wheel(:)*.020;
wheel=wheel(:)+1;
[a,aa]=sample(slot,poses,start,finish);
[b,bb]=sample(slot+L.H,poses,start,finish);
d=(b-a)/L.H;
xy=[d(:,1).*cosd(aa)+d(:,2).*sind(aa),-d(:,1).*sind(aa)+d(:,2).*cosd(aa)];
W=[1 1 -1;1 -1 1;1 -1 -1;1 1 1];
q=round(10*sum([xy L.lever*deg2rad(wrap(bb-aa))/L.H].*W(wheel,:),2)/L.K)/10;
rates=zeros(1,4);
jerks=rates;
for k=1:4
    cmd=q(wheel==k);
    dt=L.H-L.late;
    rates(k)=max(abs(diff(cmd)))/dt;
    jerks(k)=max(abs(diff(cmd,2))/dt^2+abs(diff(cmd(1:end-1)))*2*L.late/dt^3);
    assert(cmd(1)==0 && cmd(end)==0 && rates(k)<=250 && jerks(k)<=1400);
end
assert(max(abs(q))<=300);
row=struct('available',true,'time_s',edges(end),'blend',blends,'stops',stops,'clearance_mm',g,...
    'wheel_peak_rpm',max(abs(q)),'wheel_rate_rpm_s',max(rates),'wheel_jerk_rpm_s2',max(jerks),...
    'min_join_mm_s',min(joined),'edges_s',edges,'spans',poses);
end

function r=profile(len,v0,v1,L)
r=[];
best=inf;
for count=[5 8 10 15 20 25 30 40]
    b=count*L.H;
    for n=0:200
        f=n*L.H;
        v=(len-b*(v0+v1)/2)/(f+b);
        change=v-min(v0,v1);
        if v<max(v0,v1), break; end
        if v<=L.speed && 1.5*change/b<=120 && 6*change/b^2<=240
            if f+2*b<best
                r=[b f b v0 v v1];
                best=f+2*b;
            end
            break;
        end
    end
end
end

function [poses,g]=add_turn(poses,g,p,start,turn,L)
if ~turn, return; end
allYaw=start+linspace(0,turn,ceil(abs(turn))+1);
ext=zeros(1,2);
for h=allYaw
    ext=max(ext,body_extent(h,L));
end
% One-degree heading grid padded by a half-degree angular Lipschitz bound.
g=min(g,sweep(p,ext+L.track+norm(L.half)*deg2rad(.5),L,0));
sec=L.H*ceil(3.2*max(1,abs(turn)/90)/L.H);
assert(1.875*abs(turn)/sec<=90 && 6*abs(turn)/sec^2<=180 && 60*abs(turn)/sec^3<=1440);
poses(end+1)=span('turn',sec,p,[],[start turn]);
end

function s=span(kind,sec,p,q,profile)
s=struct('kind',kind,'sec',sec,'at',0,'p',p,'q',q,'profile',profile);
end

function [p,h]=sample(t,poses,start,finish)
t=t(:);
p=repmat(poses(1).p(1,:),numel(t),1);
h=repmat(start,numel(t),1);
current=start;
for i=1:numel(poses)
    s=poses(i);
    mask=t>=s.at & t<s.at+s.sec;
    ts=t(mask)-s.at;
    h(mask)=current;
    switch s.kind
        case 'line'
            r=s.profile;
            b=r(1); f=r(2); v0=r(4); v=r(5); v1=r(6);
            x=min(1,ts/b); y=max(0,min(1,(ts-b-f)/b));
            d=v0*b*x+(v-v0)*b*(x.^3-.5*x.^4)+v*max(0,min(f,ts-b))...
                +v*b*y+(v1-v)*b*(y.^3-.5*y.^4);
            p(mask,:)=s.p+d*((s.q-s.p)/norm(s.q-s.p));
        case 'blend'
            u=ts/s.sec; v=1-u;
            p(mask,:)=[v.^5 5*u.*v.^4 10*u.^2.*v.^3 10*u.^3.*v.^2 5*u.^4.*v u.^5]*s.p;
        case 'stop'
            p(mask,:)=repmat(s.p,numel(ts),1);
        case 'turn'
            p(mask,:)=repmat(s.p,numel(ts),1);
            u=ts/s.sec;
            h(mask)=s.profile(1)+s.profile(2)*u.^3.*(10-15*u+6*u.^2);
            current=s.profile(1)+s.profile(2);
    end
end
last=poses(end);
endp=last.p(end,:);
if strcmp(last.kind,'line'), endp=last.q; end
after=t>=last.at+last.sec;
p(after,:)=repmat(endp,nnz(after),1);
h(after)=finish;
end

function a=wrap(a)
a=mod(a+180,360)-180;
end

function m=sweep(P,ex,L,depth)
lo=min(P,[],1)-ex;
hi=max(P,[],1)+ex;
B=L.box;
m=min([lo L.field-hi max([B(:,1)-hi(1),B(:,2)-hi(2),lo(1)-B(:,3),lo(2)-B(:,4)],[],2)']);
if m>0 || depth==0, return; end
n=size(P,1);
le=P; ri=P;
for k=1:n
    le(k,:)=P(1,:);
    ri(n-k+1,:)=P(end,:);
    P=(P(1:end-1,:)+P(2:end,:))/2;
end
m=min(sweep(le,ex,L,depth-1),sweep(ri,ex,L,depth-1));
end

function ex=body_extent(h,L)
vertices=[1 1;1 -1;-1 1;-1 -1].*L.half(:)';
p=vertices*[cosd(h) sind(h);-sind(h) cosd(h)];
d=[-p(:,2) p(:,1)];
ex=max([p*cos(L.yaw)-d*sin(L.yaw);p*cos(L.yaw)+d*sin(L.yaw)],[],1);
ex(any(abs(atan2(d,p))<=L.yaw,1))=norm(L.half);
ex=ex+L.pos;
end
