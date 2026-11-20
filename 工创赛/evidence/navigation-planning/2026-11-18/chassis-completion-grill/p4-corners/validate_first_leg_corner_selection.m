function report=validate_first_leg_corner_selection(output)
% Offline geometric screen for one ordinary corner of a copied P3 leg.
folder=fileparts(mfilename('fullpath'));
data=jsondecode(fileread(fullfile(folder,'..','p3-path','p3_path_vectors.json')));
certificate=jsondecode(fileread(fullfile(folder,'joint_model_1.json')));
L=certificate.assumptions;
assert(data.passed && isequal(data.body_mm(:),2*L.half(:)) && ...
    data.position_error_mm==L.pos && data.heading_error_deg==rad2deg(L.yaw));
L.field=data.field_size_mm;
L.startupTrack=1;
L.startupDistance=5;
L.minExit=12*(L.H+36*L.H/2)+L.track;
leg=data.legs(1);
p=leg.points_mm;
h=leg.headings.hold;
yellow=data.yellow_boxes_mm;
result=select_corner(p,h,yellow,[0 0],L);
assert(strcmp(result.mode,'blend_candidate') && isequal(result.corner_mm,p(2,:)),result.reason);
frozen=jsondecode(fileread(fullfile(folder,'p4_corner_vectors.json')));
assert(norm(result.control_mm-frozen.routes(1).corners(1).control_points_mm,'fro')<1e-8);
assert(result.projection_mm>0 && result.curve_margin_mm>0);
checks=4;

% Copied start point is translated independently of the P3 waypoint list.
offsetResults=cell(16,1);
k=0;
for x=[-.25 .25]
    for y=[-.25 .25]
        for dx=[-2 2]
            for dh=[-.015 .015]
                q=p;
                q(2,1)=q(2,1)+dx;
                r=select_corner(q,h+dh,yellow,[x y],L);
                assert(strcmp(r.mode,'blend_candidate') && isequal(r.corner_mm,q(2,:)),r.reason);
                assert(isequal(r.start_mm,q(1,:)+[x y]));
                k=k+1;
                offsetResults{k}=r;
                checks=checks+2;
            end
        end
    end
end
startup=select_corner(p,h,yellow,[0 -.5],L);
assert(strcmp(startup.reason,'polyline_blocked'));
wide=L;
wide.startupTrack=L.track;
uniform=select_corner(p,h,yellow,[0 0],wide);
assert(strcmp(uniform.reason,'polyline_blocked'));
checks=checks+2;

% Extra occupied cells are full world boxes, with the yellow boxes retained.
alternate=select_corner(p,h,[yellow; cell_box(data.detour.occupied_ids,data.cell_size_mm)],[0 0],L);
blocked=select_corner(p,h,[yellow; cell_box(1,data.cell_size_mm)],[0 0],L);
assert(strcmp(alternate.mode,'blend_candidate') && strcmp(blocked.reason,'polyline_blocked'));
checks=checks+1;

q=[p(1:2,:);p(2,:)+[30 30]];
straight=select_corner(q,h,yellow,[0 0],L);
assert(strcmp(straight.mode,'settle_and_continue') && strcmp(straight.reason,'straight') && ...
    isequal(straight.continue_mm,q(3,:)));
q=[p(1,:);p(1,:)+[5 6];p(2:end,:)];
short=select_corner(q,h,yellow,[0 0],L);
assert(strcmp(short.mode,'settle_and_continue') && strcmp(short.reason,'short_trim') && ...
    isequal(short.continue_mm,q(3,:)),short.reason);
q=[p(2,:);p(3,:);p(3,:)+[0 25]];
exit=select_corner(q,h,yellow,[0 0],L);
assert(strcmp(exit.mode,'settle_and_continue') && strcmp(exit.reason,'short_terminal_exit') && ...
    isequal(exit.continue_mm,q(end,:)));
direct=select_corner(p([1 2],:),h,yellow,[0 0],L);
assert(strcmp(direct.mode,'line_to_station'));
checks=checks+4;

centralLeg=data.legs(4);
central=select_corner(centralLeg.points_mm,centralLeg.headings.hold,yellow,[0 0],L);
cornerBox=[1032 1339 1040 1347];
centralFallback=select_corner(centralLeg.points_mm,centralLeg.headings.hold,[yellow;cornerBox],[0 0],L);
assert(strcmp(central.mode,'blend_candidate'),central.reason);
assert(strcmp(centralFallback.mode,'settle_and_continue') && strcmp(centralFallback.reason,'curve_blocked') && ...
    centralFallback.path_margin_mm>0 && centralFallback.curve_margin_mm<=0 && ...
    isequal(centralFallback.continue_mm,centralLeg.points_mm(3,:)),centralFallback.reason);
checks=checks+2;
report=struct('version',version,'passed',true,'checks',checks,'model_only',true,...
    'deployment_allowed',false,'frozen_first_corner',result,'perturbed',{offsetResults},...
    'startup_offset_blocked',startup,'uniform_tracking_blocked',uniform,...
    'alternate_occupancy',alternate,'occupied_route_blocked',blocked,'no_angle',straight,...
    'short_trim',short,'short_terminal_exit',exit,'no_interior_point',direct,...
    'central',central,'central_fallback',centralFallback,'startup_distance_mm',L.startupDistance,...
    'startup_tracking_mm',L.startupTrack,'normal_tracking_mm',L.track,'exit_screen_mm',L.minExit);
if nargin>0
    f=fopen(output,'w');
    assert(f>=0);
    fprintf(f,'%s\n',jsonencode(report));
    fclose(f);
end
fprintf('FIRST_LEG_CORNER_GEOMETRY_PASS %d checks, %d perturbed starts, first angle %.3f deg, trim %.3f mm, path/curve %.3f/%.3f mm\n',...
    checks,numel(offsetResults),result.angle_deg,result.trim_mm,result.path_margin_mm,result.curve_margin_mm);
fprintf('Central bend with added box: polyline %.3f mm, blend %.3f mm, %s\n',...
    centralFallback.path_margin_mm,centralFallback.curve_margin_mm,centralFallback.mode);
end

function r=select_corner(p,h,boxes,startOffset,L)
assert(size(p,1)>=2 && size(p,2)==2 && all(isfinite(p(:))) && isfinite(h));
assert(size(boxes,2)==4 && all(isfinite(boxes(:))) && all(all(boxes(:,1:2)<boxes(:,3:4))));
p(1,:)=p(1,:)+startOffset;
ex=body_extent(h,L.half,L.yaw)+L.pos;
lead=p(2,:)-p(1,:);
ln=norm(lead);
assert(ln>0);
splitPoint=p(1,:)+lead*min(1,L.startupDistance/ln);
m=sweep([p(1,:);splitPoint],ex+L.startupTrack,boxes,L.field,7);
m=min(m,sweep([splitPoint;p(2,:)],ex+L.track,boxes,L.field,7));
for i=2:size(p,1)-1
    m=min(m,sweep(p(i:i+1,:),ex+L.track,boxes,L.field,7));
end
r=struct('mode','unavailable','reason','polyline_blocked','start_mm',p(1,:),...
    'corner_mm',[],'continue_mm',[],'station_mm',p(end,:),'heading_deg',h,...
    'angle_deg',[],'trim_mm',[],'available_exit_mm',[],'control_mm',[],...
    'projection_mm',[],'path_margin_mm',m,'curve_margin_mm',[]);
if m<=0, return; end
if size(p,1)==2
    r.mode='line_to_station';
    r.reason='no_interior_point';
    return;
end
r.mode='settle_and_continue';
r.corner_mm=p(2,:);
r.continue_mm=p(3,:);
leave=p(3,:)-p(2,:);
rn=norm(leave);
assert(rn>0);
u=lead/ln;
v=leave/rn;
r.angle_deg=atan2d(det([u; v]),dot(u,v));
r.reason='straight';
if abs(r.angle_deg)<1e-8, return; end
r.reason='reversal';
if abs(r.angle_deg)>150, return; end
d=min(160,.4*min(ln,rn));
r.trim_mm=d;
r.reason='short_trim';
if d<8, return; end
terminal=size(p,1)==3;
r.available_exit_mm=rn-d-.4*rn*~terminal;
r.reason='short_shared_exit';
if terminal, r.reason='short_terminal_exit'; end
% This is only an early exit-length screen, prior to per-route braking.
if r.available_exit_mm<L.minExit, return; end
c=p(2,:);
P=[c-d*u;c-2*d*u/3;c-d*u/3;c+d*v/3;c+2*d*v/3;c+d*v];
D=5*diff(P);
r.projection_mm=min(D*((u+v)/norm(u+v))');
assert(r.projection_mm>0);
A=4*diff(D);
assert(norm(A(1,:))+norm(A(end,:))<1e-8);
r.control_mm=P;
r.curve_margin_mm=sweep(P,ex+L.track,boxes,L.field,7);
r.reason='curve_blocked';
if r.curve_margin_mm<=0, return; end
r.mode='blend_candidate';
r.reason='geometry_pass';
end

function m=sweep(P,ex,boxes,field,depth)
% A Bezier segment lies in its control hull. Split when its AABB touches.
lo=min(P,[],1)-ex;
hi=max(P,[],1)+ex;
g=max([boxes(:,1)-hi(1),boxes(:,2)-hi(2),lo(1)-boxes(:,3),lo(2)-boxes(:,4)],[],2);
m=min([lo field-hi g']);
if m>0 || depth==0, return; end
n=size(P,1);
left=P;
right=P;
for k=1:n
    left(k,:)=P(1,:);
    right(n-k+1,:)=P(end,:);
    P=(P(1:end-1,:)+P(2:end,:))/2;
end
m=min(sweep(left,ex,boxes,field,depth-1),sweep(right,ex,boxes,field,depth-1));
end

function ex=body_extent(h,half,yaw)
% Each vertex projection is sinusoidal over the bounded heading interval.
vertices=[1 1;1 -1;-1 1;-1 -1].*half(:)';
projection=vertices*[cosd(h) sind(h);-sind(h) cosd(h)];
derivative=[-projection(:,2) projection(:,1)];
ex=max([projection*cos(yaw)-derivative*sin(yaw);...
        projection*cos(yaw)+derivative*sin(yaw)],[],1);
ex(any(abs(atan2(derivative,projection))<=yaw,1))=norm(half);
end

function box=cell_box(id,cellSize)
row=floor(id/5);
col=mod(id,5);
if mod(row,2), col=4-col; end
lo=cellSize*[col row];
box=[lo lo+cellSize];
end
