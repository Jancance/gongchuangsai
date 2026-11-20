function validate_p3_path(output)
% Fixed geometry from field_map.c and vehicle_tuning.h. F yaw=90 means +Y.
yellow=[550 550 1000 1000;550 1400 1000 1850;1400 550 1850 1000;1400 1400 1850 1850];
nominal=[0 0];
budget=[5 1]; % assumed total position uncertainty per F axis, and heading uncertainty
ctx=struct('run',23,'startId',2,'node',1,'zone',2,'map',4,'layout',2,'scan',700,'link',8);
grid=make_grid(ctx,[]);
stations=[150 150;1200 300;2160 1200;240 1200;1200 2160];
ids=[0 2 14 10 22];
yaws=[90 90 90 -90 180];
stops=[1 2 3 4 5 3 4 5 1];
wps={2,[4 14],10,[12 22],[12 14],10,[12 22],[2 0]};
route=cell(1,8); requested=zeros(0,2); cache=cell(25,25); seq=65533; checks=0;
for i=1:8
    a=stops(i); b=stops(i+1); from=ids(a); to=ids(b);
    q=struct('context',ctx,'from',from,'to',to,'sequence',seq);
    hit=~isempty(cache{from+1,to+1});
    if hit
        reply=cache{from+1,to+1};
    else
        requested(end+1,:)=[from to]; %#ok<AGROW>
        reply=make_reply(q,wps{i}); cache{from+1,to+1}=reply; seq=mod(seq+1,65536);
    end
    actual=stations(a,:);
    if i==6, actual=actual+[3 -4]; end % cached path enters from the new measured pose
    [reason,cells,points]=convert(grid,q,reply,stations(b,:),actual,hit);
    assert(strcmp(reason,'ok'), 'Leg %d conversion: %s', i, reason); checks=checks+1;
    assert(isequal(points(1,:),actual) && isequal(points(end,:),stations(b,:))); checks=checks+1;
    [ok,headings,margin]=clear_route(points,yaws(a),yaws(b),-1,yellow,budget);
    assert(ok,'Leg %d clearance',i); checks=checks+1;
    [ok,~,unbounded]=clear_route(points,yaws(a),yaws(b),-1,yellow,nominal);
    assert(ok && margin<=unbounded); checks=checks+1;
    alternate=[];
    if abs(wrap(yaws(b)-yaws(a)))==180
        [ccwOK,alternate]=clear_route(points,yaws(a),yaws(b),1,yellow,budget);
        assert(ccwOK && alt_turn(headings,alternate)); checks=checks+1;
    end
    route{i}=struct('from',from,'to',to,'request_seq',reply.sequence,'cache_hit',hit,...
        'waypoints',wps{i},'cells',cells,'points_mm',points,'headings',headings,'opposite_turn',alternate,...
        'nominal_field_margin_mm',unbounded,'bounded_field_margin_mm',margin);
end
route=[route{:}];
assert(isequal(requested,[0 2;2 14;14 10;10 22;22 14;22 0])); checks=checks+1;
assert(isequal(find([route.cache_hit]),[6 7]) && ~isequal(route(6).points_mm,route(3).points_mm)); checks=checks+1;
assert(all(arrayfun(@(leg) leg.headings.first==-90 && leg.headings.second==0,route([4 5 7 8])))); checks=checks+1;
assert(isequal([route.request_seq],[65533 65534 65535 0 1 65535 0 2])); checks=checks+1;

% An alternate radar reply around occupied cell 12; use the entire cell.
q=struct('context',ctx,'from',14,'to',10,'sequence',10);
direct=make_reply(q,10);
grid12=make_grid(ctx,12);
assert(strcmp(convert(grid12,q,direct,stations(4,:),stations(3,:),false),'occupied')); checks=checks+1;
detour=make_reply(q,[4 0 10]);
[reason,cells,points]=convert(grid12,q,detour,stations(4,:),stations(3,:),false);
assert(strcmp(reason,'ok') && isequal(cells,[14 5 4 3 2 1 0 9 10])); checks=checks+1;
assert(clear_route(points,90,-90,-1,[yellow;grid12.boxes],budget)); checks=checks+1;
assert(clear_route(points,90,-90,1,[yellow;grid12.boxes],budget)); checks=checks+1;
alternateDetour=struct('waypoints',detour.waypoints,'cells',cells,'points_mm',points,'occupied_ids',12);

rejected={};
for key={'run','startId','node','zone','map','layout','scan','link'}
    bad=q; name=key{1}; bad.context.(name)=bad.context.(name)+1;
    assert(strcmp(convert(grid,bad,direct,stations(4,:),stations(3,:),false),'stale_map'));
    assert(~cache_match(bad,direct));
    rejected{end+1}=['old_' name]; checks=checks+2; %#ok<AGROW>
end
mutations={...
    'wrong_txn','sequence',11,'identity'; 'wrong_status','status',1,'status'; ...
    'wrong_reverse','from',10,'identity'; 'wrong_goal','to',11,'identity'; ...
    'wrong_waypoint','waypoints',[12 11],'last'; 'bad_index','waypoints',[25 10],'format'; ...
    'duplicate','waypoints',[14 10],'repeat'; 'loop','waypoints',[12 14 10],'repeat'; ...
    'non_axis','waypoints',[4 10],'non_axis'; 'truncated','waypoints',[],'last'; ...
    'no_capacity','waypoints',[zeros(1,24) 10],'format'; 'no_ack','ack',false,'status'};
for i=1:size(mutations,1)
    bad=direct; bad.(mutations{i,2})=mutations{i,3};
    assert(strcmp(convert(grid,q,bad,stations(4,:),stations(3,:),false),mutations{i,4}),mutations{i,1});
    rejected{end+1}=mutations{i,1}; checks=checks+1; %#ok<AGROW>
end
bad=direct; bad.to=14; bad.from=10;
assert(~cache_match(q,bad)); checks=checks+1;
badGrid=grid; badGrid.valid=false;
assert(strcmp(convert(badGrid,q,direct,stations(4,:),stations(3,:),false),'stale_map')); checks=checks+1;
badGrid=make_grid(ctx,[255]); assert(~badGrid.valid); checks=checks+1;
badGrid=make_grid(ctx,0:25); assert(~badGrid.valid); checks=checks+1;
duplicates=make_grid(ctx,[4 4 24]);
assert(isequal(duplicates.mask,bitset(bitset(uint32(0),5),25))); checks=checks+1;
assert(strcmp(convert(grid,q,direct,stations(4,:),[1900 1600],true),'pose_cell')); checks=checks+1;
assert(strcmp(convert(grid,q,direct,[NaN 1200],stations(3,:),false),'pose_cell')); checks=checks+1;

% Same-cell empty list can still require a real connector. A coincident
% endpoint is a single-point hold and needs stopped/heading checks downstream.
same=struct('context',ctx,'from',0,'to',0,'sequence',0);
[reason,~,points]=convert(grid,same,make_reply(same,[]),[160 152],[150 150],false);
assert(strcmp(reason,'ok') && isequal(points,[150 150;160 152])); checks=checks+1;
assert(clear_route(points,90,90,-1,yellow,budget)); checks=checks+1;
[reason,~,points]=convert(grid,same,make_reply(same,[]),[150 150],[150 150],false);
assert(strcmp(reason,'ok') && isequal(points,[150 150])); checks=checks+1;
assert(strcmp(convert(grid,same,make_reply(same,0),[150 150],[150 150],false),'last')); checks=checks+1;

% Discrete capacity and continuous occupancy are checked independently.
long=struct('context',ctx,'from',0,'to',24,'sequence',33);
[reason,cells,points]=convert(grid,long,make_reply(long,1:24),[2150 2155],[150 150],false);
assert(strcmp(reason,'ok') && numel(cells)==25 && size(points,1)==27); checks=checks+1;
assert(~clear_route(points,90,90,-1,yellow,budget)); checks=checks+1;
yellowPath=struct('context',ctx,'from',0,'to',18,'sequence',34);
[reason,~,points]=convert(grid,yellowPath,make_reply(yellowPath,[1 18]),cell_center(18),stations(1,:),false);
assert(strcmp(reason,'ok') && ~clear_route(points,90,90,-1,yellow,budget)); checks=checks+1;
% The full body can hit an occupied neighboring cell while its center stays
% in the free center cell. A discrete visit check alone misses this overlap.
assert(~clear_route([1200 1000;1200 1199],90,90,-1,[yellow;960 480 1440 960],budget)); checks=checks+1;
assert(~clear_route([1200 2160;150 150],180,90,-1,yellow,budget)); checks=checks+1;
assert(~clear_route([150 150;1200 240],90,0,-1,yellow,budget)); checks=checks+1;

returnCases=[0 0;5 1;5 2;8 1;2 1;5 0];
returnMargin=zeros(size(returnCases,1),2);
for i=1:size(returnCases,1)
    [~,returnMargin(i,:)]=inside_return([150 150],90,returnCases(i,:),true);
end
assert(max(abs(returnMargin(1,:)-[23 8.5]))<1e-8); checks=checks+1;
assert(all(returnMargin(2,:)>0) && returnMargin(3,2)<0 && returnMargin(4,2)<0); checks=checks+1;
poseError=[2 1];
assert(inside_return([153 147],90,poseError,true)); checks=checks+1;
assert(~inside_return([150 145],90,poseError,true)); checks=checks+1;
assert(~inside_return([150 150],90,poseError,false)); checks=checks+1;
assert(~inside_return([150 150],-90,poseError,true)); checks=checks+1;
assert(~inside_return([150 150],80,poseError,true)); checks=checks+1;
% First CW quarter is targeted at 0. At observed 4 degrees the second
% absolute target remains -90, so the residual is -94, not -90.
assert(route(3).headings.terminal==-90 && wrap(route(3).headings.terminal-4)==-94); checks=checks+1;
report=struct('version',version,'assertions',checks,'field_size_mm',2400,'cell_size_mm',480,...
    'body_mm',[283 254],'yellow_boxes_mm',yellow,'position_error_mm',budget(1),...
    'heading_error_deg',budget(2),'stations_mm',stations,'requests',requested,'legs',route,...
    'detour',alternateDetour,'rejected',{rejected},'return_error_cases',returnCases,...
    'return_margin_mm',returnMargin,'passed',true);
fid=fopen(output,'w'); assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(report)); fclose(fid);
fprintf('P3_PATH_PASS %d assertions, 6 requests, 8 legs\n',checks);
end

function g=make_grid(ctx,ids)
valid=ctx.startId==2 && ctx.node==1 && ctx.zone==2 && ctx.run>0 && ...
    numel(ids)<=25 && all(ids>=0 & ids<=24 & ids==floor(ids));
g=struct('context',ctx,'valid',valid,'mask',uint32(0),'boxes',zeros(0,4));
if ~valid, return; end
for id=reshape(unique(ids),1,[])
    g.mask=bitset(g.mask,id+1);
    p=cell_center(id); g.boxes(end+1,:)=[p-240 p+240]; %#ok<AGROW>
end
end

function p=cell_center(id)
row=floor(id/5); col=mod(id,5);
if mod(row,2), col=4-col; end
p=480*[col+0.5 row+0.5];
end

function id=point_id(p)
if numel(p)~=2 || any(~isfinite(p) | p<0 | p>=2400), id=-1; return; end
c=floor(p(1)/480); r=floor(p(2)/480);
if mod(r,2), c=4-c; end
id=5*r+c;
end

function reply=make_reply(q,waypoints)
reply=q; reply.waypoints=waypoints; reply.status=0; reply.ack=true;
end

function yes=cache_match(q,r)
yes=isequal(q.context,r.context) && q.from==r.from && q.to==r.to;
end

function [reason,cells,points]=convert(grid,q,r,terminal,actual,hit)
cells=[]; points=[];
reason='stale_map'; if ~grid.valid || ~isequal(grid.context,q.context), return; end
reason='identity'; if ~cache_match(q,r) || (~hit && q.sequence~=r.sequence), return; end
reason='status'; if ~r.ack || r.status~=0, return; end
reason='format'; w=r.waypoints;
if numel(w)>24 || any(w<0 | w>24 | w~=floor(w)) || any([q.from q.to]<0 | [q.from q.to]>24), return; end
reason='last';
if q.from==q.to
    if ~isempty(w), return; end
elseif isempty(w) || w(end)~=q.to, return;
end
reason='pose_cell'; if point_id(actual)~=q.from || point_id(terminal)~=q.to, return; end
cells=q.from;
if bitget(grid.mask,q.from+1), reason='occupied'; return; end
centers=cell_center(q.from);
for id=w
    a=centers(end,:); b=cell_center(id); n=max(abs(a-b))/480;
    if n==0, reason='repeat'; return; end
    if all(a~=b), reason='non_axis'; return; end
    for k=1:n
        current=point_id(a+(b-a)*k/n);
        if any(current==cells), reason='repeat'; return; end
        if bitget(grid.mask,current+1), reason='occupied'; return; end
        cells(end+1)=current; %#ok<AGROW>
    end
    centers(end+1,:)=b; %#ok<AGROW>
end
if q.from==q.to
    points=[actual;terminal];
else
    points=[actual;centers;terminal];
end
points=points([true;any(diff(points,1,1)~=0,2)],:);
if size(points,1)>32, reason='capacity'; return; end
reason='ok';
end

function a=wrap(a)
a=mod(a+180,360)-180;
end

function yes=alt_turn(cw,ccw)
yes=cw.first==-90 && cw.second==-90 && ccw.first==90 && ccw.second==90 && ...
    abs(wrap(cw.hold-ccw.hold))==180 && cw.terminal==ccw.terminal;
end

function [ok,phases,margin]=clear_route(points,h0,h1,direction,boxes,e)
d=wrap(h1-h0); first=d; second=0;
if abs(d)==180, first=direction*90; second=first; end
hold=h0+first;
phases=struct('start',h0,'hold',wrap(hold),'terminal',wrap(h1),'first',first,'second',second);
ok=false; margin=inf; radius=hypot(141.5,127)+e(1);
if ~ismember(abs(d),[0 90 180]), return; end
if first~=0
    [pass,m]=clear_segment(points(1,:),points(1,:),[radius radius],boxes);
    margin=min(margin,m); if ~pass, return; end
end
if second~=0
    [pass,m]=clear_segment(points(end,:),points(end,:),[radius radius],boxes);
    margin=min(margin,m); if ~pass, return; end
end
ex=extents(hold,e);
for i=1:max(1,size(points,1)-1)
    [pass,m]=clear_segment(points(i,:),points(min(i+1,size(points,1)),:),ex,boxes);
    margin=min(margin,m); if ~pass, return; end
end
ok=true;
end

function [ok,margin]=clear_segment(a,b,ex,boxes)
margin=min([min(a,b)-ex 2400-max(a,b)-ex]); ok=margin>=0;
if ~ok, return; end
for i=1:size(boxes,1)
    if intersects(a,b,boxes(i,1:2)-ex,boxes(i,3:4)+ex), ok=false; return; end
end
end

function hit=intersects(a,b,lower,upper)
enter=0; exit=1;
for i=1:2
    v=b(i)-a(i);
    if v==0
        if a(i)<lower(i) || a(i)>upper(i), hit=false; return; end
    else
        u=sort(([lower(i) upper(i)]-a(i))/v);
        enter=max(enter,u(1)); exit=min(exit,u(2));
        if enter>exit, hit=false; return; end
    end
end
hit=true;
end

function ex=extents(yaw,e)
angle=yaw+[-e(2) e(2)];
a=atan2d(127,141.5); b=atan2d(141.5,127);
critical=reshape([-a;a;-b;b]+(-3:3)*180,1,[]);
theta=[angle critical(critical>=angle(1) & critical<=angle(2))];
c=abs(cosd(theta)); s=abs(sind(theta));
ex=[max(141.5*c+127*s) max(141.5*s+127*c)]+e(1);
end

function [ok,margin]=inside_return(p,yaw,e,stopped)
margin=min([p;300-p],[],1)-extents(yaw,e);
ok=stopped && abs(wrap(yaw-90))<=1 && all(margin>=0);
end
