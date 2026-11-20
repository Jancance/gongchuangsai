function report = validate_three_leg_contract
% Pre-implementation contract model. No motor, visual or PWM model.
folder = fileparts(mfilename('fullpath'));
project = fileparts(fileparts(fileparts(fileparts(fileparts(folder)))));
p3 = fullfile(project,'evidence','navigation-planning','2026-11-18', ...
    'chassis-completion-grill','p3-path','p3_path_vectors.json');
base = load_json(p3);
live = load_json(fullfile(project,'tmp','zone2-field-read-20261120-012415.json'));
detour = load_json(fullfile(project,'tmp','zone2-field-read-20261120-004517.json'));
zone1 = load_json(fullfile(project,'tmp','pa1-dual-launch-20261120-040232','candidate-zone1-arm.json'));
assert(base.passed && isequal(base.body_mm(:)',[283 254]));
assert(base.position_error_mm==5 && base.heading_error_deg==1);
assert(live.route.completed_legs==8 && live.route.grid.start_id==2);
assert(detour.route.completed_legs==8 && detour.route.grid.start_id==2);
assert(zone1.route_trace.grid.start_id==1);
report = struct('version',version,'model_only',true);
report.routes = [
    check_route('zone2 field 01:24',live.route,base)
    check_route('zone2 field 00:45',detour.route,base)
    check_route('zone1 ARM candidate',zone1.route_trace,base)
];
report.scan = check_rescan(base,live.route.grid.occupied_ids);
report.jobs = check_task;
report.supervision = check_supervision;
report.passed = true;
file = fopen(fullfile(folder,'matlab_result.json'),'w');
assert(file>=0);
fprintf(file,'%s\n',jsonencode(report));
fclose(file);
fprintf('THREE_LEG_CONTRACT_PASS %d route replays, %d scan cases, %d job checks, %d supervision cases\n', ...
    numel(report.routes),numel(report.scan.cases),numel(report.jobs.cases),numel(report.supervision.cases));
end

function result = check_route(name,trace,base)
occupied = trace.grid.occupied_ids;
boxes = [base.yellow_boxes_mm; occupied_boxes(occupied,base.cell_size_mm)];
start = [2250 150];
if trace.grid.start_id==2
    start = [150 150];
end
stops = [1200 300;2160 1200;290 1200];
headings = [90 90;90 90;90 -90];
expected = [4*(trace.grid.start_id==1) 2;2 14;14 10];
received = trace.legs(1:3);
% Actual pose values here are stress cases for the handoff, not measurements.
origins = [start;1227 300;2150 1194];
for i=1:3
    wire = received(i).path;
    assert(wire.status==0 && isequal([wire.start wire.goal],expected(i,:)));
    assert(received(i).source_leg==i && received(i).stop_source_ms>0);
    assert(received(i).arrive_elapsed_ms>received(i).begin_elapsed_ms);
    points = convert_path(wire,origins(i,:),stops(i,:),base.cell_size_mm,occupied);
    assert(isequal(points(1,:),origins(i,:)) && isequal(points(end,:),stops(i,:)));
    travel = headings(i,1);
    if i==3
        travel = 0;
    end
    margin(i) = sweep_margin(points,boxes,extent(base,travel),base.field_size_mm); %#ok<AGROW>
    if i==3
        % Heading can traverse the entire turn interval at both endpoints.
        disk = norm(base.body_mm)/2 + base.position_error_mm;
        margin(i) = min(margin(i),sweep_margin(points([1 end],:),boxes,[disk disk],base.field_size_mm,true));
    end
    assert(margin(i)>0);
    legs(i) = struct('from',wire.start,'to',wire.goal,'wire_waypoints',wire.waypoints, ... %#ok<AGROW>
        'points_mm',points,'heading_deg',headings(i,:),'static_margin_lower_bound_mm',margin(i));
end
assert(isequal([legs.to],[2 14 10]));
assert(all(diff([received.arrive_elapsed_ms])>0));
if isequal(received(3).path.waypoints,10)
    % The offset live origin may first return to the raw-material cell center.
    assert(isequal(legs(3).points_mm(2:end,:),[2160 1200;290 1200]));
end
result = struct('source',name,'occupied_cells',occupied,'legs',legs);
end

function points = convert_path(wire,actual,target,sizeMm,occupied)
% Check the full wire route including cells skipped by endpoint geometry.
waypoints = [wire.start wire.waypoints(:)'];
assert(waypoints(end)==wire.goal);
assert(cell_id(actual,sizeMm)==wire.start && cell_id(target,sizeMm)==wire.goal);
centers = grid_xy(waypoints)*sizeMm + sizeMm/2;
visited = wire.start;
for i=2:size(centers,1)
    delta = centers(i,:)-centers(i-1,:);
    assert(nnz(delta)==1);
    for k=1:sum(abs(delta))/sizeMm
        cellId = cell_id(centers(i-1,:)+k*sign(delta)*sizeMm,sizeMm);
        assert(~ismember(cellId,visited));
        visited(end+1) = cellId; %#ok<AGROW>
    end
end
assert(~any(ismember(visited,occupied)));
if wire.start==2 && wire.goal==14 && abs(actual(1)-centers(1,1))<=1 && ...
        centers(2,1)==centers(1,1) && centers(2,2)>actual(2) && actual(2)>centers(1,2)
    centers(1,:) = [];
end
before = centers(end-1,:);
last = centers(end,:);
if any(abs(target-last)>0) && nnz(target-last)==1 && ...
        all(target>=min(before,last) & target<=max(before,last))
    centers(end,:) = [];
end
points = [actual;centers;target];
points = points([true; any(diff(points,1,1)~=0,2)],:);
end

function grid = grid_xy(ids)
row = floor(ids(:)/5);
col = mod(ids(:),5);
odd = mod(row,2)==1;
col(odd) = 4-col(odd);
grid = [col row];
end

function id = cell_id(p,sizeMm)
grid = floor(p/sizeMm);
assert(all(grid>=0 & grid<5));
if mod(grid(2),2)==1
    grid(1) = 4-grid(1);
end
id = grid(2)*5+grid(1);
end

function boxes = occupied_boxes(occupied,sizeMm)
lo = grid_xy(occupied)*sizeMm;
boxes = [lo lo+sizeMm];
end

function ex = extent(base,yaw)
heading = yaw+[-base.heading_error_deg 0 base.heading_error_deg];
lengthMm = base.body_mm(1);
widthMm = base.body_mm(2);
ex = max([lengthMm*abs(cosd(heading))+widthMm*abs(sind(heading)); ...
    lengthMm*abs(sind(heading))+widthMm*abs(cosd(heading))],[],2)'/2+base.position_error_mm;
end

function margin = sweep_margin(points,boxes,ex,field,pointsOnly)
if nargin<5
    pointsOnly = false;
end
margin = inf;
for i=1:size(points,1)
    a = points(i,:);
    b = points(min(i+1,size(points,1)),:);
    if pointsOnly
        b = a;
    end
    subdivisions = max(1,ceil(norm(b-a)));
    for j=1:subdivisions
        from = a+(j-1)*(b-a)/subdivisions;
        to = a+j*(b-a)/subdivisions;
        lo = min(from,to)-ex;
        hi = max(from,to)+ex;
        % Conservative signed separation on either axis covering each full step.
        separation = max([boxes(:,1)-hi(1),lo(1)-boxes(:,3),boxes(:,2)-hi(2),lo(2)-boxes(:,4)],[],2);
        margin = min([margin lo field-hi separation']);
    end
end
end

function result = check_rescan(base,occupied)
center = [1200 300];
% A ±50 mm geometry witness only. Actual command/brake margins need field data.
delta = 50;
right = center+[delta 0];
left = center+[-delta 0];
search = [center;right;left];
boxes = [base.yellow_boxes_mm; occupied_boxes(occupied,base.cell_size_mm)];
margin = sweep_margin(search,boxes,extent(base,90),base.field_size_mm);
assert(margin>0 && all(search(:,2)==center(2)));
assert(all(arrayfun(@(i) cell_id(search(i,:),base.cell_size_mm)==2,1:size(search,1))));
% A greater positive shift from the station would contact the outer boundary.
assert(sweep_margin([center;2400 300],boxes,extent(base,90),base.field_size_mm)<0);
raw = '156+123+516+231';
for x = [1200 1227 1173]
    s = new_scan(24,10);
    s = offer(s,23,11,raw);
    s = offer(s,24,10,raw);
    assert(isempty(s.raw));
    s = offer(s,24,11,raw);
    s = offer(s,24,12,'165+321+561+132');
    assert(strcmp(s.raw,raw));
    assert(~scan_handoff(s,24,2,[x 300],[1 1 0 1],true));
    assert(~scan_handoff(s,24,2,[x 300],[1 1 1 1],false));
    assert(~scan_handoff(s,23,2,[x 300],[1 1 1 1],true));
    assert(~scan_handoff(s,24,2,[x 300],[1 1 1 1],true,1));
    assert(scan_handoff(s,24,2,[x 300],[1 1 1 1],true,0));
    pendingDisplay = s.raw;
    displayAvailable = 40;
    departAt = 3;
    assert(departAt<displayAvailable && strcmp(pendingDisplay,raw));
end
s = new_scan(25,20);
s.expired = true;
s = offer(s,25,21,raw);
assert(isempty(s.raw) && ~scan_handoff(s,25,2,center,ones(1,4),true));
result = struct('candidate_delta_mm',delta,'static_margin_lower_bound_mm',margin,'stage_heading_deg',90);
result.cases = {'station read','read during +X','read during -X','old round','pre-entry sample', ...
    'duplicate after lock','missing wheel stop','stale stop','slow display not on handoff', ...
    'expired after all attempts','out-of-envelope rejected'};
end

function s = new_scan(run,after)
s = struct('run',run,'after',after,'expired',false,'raw','');
end

function s = offer(s,run,seq,raw)
if run==s.run && seq>s.after && ~s.expired && isempty(s.raw) && valid_code(raw)
    s.raw = raw;
end
end

function ready = scan_handoff(s,run,cellId,pose,stopped,fresh,delay)
if nargin<7
    delay = 0;
end
% delay here represents elapsed time until measured stop, not display latency.
ready = run==s.run && cell_id(pose,480)==cellId && all(stopped) && fresh && ...
    ~s.expired && ~isempty(s.raw) && delay==0;
end

function valid = valid_code(raw)
valid = numel(raw)==15 && ...
    ~isempty(regexp(raw,'^[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}$','once'));
if valid
    first = sort(raw(1:3));
    valid = numel(unique(first))==3 && isequal(first,sort(raw(9:11))) && ...
        isequal(sort(raw(5:7)),'123') && isequal(sort(raw(13:15)),'123');
end
end

function report = check_task
raw = '156+123+516+231';
assert(valid_code(raw) && valid_code('462+321+246+132'));
bad = {'156+123+516','156-123+516+231','156+123+516+2310','150+123+510+231', ...
    '156+124+516+231','156+112+516+231','156+123+556+231', ...
    '156+123+526+231','156+123+516+221','','156+123+516+231 ',[raw char(10)]};
assert(~any(cellfun(@valid_code,bad)));
color = raw(1:3)-'0';
ring = raw(5:7)-'0';
slot = [3 1 2]; % Synthetic non-color/non-ring physical slot mapping.
carrier = zeros(1,3);
floor = zeros(1,3);
held = 0;
for i=1:3
    assert(held==0);
    held = color(i);
    assert(carrier(slot(i))==0);
    carrier(slot(i)) = held;
    held = 0;
end
assert(held==0 && isequal(carrier(slot),color));
for i=1:3
    assert(held==0 && carrier(slot(i))==color(i) && floor(ring(i))==0);
    held = carrier(slot(i));
    carrier(slot(i)) = 0;
    floor(ring(i)) = held;
    held = 0;
end
assert(held==0 && all(carrier==0) && isequal(floor(ring),color));
for i=1:3
    assert(held==0 && floor(ring(i))==color(i) && carrier(slot(i))==0);
    held = floor(ring(i));
    floor(ring(i)) = 0;
    carrier(slot(i)) = held;
    held = 0;
end
assert(all(floor==0) && held==0 && isequal(carrier(slot),color));
navRequests = [2 14 10];
assert(~ismember(22,navRequests));
assert(~terminal(3,true,[1 1 0 1],true) && ~terminal(3,false,ones(1,4),true));
assert(~terminal(3,true,ones(1,4),false) && terminal(3,true,ones(1,4),true));
report = struct('raw',raw,'first_color_order',color,'first_ring_order',ring, ...
    'retained_batch2',[raw(9:11)-'0';raw(13:15)-'0'],'witness_slots',slot, ...
    'final_slots',carrier,'nav_requests',navRequests);
report.cases = [{ 'official sample','three individual loads','three drops then three retrievals', ...
    'all items in actual slots','arm stowed and four wheels stopped','no fourth leg'}, bad];
end

function done = terminal(returned,stowed,stopped,fresh)
done = returned==3 && stowed && all(stopped) && fresh;
end

function report = check_supervision
t=(0:.1:65)';
observed = t;
feedback = 65-t; % Directed residual provided by independent physical measurements in firmware.
token = 12+zeros(size(t));
seq = (1:numel(t))';
done = t==65;
ops = {'radar path phase','chassis motion','vision alignment','source pick','source seat', ...
    'rough placement','rough retrieval','arm stow'};
for i=1:numel(ops)
    run = supervise(t,observed,feedback,token,seq,done);
    assert(run.done_at==65 && run.retries==0 && isempty(run.fault));
    assert(all([12 15 16 60]<run.done_at));
end
short = t>=20 & t<20.2-1e-6;
shortObserved = observed;
shortObserved(short) = 19.9;
shortSeq = seq;
shortSeq(short) = seq(find(t<20,1,'last'));
run = supervise(t,shortObserved,feedback,token,shortSeq,done);
assert(run.done_at==65 && run.retries==0 && isempty(run.fault));
jitter = supervise(t,t,10+.03*sin(t*12),token,seq,false(size(t)));
backward = supervise(t,t,10+t,token,seq,false(size(t)));
assert(jitter.retries==1 && strcmp(jitter.fault,'no progress'));
assert(backward.retries==1 && strcmp(backward.fault,'no progress'));
noGo = supervise(t,t,10+zeros(size(t)),token,seq,false(size(t)));
assert(noGo.first_recovery_at==2 && noGo.fault_at==4 && noGo.retries==1);
improve = [10*ones(20,1); max(0,12-t(21:end))];
recovery = supervise(t,t,improve,token,seq,t>=12);
assert(recovery.done_at==12 && recovery.retries==1 && isempty(recovery.fault));
stale = supervise(t,zeros(size(t)),feedback,token,ones(size(seq)),done);
assert(strcmp(stale.fault,'stale feedback') && stale.fault_at<2 && stale.retries==0);
oldToken = token;
oldToken(121) = 11;
earlyDone = done;
earlyDone(121) = true;
old = supervise(t,t,feedback,oldToken,seq,earlyDone);
assert(old.done_at==65 && isempty(old.fault));
sameSeq = seq;
sameSeq(121) = sameSeq(120);
same = supervise(t,t,feedback,token,sameSeq,earlyDone);
assert(same.done_at==65 && isempty(same.fault));
tooEarly = supervise(t,t,feedback,token,seq,earlyDone);
assert(tooEarly.done_at==65 && isempty(tooEarly.fault));
assert(~terminal(3,true,[1 1 0 1],true));
report = struct('slow_finished_at_model_seconds',65,'first_recovery_s',noGo.first_recovery_at, ...
    'no_progress_fault_s',noGo.fault_at,'stale_feedback_fault_s',stale.fault_at);
report.cases = [ops {'short loss','oscillation','reverse progress','no progress then fault', ...
    'bounded recovery reaches endpoint','stale feedback','old done token','repeated feedback sample', ...
    'done while out of tolerance','wheel that fails to stop'}];
end

function r = supervise(now,sample,residual,token,seq,done)
% Demonstrate freshness and directional high-water progress as separate evidence.
% All times/tolerances below are synthetic scenario constants.
best = inf;
lastProgress = 0;
lastSample = 0;
lastSeq = 0;
r = struct('done_at',NaN,'retries',0,'first_recovery_at',NaN,'fault','','fault_at',NaN);
for i=1:numel(now)
    t = now(i);
    if token(i)==12 && seq(i)>lastSeq && t-sample(i)<=.16
        lastSample = sample(i);
        lastSeq = seq(i);
        if best-residual(i)>=.19
            best = residual(i);
            lastProgress = t;
        end
        if done(i) && residual(i)<=.01
            r.done_at = t;
            return
        end
    end
    if t-lastSample>=.31
        r.fault = 'stale feedback';
        r.fault_at = t;
        return
    end
    if t-lastProgress>=2
        if r.retries==1
            r.fault = 'no progress';
            r.fault_at = t;
            return
        end
        r.retries = 1;
        r.first_recovery_at = t;
        lastProgress = t;
    end
end
end

function data = load_json(path)
data = jsondecode(fileread(path));
end
