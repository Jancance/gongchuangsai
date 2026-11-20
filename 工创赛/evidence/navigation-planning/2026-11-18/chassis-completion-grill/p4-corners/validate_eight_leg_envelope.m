function report=validate_eight_leg_envelope(referenceFile,output)
% Conditional containment from each affected batch start in the dynamic model.
folder=fileparts(mfilename('fullpath'));
p3=jsondecode(fileread(fullfile(folder,'..','p3-path','p3_path_vectors.json')));
m=jsondecode(fileread(referenceFile));
assert(m.passed && numel(m.legs)==8);
L=m.assumptions;
L.half=L.half(:)';
[a,b,c,d]=ndgrid(1:3);
states=[a(:) b(:) c(:) d(:)];
choices=sub2ind([3 4],states,repmat(1:4,size(states,1),1));
W=[1 1 -1;1 -1 1;1 -1 -1;1 1 1];
tail=[0 .05 .1 .25 .5];
stopDelay=[0 .02 .05 .1 .2 .3 .5];
wheelError=[0 .5 1 2];
for i=1:8
    leg=m.legs(i);
    p=p3.legs(i).points_mm;
    last=leg.spans(end);
    if strcmp(last.kind,'line')
        p(end,:)=last.q(:)';
    else
        p(end,:)=last.p(:)';
    end
    h=p3.legs(i).headings;
    t=(0:round(leg.time_s/L.H))'*L.H;
    rpm=zeros(numel(t)+1,4);
    for k=1:4
        slot=t+.02*(k-1);
        [x,theta]=sample(leg,slot,h);
        [y,theta2]=sample(leg,slot+L.H,h);
        v=(y-x)/L.H;
        body=[v(:,1).*cos(theta)+v(:,2).*sin(theta),-v(:,1).*sin(theta)+v(:,2).*cos(theta)];
        rpm(2:end,k)=round(10*[body,L.lever*wrap(theta2-theta)/L.H]*W(k,:)'/L.K)/10;
    end
    assert(max(abs(rpm(:)))<=leg.wheel_peak_rpm+.11);
    normal=normal_replay(leg,t,p,h,rpm,L,W);
    [center,heading]=sample(leg,t,h);
    distance=min(vecnorm(center-p(1,:),2,2),vecnorm(center-p(end,:),2,2));
    nearField=any([center L.field-center]<160,2);
    edge=distance<L.edgeDistance & nearField;
    track=L.track*ones(size(t));
    track(edge)=L.trackEdge;
    margin=zeros(numel(t),numel(tail),numel(wheelError));
    delayMargin=zeros(numel(t),numel(stopDelay),numel(wheelError));
    heldBad=inf(size(t));
    for k=1:numel(t)
        rpm_set=[rpm(k,:);rpm(k+1,:);zeros(1,4)];
        body=(L.K*rpm_set(choices))*W/4;
        yaw=heading(k);
        world=body(:,1:2)*[cos(yaw) sin(yaw);-sin(yaw) cos(yaw)];
        speed=max(vecnorm(body(:,1:2),2,2));
        heldBad(k)=held_witness(center(k,:),yaw,rpm(k,:),L,W);
        for j=1:numel(wheelError)
            e=L.K*wheelError(j);
            omega=(max(abs(body(:,3)))+e)/L.lever;
            horizons=L.H+L.late+[.500+tail stopDelay];
            for q=1:numel(horizons)
                horizon=horizons(q);
                angle=L.yaw+omega*horizon;
                % Track and initial placement are independent error budgets.
                error=L.pos+track(k)+horizon*e+speed*(horizon*L.yaw+.5*horizon^2*omega);
                lo=center(k,:)+horizon*min(0,min(world,[],1))-error;
                hi=center(k,:)+horizon*max(0,max(world,[],1))+error;
                ext=extent(yaw,angle,L);
                gap=clearance(lo-ext,hi+ext,L);
                if q<=numel(tail)
                    margin(k,q,j)=gap;
                else
                    delayMargin(k,q-numel(tail),j)=gap;
                end
            end
        end
    end
    [g,idx]=min(margin(:,1,1));
    rows(i)=struct('leg',i,'batch_count',numel(t),'edge_batch_count',nnz(edge),...
        'nominal_open_loop',normal,'zero_tail_zero_error_margin_mm',g,'worst_batch_s',t(idx),...
        'margin_by_tail_error_mm',reshape(min(margin,[],1),numel(tail),numel(wheelError)),...
        'edge_margin_by_tail_error_mm',sensitivity(margin,edge,tail,wheelError),...
        'interior_margin_by_tail_error_mm',sensitivity(margin,~edge,tail,wheelError),...
        'margin_by_stop_delay_error_mm',sensitivity(delayMargin,true(size(t)),stopDelay,wheelError),...
        'edge_margin_by_stop_delay_error_mm',sensitivity(delayMargin,edge,stopDelay,wheelError),...
        'held_previous_nominal_sampled_margin_mm',min(heldBad),...
        'held_previous_nominal_bad_times_s',t(heldBad<=0),...
        'zero_tail_zero_error_bad_times_s',t(margin(:,1,1)<=0)); %#ok<AGROW>
    fprintf('Leg %d: sampled open-loop peaks %.3f mm / %.3f deg, stop margin %.3f mm at %.3f s\n',...
        i,max([normal.peak_position_mm]),max([normal.peak_yaw_deg]),g,t(idx));
end
assert(numel(rows)==8 && size(states,1)==81);
assert(all(isfinite([rows.zero_tail_zero_error_margin_mm])));
report=struct('version',version,'scenarios_completed',true,'deployment_allowed',false,...
    'previous_current_zero_combinations',size(states,1),'dispatch_fault_detect_s',L.H+L.late,...
    'stop_request_to_cut_assumption_s',.500,'physical_tail_s',tail,'wheel_error_rpm',wheelError,...
    'zero_tail_stop_delay_sensitivity_s',stopDelay,...
    'all_batches_nominally_contained',all([rows.zero_tail_zero_error_margin_mm]>0),'legs',rows);
if nargin>1
    f=fopen(output,'w');
    assert(f>=0);
    fprintf(f,'%s\n',jsonencode(report));
    fclose(f);
end
end

function data=sensitivity(margin,mask,tail,wheelError)
data=[];
if any(mask), data=reshape(min(margin(mask,:,:),[],1),numel(tail),numel(wheelError)); end
end

function g=held_witness(center,heading,rpm,L,W)
% Nominal pose and one definite execution trace, with no STOP applied until
% the 500 ms software deadline after the affected batch. A negative sampled
% margin here is independent of the arbitrary-mixture approximation.
body=(rpm*W).*([L.K L.K L.K/L.lever]/4);
g=inf;
for t=linspace(0,L.H+L.late+.500,12)
    dr=t*body(3);
    mid=heading+dr/2;
    scale=1;
    if abs(dr)>1e-10, scale=sin(dr/2)/(dr/2); end
    pos=center+t*scale*body(1:2)*[cos(mid) sin(mid);-sin(mid) cos(mid)];
    ex=extent(heading+dr,0,L);
    % Within-field containment is exact for the rectangle's support.
    g=min(g,min([pos-ex L.field-pos-ex]));
end
end

function out=normal_replay(leg,t,p,h,rpm,L,W)
% Two illustrative latencies. Exact constant-body integration between events.
for j=0:1
    [base,k]=ndgrid(t,0:3);
    events=base+.02*k+j*L.late*mod(round(base/L.H)+k,2);
    batch=repmat((2:numel(t)+1)',1,4);
    [events,order]=sort(events(:));
    k=k(:)+1;
    k=k(order);
    batch=batch(order);
    ref=(0:.005:events(end))';
    [times,order]=sort([events;ref]);
    eventCount=numel(events);
    [position,yaw]=sample(leg,ref,h);
    pose=p(1,:);
    heading=deg2rad(h.start);
    held=zeros(1,4);
    last=0;
    positionError=zeros(numel(ref),1);
    yawError=positionError;
    for n=1:numel(times)
        dt=times(n)-last;
        body=(held*W).*([L.K L.K L.K/L.lever]/4);
        dr=body(3)*dt;
        mid=heading+dr/2;
        scale=1;
        if abs(dr)>1e-10, scale=sin(dr/2)/(dr/2); end
        pose=pose+dt*scale*body(1:2)*[cos(mid) sin(mid);-sin(mid) cos(mid)];
        heading=heading+dr;
        last=times(n);
        id=order(n);
        if id<=eventCount
            held(k(id))=rpm(batch(id),k(id));
        else
            r=id-eventCount;
            positionError(r)=norm(pose-position(r,:));
            yawError(r)=abs(wrap(heading-yaw(r)));
        end
    end
    distance=min(vecnorm(position-p(1,:),2,2),vecnorm(position-p(end,:),2,2));
    edge=distance<L.edgeDistance & any([position L.field-position]<160,2);
    out(j+1)=struct('latency_pattern',j,'peak_position_mm',max(positionError),...
        'peak_yaw_deg',rad2deg(max(yawError)),'peak_edge_position_mm',max(positionError(edge)),...
        'final_position_mm',positionError(end)); %#ok<AGROW>
end
end

function [p,h]=sample(leg,time,heading)
time=time(:);
start=leg.spans(1).p;
p=repmat(start(:)',numel(time),1);
h=repmat(deg2rad(heading.start),numel(time),1);
current=deg2rad(heading.start);
for i=1:numel(leg.spans)
    s=leg.spans(i);
    mask=time>=s.at-1e-10 & time<s.at+s.sec-1e-10;
    t=max(0,time(mask)-s.at);
    h(mask)=current;
    switch s.kind
        case 'line'
            from=s.p(:)';
            to=s.q(:)';
            a=s.profile(1); f=s.profile(2); b=s.profile(3);
            v0=s.profile(4); v=s.profile(5); v1=s.profile(6);
            x=min(1,t/a); y=max(0,min(1,(t-a-f)/b));
            d=v0*a*x+(v-v0)*a*(x.^3-.5*x.^4)+v*max(0,min(f,t-a))...
                +v*b*y+(v1-v)*b*(y.^3-.5*y.^4);
            p(mask,:)=from+d*((to-from)/norm(to-from));
        case 'blend'
            u=t/s.sec; v=1-u;
            p(mask,:)=[v.^5 5*u.*v.^4 10*u.^2.*v.^3 10*u.^3.*v.^2 5*u.^4.*v u.^5]*s.p;
        case {'stop','turn'}
            p(mask,:)=repmat(s.p(:)',numel(t),1);
            if strcmp(s.kind,'turn')
                u=t/s.sec;
                h(mask)=deg2rad(s.profile(1)+s.profile(2)*u.^3.*(10-15*u+6*u.^2));
                current=deg2rad(sum(s.profile));
            end
    end
end
after=time>=leg.time_s-1e-10;
last=leg.spans(end);
point=last.p;
if strcmp(last.kind,'line'), point=last.q; end
p(after,:)=repmat(point(:)',nnz(after),1);
h(after)=deg2rad(heading.terminal);
end

function angle=wrap(angle)
angle=mod(angle+pi,2*pi)-pi;
end

function ex=extent(heading,budget,L)
v=[1 1;1 -1;-1 1;-1 -1].*L.half;
p=v*[cos(heading) sin(heading);-sin(heading) cos(heading)];
d=[-p(:,2) p(:,1)];
ex=max([p*cos(budget)-d*sin(budget);p*cos(budget)+d*sin(budget)],[],1);
ex(any(abs(atan2(d,p))<=budget,1))=norm(L.half);
end

function d=clearance(lo,hi,L)
b=L.box;
d=min([lo L.field-hi max([b(:,1)-hi(1),b(:,2)-hi(2),lo(1)-b(:,3),lo(2)-b(:,4)],[],2)']);
end
