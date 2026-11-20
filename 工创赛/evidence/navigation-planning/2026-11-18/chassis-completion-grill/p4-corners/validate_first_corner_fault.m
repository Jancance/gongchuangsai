function result=validate_first_corner_fault(p4file,jointfile,p3file,output,tail_s,wheel_error_rpm)
% Continuous containment after partial F6 during the two first-leg blends.
if nargin<5 || isempty(tail_s), tail_s=[0 .1 .25 .5 .75 1]; end
if nargin<6 || isempty(wheel_error_rpm), wheel_error_rpm=[0 .5 1 2]; end
assert(isvector(tail_s) && all(isfinite(tail_s) & tail_s>=0) && ...
    isvector(wheel_error_rpm) && all(isfinite(wheel_error_rpm) & wheel_error_rpm>=0));
p4=jsondecode(fileread(p4file)); j=jsondecode(fileread(jointfile));
p3=jsondecode(fileread(p3file)); L=j.assumptions; leg=p4.routes(1);
L.half=L.half(:)';
assert(p4.passed && p3.passed && p3.field_size_mm==2400);
assert(isequal(leg.points_mm,[150 150;240 240;1200 240;1200 300]));
assert(isequal(leg.points_mm,p3.legs(1).points_mm));
assert(isequal(p3.body_mm(:),2*L.half(:)) && all([j.rows(1:2).accepted]));

% Time from the start of the affected batch to the first STOP request is
% an offline timing assumption. The 500 ms is an ACK/cut-VM request limit.
% Physical command application, relay reaction and coast belong in the tail.
detect=L.H+L.late; stop_software=.500;
boxes=p3.yellow_boxes_mm;
trace=validate_first_leg_continuous(p4file,p3file,jointfile);
[a,b,c,d]=ndgrid(1:3);
states=[a(:) b(:) c(:) d(:)];
rows=struct([]);
for i=1:2
    curve=leg.corners(i); cert=j.rows(i);
    assert(isequal(curve.center_mm,cert.center_mm) && curve.heading_deg==90);
    bounds=prepare(curve,cert,L,states,trace,i);
    for error_idx=1:numel(wheel_error_rpm)
        err=wheel_error_rpm(error_idx);
        for tail=tail_s(:)'
            [gap,field,occupied,exit,phase]=sweep(bounds,L,p3.field_size_mm,...
                boxes,leg.points_mm(i+2,:),detect+stop_software+tail,err);
            rows=[rows;struct('id',cert.id,'tail_s',tail,'wheel_error_rpm',err,...
                'clearance_mm',gap,'field_margin_mm',field,'occupied_margin_mm',occupied,...
                'next_waypoint_margin_mm',exit,'worst_phase',phase)]; %#ok<AGROW>
            fprintf('%s error=%.2f rpm tail=%.3f s clearance=%.6f mm phase=%d\n',...
                cert.id,err,tail,gap,phase);
        end
    end
    [gap0,field0,occupied0,~,phase0]=sweep(bounds,L,p3.field_size_mm,...
        boxes,leg.points_mm(i+2,:),detect+stop_software,0);
    obstructed=sweep(bounds,L,p3.field_size_mm,[boxes;100 100 1300 300],...
        leg.points_mm(i+2,:),detect+stop_software,0);
    assert(gap0>0 && obstructed<=0 && size(states,1)==81);
    for error_idx=1:numel(wheel_error_rpm)
        err=wheel_error_rpm(error_idx);
        lo=0; hi=5;
        low=sweep(bounds,L,p3.field_size_mm,boxes,leg.points_mm(i+2,:),...
            detect+stop_software,err);
        high=sweep(bounds,L,p3.field_size_mm,boxes,leg.points_mm(i+2,:),...
            detect+stop_software+hi,err);
        if low<=0
            lo=NaN; hi=0;
        elseif high>0
            lo=hi; hi=NaN;
        else
            for step=1:24
                mid=(lo+hi)/2;
                m=sweep(bounds,L,p3.field_size_mm,boxes,leg.points_mm(i+2,:),...
                    detect+stop_software+mid,err);
                if m>0, lo=mid; else, hi=mid; end
            end
        end
        fprintf('%s error=%.2f rpm tail_limit_s bracket [%.6f, %.6f]\n',...
            cert.id,err,lo,hi);
        limits(i,error_idx)=struct( ...
            'id',cert.id,'wheel_error_rpm',err,'positive_tail_s',lo,...
            'nonpositive_tail_s',hi); %#ok<AGROW>
    end
    nominal(i)=struct('id',cert.id,'phase_count',size(bounds.start_lo,1),...
        'minimum_clearance_mm',gap0,'field_margin_mm',field0,...
        'occupied_margin_mm',occupied0,'worst_phase',phase0,...
        'obstructed_control_mm',obstructed); %#ok<AGROW>
end
result=struct('version',version,'path_mm',leg.points_mm,'yellow_boxes_mm',boxes,...
    'full_leg_commands_used',true,...
    'physical_tail_is_measured',false,'batch_start_to_stop_request_assumption_s',detect,...
    'stop_software_s',stop_software,'previous_next_zero_states',size(states,1),...
    'zero_tail_zero_error',nominal,'rows',rows,'limits',limits);
result.full_leg_uniform_budget_passed=trace.uniform_budget_passed;
result.full_leg_startup_conditioned_model_passed=trace.startup_conditioned_model_passed;
f=fopen(output,'w'); assert(f>=0);
fprintf(f,'%s\n',jsonencode(result)); fclose(f);
fprintf('FIRST_CORNER_FAULT_COMPLETED %d rows\n',numel(rows));
end

function C=prepare(c,cert,L,states,trace,corner)
P=c.control_points_mm; n=cert.batches; H=L.H;
assert(all(all(diff(P)>=0)));
t=(-1:n+1)'*H;
xy=position(P,n*H,t);
R=[0 -1;1 0]; W=[1 1;1 -1;1 -1;1 1];
rpm=zeros(n+2,4);
for w=1:4
    slot=t(1:end-1)+(w-1)*.020;
    assert(abs(trace.edges_s(2*corner+1)-trace.edges_s(2*corner)-n*H)<1e-8);
    slot=slot+trace.edges_s(2*corner);
    idx=find(trace.wheel==w);
    [distance,nearest]=min(abs(slot-trace.slot_ms(idx)'/1000),[],2);
    assert(max(distance)<1e-8);
    rpm(:,w)=trace.command_rpm(idx(nearest));
end
C.start_lo=xy(2:end-1,:); C.start_hi=xy(3:end,:);
C.exit_direction=5*(P(end,:)-P(end-1,:));
C.exit_direction=C.exit_direction/norm(C.exit_direction);
C.phase=(0:n)';
% The last interval extends onto the next straight with a 90 mm/s^2 cap.
C.start_hi(end,:)=C.start_hi(end,:)+.5*90*H^2*C.exit_direction;
C.position_mm=L.pos+L.track+max(cert.position_error_mm,...
    max([trace.normal_patterns.max_reference_offset_mm]));
C.angle_rad=max(deg2rad(cert.yaw_error_deg),L.yaw+deg2rad(max([trace.normal_patterns.max_heading_error_deg])));
for k=1:n+1
    choices=[rpm(k,:);rpm(k+1,:);zeros(1,4)];
    w=L.K*choices(sub2ind([3 4],states,repmat(1:4,size(states,1),1)));
    body=w*W/4;
    velocity=body*R';
    omega=w*[-1;1;-1;1]/(4*L.lever);
    C.vel_lo(k,:)=min(velocity,[],1);
    C.vel_hi(k,:)=max(velocity,[],1);
    C.speed(k,1)=max(vecnorm(body,2,2));
    C.omega(k,1)=max(abs(omega));
end
end

function xy=position(P,T,t)
u=max(0,min(1,t/T));
xy=zeros(numel(t),2);
for q=0:5
    xy=xy+(nchoosek(5,q)*u.^q.*(1-u).^(5-q))*P(q+1,:);
end
xy=xy+min(t/T,0)*5*(P(2,:)-P(1,:))+max(t/T-1,0)*5*(P(end,:)-P(end-1,:));
end

function [gap,field,occupied,exit,phase]=sweep(C,L,field_size,boxes,next,horizon,err)
% The wheel box admits arbitrary switching, asynchronous zeroing and
% independent tracking errors throughout the horizon. Mecanum translation
% error from a per-wheel error <= K*err is bounded by K*err in 2-norm.
v_err=L.K*err;
omega=C.omega+v_err/L.lever;
angle=C.angle_rad+omega*horizon;
% Translational rotation error <= |angle|*speed. Integrating its linear
% upper bound covers every instant between command/SW/physical transitions.
rot=C.speed.*(horizon*C.angle_rad+.5*horizon^2*omega);
lo=C.start_lo+horizon*min(0,C.vel_lo)-rot-horizon*v_err-C.position_mm;
hi=C.start_hi+horizon*max(0,C.vel_hi)+rot+horizon*v_err+C.position_mm;
% Nominal heading is 90 degrees. |cos(theta)| <= 1 and |sin(theta)| <= |theta|.
half=fliplr(L.half)+min(1,angle)*L.half;
field_by_phase=min([lo-half,field_size-hi-half],[],2);
obstacle_by_phase=inf(size(field_by_phase));
for k=1:size(boxes,1)
    separation=max([boxes(k,1:2)-hi-half,lo-half-boxes(k,3:4)],[],2);
    obstacle_by_phase=min(obstacle_by_phase,separation);
end
field=min(field_by_phase); occupied=min(obstacle_by_phase);
[gap,idx]=min(min(field_by_phase,obstacle_by_phase));
phase=C.phase(idx);
exit=min((next-hi)*C.exit_direction');
end
