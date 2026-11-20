function validate_joint_corners(p4file,p3file,output)
% Fixed-heading feedforward corner qualification in wheel displacement units.
f=jsondecode(fileread(p4file)); p3=jsondecode(fileread(p3file));
assert(f.passed && f.accepted_windows==25 && p3.passed);
car=struct('H',.080,'late',.006,'age',.160,'phase',.060,'K',77*pi/60,...
    'lever',209,'half',[141.5 127],'pos',5,'track',2,'yaw',pi/180);
yellow=p3.yellow_boxes_mm;
rows=struct([]); blocked={};
for leg=1:8
    c=f.routes(leg).corners;
    for i=1:numel(c)
        if strcmp(c(i).reason,'ok')
            rows=[rows; qualify(c(i),sprintf('leg%d.%d',leg,i),yellow,car)]; %#ok<AGROW>
        else
            blocked{end+1}=sprintf('leg%d.%d:%s',leg,i,c(i).reason); %#ok<AGROW>
        end
    end
end
assert(numel(rows)==9 && isequal(blocked,{'leg6.1:too_short'}));
groups={f.outer_both_directions,f.occupied_detour,f.other_angles,f.adjacent_accepted};
boxes={yellow,[yellow; 960 960 1440 1440],zeros(0,4),zeros(0,4)};
names={'outer','occupied','angle','adjacent'};
assert(p3.detour.occupied_ids==12);
for g=1:4
    cases=groups{g};
    for i=1:numel(cases)
        rows=[rows;qualify(cases(i),sprintf('%s.%d',names{g},i),boxes{g},car)]; %#ok<AGROW>
    end
end
assert(numel(rows)==25 && numel(unique({rows.id}))==25);
for k=1:25
    r=rows(k);
    fprintf('%-12s %-20s n=%d brake=%d speed=%.3f sweep=%.3f exit=%.3f\n',...
        r.id,r.result,r.batches,r.brake_batches,r.min_speed_mm_s,r.sweep_mm,r.exit_mm);
    if r.accepted
        assert(r.sweep_mm>0 && r.exit_mm>0 && r.min_speed_mm_s>=12);
        assert(r.rate_rpm_s<=250 && r.rate_change_rpm_s2<=1400 && r.raw_max<=3000);
    end
end
% Refusal controls: all four separation axes overlap for a box around the corner.
c=f.routes(4).corners; overlap=[900 900 1500 1500];
adverse=qualify(c,'box-over-path',overlap,car); assert(~adverse.accepted);
c.available_post_mm=1; adverse=qualify(c,'short-exit',yellow,car);
assert(~adverse.accepted && adverse.exit_mm<0);
% Parent-to-leaf interval bounds must reject a footprint tangent to the field edge.
assert(clearance([10 100;10 200],[10 10],zeros(0,4),3)==0);
result=struct('version',version,'assumptions',car,'original',25,'field_original',9,...
    'field_model_qualified',nnz([rows(1:9).accepted]),'all_model_qualified',nnz([rows.accepted]),...
    'preexisting_refusals',{blocked},'rows',{rows});
file=fopen(output,'w'); assert(file>=0); fprintf(file,'%s\n',jsonencode(result)); fclose(file);
fprintf('JOINT_CORNERS_COMPLETED field=%d/9 total=%d/25\n',result.field_model_qualified,result.all_model_qualified);
end

function out=qualify(c,id,boxes,L)
H=L.H; K=L.K; P=c.control_points_mm; D=5*diff(P); h=c.heading_deg;
R=[cosd(h) -sind(h);sind(h) cosd(h)];
W=[1 1;1 -1;1 -1;1 1]/K;
u=D(1,:)/norm(D(1,:)); v=D(end,:)/norm(D(end,:));
% Oldest qualified observation, next batch, then overlap of previously
% planned wheel intervals. Application jitter is accounted in the ZOH bound.
reaction=L.age+H+L.phase;
old=c.nominal_batch_count;
out=struct('id',id,'center_mm',c.center_mm,'accepted',false,'result','unqualified',...
    'batches',0,'brake_batches',0,'min_speed_mm_s',0,'sweep_mm',0,'exit_mm',0,...
    'position_error_mm',0,'yaw_error_deg',0,'wheel_distance_error_mm',0,...
    'rate_rpm_s',0,'rate_change_rpm_s2',0,'raw_max',0,'endpoint_reach_mm',0);
best=-inf;
for n=old:floor(old*c.min_speed_mm_s/12)
    T=n*H; s=old/n; V=c.max_speed_bound_mm_s*s;
    A=c.acc_bound_mm_s2*s^2; J=c.jerk_bound_mm_s3*s^3;
    w=c.wheel_bounds(1)*s; wa=c.wheel_bounds(2)*s^2; wj=c.wheel_bounds(3)*s^3;
    join=c.join_speed_mm_s*s;
    % Euclidean recovered body speed error <= maximum wheel surface speed
    % error for this orthogonal mecanum map. Includes integer observation.
    speed=c.min_speed_mm_s*s-K*(wa*(H/2+L.late)+.55);
    for b=1:40
        B=b*H;
        abr=A+1.5*V/B; jbr=J+4.5*A/B+6*V/B^2;
        wabr=wa+1.5*w/B; wjbr=wj+4.5*wa/B+6*w/B^2;
        if abr>130 || jbr>260 || wabr>220 || wjbr>1240, continue; end
        minGap=H-L.late;
        wrate=(wabr*H+.1)/minGap;
        wjerk=((wjbr*H^2+.2)/minGap+(wabr*H+.1)*2*L.late/minGap^2)/minGap;
        if wrate>250 || wjerk>1400, continue; end
        reach=join*(reaction+B/2);
        duration=H+T+reaction+B+H+L.late;
        % Uniform ideal-grid partial prefix, command quantization, jitter.
        % The Bezier velocity-control polygon bounds per-wheel total variation.
        % Multiplying by a monotone 1->0 brake adds at most the peak wheel speed.
        variation=max(sum(abs(diff(D*R*W'/T)),1))+w;
        transitions=ceil(duration/H)+2;
        dist=K*(wabr*H^2/8+.05*duration+L.late*(variation+.1*transitions));
        angle=L.yaw+dist/L.lever;
        % Include the pose-to-world orientation error for the full travel,
        % in addition to rotation of the rectangular chassis footprint.
        length=H*join+c.length_upper_mm+reach;
        travel=length+K*(wabr*(H/2+L.late)+.05)*duration;
        position=dist+angle*travel;
        ex=abs(R)*L.half'; ex=ex'+L.pos+L.track+position+norm(L.half)*angle;
        sweep=min([clearance(P,ex,boxes,7),...
            clearance([P(1,:)-H*join*u;P(1,:)],ex,boxes,0),...
            clearance([P(end,:);P(end,:)+reach*v],ex,boxes,0)]);
        exit=c.available_post_mm-reach-position-L.track-L.pos*sum(abs(v));
        merit=min([sweep,exit,(speed-12)*H]);
        if merit>best
            best=merit; out.batches=n; out.brake_batches=b; out.min_speed_mm_s=speed;
            out.sweep_mm=sweep; out.exit_mm=exit; out.position_error_mm=position;
            out.yaw_error_deg=angle*180/pi; out.wheel_distance_error_mm=dist;
            out.rate_rpm_s=wrate; out.rate_change_rpm_s2=wjerk;
            out.endpoint_reach_mm=reach; out.raw_max=ceil(10*w);
        end
        if sweep>0 && exit>0 && speed>=12
            out.accepted=true; out.result='model_qualified'; return;
        end
        % Longer brakes add travel and quantization time, but reduce the
        % derivative bound; keep the search open across both tradeoffs.
    end
end
if out.batches==0, out.result='derivatives'; 
elseif out.sweep_mm<=0, out.result='sweep_bound';
elseif out.exit_mm<=0, out.result='exit_bound';
else, out.result='speed_bound';
end
end

function m=clearance(P,ex,boxes,depth)
lo=min(P,[],1)-ex; hi=max(P,[],1)+ex;
g=max([boxes(:,1)-hi(1),boxes(:,2)-hi(2),lo(1)-boxes(:,3),lo(2)-boxes(:,4)],[],2);
m=min([lo 2400-hi g']);
if m>0 || depth==0, return; end
left=P; right=P; Q=P;
for k=1:size(P,1)
    left(k,:)=Q(1,:); right(end-k+1,:)=Q(end,:);
    Q=(Q(1:end-1,:)+Q(2:end,:))/2;
end
m=min(clearance(left,ex,boxes,depth-1),clearance(right,ex,boxes,depth-1));
end
