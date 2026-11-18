function report=validate_zone2_turn_timing(output)
% Pure 90-degree pivot from the eight-leg reference. All times are model-only.
folder=fileparts(mfilename('fullpath'));
ref=jsondecode(fileread(fullfile(folder,'eight_leg_reference_250.json')));
L=ref.assumptions;
assert(ref.passed);
turns=struct([]);
for i=1:numel(ref.legs)
    spans=ref.legs(i).spans;
    pivots=spans(strcmp({spans.kind},'turn'));
    turns=[turns;pivots(:)]; %#ok<AGROW>
end
assert(numel(turns)==8 && all([turns.sec]==3.2));
assert(all(arrayfun(@(s) abs(s.profile(2))==90,turns)));
centers=reshape([turns.p],2,[])';
disk=norm(L.half);
angles=[-1;1;-1;1];
W=[1 1 -1;1 -1 1;1 -1 -1;1 1 1];
H=L.H;
dt=H-L.late;
% Same-sample feedback limits, with independently bounded wheel error.
yawCorrection=3;
translationCorrection=12;
wheelError=.5;
wheelYawUncertainty=rad2deg(L.K*wheelError/L.lever);
qreserve=sqrt(2)*translationCorrection/L.K+deg2rad(yawCorrection)*L.lever/L.K;
astep=(sqrt(2)*28+deg2rad(10)*L.lever)*H/L.K;
[s1,s2,s3,s4]=ndgrid(1:3);
states=[s1(:) s2(:) s3(:) s4(:)];
select=sub2ind([3 4],states,repmat(1:4,size(states,1),1));
tail=[0 .05 .10 .25 .50];
faultHorizon=H+L.late+.5+tail;
stopDelay=[0 .02 .05 .1 .2 .3 .5];
fastHorizon=H+L.late+stopDelay;
floorMargin=min(disk_margin(centers,disk+L.pos+L.track+.5,L));
assert(floorMargin>0);
ticks=[23:30 32 35 40];
for k=1:numel(ticks)
    T=H*ticks(k);
    t=(-2:ticks(k)+2)'*H+(0:3)*.020;
    rpm=round(10*deg2rad(pivot(t+H,T)-pivot(t,T)).*angles'*L.lever/(H*L.K))/10;
    change=abs(diff(rpm,1,1));
    curvature=abs(diff(rpm,2,1));
    % A correction step changes between batches by astep at most. Quantization
    % can add 0.2 RPM to first and 0.4 RPM to second differences relative to
    % the already quantized nominal commands.
    dq=change+astep+.2;
    rate=max(dq(:))/dt;
    d2q=curvature+2*astep+.4;
    jerk=d2q/dt^2+dq(1:end-1,:)*2*L.late/dt^3;
    jerk=max(jerk(:));
    speed=max(abs(rpm(:)))+qreserve+.1;
    yaw=90*1.875/T;
    yawA=90*(10/sqrt(3))/T^2;
    yawJ=90*60/T^3;
    yawHeadroom=90-yaw-yawCorrection-wheelYawUncertainty;
    horizons=[faultHorizon fastHorizon];
    faultMargin=inf(size(horizons));
    for i=2:size(rpm,1)
        commands=[rpm(i-1,:);rpm(i,:);zeros(1,4)];
        body=L.K*commands(select)*W/4;
        % Rotation leaves a full disk valid for all heading errors. Bound the
        % translation from asymmetric wheel states regardless of instantaneous yaw.
        mixedPeak=max(vecnorm(body(:,1:2),2,2));
        % Translation correction can also become asymmetric when some controls
        % are stopped. Bound the coupled correction and per-wheel speed error.
        uncertainV=mixedPeak+L.K*(qreserve+.1+wheelError);
        for j=1:numel(horizons)
            swept=disk+L.pos+L.track+.5+horizons(j)*uncertainV;
            faultMargin(j)=min(faultMargin(j),min(disk_margin(centers,swept,L)));
        end
    end
    savings=numel(turns)*(3.2-T);
    rows(k)=struct('turn_s',T,'yaw_deg_s',yaw,'yaw_a_deg_s2',yawA,...
        'yaw_j_deg_s3',yawJ,'yaw_headroom_with_uncertainty_deg_s',yawHeadroom,...
        'wheel_peak_bound_rpm',speed,'wheel_rate_bound_rpm_s',rate,...
        'wheel_jerk_bound_rpm_s2',jerk,'disk_margin_mm',floorMargin,...
        'partial_stop_disk_margin_mm',faultMargin(1:numel(tail)),...
        'zero_tail_stop_delay_disk_margin_mm',faultMargin(numel(tail)+1:end),'chain_saved_s',savings,...
        'elapsed_without_radar_rotations_s',175.990+6.16+2.5-savings,...
        'stale_yaw_motion_deg',[.020 .080 .160]*(yaw+yawCorrection+wheelYawUncertainty),...
        'passes_nominal_limits',yawHeadroom>=0 && yawA+10<=180 && yawJ<=1440 && ...
        speed<=300 && rate<=250 && jerk<=1400); %#ok<AGROW>
    fprintf('%.2fs  yaw+guard headroom %6.2f deg/s  wheel %5.1f/%5.1f/%6.1f  STOP bound %7.2f mm  elapsed %.3f s  normal pass %d\n',...
        T,yawHeadroom,speed,rate,jerk,faultMargin(1),rows(k).elapsed_without_radar_rotations_s,rows(k).passes_nominal_limits);
end
first=find([rows.passes_nominal_limits],1);
assert(~isempty(first));
report=struct('version',version,'model_only',true,'deployment_allowed',false,'pivot_count',numel(turns),...
    'position_budget_mm',L.pos,'tracking_budget_mm',L.track,'wheel_error_rpm',wheelError,...
    'yaw_correction_deg_s',yawCorrection,'translation_correction_mm_s',translationCorrection,...
    'fault_horizon_s',faultHorizon,'zero_tail_stop_delay_s',stopDelay,'stale_yaw_age_s',[.020 .080 .160],...
    'earliest_nominal_limit_pass_s',rows(first).turn_s,'rows',rows);
if nargin>0
    fid=fopen(output,'w');
    assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(report));
    fclose(fid);
end
end

function h=pivot(t,T)
u=max(0,min(1,t/T));
h=90*u.^3.*(10+u.*(6*u-15));
end

function m=disk_margin(p,r,L)
lo=p-r;
hi=p+r;
m=min([lo L.field-hi],[],2);
for b=L.box'
    d=max([b(1)-hi(:,1),b(2)-hi(:,2),lo(:,1)-b(3),lo(:,2)-b(4)],[],2);
    m=min(m,d);
end
end
