function result = validate_first_leg_feedback(p4file, p3file, certificate, output)
% Same-time pose feedback on the unchanged first-leg reference.
[nominal,reference] = validate_first_leg_continuous(p4file,p3file,certificate);
p3 = jsondecode(fileread(p3file));
cert = jsondecode(fileread(certificate));
L = cert.assumptions;
D = struct('position_gain_hz',1.4,'position_cap_mm_s',12,'position_slew_mm_s2',28,...
    'yaw_gain_hz',2.2,'yaw_cap_deg_s',3,'yaw_slew_deg_s2',10,...
    'terminal_error_mm',2.5,'terminal_yaw_deg',.10,'terminal_max_s',2,...
    'terminal_zero_hold_s',.24,'sample_age_ms',160,'prepare_lead_ms',10);
% Launch yaw, source period/lag, missed frame, wheel delay pattern,
% constant map-position and heading biases, sustained outage onset.
cases = [...
    0   50  10  0  0    0   0    0    inf
    1   60  20 12  1    0   0    0    inf
   -1   60  20 12  1    0   0    0    inf
    1  110  30  0  1    0   0    0    inf
   -1  110  30  0  2    0   0    0    inf
    1   60  20 12  2   .6 -.8  .05   inf
   -1   60  20 12  2  -.6  .8 -.05   inf
    1  110  30  5  1    0   0    0    inf
   -1   60  20  0  2    0   0    0   7000
    1   60  20  0  1    0   0    0  16660
    0   60  20  0  1    0   0    0      0
    0   60  20  0  1    0   0    0      1
    0   60  20  0  1    0   0    0    inf];
terminalBias = [zeros(size(cases,1)-1,1);-40];
result = struct('version',version,'constants',D,'columns',...
    {{'initial_yaw_deg','sample_period_ms','sample_lag_ms','missed_frame','late_pattern',...
      'bias_x_mm','bias_y_mm','bias_yaw_deg','outage_ms'}},'inputs',cases,'scenarios',[],...
    'injected_terminal_y_bias_mm',terminalBias,'stop_model','synthetic phase-held wheel zero',...
    'frozen_map_only',true,'continuous_latency_bound_verified',false,'deployment_allowed',false);
for i=1:size(cases,1)
    result.scenarios = [result.scenarios; simulate(cases(i,:),terminalBias(i),nominal,reference,L,D,p3)];
end
for i=1:numel(result.scenarios)
    s=result.scenarios(i);
    fprintf(['case %02d err %.4fmm yaw %.4fdeg start gap %.4fmm (' ...
        'extra yaw %.4fmm) gap %.4fmm deviation %.4fmm startup %.4fmm ' ...
        'corner %.3f/%.3fmm/s rpm %.3f accel %.3f jerk %.3f ' ...
        'simulation end %.3fs stop request %.3fs age %.3fms reused %d wheel zero %.3fs\n'],i,...
        s.final_error_mm,s.final_yaw_error_deg,s.startup_gap_mm,s.startup_extra_yaw_gap_mm,...
        s.sweep_gap_mm,s.max_offset_mm,s.startup_offset_mm,s.corner_speed_mm_s,...
        s.max_rpm,s.max_rpm_s,s.max_rpm_s2,s.simulation_end_s,s.stop_request_s,s.max_age_ms,...
        s.reused_fresh_samples,s.first_zero_s);
end
ok=result.scenarios(1:7);
fault=result.scenarios(8:12);
timeout=result.scenarios(13);
assert(all([ok.arrived]) && ~any([ok.stale]) && all([ok.final_error_mm]<=5));
assert(all([ok.final_yaw_error_deg]<=1) && all([ok.sweep_gap_mm]>0));
assert(all([ok.startup_offset_mm]<=1) && all([ok.startup_gap_mm]>0) && all([ok.max_yaw_deg]<=1+1e-10));
assert(all([ok(2:end).startup_extra_yaw_gap_mm]<0));
assert(all([ok.max_rpm]<=300) && all([ok.max_rpm_s]<=250) && all([ok.max_rpm_s2]<=1400));
assert(all(reshape([ok.corner_speed_mm_s],2,[])>=12,'all'));
assert(all([ok.qualified_sample_s]>[ok.initial_zero_s]) && ...
    all([ok.first_nonzero_applied_s]>[ok.qualified_sample_s]));
assert(all([ok(2:end).reused_fresh_samples]>0));
assert(all([fault.stale]) && ~any([fault.arrived]) && all([fault.final_wheels_zero]));
assert(all([fault.first_zero_s]-[fault.stop_request_s]<=.066+1e-9) && all([fault.sweep_gap_mm]>0));
assert(all(isnan([fault(4:5).qualified_sample_s])) && all(isnan([fault(4:5).first_nonzero_applied_s])) && ...
    ~any([fault(4:5).max_rpm]));
assert(timeout.terminal_timeout && ~timeout.arrived && ~timeout.stale && timeout.final_wheels_zero);
assert(abs(timeout.stop_request_s-timeout.trajectory_start_s-nominal.edges_s(end)-D.terminal_max_s)<.0101);
assert(timeout.sweep_gap_mm>0 && timeout.max_rpm<=300 && timeout.max_rpm_s<=250 && timeout.max_rpm_s2<=1400);
result.conditional_model_passed=true;
if nargin>=4
    f=fopen(output,'w'); assert(f>=0); fprintf(f,'%s\n',jsonencode(result)); fclose(f);
end
fprintf('FIRST_LEG_FEEDBACK_CONDITIONAL %d arrivals / %d stale requests / 1 terminal timeout\n',numel(ok),numel(fault));
end

function s=simulate(c,terminalBias,nominal,reference,L,D,p3)
dt=.002;
N=round((nominal.edges_s(end)+D.terminal_max_s+D.terminal_zero_hold_s+.40)/dt);
t=(0:N)'*dt;
ref=reference(t);
refYaw=pi/2;
q=zeros(N+1,3); q(1,:)=[ref(1,:) refYaw+c(1)*pi/180];
W=[1 1;1 -1;1 -1;1 1];
z=[-1 1 -1 1];
slots=(0:3)*20;
held=zeros(1,4); pending=zeros(1,4); application=slots+wheelLate(c(5),0);
initialZeroMs=max(application);
allRPM=zeros(0,4); correction=[0 0]; yawCorrection=0;
sample=NaN; samplePose=[NaN NaN NaN]; maxAge=NaN;
startMs=NaN; qualifiedSample=NaN; lastUsedSample=NaN; reused=0; firstNonzero=NaN;
bias=[c(6:7) c(8)*pi/180];
startGap=inf; startExtraYawGap=inf; gap=inf; startOffset=0; offset=0; maxYaw=0;
cornerSpeed=[inf inf]; firstZero=NaN; stopTime=NaN; stopRequest=NaN; settled=false;
stale=false; terminalTimeout=false;
finishRequested=false;
for k=1:N
    ms=(k-1)*2;
    capture=ms-c(3);
    if capture>=0 && mod(capture,c(2))==0 && capture<c(9)
        frame=capture/c(2)+1;
        if frame~=c(4)
            sample=capture;
            samplePose=q(capture/2+1,:);
            if ~isnan(startMs) && capture>=startMs+round(1000*nominal.edges_s(end))
                samplePose(2)=samplePose(2)+terminalBias;
            end
        end
    end
    age=ms-sample;
    trajectoryMs=max(0,ms-startMs);
    if mod(ms,10)==0 && isnan(stopRequest)
        if ~isnan(age), maxAge=max(maxAge,age); end
        postZeroSample=sample>initialZeroMs && age<=D.sample_age_ms;
        stale=age>D.sample_age_ms || (isnan(startMs) && ms>D.sample_age_ms && ~postZeroSample);
        terminalTimeout=~isnan(startMs) && trajectoryMs>=round(1000*(nominal.edges_s(end)+D.terminal_max_s));
        if stale || terminalTimeout
            stopRequest=t(k);
            % Synthetic plant input after the software stop decision.
            application=ms+slots+wheelLate(c(5),floor(ms/80));
            pending=zeros(1,4);
            firstZero=NaN;
        end
    end
    if isnan(stopRequest) && ~finishRequested && mod(ms+D.prepare_lead_ms,80)==0 &&...
            (postZeroSample || ~isnan(startMs))
        base=ceil(ms/80)*80;
        if isnan(startMs)
            startMs=base;
            qualifiedSample=sample;
        end
        if sample==lastUsedSample, reused=reused+1; end
        lastUsedSample=sample;
        slot=(base+slots-startMs)'/1000;
        observed=samplePose+bias;
        e=ref(max(0,(sample-startMs)/2)+1,:)-observed(1:2);
        yawError=atan2(sin(refYaw-observed(3)),cos(refYaw-observed(3)));
        targetCorrection=capVector(D.position_gain_hz*e,D.position_cap_mm_s);
        correction=correction+capVector(targetCorrection-correction,D.position_slew_mm_s2*L.H);
        targetYaw=max(-D.yaw_cap_deg_s,min(D.yaw_cap_deg_s,D.yaw_gain_hz*yawError*180/pi));
        yawCorrection=yawCorrection+max(-D.yaw_slew_deg_s2*L.H,...
            min(D.yaw_slew_deg_s2*L.H,targetYaw-yawCorrection));
        speedMap=(reference(slot+L.H)-reference(slot))/L.H+correction;
        R=[cos(observed(3)) -sin(observed(3)); sin(observed(3)) cos(observed(3))];
        cmd=(sum((speedMap*R).*W,2)'+z*L.lever*yawCorrection*pi/180)/L.K;
        if trajectoryMs>=round(1000*nominal.edges_s(end)) && sample>=startMs+round(1000*nominal.edges_s(end)) &&...
                norm(e)<=D.terminal_error_mm &&...
                abs(yawError)*180/pi<=D.terminal_yaw_deg
            cmd=zeros(1,4);
            finishRequested=true;
        end
        pending=round(10*cmd)/10;
        application=base+slots+wheelLate(c(5),floor(base/80));
        allRPM=[allRPM; pending];
    end
    apply=application==ms;
    held(apply)=pending(apply);
    if any(held~=0) && isnan(firstNonzero), firstNonzero=t(k); end
    if all(held==0) && all(application<=ms) && (finishRequested || ~isnan(stopRequest))
        if isnan(firstZero), firstZero=t(k); end
        if ~isnan(stopRequest)
            stopTime=t(k);
            break;
        end
        if t(k)-firstZero>=D.terminal_zero_hold_s && sample>=round(1000*firstZero)
            stopTime=t(k);
            stopObservation=samplePose+bias;
            endYawError=atan2(sin(refYaw-stopObservation(3)),cos(refYaw-stopObservation(3)));
            settled=norm(ref(end,:)-stopObservation(1:2))<=D.terminal_error_mm &&...
                abs(endYawError)*180/pi<=D.terminal_yaw_deg;
            break;
        end
    end
    body=held*W*L.K/4;
    speed=norm(body);
    dYaw=held*z'*L.K/(4*L.lever)*dt;
    midpoint=q(k,3)+dYaw/2;
    factor=1;
    if abs(dYaw)>1e-12, factor=sin(dYaw/2)/(dYaw/2); end
    q(k+1,:)=q(k,:)+[body*[cos(midpoint) sin(midpoint);-sin(midpoint) cos(midpoint)]*dt*factor dYaw];
    trajectoryTime=trajectoryMs/1000;
    refIndex=round(trajectoryTime/dt)+1+(ms>=startMs);
    delta=norm(q(k+1,1:2)-ref(refIndex,:));
    offset=max(offset,delta);
    actualYaw=max(abs(q(k:k+1,3)-refYaw));
    maxYaw=max(maxYaw,actualYaw);
    interp=speed*abs(dYaw)*dt/8;
    track=L.track;
    if trajectoryTime+dt<=nominal.startup_tracking_until_s+1e-9
        track=nominal.startup_tracking_mm;
        startOffset=max(startOffset,delta);
        startExtraYawGap=min(startExtraYawGap,sweep(q(k:k+1,1:2),...
            actualYaw+L.yaw,L.half,L.pos+track+interp,p3.yellow_boxes_mm));
    end
    % The 1 degree envelope includes the simulated initial heading error.
    available=sweep(q(k:k+1,1:2),max(actualYaw,L.yaw),L.half,...
        L.pos+track+interp,p3.yellow_boxes_mm);
    gap=min(gap,available);
    if trajectoryTime+dt<=nominal.startup_tracking_until_s+1e-9, startGap=min(startGap,available); end
    for j=1:2
        if trajectoryTime+dt>nominal.edges_s(2*j) && trajectoryTime<nominal.edges_s(2*j+1)
            cornerSpeed(j)=min(cornerSpeed(j),speed);
        end
    end
end
deltaRPM=diff([zeros(2,4); allRPM],1,1);
lower=L.H-L.late;
acc=abs(deltaRPM)/lower;
jerk=(abs(diff(deltaRPM,1,1))+abs(deltaRPM(1:end-1,:))*2*L.late/lower)/lower^2;
s=struct('arrived',settled,'stale',stale,'terminal_timeout',terminalTimeout,...
    'initial_zero_s',initialZeroMs/1000,'qualified_sample_s',qualifiedSample/1000,...
    'trajectory_start_s',startMs/1000,'first_nonzero_applied_s',firstNonzero,'reused_fresh_samples',reused,...
    'stop_request_s',stopRequest,'simulation_end_s',stopTime,...
    'first_zero_s',firstZero,'final_wheels_zero',all(held==0),'max_age_ms',maxAge,...
    'final_error_mm',norm(q(k,1:2)-ref(end,:)),'final_yaw_error_deg',abs(q(k,3)-refYaw)*180/pi,...
    'startup_gap_mm',startGap,'startup_extra_yaw_gap_mm',startExtraYawGap,...
    'sweep_gap_mm',gap,'max_offset_mm',offset,'startup_offset_mm',startOffset,...
    'max_yaw_deg',maxYaw*180/pi,'corner_speed_mm_s',cornerSpeed,'max_rpm',max([0; abs(allRPM(:))]),...
    'max_rpm_s',max([0; acc(:)]),'max_rpm_s2',max([0; jerk(:)]));
end

function late=wheelLate(pattern,batch)
late=zeros(1,4);
if pattern==1, late=6*mod(batch+(1:4),2); end
if pattern==2, late=2*mod(3*batch+2*(1:4)+floor(batch/3),4); end
end

function v=capVector(v,maximum)
n=norm(v);
if n>maximum, v=v*(maximum/n); end
end

function gap=sweep(p,angle,h,padding,boxes)
assert(angle<atan(min(h)/max(h)));
half=[h(2) h(1)]*cos(angle)+[h(1) h(2)]*sin(angle)+padding;
lo=min(p)-half;
hi=max(p)+half;
separate=max([boxes(:,1)-hi(1),boxes(:,2)-hi(2),lo(1)-boxes(:,3),lo(2)-boxes(:,4)],[],2);
gap=min([lo 2400-hi separate']);
end
