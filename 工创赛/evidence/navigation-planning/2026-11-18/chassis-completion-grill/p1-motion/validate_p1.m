function validate_p1()
L.W=[1 1 -209;1 -1 209;1 -1 -209;1 1 209]/(77*pi/60);
L.componentV=400; L.xyA=150; L.xyJ=300;
L.wV=90*pi/180; L.wA=180*pi/180; L.wJ=1440*pi/180;
L.rpmV=300; L.rpmA=250; L.rpmJ=1400; % wheel derivative sensitivity assumptions
vectors={...
 "forward", [0 0 0],[0 0 0],[400 0 0],true;...
 "reverse", [0 0 0],[0 0 0],[-400 0 0],true;...
 "left", [0 0 0],[0 0 0],[0 400 0],true;...
 "right", [0 0 0],[0 0 0],[0 -400 0],true;...
 "NE", [0 0 0],[0 0 0],[400 400 0],true;...
 "NW", [0 0 0],[0 0 0],[-400 400 0],true;...
 "SW", [0 0 0],[0 0 0],[-400 -400 0],true;...
 "SE", [0 0 0],[0 0 0],[400 -400 0],true;...
 "reverse_through_zero", [200 0 0],[20 0 0],[-200 0 0],true;...
 "lateral_cross_zero", [160 -30 0],[20 40 0],[165 40 0],true;...
 "brake_outward_a", [350 0 0],[90 0 0],[0 0 0],true;...
 "brake_inward_a", [350 0 0],[-90 0 0],[0 0 0],true;...
 "near_cap_analytic", [399.9 0 0],[3 0 0],[0 0 0],true;...
 "brake_infeasible_at_cap", [400 0 0],[50 0 0],[0 0 0],false;...
 "target_overspeed", [0 0 0],[0 0 0],[401 0 0],false;...
 "start_overaccel", [20 20 0],[110 110 0],[0 0 0],false;...
 "positive_turn", [0 0 0],[0 0 0],[0 0 pi/2],true;...
 "negative_turn", [0 0 0],[0 0 0],[0 0 -pi/2],true;...
 "pd_change_when_translating", [200 20 0.1],[30 -30 0.1],[220 -20 -0.3],true;...
 "joint_velocity_with_turn", [0 0 0],[0 0 0],[400 400 pi/2],true;...
 "brake_with_angular_rate", [225 30 -0.1],[40 0 -0.1],[0 0 0],true;...
 "budget_stationary", [0 0 0],[0 0 0],[0 0 0],true};
checks=0; trace=struct([]);
for i=1:size(vectors,1)
    q0=vectors{i,2}; a0=vectors{i,3}; q1=vectors{i,4};
    [found,curve]=velocity_candidate(q0,a0,q1,20,L);
    assert(found==vectors{i,5},'candidate expectation %s',vectors{i,1}); checks=checks+1;
    if found
        p=curve.p; T=curve.duration_ms/1000; s=linspace(0,1,401)';
        [q,a,j]=trace_curve(curve,s);
        wheel=q*L.W'; raw=floor(abs(wheel)*10+0.5);
        directions=xor(repmat(logical([0 1 0 1]),size(raw,1),1),raw~=0 & wheel<0);
        assert(max(abs(q(1,:)-q0))<1e-8 && max(abs(q(end,:)-q1))<1e-8);
        assert(max(abs(a(1,:)-a0))<1e-8 && max(abs(a(end,:)))<1e-8);
        assert(all(all(abs(q(:,1:2))<=L.componentV+1e-8)) && all(abs(q(:,3))<=L.wV+1e-8));
        assert(all(hypot(a(:,1),a(:,2))<=L.xyA+1e-8) && all(abs(a(:,3))<=L.wA+1e-8));
        assert(all(hypot(j(:,1),j(:,2))<=L.xyJ+1e-8) && all(abs(j(:,3))<=L.wJ+1e-8));
        assert(max(max(abs(a*L.W')))<=L.rpmA+1e-8 && max(max(abs(j*L.W')))<=L.rpmJ+1e-8);
        assert(max(raw(:))<=3000 && max(max(abs(raw*0.1-abs(wheel))))<=0.05+1e-8);
        assert(T*trapz(s,hypot(q(:,1),q(:,2)))<=curve.distance_mm+0.003);
        assert(T*trapz(s,abs(q(:,3)))<=curve.rotation_rad+1e-6);
        assert(all(all(directions(raw==0)==repmat_dirs(raw))));
        checks=checks+10;
        trace(i).duration_ms=curve.duration_ms;
        trace(i).distance_mm=curve.distance_mm;
        trace(i).angular_rad=curve.rotation_rad;
        trace(i).control=p;
        trace(i).raw_at_samples=raw(1:100:end,:);
        trace(i).direction_at_samples=directions(1:100:end,:);
        trace(i).qva_samples=[q(1:100:end,:) a(1:100:end,:) j(1:100:end,:)];
    end
    trace(i).name=vectors{i,1};
    trace(i).q0=q0; trace(i).a0=a0; trace(i).q1=q1; trace(i).accepted=found;
end
% Stop reservation examples and feedback age wrap.
r=struct('now_ms',35,'source_ms',[2^32-15 3 14 12 18],...
 'max_age_ms',100,'compute_ms',5,'queue_ms',10,'batch_ms',28,'dt_ms',20,...
 'tangent',[1 0],'remaining_mm',1000,'back_mm',60,'lateral_mm',25,'angle_rad',pi,...
 'turn_sign',0,'feedback_xy_mm_s',2,'feedback_yaw_rad_s',0.001,'mixed_mm',0.2,'mixed_rad',0.001,'mixed_lost',false);
[ok,m]=brake_reservation([350 0 0],[90 0 0],r,L);
assert(ok && m.held_ms==93); checks=checks+1;
assert(m.distance_mm>650 && m.distance_mm<1000); checks=checks+1;
base=m;
r.remaining_mm=base.distance_mm-0.001;
[ok,m]=brake_reservation([350 0 0],[90 0 0],r,L);
assert(~ok && m.reason=="distance"); checks=checks+1;
r.remaining_mm=base.distance_mm+0.001;
[ok,m]=brake_reservation([350 0 0],[90 0 0],r,L);
assert(ok && m.reason=="ok"); checks=checks+1;
r.batch_ms=200;
[ok,m]=brake_reservation([350 0 0],[90 0 0],r,L);
assert(~ok && m.reason=="distance" && m.distance_mm>base.distance_mm); checks=checks+1;
r.remaining_mm=1000; r.batch_ms=28; r.lateral_mm=1;
[ok,m]=brake_reservation([350 0 0],[90 0 0],r,L);
assert(~ok && m.reason=="lateral"); checks=checks+1;
r.lateral_mm=25; r.source_ms(1)=2^32-66;
[ok,m]=brake_reservation([350 0 0],[90 0 0],r,L);
assert(~ok && m.reason=="stale_source"); checks=checks+1;
r.source_ms(1)=2^32-15; r.mixed_lost=true;
[ok,m]=brake_reservation([350 0 0],[90 0 0],r,L);
assert(~ok && m.reason=="partial_delivery"); checks=checks+1;
r.mixed_lost=false;
[ok,m]=brake_reservation([400 0 0],[50 0 0],r,L);
assert(~ok && m.reason=="no_brake_curve"); checks=checks+1;
r.angle_rad=0.08;
[ok,m]=brake_reservation([0 0 pi/2],[0 0 0],r,L);
assert(~ok && m.reason=="angle"); checks=checks+1;
r.angle_rad=pi/2;
[ok,m]=brake_reservation([0 0 pi/2],[0 0 0],r,L);
assert(ok && m.rotation_rad>0.5 && m.rotation_rad<r.angle_rad); checks=checks+1;
% Locked-turn profiles use absolute angular budget and may not reverse.
r.turn_sign=1;
[ok,m]=brake_reservation([0 0 0.005],[0 0 -pi],r,L);
assert(~ok && m.reason=="turn_reverse"); checks=checks+1;
[ok,m]=brake_reservation([0 0 -0.1],[0 0 0],r,L);
assert(~ok && m.reason=="turn_reverse"); checks=checks+1;
r.turn_sign=-1;
[ok,m]=brake_reservation([0 0 -pi/2],[0 0 0],r,L);
assert(ok && m.rotation_rad>0.5); checks=checks+1;
r.turn_sign=0;
limited=L; limited.rpmV=270;
[ok,~]=velocity_candidate([0 0 0],[0 0 0],[400 400 pi/2],20,limited);
assert(~ok && max(abs(L.W*[400;400;pi/2]))<300); checks=checks+1;
% Explicit counterexamples for nonfinite and negative reserve inputs.
r.angle_rad=pi; r.back_mm=60;
badFields={'feedback_xy_mm_s','mixed_rad','batch_ms','max_age_ms','back_mm'};
for i=1:numel(badFields)
    bad=r; bad.(badFields{i})=NaN;
    [ok,m]=brake_reservation([200 0 0],[0 0 0],bad,L);
    assert(~ok && m.reason=="input"); checks=checks+1;
end
bad=r; bad.tangent=[NaN 0];
[ok,m]=brake_reservation([200 0 0],[0 0 0],bad,L);
assert(~ok && m.reason=="input"); checks=checks+1;
bad=r; bad.source_ms(1)=-1;
[ok,m]=brake_reservation([200 0 0],[0 0 0],bad,L);
assert(~ok && m.reason=="input"); checks=checks+1;
bad=r; bad.mixed_mm=-0.02;
[ok,m]=brake_reservation([200 0 0],[0 0 0],bad,L);
assert(~ok && m.reason=="input"); checks=checks+1;
bad=r; bad.mixed_rad=0.2;
[ok,m]=brake_reservation([200 0 0],[0 0 0],bad,L);
assert(~ok && m.reason=="lateral"); checks=checks+1;
bad=r; bad.source_ms(1)=r.now_ms+1;
[ok,m]=brake_reservation([200 0 0],[0 0 0],bad,L);
assert(~ok && m.reason=="stale_source"); checks=checks+1;
[ok,m]=brake_reservation([-200 0 0],[0 0 0],r,L);
assert(~ok && m.reason=="back"); checks=checks+1;

rpmPerRad=209/(77*pi/60);
headingDps=8; holdCorrectionRPM=0.5*headingDps; % example signed PD wheel differential
qHold=[220 0 holdCorrectionRPM/rpmPerRad];
[decision,x]=proposal_reservation([180 0 0],[10 0 0],qHold,r,L);
assert(decision=="advance" && all(isfinite(x.prefix_a))); checks=checks+1;
assert(x.prefix_q(1)>180 && x.prefix_q(3)>0 && x.prefix_a(1)>0); checks=checks+1;
[~,room]=brake_reservation([180 0 0],[10 0 0],r,L);
narrow=r; narrow.remaining_mm=room.distance_mm+0.01;
[decision,xNarrow]=proposal_reservation([180 0 0],[10 0 0],qHold,narrow,L);
assert(decision=="brake"); checks=checks+1;
narrow.remaining_mm=room.distance_mm-0.01;
[decision,~]=proposal_reservation([180 0 0],[10 0 0],qHold,narrow,L);
assert(decision=="fault"); checks=checks+1;
[decision,~]=proposal_reservation([180 0 0],[10 0 0],[401 0 0],r,L);
assert(decision=="brake"); checks=checks+1;
% Sweep real candidate samples across retained profiles. Position here is the
% integrated ideal body velocity at fixed heading; no physical tracking claim.
trajectory=struct();
for diagonal=0:1
    cruise=[400 400*diagonal 0]; route=r; route.tangent=cruise(1:2)/norm(cruise(1:2));
    route.remaining_mm=5000;
    [ok,start]=velocity_candidate([0 0 0],[0 0 0],cruise,20,L);
    assert(ok); checks=checks+1;
    [ok,stop]=velocity_candidate(cruise,[0 0 0],[0 0 0],20,L);
    assert(ok); checks=checks+1;
    [traj,total]=run_retained([start stop],route,L);
    assert(all(max(abs(traj(:,2:4)),[],1)==abs(cruise)));
    assert(traj(end,11)>0 && total>0 && total<5000);
    assert(max(abs(traj(end,2:7)))<1e-8);
    checks=checks+3;
    if diagonal, trajectory.diagonal=traj; else, trajectory.straight=traj; end
end
[ok,oldPlan]=velocity_candidate([180 0 0],[10 0 0],[0 0 0],20,L);
assert(ok && xNarrow.reason~="ok"); checks=checks+1;
narrow.remaining_mm=room.distance_mm+0.01;
[retained,total]=run_retained(oldPlan,narrow,L);
assert(total<narrow.remaining_mm && max(abs(retained(end,2:7)))<1e-8);
assert(max(abs(retained(1,2:7)-[180 0 0 10 0 0]))<1e-8); checks=checks+2;
trajectory.short=retained;
% Adjacent commands are subjected to a real four-row geometry and
% independent Tx/ACK timeline. ACK loss may already have applied the wheel.
batch=held_batch([200 0 0],[197 5 0.01],[0 7 14 21],[5 13 18 28],28,L);
assert(batch.complete && batch.latest_ack_ms==28); checks=checks+1;
assert(batch.mismatch_mm>0 && batch.mismatch_rad>0); checks=checks+1;
assert(all(abs(batch.signed_raw(:))<=3000)); checks=checks+1;
assert(batch.gap_abs_integral_rpm_s>0 && numel(batch.rigid_residual_rpm)==16);
assert(all(abs(batch.rigid_residual_rpm)<=batch.max_mask_residual_rpm)); checks=checks+2;
batchLost=held_batch([200 0 0],[197 5 0.01],[0 7 NaN NaN],[5 NaN NaN NaN],30,L);
assert(~batchLost.complete && batchLost.assume_applied(2) && ~batchLost.assume_applied(3));
assert(batchLost.mismatch_mm>0 && batchLost.at_risk); checks=checks+2;
assert(seq_after(3,2^32-2) && ~seq_after(2^32-2,3) && ~seq_after(3,3)); checks=checks+1;
out=struct('version',version,'scenarios',size(vectors,1)+37,'assertion_groups',checks,...
 'limits',L,'brake',base,'proposal',x,'narrow',xNarrow,'all_ack',batch,'lost_ack',batchLost,'passed',true);
write_json('p1_kernel_summary.json',out);
write_json('p1_kernel_vectors.json',trace);
write_json('p1_retained_trajectories.json',trajectory);
fprintf('P1_KERNEL_PASS %d scenarios, %d assertion groups\n',out.scenarios,checks);
end

function [q,a,j]=trace_curve(curve,s)
T=curve.duration_ms/1000; p=curve.p;
q=[(1-s).^3 3*s.*(1-s).^2 3*s.^2.*(1-s) s.^3]*p;
a=[(1-s).^2 2*s.*(1-s) s.^2]*3*diff(p,1,1)/T;
j=[(1-s) s]*6*diff(p,2,1)/T^2;
end

function dirs=repmat_dirs(raw)
dirs=repmat(logical([0 1 0 1]),size(raw,1),1);
dirs=dirs(raw==0);
end

function batch=held_batch(old,new,tx,ack,deadline,L)
W=L.W; B=pinv(W);
w=[old;new]*W';
signedRaw=sign(w).*floor(abs(w)*10+0.5);
observed=isfinite(tx) & tx<=deadline;
agreed=observed & isfinite(ack) & ack>=tx & ack<=deadline;
times=unique([0 tx(observed) deadline]); previous=0; mixed=signedRaw(1,:)/10;
full=signedRaw(2,:)/10;
xy=0; yaw=0; residualIntegral=0;
% [-] residual is invisible to the forward projection for this wheel order.
nullRow=[1 1 -1 -1]/4;
residuals=zeros(1,16);
for id=0:15
    mask=logical(bitget(uint32(id),1:4));
    trial=signedRaw(1,:)/10; trial(mask)=full(mask);
    residuals(id+1)=dot(trial,nullRow);
end
for t=times
    mismatch=(mixed-full)*B'; elapsed=(t-previous)/1000;
    xy=xy+elapsed*norm(mismatch(1:2)); yaw=yaw+elapsed*abs(mismatch(3));
    residualIntegral=residualIntegral+elapsed*abs(dot(mixed,nullRow));
    mask=observed & tx==t;
    mixed(mask)=full(mask);
    previous=t;
end
batch=struct('complete',all(agreed),'latest_ack_ms',max(ack(agreed)),...
    'at_risk',any(observed & ~agreed),'assume_applied',observed,...
    'signed_raw',signedRaw,'mismatch_mm',xy,'mismatch_rad',yaw,...
    'rigid_residual_rpm',residuals,'max_mask_residual_rpm',max(abs(residuals)),...
    'gap_abs_integral_rpm_s',residualIntegral);
end

function newer=seq_after(t0,t1)
dt=mod(double(t0)-double(t1),2^32);
newer=dt>0 && dt<2^31;
end

function write_json(filename,out)
fid=fopen(filename,'w'); assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(out)); fclose(fid);
end

function [trajectory,distance]=run_retained(curves,request,L)
distance=0; time=0; trajectory=[];
for curve=curves
    indices=0:request.dt_ms:curve.duration_ms;
    s=indices'/curve.duration_ms;
    [q,a,j]=trace_curve(curve,s);
    travel=curve.duration_ms/1000*cumtrapz(s,hypot(q(:,1),q(:,2)));
    sample=[indices'+time q a j travel+distance];
    assert(all(hypot(a(:,1),a(:,2))<=L.xyA+1e-7) && all(hypot(j(:,1),j(:,2))<=L.xyJ+1e-7));
    assert(all(all(abs(q*L.W')<=L.rpmV+1e-7)));
    if ~isempty(trajectory)
        assert(all(abs(trajectory(end,2:7)-sample(1,2:7))<=1e-7));
        sample=sample(2:end,:);
    end
    trajectory=[trajectory;sample]; %#ok<AGROW>
    time=time+curve.duration_ms;
    distance=distance+travel(end);
end
end
