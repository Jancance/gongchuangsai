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
 "brake_infeasible_at_cap", [400 0 0],[50 0 0],[0 0 0],false;...
 "target_overspeed", [0 0 0],[0 0 0],[401 0 0],false;...
 "start_overaccel", [20 20 0],[110 110 0],[0 0 0],false;...
 "positive_turn", [0 0 0],[0 0 0],[0 0 pi/2],true;...
 "negative_turn", [0 0 0],[0 0 0],[0 0 -pi/2],true;...
 "pd_change_when_translating", [200 20 0.1],[30 -30 0.1],[220 -20 -0.3],true;...
 "joint_wheel_overspeed", [0 0 0],[0 0 0],[400 400 pi/2],true;...
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
 'tangent',[1 0],'remaining_mm',1000,'lateral_mm',25,'angle_rad',pi,...
 'feedback_xy_mm_s',2,'feedback_yaw_rad_s',0.001,'mixed_mm',0.2,'mixed_rad',0.001,'mixed_lost',false);
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
r.batching_stress_ms=120; r.batch_ms=r.batching_stress_ms;
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
% Adjacent commands are subjected to a real four-row geometry and
% independent Tx/ACK timeline. ACK loss may already have applied the wheel.
batch=held_batch([200 0 0],[197 5 0.01],[0 7 14 21],[5 13 18 28],28,L);
assert(batch.complete && batch.latest_ack_ms==28); checks=checks+1;
assert(batch.mismatch_mm>0 && batch.mismatch_rad>0); checks=checks+1;
assert(all(abs(batch.signed_raw(:))<=3000)); checks=checks+1;
batchLost=held_batch([200 0 0],[197 5 0.01],[0 7 NaN NaN],[5 NaN NaN NaN],30,L);
assert(~batchLost.complete && batchLost.assume_applied(2) && ~batchLost.assume_applied(3));
assert(batchLost.mismatch_mm>0 && batchLost.at_risk); checks=checks+2;
assert(seq_after(3,2^32-2) && ~seq_after(2^32-2,3) && ~seq_after(3,3)); checks=checks+1;
out=struct('version',version,'scenarios',size(vectors,1)+14,'assertion_groups',checks,...
 'limits',L,'brake',base,'all_ack',batch,'lost_ack',batchLost,'passed',true);
write_json('p1_kernel_summary.json',out);
write_json('p1_kernel_vectors.json',trace);
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
xy=0; yaw=0;
for t=times
    mismatch=(mixed-full)*B'; elapsed=(t-previous)/1000;
    xy=xy+elapsed*norm(mismatch(1:2)); yaw=yaw+elapsed*abs(mismatch(3));
    mask=observed & tx==t;
    mixed(mask)=full(mask);
    previous=t;
end
batch=struct('complete',all(agreed),'latest_ack_ms',max(ack(agreed)),...
    'at_risk',any(observed & ~agreed),'assume_applied',observed,...
    'signed_raw',signedRaw,'mismatch_mm',xy,'mismatch_rad',yaw);
end

function newer=seq_after(t0,t1)
dt=mod(double(t0)-double(t1),2^32);
newer=dt>0 && dt<2^31;
end

function write_json(filename,out)
fid=fopen(filename,'w'); assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(out)); fclose(fid);
end
