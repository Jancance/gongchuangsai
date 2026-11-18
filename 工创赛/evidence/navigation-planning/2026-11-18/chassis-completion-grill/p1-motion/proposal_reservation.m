function [decision,out]=proposal_reservation(q0,a0,q1,request,L)
% Test the old brake certificate first. A new prefix must preserve room for its
% own stop. Current-certificate failure asks the caller for the fault/STOP path.
[nowOK,before]=brake_reservation(q0,a0,request,L);
out=struct('reason',before.reason,'before',before,'after',[],'duration_ms',0,...
    'prefix_mm',0,'prefix_q',q0,'prefix_a',a0);
decision="fault";
if ~nowOK
    return
end
decision="brake";
[valid,curve]=velocity_candidate(q0,a0,q1,request.dt_ms,L);
if ~valid
    out.reason="no_candidate_curve"; return
end
if request.turn_sign~=0 && any(request.turn_sign*curve.p(:,3)<-1e-9)
    out.reason="turn_reverse"; return
end
s=request.dt_ms/curve.duration_ms;
p=curve.p; t=curve.duration_ms/1000;
a=(1-s)*p(1:3,:)+s*p(2:4,:);
b=(1-s)*a(1:2,:)+s*a(2:3,:);
v=(1-s)*b(1,:)+s*b(2,:);
nextA=3*(b(2,:)-b(1,:))/t;
% b2-b1 is dP/ds/3. This evaluation preserves acceleration at the prefix end.
prefix=[p(1,:);a(1,:);b(1,:);v];
dt=request.dt_ms/1000;
prefixArc=dt/4*sum(hypot(prefix(:,1),prefix(:,2)));
prefixYaw=dt/4*sum(abs(prefix(:,3)));
n=[-request.tangent(2) request.tangent(1)];
rotateMargin=min(2,prefixYaw);
prefixBack=dt/4*sum(max(0,-prefix(:,1:2)*request.tangent'))+prefixArc*rotateMargin;
prefixLateral=dt/4*sum(abs(prefix(:,1:2)*n'))+prefixArc*rotateMargin;
% Budgets below already include future source/queue/mixed hold via brake_reservation.
% Rotating the start basis of the suffix over a prefix costs another angular margin.
refined=request;
refined.remaining_mm=max(0,request.remaining_mm-prefixArc);
refined.angle_rad=max(0,request.angle_rad-prefixYaw);
[nextOK,after]=brake_reservation(v,nextA,refined,L);
tailYawMargin=after.distance_mm*rotateMargin;
out.after=after; out.prefix_mm=prefixArc; out.prefix_q=v; out.prefix_a=nextA;
out.duration_ms=curve.duration_ms; out.reason=after.reason;
if ~nextOK
    return
end
if prefixArc+after.distance_mm>request.remaining_mm || ...
   prefixBack+after.back_mm+tailYawMargin>request.back_mm || ...
   prefixLateral+after.lateral_mm+tailYawMargin>request.lateral_mm || ...
   prefixYaw+after.rotation_rad>request.angle_rad
    out.reason="prefix_space"; return
end
decision="advance"; out.reason="ok";
end
