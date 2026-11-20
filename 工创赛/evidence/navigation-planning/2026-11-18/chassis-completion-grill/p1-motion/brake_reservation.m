function [accepted,out]=brake_reservation(q,a,request,L)
% Hold inputs in milliseconds. Feedback error bounds are engineering assumptions.
% Test a braking trajectory from the supplied actuation boundary; report reserves
% in the starting body-frame tangent/normal basis provided by request.tangent.
out=struct('duration_ms',0,'held_ms',0,'distance_mm',0,'lateral_mm',0,...
           'rotation_rad',0,'back_mm',0,'reason',"");
accepted=false;
times=[request.now_ms request.source_ms];
parameters=[request.max_age_ms request.compute_ms request.queue_ms request.batch_ms ...
    request.feedback_xy_mm_s request.feedback_yaw_rad_s request.mixed_mm request.mixed_rad ...
    request.remaining_mm request.back_mm request.lateral_mm request.angle_rad];
if numel(request.source_ms)~=5 || any(~isfinite(times)) || ...
   any(times<0 | times>=2^32 | times~=floor(times)) || ...
   any(~isfinite(parameters) | parameters<0) || ...
   numel(request.tangent)~=2 || any(~isfinite(request.tangent)) || ...
   abs(norm(request.tangent)-1)>1e-7 || ~any(request.turn_sign==[-1 0 1])
    out.reason="input"; return
end
ages=mod(double(request.now_ms)-double(request.source_ms),2^32);
if any(ages>request.max_age_ms)
    out.reason="stale_source"; return
end
if request.mixed_lost
    out.reason="partial_delivery"; return
end
[valid,curve]=velocity_candidate(q,a,[0 0 0],request.dt_ms,L);
if ~valid
    out.reason="no_brake_curve"; return
end
if request.turn_sign~=0 && any(request.turn_sign*curve.p(1:2,3)<-1e-9)
    out.reason="turn_reverse"; return
end
t=curve.duration_ms/1000;
heldMs=max(ages)+request.compute_ms+request.queue_ms+request.batch_ms;
held=heldMs/1000;
n=[-request.tangent(2) request.tangent(1)];
turnMargin=min(2,held*(abs(q(3))+request.feedback_yaw_rad_s)+curve.rotation_rad+...
    request.feedback_yaw_rad_s*t+request.mixed_rad);
mmError=request.feedback_xy_mm_s*(held+t);
predictedHoldMm=held*norm(q(1:2));
% Hold and mixed-actuation mismatch are budgeted independently of the ideal curve.
% reserve is conditional on a configured upper bound for mixed-window displacement.
curveLateral=t/4*sum(abs(curve.p(:,1:2)*n'));
heldLateral=held*abs(dot(q(1:2),n));
idealArc=curve.distance_mm+predictedHoldMm;
backward=t/4*sum(max(0,-curve.p(:,1:2)*request.tangent'))+...
    held*max(0,-dot(q(1:2),request.tangent));
out.back_mm=backward+idealArc*turnMargin+mmError+request.mixed_mm;
out.duration_ms=curve.duration_ms;
out.held_ms=heldMs;
out.distance_mm=curve.distance_mm+predictedHoldMm+mmError+request.mixed_mm;
out.lateral_mm=curveLateral+heldLateral+...
    (curve.distance_mm+predictedHoldMm)*turnMargin+mmError+request.mixed_mm;
out.rotation_rad=curve.rotation_rad+held*abs(q(3))+...
    request.feedback_yaw_rad_s*(held+t)+request.mixed_rad;
if out.distance_mm>request.remaining_mm
    out.reason="distance"; return
end
if out.back_mm>request.back_mm
    out.reason="back"; return
end
if out.lateral_mm>request.lateral_mm
    out.reason="lateral"; return
end
if out.rotation_rad>request.angle_rad
    out.reason="angle"; return
end
out.reason="ok"; accepted=true;
end
