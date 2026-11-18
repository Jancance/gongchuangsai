function [accepted,out]=brake_reservation(q,a,request,L)
% Hold inputs in milliseconds. Feedback error bounds are engineering assumptions.
% Test a braking trajectory from the supplied actuation boundary; report reserves
% in the starting body-frame tangent/normal basis provided by request.tangent.
out=struct('duration_ms',0,'held_ms',0,'distance_mm',0,'lateral_mm',0,...
           'rotation_rad',0,'reason',"");
accepted=false;
ages=mod(double(request.now_ms)-double(request.source_ms),2^32);
if any(ages>request.max_age_ms)
    out.reason="stale_source"; return
end
if request.mixed_lost
    out.reason="partial_delivery"; return
end
if ~isfinite(request.remaining_mm) || request.remaining_mm<0 || ...
   ~isfinite(request.lateral_mm) || request.lateral_mm<0 || ...
   ~isfinite(request.angle_rad) || request.angle_rad<0 || ...
   abs(norm(request.tangent)-1)>1e-7
    out.reason="input"; return
end
[valid,curve]=velocity_candidate(q,a,[0 0 0],request.dt_ms,L);
if ~valid
    out.reason="no_brake_curve"; return
end
t=curve.duration_ms/1000;
heldMs=max(ages)+request.compute_ms+request.queue_ms+request.batch_ms;
held=heldMs/1000;
n=[-request.tangent(2) request.tangent(1)];
turnMargin=min(2,held*(abs(q(3))+request.feedback_yaw_rad_s)+curve.rotation_rad+...
    request.feedback_yaw_rad_s*t);
mmError=request.feedback_xy_mm_s*(held+t);
predictedHoldMm=held*norm(q(1:2));
% Hold and mixed-actuation mismatch are budgeted independently of the ideal curve.
% reserve is conditional on a configured upper bound for mixed-window displacement.
curveLateral=t/4*sum(abs(curve.p(:,1:2)*n'));
heldLateral=held*abs(dot(q(1:2),n));
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
if out.lateral_mm>request.lateral_mm
    out.reason="lateral"; return
end
if out.rotation_rad>request.angle_rad
    out.reason="angle"; return
end
out.reason="ok"; accepted=true;
end
