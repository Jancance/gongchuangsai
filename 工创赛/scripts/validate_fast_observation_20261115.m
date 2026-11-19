function validate_fast_observation_20261115
% Validate replacing whole-turn learning with saved fixed-view geometry and
% two CURRENT observations. Saved geometry is not physical grasp acceptance.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-15','source-rim-reacquire');
out=fullfile(base,'fast-observation');if ~exist(out,'dir'),mkdir(out);end
rows=jsondecode(fileread(fullfile(base,'field-no-pick','observations-summary.json')));
cfg=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','source-continuous','prediction-config.json')));
points=[];rim=[];times=[];
for n=1:numel(rows)
 row=rows(n);
 if row.state.state==2 && ~isempty(row.body),rim=row.body.center_px(:)';end
 if row.state.state~=6,continue;end
 times(end+1)=row.observation_host_timestamp; %#ok<AGROW>
 for color=[1 2 5]
  regions=row.regions;if isempty(regions),continue;end
  found=find([regions.color_id]==color);
  if numel(found)==1
   p=regions(found);points(end+1,:)=[p.center_x_px p.center_y_px]; %#ok<AGROW>
  end
 end
end
% This is a recorded image orbit. Quantify its residual instead of silently
% labelling a conic fitted to silhouettes as the true material-centre orbit.
[center,W]=ellipse(points);
radiusError=abs(vecnorm((W*(points-center)'),2,1)-1)/norm(W,2);
sorted=sort(radiusError);p95=sorted(ceil(.95*numel(sorted)));
checks=0;maximumError=0;
% atan2 with separate columns keeps compatibility with installed R2020a.
tool=(W*(cfg.target_tool_points_px-center)')';target=atan2(tool(:,2),tool(:,1));
for omega=[-1.1 -.36 .36 1.1]
 for theta=linspace(-pi,pi,13)
  for dt=[.04 .10 .24]
   a=theta;b=theta+omega*dt;
   measured=atan2(sin(b-a),cos(b-a))/dt;
   assert(abs(measured-omega)<1e-10);checks=checks+1;
   for k=1:3
    lead=cfg.lead_ms_by_region(k)/1000;period=2*pi/abs(measured);
    if measured>0,wait=mod(target(k)-b,2*pi)/abs(measured);
    else,wait=mod(b-target(k),2*pi)/abs(measured);end
    if wait<lead,wait=wait+ceil((lead-wait)/period)*period;end
    delay=wait-lead;phase=b+omega*(delay+lead);
    err=abs(atan2(sin(phase-target(k)),cos(phase-target(k))));
    maximumError=max(maximumError,err);assert(delay>=0 && err<1e-10);checks=checks+2;
   end
  end
 end
end
% Existing freshness: a gap breaks the pair; a new pair resumes immediately.
stamps=[0 .1 1 1.1];valid=diff(stamps)<=.25;
assert(isequal(valid,[true false true]));checks=checks+1;
assert(all(isfinite([center(:);W(:);rim(:)])));checks=checks+1;
orbit=struct('center_px',center,'whitening',W,'calibration_rim_center_px',rim,...
 'target_tool_points_px',cfg.target_tool_points_px,'lead_ms_by_region',cfg.lead_ms_by_region,...
 'maximum_frame_interval_s',.25,'learn_one_revolution',false,...
 'source','same-session recorded silhouette orbit; physical grasp centre NOT calibrated',...
 'observed_radius_residual_p95_px',p95,'physical_prediction_validation_passed',false);
f=fopen(fullfile(out,'orbit-config.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(orbit));fclose(f);
report=struct('passed',true,'local_matlab',true,'checks',checks,...
 'two_frame_constant_speed_math_passed',true,'maximum_phase_error_rad',maximumError,...
 'recorded_observe_seconds',times(end)-times(1),'recorded_observe_frames',numel(times),...
 'recorded_orbit_radius_residual_p95_px',p95,...
 'live_deployment_approved_by_this_result',false,'physical_grasp_validation',false,...
 'geometry_and_current_red_threshold_need_field_verification',true);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS fast two-frame scheduling mathematics: %u checks; recorded geometry residual P95 %.2fpx, NOT physical acceptance\n',checks,p95);
end

function [center,W]=ellipse(points)
origin=mean(points,1);s=max(std(points));q=(points-origin)/s;
x=q(:,1);y=q(:,2);v=[x.^2 x.*y y.^2 x y]\ones(size(x));
Q=[v(1) v(2)/2;v(2)/2 v(3)];o=-.5*(Q\v(4:5));k=1+o'*Q*o;
[V,D]=eig(Q);assert(all(diag(D)>0) && k>0);
center=origin+s*o';W=V*diag(sqrt(diag(D)/k))*V'/s;
end
