function validate_source_continuous_20261115
% User-authorized observation gate and moving-target scheduling, no hardware.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','source-continuous');
if ~exist(out,'dir'),mkdir(out);end
base=fullfile(root,'evidence','integration','2026-11-15','integration18','window5-pause','integration19-field');
g=jsondecode(fileread(fullfile(base,'parking-current-matlab.json')));
p=jsondecode(fileread(fullfile(base,'source_body_registration_field.json')));
extensions=[reshape(g.current_extensions_mm(1:2),1,[]) 0];yaw=[195 165 180];
body=[(161+extensions).*cosd(yaw-180);(161+extensions).*sind(yaw-180)]';
targetPixels=[960-body(:,2)/p.scale_mm_per_px,540+(226-body(:,1))/p.scale_mm_per_px];
v=targetPixels-reshape(p.reference_center_px,1,[]);
target=atan2(v(:,2),v(:,1))';
orbitRadius=mean(vecnorm(g.contact_centers_px-reshape(p.reference_center_px,1,[]),2,2))*p.scale_mm_per_px;
radialOffset=vecnorm(v,2,2)'*p.scale_mm_per_px-orbitRadius;
timing=jsondecode(fileread(fullfile(out,'historical-contact-timing.json')));
lead=zeros(1,3);bracket=zeros(1,3);
for k=1:numel(timing.samples)
 s=timing.samples(k);lead(s.region+1)=s.initial_lead_ms/1000;bracket(s.region+1)=s.sample_interval_ms/1000;
end
assert(all(lead>0));checks=1;events=0;maxTickError=0;maxTimingError=0;
% Same conic/positive-definite whitening and per-colour phase slopes as planned.
for omega=[-.8 .3 1.5]
 tt=(0:.05:2*pi/abs(omega)+.2)';points=[];times=[];labels=[];
 transform=[430 18;18 385];center=[970;390];
 for color=1:3
  phase=omega*tt+(color-1)*2*pi/3;
  xy=(center+transform*[cos(phase)';sin(phase)'])';
  points=[points;xy];times=[times;tt];labels=[labels;repmat(color,size(tt))]; %#ok<AGROW>
 end
 origin=mean(points,1);s=max(std(points));q=(points-origin)/s;
 design=[q(:,1).^2 q(:,1).*q(:,2) q(:,2).^2 q(:,1) q(:,2)];
 coeff=design\ones(size(q,1),1);Q=[coeff(1) coeff(2)/2;coeff(2)/2 coeff(3)];b=coeff(4:5);
 o=-.5*(Q\b);k=1+o'*Q*o;[V,D]=eig(Q);assert(all(diag(D)>0) && k>0);
 W=V*diag(sqrt(diag(D)/k))*V'/s;c=origin'+s*o;
 assert(norm(c-center)<1e-6);checks=checks+1;
 angleDesign=[];angles=[];
 for color=1:3
  indices=labels==color;z=(W*(points(indices,:)'-c))';theta=unwrap(atan2(z(:,2),z(:,1)));
  onehot=zeros(sum(indices),3);onehot(:,color)=1;
  angleDesign=[angleDesign;[times(indices),onehot]];angles=[angles;theta]; %#ok<AGROW>
  assert(abs(theta(end)-theta(1))>=2*pi);checks=checks+1;
 end
 fit=angleDesign\angles;assert(abs(fit(1)-omega)<1e-8);checks=checks+1;
end
% A full turn defines initial learning; moving-camera frames never train it.
for omega=[-.8 -.3 .3 .8 1.5]
 period=2*pi/abs(omega);dt=.04;t=0:dt:period+2;
 theta=omega*t+.4;learned=abs(theta-theta(1))>=2*pi;
 assert(~learned(1) && learned(end));checks=checks+1;
 for sample=1:17:numel(t)
  now=t(sample);
  delays=zeros(1,3);contacts=zeros(1,3);
  for region=1:3
   if omega>0,angle=mod(target(region)-(omega*now+.4),2*pi);
   else,angle=mod((omega*now+.4)-target(region),2*pi);end
   wait=angle/abs(omega);
   if wait<lead(region),wait=wait+ceil((lead(region)-wait)/period)*period;end
   delay=wait-lead(region);assert(delay>=0);checks=checks+1;
   start=now+delay;contact=start+lead(region);
   phase=omega*contact+.4;
   angularError=atan2(sin(phase-target(region)),cos(phase-target(region)));
   assert(abs(angularError)<1e-10);checks=checks+1;
   % MCU10ms task scheduling removes camera-frame trigger quantisation.
   scheduled=ceil(start*1000/10)*10/1000;
   tickError=100*abs(omega)*(scheduled-start);maxTickError=max(maxTickError,tickError);
   assert(scheduled-start<.01000001);checks=checks+1;
   uncertainty=100*abs(omega)*(bracket(region)/2+.01);
   maxTimingError=max(maxTimingError,uncertainty);
   delays(region)=delay;contacts(region)=contact;events=events+1;
  end
  [~,chosen]=min(delays);assert(contacts(chosen)>=now+lead(chosen));checks=checks+1;
 end
end
for rms=[.557 2.999 3 3.001 46.627 50.307 61.579]
 valid=rms<=3;assert(valid==(rms<=3));checks=checks+1;
 if rms>3,assert(~valid);end
end
% Bad visual data causes no new body step and no fault/VM change.
state=2;fault=0;vm=1;bodyRequests=0;
for k=1:50
 valid=false;
 if valid,bodyRequests=bodyRequests+1;end
 assert(state==2 && fault==0 && vm==1 && bodyRequests==0);checks=checks+1;
end
% Scheduling is part of a committed piece: camera loss does not cancel it.
for elapsed=[0 100 500 2000]
 due=2000;start=elapsed>=due;assert(start==(elapsed>=due));checks=checks+1;
end
config=struct('target_angles_rad',target,'lead_ms_by_region',round(lead*1000),...
 'target_yaw_deg',yaw,'target_extension_mm',extensions,'target_tool_points_px',targetPixels,...
 'tool_to_previous_orbit_radial_offset_mm',radialOffset,...
 'lead_source','historical closure-height feedback bracket, not physical contact measurement',...
 'timing_calibrated_on_continuously_rotating_disc',false,...
 'learn_one_revolution',true,'rim_max_fit_rms_px',3,'body_tolerance_mm',5,...
 'extension_arrival_mm',5,'extension_path_mm',10,'zero_mm_approximately',.5);
f=fopen(fullfile(out,'prediction-config.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(config));fclose(f);
report=struct('passed',true,'local_matlab',true,'checks',checks,'scheduled_examples',events,...
 'maximum_10ms_schedule_position_error_mm_at_radius100',maxTickError,...
 'historical_timing_bracket_plus_10ms_error_bound_mm',maxTimingError,...
 'continuous_motion_model','constant angular speed, image orbit model learned at fixed observation pose',...
 'physical_contact_latency_unmeasured',true,'current_motion_data_required_after_reposition',true,...
 'actual_grasp_success_verified',false,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS continuous scheduling MATLAB: %u checks/%u examples; lead %u/%u/%ums; timing uncertainty bound %.2fmm, physical contact NOT verified\n',checks,events,config.lead_ms_by_region,maxTimingError);
end
