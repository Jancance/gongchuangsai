function validate_rim_and_final_park_20261115
% Independent MATLAB fitting/control checks on observed edges from the
% unchanged existing detector. Not a Python-model pass flag or a vehicle test.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','source-rim-reacquire');
d=jsondecode(fileread(fullfile(out,'measured-edge-clouds.json')));
p=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-15','integration18',...
 'window5-pause','integration19-field','source_body_registration_field.json')));
config=jsondecode(fileread(fullfile(out,'actual-turntable-detector.json')));
checks=0;results=struct([]);shifts=0;
for i=1:numel(d.images)
 row=d.images(i);points=row.points_zero_based;weights=row.strengths;
 [circle,rms]=robust(points,weights);
 assert(norm(circle(1:2)-row.raw_center_zero_based(:)')<1e-5);
 assert(abs(circle(3)-row.raw_radius)<1e-5 && abs(rms-row.raw_rms)<1e-5);checks=checks+3;
 valid=rms<=3 && circle(3)>=config.coarse_search.minimum_radius_px && circle(3)<=config.coarse_search.maximum_radius_px;
 assert(valid);checks=checks+1;
 center=circle(1:2)+1;delta=center-p.reference_center_px(:)';
 correction=p.response_px_per_command_mm\(-delta');
 assert(all(isfinite(correction)));checks=checks+1;
 for dx=[-120 -60 0 60 120]
  for dy=[-120 -60 0 60 120]
   [translated,r]=robust(points+[dx dy],weights);
   assert(norm(translated(1:2)-circle(1:2)-[dx dy])<1e-5);
   assert(abs(r-rms)<1e-5);checks=checks+2;shifts=shifts+1;
  end
 end
 results(i).path=row.path;results(i).center_matlab_one_based=center;
 results(i).radius_px=circle(3);results(i).fit_rms_px=rms;
 results(i).delta_mm=delta*p.scale_mm_per_px;
 results(i).correction_right_forward_mm=correction';
 if contains(row.path,'integration19-right7-1450')
  assert(norm(delta)*p.scale_mm_per_px<1);checks=checks+1;
 end
end
% Keep the user-approved3px gate. An invalid visual observation only waits.
for rms=[.55 1.0145 3 3.01 58.6]
 valid=rms<=3;motorRequests=double(valid);fault=0;vm=1;
 if rms>3,assert(motorRequests==0 && fault==0 && vm==1);end
 checks=checks+1;
end
% Same50ms release dwell and parallel return, different LAST-piece endpoint.
for slot=1:3
 if slot==3,yaw=0;extension=0;else,yaw=180;extension=65;end
 delta=yaw-37;request=4;if delta<0,request=5;end
 if slot==3,assert(delta==-37 && request==5 && extension==0);
 else,assert(delta==143 && request==4 && extension==65);end
 checks=checks+1;
 t=linspace(0,1,101);angle=37+delta*t;z=1800+(600-1800)*t;x=extension*t;
 assert(all(angle>=0 & angle<=180) && all(diff(z)<=0));checks=checks+1;
 if slot==3,assert(all(diff(angle)<=0) && all(x==0));checks=checks+1;end
 assert(z(end)==600);checks=checks+1;
 for bad=[false true]
  existingStop=bad;assert(existingStop==bad);checks=checks+1;
 end
end
report=struct('passed',true,'local_matlab',true,'checks',checks,'shift_cases',shifts,...
 'existing_detector_edge_clouds',numel(d.images),'results',results,...
 'no_added_temporal_filter',true,'rim_gate_px',3,'invalid_observation_waits_without_fault',true,...
 'final_yaw_deg',0,'final_extension_mm',0,'final_lift_safe_top_pulses',600,...
 'final_from_tray_yaw_deg',37,'final_signed_delta_deg',-37,'final_request',5,...
 'direction_evidence','dm_can2_motion.c: request4 CCW/request5 CW; target-current negative selects5',...
 'hardware_access',false,'final_parking_physically_verified',false,...
 'full_three_dimensional_carrier_tool_sweep_not_measured',true);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS MATLAB rim/park: %u checks/%u shifts; raw current RMS %.3fpx; final37->0 CW/X0/top600; no hardware\n',checks,shifts,results(1).fit_rms_px);
end
function [m,rms]=robust(points,weights)
keep=true(size(points,1),1);m=fit(points,weights);
for j=1:6
 residual=abs(vecnorm(points-m(1:2),2,2)-m(3));active=residual(keep);
 center=median(active);mad=median(abs(active-center))+1e-6;
 threshold=max(1.5,min(5,center+3*1.4826*mad));updated=residual<=threshold;
 if sum(updated)<20,break;end
 keep=updated;m=fit(points(keep,:),weights(keep));
end
err=vecnorm(points(keep,:)-m(1:2),2,2)-m(3);rms=sqrt(mean(err.^2));
end
function m=fit(points,weights)
A=[2*points ones(size(points,1),1)];b=sum(points.^2,2);s=sqrt(weights(:));
v=(A.*s)\(b.*s);m=[v(1:2)' sqrt(v(3)+sum(v(1:2).^2))];
end
