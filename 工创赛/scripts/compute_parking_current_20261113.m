function compute_parking_current_20261113(session)
% New-image calculation only. Prior measured response is a candidate model.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<1,session='align-new-20261113-capture';end
out=fullfile(root,'evidence','integration','2026-11-13',session);
if isfolder(session),out=session;end
d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
prior=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-13','source-resume','pose','matlab-result.json')));
response=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-13','source-resume','body-align','two-axis-result.json')));
assert(prior.passed && response.passed);
scale=prior.scale_mm_per_px;c=zeros(3,2);radii=zeros(1,3);rms=zeros(1,3);
for k=1:3
 p=d.arcs(k).boundary_px;assert(size(p,1)>150);keep=true(size(p,1),1);
 for j=1:5
  o=mean(p(keep,:),1);q=p(keep,:)-o;v=[2*q ones(size(q,1),1)]\sum(q.^2,2);
  center=v(1:2)'+o;r=sqrt(v(3)+sum(v(1:2).^2));err=vecnorm(p-center,2,2)-r;
  med=median(err(keep));mad=median(abs(err(keep)-med));keep=abs(err-med)<max(1.5,3*1.4826*mad);
 end
 c(k,:)=center;radii(k)=r*scale;rms(k)=sqrt(mean(err(keep).^2))*scale;
 assert(radii(k)>18 && radii(k)<32 && rms(k)<1.5);
end
target=[960,540+65/scale];delta=target-c(3,:);
J=response.response_px_per_command_mm;assert(cond(J)<10);
u=J\delta';assert(all(abs(u)<=20));
best=Inf;command=[0;0];
for right=-20:20
 for forward=-20:20
  cost=norm(c(3,:)+(J*[right;forward])'-target)^2;
  if cost<best,best=cost;command=[right;forward];end
 end
end
corrected=c+(J*command)';
current_positions=[226-(c(:,2)-540)*scale,-(c(:,1)-960)*scale];
current_yaw=180+atan2d(current_positions(:,2),current_positions(:,1));
current_extension=vecnorm(current_positions,2,2)-161;
current_error=(c(3,:)-target)*scale;
positions=[226-(corrected(:,2)-540)*scale,-(corrected(:,1)-960)*scale];
yaw=180+atan2d(positions(:,2),positions(:,1));extension=vecnorm(positions,2,2)-161;
residual=(corrected(3,:)-target)*scale;
report=struct('passed',true,'engine','local MATLAB','source_session',d.source_session, ...
 'image_sha256',d.image_sha256,'colors_by_region',[d.arcs.color_id], ...
 'contact_centers_px',c,'radii_mm',radii,'fit_rms_mm',rms, ...
 'scale_mm_per_px',scale,'target_near_px',target,'desired_pixel_change',delta, ...
 'current_arm_angles_deg',current_yaw','current_extensions_mm',current_extension', ...
 'current_near_error_mm',current_error,'current_near_error_norm_mm',norm(current_error), ...
 'prior_response_px_per_command_mm',J,'continuous_right_forward_mm',u', ...
 'selected_right_forward_mm',command','predicted_near_residual_mm',residual, ...
 'predicted_arm_angles_deg',yaw','predicted_extensions_mm',extension', ...
 'prior_response_revalidated_current_session',false,'physical_accuracy_verified',false, ...
 'fresh_image_required_after_each_step',true,'motion_allowed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('Current near center %.3f %.3f; candidate right/forward %.1f %.1f mm; predicted residual %.3f mm\n',c(3,:),command,norm(residual));
fprintf('Current contacts radii %.3f %.3f %.3f mm; fit RMS %.3f %.3f %.3f mm\n',radii,rms);
end
