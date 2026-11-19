function compute_body_two_axis_20261113
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-13','source-resume','body-align');
d=jsondecode(fileread(fullfile(out,'two-axis-input.json')));
m=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-13','source-resume','pose','matlab-result.json')));assert(m.passed);
c=zeros(3,2);rms=zeros(1,3);
for k=1:3
 p=d.observations(k).arc_px;assert(size(p,1)>150);keep=true(size(p,1),1);
 for j=1:5
  o=mean(p(keep,:),1);q=p(keep,:)-o;v=[2*q ones(size(q,1),1)]\sum(q.^2,2);center=v(1:2)'+o;
  r=sqrt(v(3)+sum(v(1:2).^2));err=vecnorm(p-center,2,2)-r;
  med=median(err(keep));mad=median(abs(err(keep)-med));keep=abs(err-med)<max(1.5,3*1.4826*mad);
 end
 c(k,:)=center;rms(k)=sqrt(mean(err(keep).^2))*m.scale_mm_per_px;
 assert(r*m.scale_mm_per_px>18 && r*m.scale_mm_per_px<32 && rms(k)<1.5);
end
J=[(c(2,:)-c(1,:))'/d.right_command_mm,(c(3,:)-c(2,:))'/d.forward_command_mm];
assert(cond(J)<10);target=[960,540+65/m.scale_mm_per_px];error=target-c(3,:);u=J\error';assert(all(abs(u)<=20));
% Search executable 1mm steps, allow an axis to stay still inside tolerance.
best=Inf;command=[0;0];
for right=-15:15
 for forward=-15:15
  predicted=c(3,:)+(J*[right;forward])';cost=norm(predicted-target)^2;
  if cost<best,best=cost;command=[right;forward];end
 end
end
residual=(c(3,:)+(J*command)'-target)*m.scale_mm_per_px;assert(norm(residual)<2);
report=struct('passed',true,'local_matlab',true,'sessions',{{d.observations.session}}, ...
 'centers_px',c,'fit_rms_mm',rms,'response_px_per_command_mm',J,'condition_number',cond(J), ...
 'target_near_px',target,'desired_pixel_change',error,'continuous_right_forward_mm',u', ...
 'selected_right_forward_mm',command','predicted_residual_mm',residual, ...
 'fresh_image_required_after_correction',true,'physical_accuracy_verified',false,'motion_allowed',false);
f=fopen(fullfile(out,'two-axis-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('NEAR centers current [%.3f %.3f]; right/forward commands [%.1f %.1f]mm; predicted residual %.3fmm\n',c(3,:),command,norm(residual));
end
