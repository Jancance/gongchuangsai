function align_top_roi_20261114(session)
% Reuse the latest measured chassis response, not the older 11/13 response.
root=fileparts(fileparts(mfilename('fullpath')));
d=jsondecode(fileread(fullfile(session,'measurement-input.json')));
prior=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-14','source-top-repark-20261114-initial','matlab-result.json')));
response=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-14','current-response.json')));
p=d.arcs(3).boundary_px;assert(size(p,1)>150);keep=true(size(p,1),1);
for j=1:5
 o=mean(p(keep,:),1);q=p(keep,:)-o;v=[2*q ones(size(q,1),1)]\sum(q.^2,2);
 center=v(1:2)'+o;r=sqrt(v(3)+sum(v(1:2).^2));err=vecnorm(p-center,2,2)-r;
 med=median(err(keep));mad=median(abs(err(keep)-med));keep=abs(err-med)<max(1.5,3*1.4826*mad);
end
scale=prior.scale_mm_per_px;radius=r*scale;rms=sqrt(mean(err(keep).^2))*scale;
assert(prior.passed && response.passed && radius>18 && radius<32 && rms<1.5);
target=prior.target_near_px(:)';J=response.response_px_per_command_mm;
u=J\(target-center)';assert(cond(J)<10 && all(abs(u)<=20));
best=Inf;command=[0;0];
for right=-20:20
 for forward=-20:20
  cost=norm(center+(J*[right;forward])'-target)^2;
  if cost<best,best=cost;command=[right;forward];end
 end
end
report=struct('passed',true,'engine','local MATLAB','source_session',d.source_session,...
 'image_sha256',d.image_sha256,'near_center_px',center,'radius_mm',radius,'fit_rms_mm',rms,...
 'scale_mm_per_px',scale,'target_near_px',target,'current_near_error_mm',(center-target)*scale,...
 'current_near_error_norm_mm',norm((center-target)*scale),'response_px_per_command_mm',J,...
 'continuous_right_forward_mm',u','selected_right_forward_mm',command',...
 'fresh_image_required_after_step',true,'physical_accuracy_verified',false,'motion_allowed',false);
f=fopen(fullfile(session,'alignment-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('Near %.3f %.3f, error %.3fmm; candidate right %.0f forward %.0fmm\n',center,report.current_near_error_norm_mm,command);
end
