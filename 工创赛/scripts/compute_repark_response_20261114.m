function compute_repark_response_20261114
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-14');
names={'source-repark-20261114-a','source-repark-left3-20261114-a','source-repark-forward3-20261114-a'};
c=zeros(3,2);
for k=1:3
 d=jsondecode(fileread(fullfile(base,names{k},'matlab-result.json')));
 assert(d.passed);c(k,:)=d.contact_centers_px(3,:);
end
% Three distinct command/image pairs from the current parked vehicle.
J=[-(c(2,:)-c(1,:))'/3,(c(3,:)-c(2,:))'/3];
assert(cond(J)<10);target=d.target_near_px(:)';u=J\(target-c(3,:))';
assert(all(abs(u)<=20));best=Inf;command=[0;0];
for right=-20:20
 for forward=-20:20
  cost=norm(c(3,:)+(J*[right;forward])'-target)^2;
  if cost<best,best=cost;command=[right;forward];end
 end
end
r=struct('passed',true,'engine','local MATLAB','sessions',{names}, ...
 'current_centers_px',c,'response_px_per_command_mm',J, ...
 'condition_number',cond(J),'target_near_px',target, ...
 'continuous_right_forward_mm',u','selected_right_forward_mm',command', ...
 'predicted_residual_mm',(c(3,:)+(J*command)'-target)*d.scale_mm_per_px, ...
 'fresh_image_required_after_each_step',true,'physical_accuracy_verified',false, ...
 'motion_allowed',false);
f=fopen(fullfile(base,'current-response.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Current-response candidate right/forward %.1f %.1fmm; residual prediction%.3fmm\n',command,norm(r.predicted_residual_mm));
end
