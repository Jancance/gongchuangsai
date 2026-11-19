function compute_source_pose_20261113
% Current-image planning only; retained scale/TCP assumptions, no motor IO.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-13','source-resume','pose');
d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
m=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-11','source-view-entry','paper50-current-20261112','matlab-result.json')));
assert(m.passed);scale=m.models(1).image_center_scale_mm_per_px(1);
centers=zeros(3,2);radii=zeros(1,3);rms=zeros(1,3);
for k=1:3
 p=d.arcs(k).boundary_px;assert(size(p,1)>150);keep=true(size(p,1),1);
 for j=1:5
  o=mean(p(keep,:),1);q=p(keep,:)-o;v=[2*q ones(size(q,1),1)]\sum(q.^2,2);
  c=v(1:2)'+o;r=sqrt(v(3)+sum(v(1:2).^2));err=vecnorm(p-c,2,2)-r;
  med=median(err(keep));mad=median(abs(err(keep)-med));keep=abs(err-med)<max(1.5,3*1.4826*mad);
 end
 centers(k,:)=c;radii(k)=r*scale;rms(k)=sqrt(mean(err(keep).^2))*scale;
 assert(radii(k)>18 && radii(k)<32 && rms(k)<1.5);
end
% Source-frame +X image-up at logical180; +Y image-left.
% 161mm retracted pivot radius and image-center TCP remain model inputs.
positions=[226-(centers(:,2)-540)*scale,-(centers(:,1)-960)*scale];
angles=180+atan2d(positions(:,2),positions(:,1));ext=vecnorm(positions,2,2)-161;
assert(all(ext>=0 & ext<=170));
range=zeros(3,2);range(:,1)=Inf;range(:,2)=-Inf;
for factor=[.99 1 1.01]
 for dx=[-2 0 2]
  for dy=[-2 0 2]
   p=[226-(centers(:,2)+dy-540)*scale*factor,-(centers(:,1)+dx-960)*scale*factor];
   e=vecnorm(p,2,2)-161;range(:,1)=min(range(:,1),e);range(:,2)=max(range(:,2),e);
  end
 end
end
report=struct('engine','local MATLAB','source_session',d.source_session,'scale_mm_per_px',scale, ...
 'colors_by_region',[1 2 5],'contact_centers_px',centers,'radii_mm',radii,'fit_rms_mm',rms, ...
 'source_positions_mm',positions,'arm_angles_deg',angles','extensions_mm',ext', ...
 'declared_scale_1percent_pixel_2_sensitivity_mm',range,'within_170_nominal',all(ext<=170), ...
 'within_170_declared_sensitivity',all(range(:,1)>=0 & range(:,2)<=170), ...
 'pickup_order',[2 1 5],'requires_body_move_nominal',false,'TCP_image_center_is_retained_input',true, ...
 'body_axis_sign_verified',false,'motion_allowed',false,'passed',true);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('Red/yellow/black angles %.3f %.3f %.3f; extension %.3f %.3f %.3f mm\n',angles,ext);
fprintf('Sensitivity max %.3f %.3f %.3f mm; no hardware command\n',range(:,2));
end
