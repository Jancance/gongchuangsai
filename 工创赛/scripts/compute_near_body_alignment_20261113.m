function compute_near_body_alignment_20261113
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-13','source-resume','body-align');
d=jsondecode(fileread(fullfile(out,'near-input.json')));
m=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-13','source-resume','pose','matlab-result.json')));
p=d.contact_arc_px;assert(size(p,1)>150 && m.passed);keep=true(size(p,1),1);
for j=1:5
 o=mean(p(keep,:),1);q=p(keep,:)-o;v=[2*q ones(size(q,1),1)]\sum(q.^2,2);c=v(1:2)'+o;
 r=sqrt(v(3)+sum(v(1:2).^2));err=vecnorm(p-c,2,2)-r;
 med=median(err(keep));mad=median(abs(err(keep)-med));keep=abs(err-med)<max(1.5,3*1.4826*mad);
end
scale=m.scale_mm_per_px;xy=[226-(c(2)-540)*scale,-(c(1)-960)*scale];offset=xy-[161 0];
assert(r*scale>18 && r*scale<32 && sqrt(mean(err(keep).^2))*scale<1.5);
assert(offset(1)>0 && offset(1)<10 && abs(offset(2))<10);
right_step=round(offset(1));assert(right_step>=1 && right_step<=20);
after=xy-[right_step 0];assert(abs(after(1)-161)<.51);
report=struct('passed',true,'local_matlab',true,'session',d.session,'near_center_px',c, ...
 'near_source_xy_mm',xy,'offset_from_180deg_zero_extension_mm',offset, ...
 'first_right_step_mm',right_step,'forward_probe_mm',3,'right_direction_user_verified',true, ...
 'image_horizontal_body_sign_verified',false,'after_right_step_model_xy_mm',after, ...
 'goal_yaw_deg',180,'goal_extension_mm',0,'new_image_required_after_steps',true,'motion_allowed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('NEAR offset [%.3f %.3f]mm; right %.1fmm then bounded forward3mm sign observation\n',offset,right_step);
end
