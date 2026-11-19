function compute_source_pose_20261112
% One current-image computation with the already checked paper/ruler scale.
root=fileparts(fileparts(mfilename('fullpath')));base=fullfile(root,'evidence','integration','2026-11-11','source-view-entry');
out=fullfile(base,'pose-current-20261112');d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
m=jsondecode(fileread(fullfile(base,'paper50-current-20261112','matlab-result.json')));
v=jsondecode(fileread(fullfile(base,'ruler-left-20261112','matlab-result.json')));assert(m.passed && v.passed);
patch=d.patches([d.patches.correlation]>=.9);assert(numel(patch)>=6);
reference=reshape([patch.reference_px],2,[])';current=reshape([patch.current_px],2,[])';
registration=similarity(current,reference);res=vecnorm(project(registration,current)-reference,2,2);
assert(max(res)<1);H=m.models(1).H_image_to_paper_plane*registration;
tcp=project(H,[960 540]);f=project(H,[960 539])-tcp;f=f/norm(f);
left=project(H,[959 540])-tcp;left=left/norm(left);r0=161;extension=65;
T=[f r0+extension-dot(tcp,f);left -dot(tcp,left);0 0 1];
items=cell(1,3);p=zeros(3,2);
for k=1:3
 [c,r,e]=circle(project(H,d.current_arcs(k).boundary_px));
 [emptyC,emptyR,~]=circle(project(m.models(1).H_image_to_paper_plane,d.empty_circles(k).boundary_px));
 assert(size(d.current_arcs(k).boundary_px,1)>100 && r>18 && r<32 && sqrt(mean(e.^2))<1.5);
 p(k,:)=project(T,c);
 items{k}=struct('region',d.current_arcs(k).id,'color_code',d.current_colors_by_region(k), ...
  'contact_center_paper_mm',c,'contact_center_image_px',project(inv(H),c),'radius_mm',r, ...
  'fit_rms_mm',sqrt(mean(e.^2)),'difference_from_registered_empty_drawn_center_mm',norm(c-emptyC), ...
  'empty_drawn_radius_mm',emptyR,'empty_drawn_center_is_not_material_truth',true);
end
near=p(3,:);axis=mean(p(1:2,:),1)-near;axis=axis/norm(axis);yaw=atan2d(axis(2),axis(1));
rotation=[axis' [-axis(2);axis(1)]];pivot=near-r0*axis;aligned=(p-pivot)*rotation;
strokes=vecnorm(aligned,2,2)-r0;angles=180+atan2d(aligned(:,2),aligned(:,1));
assert(all(strokes>=-1e-6 & strokes<=170));
report=struct('engine','local MATLAB','release',version('-release'),'source_session',d.source_session, ...
 'registration_current_to_reference',registration,'registration_max_residual_px',max(res), ...
 'H_current_image_to_paper_plane',H,'source_frame_T',T,'materials',[items{:}], ...
 'source_frame_definition','pivot origin; +X outward/image-up at180deg; +Y image-left; chassis signs not independently verified', ...
 'current_target_source_mm',p,'yaw_error_source_deg',yaw,'pivot_target_shift_source_mm',pivot, ...
 'aligned_arm_angles_deg',angles','aligned_extensions_mm',strokes', ...
 'current_arm_angles_deg',mod(180+atan2d(p(:,2),p(:,1)),360)', ...
 'current_extensions_mm',(vecnorm(p,2,2)-r0)', ...
 'retained_retracted_pivot_TCP_mm',r0,'retained_TCP_image_center_input',true, ...
 'task_code',d.task_code,'task_source',d.task_source,'pickup_color_order',[2 1 5], ...
 'reobserve_after_yaw',true,'body_axis_sign_verified',false,'motion_allowed',false,'passed',true);
fout=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(fout~=-1);fprintf(fout,'%s\n',jsonencode(report));fclose(fout);
fprintf('CURRENT POSE yaw %.4fdeg; conditional pivot shift [%.3f %.3f]mm\n',yaw,pivot);
fprintf('After align: angles [%.3f %.3f %.3f]deg; extensions [%.3f %.3f %.3f]mm\n',angles,strokes);
fprintf('Before align: angles [%.3f %.3f %.3f]deg; extensions [%.3f %.3f %.3f]mm\n',report.current_arm_angles_deg,report.current_extensions_mm);
end
function H=similarity(p,q)
A=zeros(2*size(p,1),4);b=reshape(q',[],1);
for i=1:size(p,1),A(2*i-1,:)=[p(i,1) -p(i,2) 1 0];A(2*i,:)=[p(i,2) p(i,1) 0 1];end
x=A\b;H=[x(1) -x(2) x(3);x(2) x(1) x(4);0 0 1];
end
function p=project(H,p)
p=[p ones(size(p,1),1)]*H';p=p(:,1:2)./p(:,3);
end
function [c,r,e]=circle(p)
keep=true(size(p,1),1);
for k=1:5
 o=mean(p(keep,:),1);q=p(keep,:)-o;x=[2*q ones(size(q,1),1)]\sum(q.^2,2);c=x(1:2)'+o;r=sqrt(x(3)+sum(x(1:2).^2));
 e=vecnorm(p-c,2,2)-r;med=median(e(keep));mad=median(abs(e(keep)-med));keep=abs(e-med)<=max(.5,3*1.4826*mad);
end
e=e(keep);
end
