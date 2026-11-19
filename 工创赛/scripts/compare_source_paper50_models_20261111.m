function compare_source_paper50_models_20261111
% 同一实图比较模型，避免小纸四点投影模型过拟合；仍不放行运动。
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-11','source-view-entry','paper50-geometry');
d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
base=jsondecode(fileread(fullfile(out,'matlab-result.json')));assert(base.passed);
p=d.corners_ABCD_px;q=[0 0;50 0;50 50;0 50];rim=d.frame_boundary_points_px{end};
names={'similarity','affine','projective'};modelCells=cell(1,3);
for j=1:3
 H=fit(p,q,j);[c,r,e]=circle(project(H,rim));
 shift=zeros(256,1);radius=zeros(256,1);
 for k=0:255
  delta=reshape(2*double(bitget(uint32(k),1:8))-1,4,2);
  Hp=fit(p+delta,q,j);[cp,rp,~]=circle(project(Hp,rim));
  shift(k+1)=norm(cp-c);radius(k+1)=rp;
 end
 Hfirst=fit(d.first_corners_ABCD_px,q,j);[cf,rf,~]=circle(project(Hfirst,rim));
 corner=vecnorm(project(H,p)-q,2,2);
 modelCells{j}=struct('model',names{j},'H_image_to_paper_plane',H, ...
  'corner_fit_rms_mm',sqrt(mean(corner.^2)),'corner_fit_max_mm',max(corner), ...
  'last_rim_center_plane_mm',c,'last_rim_radius_mm',r,'last_rim_fit_rms_mm',sqrt(mean(e.^2)), ...
  'declared_corner_perturbation_px',1,'declared_corner_sensitivity_center_max_mm',max(shift), ...
  'declared_corner_sensitivity_radius_range_mm',[min(radius) max(radius)], ...
  'first_last_corner_extraction_rim_center_change_mm',norm(cf-c), ...
  'first_last_corner_extraction_rim_radius_change_mm',rf-r);
 item=modelCells{j};
 fprintf('%s: corner RMS%.3f/max%.3fmm rimR%.3f/RMS%.3fmm; +/-1px center sensitivity%.3fmm\n', ...
  names{j},item.corner_fit_rms_mm,max(corner),r,item.last_rim_fit_rms_mm,max(shift));
end
report=struct('engine','local MATLAB','release',version('-release'),'models',[modelCells{:}], ...
 'known_shape_rim_circle_is_not_independent_metric_ground_truth',true, ...
 'new_measured_length_outside_paper_required',true,'plane_to_chassis_verified',false, ...
 'model_selected_for_motion',false,'motion_allowed',false);
f=fopen(fullfile(out,'model-comparison.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(report));fclose(f);
end
function H=fit(p,q,kind)
o=mean(p,1);s=sqrt(2)/mean(vecnorm(p-o,2,2));T=[s 0 -s*o(1);0 s -s*o(2);0 0 1];p=project(T,p);
if kind==1
 A=zeros(8,4);b=zeros(8,1);
 for k=1:4,A(2*k-1,:)=[p(k,1) -p(k,2) 1 0];A(2*k,:)=[p(k,2) p(k,1) 0 1];b(2*k-1:2*k)=q(k,:)';end
 z=A\b;H=[z(1) -z(2) z(3);z(2) z(1) z(4);0 0 1]*T;
elseif kind==2
 a=[p ones(4,1)]\q;H=[a';0 0 1]*T;
else
 A=zeros(8,8);b=zeros(8,1);
 for k=1:4
  x=p(k,1);y=p(k,2);u=q(k,1);v=q(k,2);
  A(2*k-1,:)=[x y 1 0 0 0 -u*x -u*y];A(2*k,:)=[0 0 0 x y 1 -v*x -v*y];b(2*k-1:2*k)=[u;v];
 end
 z=A\b;H=[z(1:3)';z(4:6)';z(7:8)' 1]*T;
end
H=H/H(3,3);
end
function p=project(H,p)
p=[p ones(size(p,1),1)]*H';p=p(:,1:2)./p(:,3);
end
function [c,r,e]=circle(p)
keep=true(size(p,1),1);
for k=1:5
 [c,r,e0]=plain(p(keep,:));e=vecnorm(p-c,2,2)-r;m=median(e(keep));mad=median(abs(e(keep)-m));
 keep=abs(e-m)<=max(1.5,3*1.4826*mad);
end
[c,r,e]=plain(p(keep,:));
end
function [c,r,e]=plain(p)
o=mean(p,1);q=p-o;z=[2*q ones(size(q,1),1)]\sum(q.^2,2);c=z(1:2)'+o;
r=sqrt(z(3)+sum(z(1:2).^2));e=vecnorm(p-c,2,2)-r;
end
