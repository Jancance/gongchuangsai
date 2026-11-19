function validate_source_paper50_20261112
% Current-pose numerical/sensitivity validation, not field acceptance.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-11','source-view-entry','paper50-current-20261112');
d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
p=d.corners_ABCD_px;q=[0 0;50 0;50 50;0 50];rim=d.rim_points_px;
checks=struct('name',{},'passed',{});
check('retained measured 50mm paper',d.paper_width_mm==50 && d.paper_height_mm==50);
check('current full resolution',isequal(d.image_size(:)',[1920 1080]));
check('line support and residual',all(d.border_line_support_counts>80) && all(d.border_line_rms_px<1));
check('seven fixed features unchanged at integer resolution',numel(d.fixed_surface_patches)==7 && all(arrayfun(@(x)x.correlation>.9 && all(x.integer_shift_px==0),d.fixed_surface_patches)));
check('observation only',strcmp(d.serial,'DISABLED') && ~d.motion_allowed && d.frames==1);
names={'similarity','affine','projective'};items=cell(1,3);
for j=1:3
 H=fit(p,q,j);corner=vecnorm(project(H,p)-q,2,2);
 [c,r,e]=circle(project(H,rim));shift=zeros(256,1);
 for k=0:255
  delta=reshape(2*double(bitget(uint32(k),1:8))-1,4,2);
  Hp=fit(p+delta,q,j);[cp,~,~]=circle(project(Hp,rim));shift(k+1)=norm(cp-c);
 end
 t=project(H,[960 540;961 540;960 541]);
 items{j}=struct('model',names{j},'H_image_to_paper_plane',H,'corner_fit_rms_mm',sqrt(mean(corner.^2)), ...
  'rim_center_paper_mm',c,'rim_radius_mm',r,'rim_fit_rms_mm',sqrt(mean(e.^2)), ...
  'image_center_scale_mm_per_px',[norm(t(2,:)-t(1,:)) norm(t(3,:)-t(1,:))], ...
  'declared_corner_perturbation_px',1,'sensitivity_cases',256,'declared_center_sensitivity_max_mm',max(shift));
 check([names{j} ' finite mapping'],all(isfinite(H(:))) && all(isfinite(c)) && isfinite(r) && r>0);
 fprintf('%s: corner RMS %.4fmm, rim radius %.4fmm, declared center sensitivity %.4fmm\n',names{j},sqrt(mean(corner.^2)),r,max(shift));
end
check('similarity paper residual below1mm engineering check',items{1}.corner_fit_rms_mm<1);
check('declared similarity sensitivity below1mm engineering check',items{1}.declared_center_sensitivity_max_mm<1);
r=struct('engine','local MATLAB','release',version('-release'),'models',[items{:}], ...
 'checks',checks,'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'passed',all([checks.passed]), ...
 'paper_plane_axes','A origin, AB positive X, AD positive Y; not chassis axes', ...
 'single_frame_only',true,'independent_paper_external_metric_check_verified',false, ...
 'old_ruler_check_is_current_pose_truth',false,'body_axis_binding_verified',false, ...
 'material_top_plane_verified',false,'field_accuracy_accepted',false,'motion_allowed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(r));fclose(f);
save(fullfile(out,'matlab-result.mat'),'r');fprintf('MATLAB %d checks / %d failures\n',r.check_count,r.failure_count);assert(r.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
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
