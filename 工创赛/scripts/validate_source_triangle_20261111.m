function validate_source_triangle_20261111
% 当前真实空盘+遮挡物料档案，验证中线几何和接触弧拟合可行性。
% 不实现轮子控制；不把手画圈中心当作物料真实底心的独立真值。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-11','source-view-entry');out=fullfile(base,'triangle-feasibility');
clean=jsondecode(fileread(fullfile(base,'clean-disk-review','matlab-result.json')));
arcs=jsondecode(fileread(fullfile(out,'contact-arc-input.json')));assert(clean.passed);
H=clean.homography_image_to_old_paper_plane;T=clean.source_frame_T_candidate;
p=clean.target_source_frame_candidate_mm;r0=clean.retained_retracted_radius_candidate_mm;
near=p(3,:);farMid=mean(p(1:2,:),1);axis=farMid-near;axis=axis/norm(axis);
theta=atan2d(axis(2),axis(1));R=[axis' [-axis(2);axis(1)]];
% Exact conditional pivot pose that aligns this median AND places near at0mm.
pivot=near-r0*axis;aligned=(p-pivot)*R;
stroke=vecnorm(aligned,2,2)-r0;angle=180+atan2d(aligned(:,2),aligned(:,1));
perp=[-axis(2) axis(1)];viewTCP=[r0+65 0];
lateralAtView=dot(near-viewTCP,perp);
checks=struct('name',{},'passed',{});
check('median aligns and near maps to actual0mm target in declared model', ...
 norm(aligned(3,:)-[r0 0])<1e-9 && abs(diff(aligned(1:2,1)))<1);
check('conditional aligned extensions stay in0..170mm',all(stroke>=-1e-9 & stroke<=170));
check('median yaw does not remove translation',abs(lateralAtView)>1 && norm(pivot)>1);
% Changing chassis rotation-centre offset changes the required translation;
% this is why yaw first must be followed by a NEW view, not a blind XY move.
[a,b]=meshgrid(-200:50:200);offset=[a(:) b(:)];yawLever=offset*(R'-eye(2));
translation=ones(size(offset,1),1)*pivot-yawLever;
realCases=struct([]);caseIndex=0;
for k=1:numel(arcs.samples)
 for j=1:3
  a=arcs.samples(k).arcs(j);pts=a.boundary_image_px;
  % Registered old material image -> clean empty image before using its H.
  plane=project(H,pts+[1 0]);[c,r,e,keep]=robust_circle(plane);
  gt=clean.circles(j).center_plane_mm(:)';
  caseIndex=caseIndex+1;realCases(caseIndex).image=arcs.samples(k).image_path;
  realCases(caseIndex).id=a.id;realCases(caseIndex).estimated_center_plane_mm=c;
  realCases(caseIndex).estimated_center_material_image_px=project(inv(H),c)-[1 0];
  realCases(caseIndex).estimated_radius_mm=r;
  realCases(caseIndex).fit_rms_mm=sqrt(mean(e.^2));realCases(caseIndex).retained_points=sum(keep);
  realCases(caseIndex).difference_from_drawn_ring_center_mm=norm(c-gt);
  realCases(caseIndex).drawn_center_is_not_true_material_center=true;
  check(sprintf('contact arc finite and plausible %d',caseIndex),all(isfinite(c)) && r>18 && r<32 && sum(keep)>100 && sqrt(mean(e.^2))<1.5);
 end
end
% Synthetic visibility/phase checks use known geometric slots. They do not
% turn predicted slot locations into observed/present materials.
phaseCases=0;maxRecovery=0;
center=clean.target_orbit_center_plane_mm(:)';radius=clean.target_orbit_radius_mm;
for phase=-180:5:175
 slots=center+radius*[cosd(phase+[0;120;240]) sind(phase+[0;120;240])];
 for observed=1:3
  unit=(slots(observed,:)-center)/radius;
  rot=[unit' [-unit(2);unit(1)]];
  reconstruction=center+radius*[cosd([0;120;240]-120*(observed-1)) sind([0;120;240]-120*(observed-1))]*rot';
  err=max(vecnorm(reconstruction-slots,2,2));maxRecovery=max(maxRecovery,err);phaseCases=phaseCases+1;
 end
end
check('known-slot one-anchor phase reconstruction numerical only',maxRecovery<1e-8);
% Removal of a material does not delete its geometric slot or redefine near.
orders=perms(1:3);orderCases=0;
for k=1:6
 visible=true(1,3);expected=true(1,3);
 for item=1:3
  desired=orders(k,item);assert(visible(desired) && expected(desired));
  expected(desired)=false;visible(desired)=false;
  assert(sum(expected)==3-item && sum(visible)==3-item);orderCases=orderCases+1;
 end
end
check('all6 task orders preserve removal state and fixed geometric near slot',orderCases==18);
report=struct('engine','local MATLAB','release',version('-release'),'median_axis_angle_source_deg',theta, ...
 'median_axis_source_unit',axis,'source_view_lateral_mismatch_at65mm_mm',lateralAtView, ...
 'conditional_arm_pivot_shift_source_mm',pivot,'aligned_target_source_mm',aligned, ...
 'aligned_extension_candidate_mm',stroke','aligned_arm_angle_candidate_deg',angle', ...
 'unknown_rotation_centre_offset_assumption_grid_mm',offset, ...
 'corresponding_translation_range_mm',[min(translation,[],1);max(translation,[],1)], ...
 'real_contact_cases',realCases,'real_case_count',caseIndex, ...
 'real_materials_same_static_phase_only',true,'empty_template_prior_used',true, ...
 'synthetic_known_slot_phase_cases',phaseCases,'synthetic_phase_max_error_mm',maxRecovery, ...
 'observed_material_not_inferred_from_predicted_slot',true,'task_order_cases',orderCases, ...
 'near_geometry_slot_must_not_jump_after_pick',true, ...
 'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'checks',checks,'passed',all([checks.passed]), ...
 'formal_unknown_phase_acquisition_verified',false,'source_motion_or_stop_window_verified',false, ...
 'true_material_center_error_measured',false,'body_axis_sign_verified',false,'motion_allowed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('TRIANGLE MATLAB%d checks/%d failures; median%.3fdeg; conditional pivot[%.3f %.3f]mm; aligned ext[%.3f %.3f %.3f]mm\n', ...
 report.check_count,report.failure_count,theta,pivot,stroke);
for k=1:caseIndex,fprintf('Real arc%d %s radius%.3f/RMS%.3fmm; difference from drawn ring%.3fmm (NOT true material error)\n', ...
 k,realCases(k).id,realCases(k).estimated_radius_mm,realCases(k).fit_rms_mm,realCases(k).difference_from_drawn_ring_center_mm);end
assert(report.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
end
function p=project(H,p)
p=[p ones(size(p,1),1)]*H';p=p(:,1:2)./p(:,3);
end
function [c,r,e,keep]=robust_circle(p)
keep=true(size(p,1),1);
for k=1:5
 [c,r,~]=plain(p(keep,:));e=vecnorm(p-c,2,2)-r;m=median(e(keep));mad=median(abs(e(keep)-m));keep=abs(e-m)<=max(.5,3*1.4826*mad);
end
[c,r,e]=plain(p(keep,:));
end
function [c,r,e]=plain(p)
o=mean(p,1);q=p-o;z=[2*q ones(size(q,1),1)]\sum(q.^2,2);c=z(1:2)'+o;r=sqrt(z(3)+sum(z(1:2).^2));e=vecnorm(p-c,2,2)-r;
end
