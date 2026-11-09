function validate_source_current_pair255_94_matlab_20261105
% 当前尺量255/94mm；独立重算原235/70假设下的几何与新单停车点比较。
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-05','source-markers','current-221545-a3467c481720');
out=fullfile(base,'mechanics255-e94');if ~exist(out,'dir'),mkdir(out);end
ref=read_json(fullfile(root,'calibration','source_tcp_center_reference_20261105.json'));
scene=read_json(fullfile(base,'matlab-result.json'));
assert(scene.all_three_marker_diagnostics_passed && scene.same_image_pose_check_passed);
L=ref.mechanics.current_rotation_pivot_to_tcp_mm;E=ref.mechanics.current_extension_ruler_mm;
assert(L==255 && E==94);
zero=L-E;maximum=ref.mechanics.extension_max_from_home_mm;
offsets=scene.marker_offsets_image_right_extension_forward_mm;
% 使用原图已验证的三圈/外圆；只改变本轮静态尺量，未调H迎合净距。
old_disk=scene.rim.center_plane_mm(:)';H=scene.homography_image_to_paper;
pixel=ref.reference_image_px(:)';p0=project(H,pixel);
direction=ref.extension_image_direction(:)';direction=direction/norm(direction);
forward=project(H,pixel+direction)-p0;forward=forward/norm(forward);right=[-forward(2),forward(1)];
if dot(right,project(H,pixel+[1,0])-p0)<0,right=-right;end
disk_relative=(old_disk-p0)*[right',forward'];
disk=[L+disk_relative(2),-disk_relative(1)];radius=scene.rim.planning_radius_mm;
b=ref.scene_measurements.chassis_bounds_from_dm_axis_mm;
user_gap=ref.scene_measurements.current_chassis_to_disk_min_gap_mm;
current_gap=gap(disk,b,radius);gap_error=current_gap-user_gap;
targets=[L+offsets(:,2),-offsets(:,1)];
initial=solve(targets,[0,0],zero,maximum);
checks=struct('name',{},'passed',{});
check('current ruler pair implies161mm retracted radius',zero==161);
check('current ruler94mm can be reconstructed from current255mm radius',abs((zero+E)-L)<1e-10);
check('image H unchanged from independent new-view result',isequal(H,scene.homography_image_to_paper));
check('changing pivot-to-TCP235to255 changes model gap by20mm',abs(current_gap-scene.current_model_disk_gap_mm-20)<1e-8);

% 保守筛查：将当前模型比尺量多出的间隙作为附加障碍膨胀，非独立标定。
extra_screening=max(0,gap_error);screen_radius=radius+extra_screening;
% 用户已明确当前94+剩余76=170mm，不采用194mm假设。
assert(maximum==170 && ~ref.mechanics.extension_max_170mm_requires_reconfirmation);
maxima=maximum;comparison=struct('max_extension_mm',{},'configured_current_max',{}, ...
 'reconfirmed_for_current_ruler_pair',{},'grid_count',{},'endpoints_with10mm_margin',{}, ...
 'best_nearby_single_stop',{});
for scenario=1:numel(maxima)
 maxE=maxima(scenario);rmax=zero+maxE;lo=max(targets-rmax,[],1);hi=min(targets+rmax,[],1);
 [x,y]=meshgrid(floor(lo(1)):1:ceil(hi(1)),floor(lo(2)):1:ceil(hi(2)));p=[x(:),y(:)];
 ext=zeros(size(p,1),3);angle=ext;
 for k=1:3
  d=targets(k,:)-p;ext(:,k)=hypot(d(:,1),d(:,2))-zero;angle(:,k)=mod(180+atan2d(d(:,2),d(:,1)),360);
 end
 margins=min([ext,maxE-ext],[],2);g=gap(disk-p,b,screen_radius);
 valid=margins>=0 & g>=50 & all(angle>=2 & angle<=358,2);
 strict=valid & margins>=10;near=valid & hypot(p(:,1),p(:,2))<=120;
 best=struct;ids=find(near);
 if ~isempty(ids)
  [~,j]=sortrows([-margins(near),hypot(p(near,1),p(near,2))],[1,2]);i=ids(j(1));seed=p(i,:);
  [x,y]=meshgrid(seed(1)-1:.1:seed(1)+1,seed(2)-1:.1:seed(2)+1);fp=[x(:),y(:)];
  fe=zeros(size(fp,1),3);fa=fe;
  for k=1:3,d=targets(k,:)-fp;fe(:,k)=hypot(d(:,1),d(:,2))-zero;fa(:,k)=mod(180+atan2d(d(:,2),d(:,1)),360);end
  fm=min([fe,maxE-fe],[],2);fg=gap(disk-fp,b,screen_radius);
  fvalid=fm>=0 & fg>=50 & all(fa>=2 & fa<=358,2);fi=find(fvalid);
  [~,j]=sortrows([-fm(fvalid),hypot(fp(fvalid,1),fp(fvalid,2))],[1,2]);i=fi(j(1));shift=fp(i,:);
  path=zeros(1,101);for k=1:101,t=(k-1)/100;path(k)=gap(disk-t*shift,b,screen_radius);end
  [~ ,~ , reconstructed]=solve(targets,shift,zero,maxE);
  check(sprintf('candidate max%.0f inverse and forward agree',maxE),max_norm(reconstructed-targets)<1e-8);
  best=struct('chassis_right_forward_shift_mm',shift,'extension_mm',fe(i,:),'logical_deg',fa(i,:), ...
   'minimum_stroke_margin_mm',fm(i),'screened_endpoint_gap_mm',fg(i), ...
   'sampled_screened_path_min_gap_mm',min(path),'screened_path_lower_bound_mm',min(path)-norm(shift)/200, ...
   'sampled_path_clearance_monotonic',all(diff(path)>=-1e-8),'whole_path_meets50mm',min(path)-norm(shift)/200>=50, ...
   'meets10mm_stroke_margin',fm(i)>=10,'is_diagnostic_not_command',true);
  % 新的当前94与旧170最大值的误差不是同一次相加测量，不强加旧相关关系。
  definitions=[0,0,0,3,0;1,1,2,2,1;1,1,2,3,1];stress=struct('ruler_L_error_mm',{}, ...
   'ruler_current_E_error_mm',{},'maxE_error_mm',{},'target_error_mm',{},'base_error_mm',{}, ...
   'sets',{},'rejected_sets',{},'minimum_stroke_margin_mm',{});
  for cs=1:size(definitions,1)
   v=definitions(cs,:);worst=Inf;sets=0;rejected=0;
   for dl=unique([-v(1),0,v(1)])
    for de=unique([-v(2),0,v(2)])
     for dm=unique([-v(3),0,v(3)])
      for dx=unique([-v(4),0,v(4)])
       for dy=unique([-v(4),0,v(4)])
        for bx=unique([-v(5),0,v(5)])
         for by=unique([-v(5),0,v(5)])
          pts=[L+dl+offsets(:,2),-offsets(:,1)]+[dx,dy]-(shift+[bx,by]);
          ee=hypot(pts(:,1),pts(:,2))-((L+dl)-(E+de));limit=maxE+dm;
          sets=sets+1;rejected=rejected+~all(ee>=0 & ee<=limit);worst=min(worst,min([ee;limit-ee]));
         end
        end
       end
      end
     end
    end
   end
   stress(cs)=struct('ruler_L_error_mm',v(1),'ruler_current_E_error_mm',v(2),'maxE_error_mm',v(3), ...
    'target_error_mm',v(4),'base_error_mm',v(5),'sets',sets,'rejected_sets',rejected,'minimum_stroke_margin_mm',worst);
  end
  best.declared_sensitivity=stress;
 end
 reconfirmed=false;
 if isfield(ref.mechanics,'extension_max_170mm_requires_reconfirmation') && ~ref.mechanics.extension_max_170mm_requires_reconfirmation
  reconfirmed=maxE==maximum;
 end
 comparison(scenario)=struct('max_extension_mm',maxE,'configured_current_max',maxE==maximum, ...
  'reconfirmed_for_current_ruler_pair',reconfirmed,'grid_count',size(p,1),'endpoints_with10mm_margin',sum(strict), ...
  'best_nearby_single_stop',best);
end
report=struct;report.engine='local MATLAB';report.version=version;report.release=version('-release');
report.reference=ref;report.image_sha256=scene.image_sha256;report.calibration_id=scene.calibration_id;
report.current_measured_pivot_to_tcp_mm=L;report.current_measured_extension_mm=E;
report.retracted_radius_derived_mm=zero;report.previous_retracted_radius_candidate_mm=165;
report.zero_radius_change_mm=zero-165;report.configured_maximum_extension_mm=maximum;
report.current_remaining_extension_from_configured_max_mm=maximum-E;
report.current_pose_extension_targets_mm=initial;report.current_model_disk_gap_mm=current_gap;
report.current_user_outermost_gap_mm=user_gap;report.model_minus_user_gap_mm=gap_error;
report.existing5mm_gap_consistency_check_passed=abs(gap_error)<=5;
report.screening_obstacle_inflation_mm=extra_screening;
report.screening_inflation_is_verified_error_bound=false;
report.same_stop_comparison=comparison;report.checks=checks;report.check_count=numel(checks);report.failure_count=sum(~[checks.passed]);
report.math_processing_passed=all([checks.passed]);report.field_calibration_accepted=false;
report.current_encoder_reference_restored=false;report.gray_lane_verified=false;
report.full_motion_validation_passed=false;report.hardware_operated=false;report.motion_allowed=false;
report.pending={'remaining5.2mm image/TCP/body-gap disagreement','current gap46.5mm below50mm engineering requirement', ...
 'initial below-margin retreat requires separate validation, not waived by endpoint feasibility', ...
 'lift calibration and user50mm-height loaded clearance','actual reference/encoder/fault/lane validation'};
write_json(fullfile(out,'matlab-result.json'),report);save(fullfile(out,'matlab-result.mat'),'report','targets','disk','offsets','ref');
fprintf('MATLAB current255/94 checks=%d failures=%d derived zero=%.3fmm model_gap=%.3f actual_gap=%.3f diff=%.3fmm motion=0\n', ...
 report.check_count,report.failure_count,zero,current_gap,user_gap,gap_error);
for k=1:numel(comparison),c=comparison(k);fprintf('maxE %.0f reconfirmed%d strict10 endpoints%d\n',c.max_extension_mm,c.reconfirmed_for_current_ruler_pair,c.endpoints_with10mm_margin);disp(c.best_nearby_single_stop);end
assert(report.math_processing_passed,'Current mechanical pair math checks failed');

 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed)); %#ok<AGROW>
 end
end
function [ext,angle,reconstruction]=solve(targets,shift,zero,maximum)
d=targets-shift;r=hypot(d(:,1),d(:,2));ext=r-zero;angle=mod(180+atan2d(d(:,2),d(:,1)),360);
reconstruction=[(zero+ext).*cosd(angle-180),(zero+ext).*sind(angle-180)]+shift;
assert(maximum>0);
end
function gap=gap(c,b,r)
dx=max(max(-b.left-c(:,1),c(:,1)-b.right),0);dy=max(max(-b.rear-c(:,2),c(:,2)-b.front),0);gap=hypot(dx,dy)-r;
end
function q=project(H,p)
v=[p,ones(size(p,1),1)]*H';assert(all(isfinite(v(:))) && all(abs(v(:,3))>1e-10));q=v(:,1:2)./v(:,3);
end
function d=max_norm(p)
d=max(sqrt(sum(p.^2,2)));
end
function value=read_json(path)
fid=fopen(path,'r','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
value=jsondecode(fscanf(fid,'%c'));
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8');assert(fid~=-1);clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value));
end
