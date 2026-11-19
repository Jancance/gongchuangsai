function validate_source_ruler_20261112
% Independent current ruler strip; never fits H with the ruler.
root=fileparts(fileparts(mfilename('fullpath')));base=fullfile(root,'evidence','integration','2026-11-11','source-view-entry');
out=fullfile(base,'ruler-current-20261112');d=jsondecode(fileread(fullfile(out,'measurement-input.json')));
m=jsondecode(fileread(fullfile(base,'paper50-current-20261112','matlab-result.json')));assert(m.passed);
allPoints=reshape([d.ticks.point_px],2,[])';allTruth=[d.ticks.position_mm]';accepted=[d.ticks.accepted_for_metric]';
p=allPoints(accepted,:);truth=allTruth(accepted);checks=struct('name',{},'passed',{});
check('single current full frame',d.frames==1 && isequal(d.image_size(:)',[1920 1080]) && isempty(d.error));
check('serial disabled and no movement',strcmp(d.serial,'DISABLED') && ~d.motion_allowed);
check('six fixed surface patches unchanged',numel(d.fixed_surface_patches)==6 && all(arrayfun(@(x)x.correlation>.9 && all(x.integer_shift_px==0),d.fixed_surface_patches)));
check('ordered unambiguous marks and at least150mm strip',numel(truth)>=10 && max(truth)-min(truth)>=150 && all(diff(p(:,2))>0));
ij=nchoosek(1:numel(truth),2);expected=truth(ij(:,2))-truth(ij(:,1));items=cell(1,3);
for k=1:3
 H=m.models(k).H_image_to_paper_plane;z=project(H,p);measured=vecnorm(z(ij(:,2),:)-z(ij(:,1),:),2,2);err=measured-expected;
 [largest,i]=max(expected);
 items{k}=struct('model',m.models(k).model,'ruler_used_to_fit_model',false,'pair_count',size(ij,1), ...
  'pair_expected_mm',expected','pair_measured_mm',measured','pair_error_mm',err', ...
  'pair_rms_error_mm',sqrt(mean(err.^2)),'pair_max_abs_error_mm',max(abs(err)), ...
  'longest_interval_truth_mm',largest,'longest_interval_measured_mm',measured(i), ...
  'strip_engineering_limit_mm',3,'strip_within_limit',max(abs(err))<=3);
 fprintf('%s: %.0fmm -> %.4fmm; %d shared-endpoint pairs max error %.4fmm\n',items{k}.model,largest,measured(i),size(ij,1),max(abs(err)));
end
check('paper similarity meets retained3mm strip engineering check',items{1}.strip_within_limit);
r=struct('engine','local MATLAB','release',version('-release'),'models',[items{:}], ...
 'accepted_tick_labels_cm',truth'/10,'rejected_tick_labels_cm',allTruth(~accepted)'/10, ...
 'excluded_glare_cm',d.excluded_cm,'pairs_are_not_independent_trials',true, ...
 'checks',checks,'check_count',numel(checks),'failure_count',sum(~[checks.passed]),'passed',all([checks.passed]), ...
 'ruler_surface_height_above_disk_mm',[],'perpendicular_axis_checked',false,'full_plane_accuracy_accepted',false, ...
 'plane_to_chassis_verified',false,'pose_TCP_verified',false,'motion_allowed',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');assert(f~=-1);fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('RULER MATLAB %d checks / %d failures\n',r.check_count,r.failure_count);assert(r.passed);
 function check(name,passed)
  checks(end+1)=struct('name',name,'passed',logical(passed));
 end
end
function p=project(H,p)
p=[p ones(size(p,1),1)]*H';p=p(:,1:2)./p(:,3);
end
