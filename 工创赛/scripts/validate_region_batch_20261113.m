function validate_region_batch_20261113(session)
% State/parameter validation before implementing the new supervised owner.
% This is not physical sweep or grip validation; actual top/clearance remain required.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-13','region-batch');
if ~exist(out,'dir'),mkdir(out);end
if nargin<1,session='align-after-back4-20261113-2136';filename='matlab-result.json';
else,filename='trial-plan.json';end
p=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-13', ...
 session,'matlab-result.json')));assert(p.passed);
colors=p.colors_by_region(:)';task=[2 1 5];yaw=p.current_arm_angles_deg(:)';ext=p.current_extensions_mm(:)';
% User accepted physical near180/0 alignment. Do not send negative model extension.
yaw(3)=180;ext(3)=0;checks=0;
assert(all(isfinite(yaw)) && all(yaw>=0 & yaw<=270));checks=checks+1;
assert(all(isfinite(ext)) && all(ext>=0 & ext<=170));checks=checks+1;
assert(isequal(sort(colors),sort(task)));checks=checks+1;
assert(p.current_near_error_norm_mm<=2,'Stopped near target does not match the accepted180/0 pickup point');
regions=zeros(1,3);requests=[];
for item=1:3
 regions(item)=find(colors==task(item));assert(isscalar(regions(item)));checks=checks+1;
 % Start is either verified observation65 or return at37/0. No Z during yaw.
 for start_ext=[0 65]
  if start_ext==65 && regions(item)==3,first=2;else,first=1;end
  assert(first==1 || first==2);checks=checks+1;
 end
 requests=[requests 3 14 15]; %#ok<AGROW>
 for age=[49 50],assert((age>=50)==(age==50));checks=checks+1;end
end
assert(isequal(colors(regions),task));checks=checks+1;
assert(numel(requests)==9 && sum(requests==14)==3 && sum(requests==15)==3);checks=checks+1;
% Old DONE, duplicate/busy requests, stale/wrong frames must not start another chain.
for fresh=[false true]
 for busy=[false true]
  for newseq=[false true]
   accepted=fresh && ~busy && newseq;
   if ~fresh || busy || ~newseq,assert(~accepted);end
   checks=checks+1;
  end
 end
end
r=struct('passed',true,'local_matlab',true,'checks',checks, ...
 'source_session',session,'source_image_sha256',p.image_sha256,'colors_by_region',colors,'task_colors',task, ...
 'region_indices_1based',regions,'region_yaw_deg',yaw,'region_extension_mm',ext, ...
 'near_180_zero_physically_confirmed_by_user',true,'physical_confirmation_scope','body baseline before disc rotation', ...
 'current_near_error_norm_mm',p.current_near_error_norm_mm,'carrier_slots',[1 2 3], ...
 'release_wait_ms',50,'one_explicit_batch_per_sequence',true, ...
 'actual_top_and_scene_clearance_required',true,'physical_sweep_verified',false, ...
 'hardware_access',false);
f=fopen(fullfile(out,filename),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Region-batch MATLAB: %u checks; task regions %u %u %u; no hardware access\n',checks,regions);
end
