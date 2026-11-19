% Evaluate independent transferred-model probes, then fit current local response.
% Original frozen candidate remains unchanged; no effective calibration flag.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-18','stage3-resume');
pre=jsondecode(fileread(fullfile(base,'repark-matlab-prevalidation-v2.json')));
names={'base','right5','forward5','turn2'};features=zeros(4,3);hashes=cell(4,1);
for k=1:4
 m=jsondecode(fileread(fullfile(base,['repark-' names{k} '-measurement.json'])));
 features(k,:)=[double(m.center_px(:)).' double(m.row_yaw_deg)];hashes{k}=m.image_sha256;
end
C=diag([5 5 2]);D=diff(features,1,1);J=(C\D).';
assert(rank(J)==3 && all(isfinite(J(:))));
old=double(pre.J_pixel_per_right_forward_ccw);residual=D-C*old.';
A=double(pre.A2_nominal);nominalResidual=(A*residual(:,1:2).').';
e=features(end,:).'-[960;540;0];planar=-(J(1:2,1:2)\e(1:2));
assert(all(isfinite(planar)) && all(abs(planar)<=30));
cases=0;unclosed=0;
for x=[-60 0 60]
 for y=[-80 0 80]
  for yaw=[-5 0 5]
   error=[x;y;yaw];
   for n=1:20
    if norm(A*error(1:2))<=2 && abs(error(3))<=1,break;end
    if abs(error(3))>1
     axis=3;v=-error(3)/J(3,3);v=sign(v)*min(max(abs(v),1.001),10);
    else
     p=-(J(1:2,1:2)\error(1:2));[~,axis]=max(abs(p));v=sign(p(axis))*min(abs(p(axis)),30);
     if abs(v)<1,break;end
    end
    error=error+J(:,axis)*v;
   end
   cases=cases+1;if norm(A*error(1:2))>2 || abs(error(3))>1,unclosed=unclosed+1;end
  end
 end
end
assert(unclosed==0,'Current measured model did not close simulations');
result=struct('matlab_version',version,'features',features,'image_sha256',{hashes}, ...
 'J_current_pixel_per_right_forward_ccw',J,'independent_old_model_residual_px_px_deg',residual, ...
 'independent_nominal_translation_residual_mm',sqrt(sum(nominalResidual.^2,2)), ...
 'current_fit_residual',norm(D-C*J.','fro'),'current_fit_is_independent_validation',false, ...
 'next_planar_right_forward_mm',planar.','predicted_after_planar', ...
 (features(end,:).'+J(:,1:2)*planar).','synthetic_cases',cases,'synthetic_unclosed',unclosed, ...
 'target_user_confirmed',[960 540 0],'calibration_verified',false,'physical_verified',false, ...
 'note','Local diagnostic step plan only. Original frozen candidate/live flags untouched. Corrections require new images.');
path=fullfile(base,'repark-matlab-measured.json');assert(~isfile(path));
fid=fopen(path,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(result));clear cleanup;
disp(result);disp(J);fprintf('NEXT right %.6f mm; forward %.6f mm; %d synthetic cases passed\n',planar(1),planar(2),cases);
