function validate_disc_body_registration_20261114
% Fixed rim observation works even when no material remains or the disc rotates.
root=fileparts(fileparts(mfilename('fullpath')));
base=fullfile(root,'evidence','integration','2026-11-14','top-pick-pb9');
out=fullfile(root,'evidence','integration','2026-11-14','overlap200-commit');
if ~exist(out,'dir'),mkdir(out);end
names={'source-top-roi-20261114-a','source-top-roi-left11-20261114-a','source-top-roi-final-20261114-a'};
centers=zeros(3,2);near=zeros(3,2);radii=zeros(3,1);fitrms=zeros(3,1);
for k=1:3
 frame=imread(fullfile(base,names{k},'last-raw.png'));
 [centers(k,:),radii(k),fitrms(k)]=rim(frame,[950 430],610);
 d=jsondecode(fileread(fullfile(base,names{k},'alignment-result.json')));
 near(k,:)=d.near_center_px(:)';
end
expected=near-near(3,:);measured=centers-centers(3,:);
error=vecnorm(measured-expected,2,2)*d.scale_mm_per_px;
assert(all(error<=2));
J=[-(near(2,:)-near(1,:))'/11,(near(3,:)-near(2,:))'/3];assert(cond(J)<10);
r=struct('passed',true,'local_matlab',true,'sessions',{names},'rim_centers_px',centers,...
 'rim_radii_px',radii,'rim_fit_rms_px',fitrms,'expected_translation_px',expected,...
 'rim_translation_px',measured,'crosscheck_error_mm',error,...
 'reference_session',names{3},'reference_center_px',centers(3,:),...
 'reference_radius_px',radii(3),'response_px_per_command_mm',J,...
 'existing_position_tolerance_mm',2,'existing_max_step_mm',20,...
 'material_colors_not_used',true,'fixed_view_required',true,...
 'hardware_access',false,'physical_auto_correction_passed',false);
f=fopen(fullfile(out,'body-registration-matlab.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('PASS fixed-rim translation: errors %.3f %.3f %.3fmm; reference %.2f %.2f r%.2f\n',error,centers(3,:),radii(3));
end
function [center,radius,rms]=rim(frame,origin,nominal)
I=double(frame);I=.299*I(:,:,1)+.587*I(:,:,2)+.114*I(:,:,3);
angles=deg2rad([0:2:70,110:2:180]);rr=(nominal-100):.5:(nominal+100);points=[];
for angle=angles
 x=origin(1)+rr*cos(angle);y=origin(2)+rr*sin(angle);
 z=interp2(I,x,y,'linear',NaN);
 score=(z(1:end-16)+z(17:end))/2-z(9:end-8);
 score(~isfinite(score))=-Inf;[peak,index]=max(score);
 if isfinite(peak)
  index=index+8;points(end+1,:)=[x(index),y(index)]; %#ok<AGROW>
 end
end
assert(size(points,1)>20);keep=true(size(points,1),1);
for j=1:8
 o=mean(points(keep,:),1);q=points(keep,:)-o;
 v=[2*q ones(size(q,1),1)]\sum(q.^2,2);center=v(1:2)'+o;
 radius=sqrt(v(3)+sum(v(1:2).^2));err=vecnorm(points-center,2,2)-radius;
 med=median(err(keep));mad=median(abs(err(keep)-med));keep=abs(err-med)<max(1.5,3*1.4826*mad);
end
rms=sqrt(mean(err(keep).^2));
end
