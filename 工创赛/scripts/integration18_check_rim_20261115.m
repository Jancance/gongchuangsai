function integration18_check_rim_20261115(session,profilePath)
% Current-image check using the existing fixed-rim model. No hardware writes.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<2,profilePath=fullfile(root,'output','source_body_registration_20261114.json');end
profile=jsondecode(fileread(profilePath));
index=jsondecode(fileread(fullfile(session,'image-index.json')));
names={'first-raw.png','last-raw.png'};
centers=zeros(2,2);radii=zeros(2,1);rms=zeros(2,1);
for k=1:2
    frame=imread(fullfile(session,names{k}));
    assert(isequal(size(frame),[1080 1920 3]));
    [centers(k,:),radii(k),rms(k)]=rim(frame,profile.sampling_origin_px,profile.sampling_radius_px);
end
delta=centers(2,:)-profile.reference_center_px(:)';
J=profile.response_px_per_command_mm;
u=J\(-delta');
error=delta*profile.scale_mm_per_px;
within=norm(error)<=profile.position_tolerance_mm;
distance=0;direction=-1;
if ~within
    [~,axis]=max(abs(u));
    distance=max(1,min(profile.max_step_mm,round(abs(u(axis)))));
    if axis==1
        if u(axis)>0,direction=2;else,direction=1;end
    else
        if u(axis)>0,direction=0;else,direction=3;end
    end
end
report=struct('passed',true,'engine','local MATLAB','local_matlab',true,...
    'profile_id',profile.profile_id,'source_session',session,...
    'last_image_sha256',index.last_image_sha256,'centers_px',centers,...
    'radii_px',radii,'fit_rms_px',rms,'frame_shift_mm',norm(diff(centers))*profile.scale_mm_per_px,...
    'error_mm',error,'error_norm_mm',norm(error),'within_tolerance',within,...
    'continuous_right_forward_mm',u','candidate_direction',direction,'candidate_distance_mm',distance,...
    'prior_response_revalidated_current_session',false,'physical_accuracy_verified',false,...
    'motion_allowed',false,'hardware_access',false);
f=fopen(fullfile(session,'rim-matlab.json'),'w','n','UTF-8');
fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('CURRENT RIM: center %.3f %.3f; error %.3fmm; first/last shift %.3fmm; candidate dir%d %.0fmm\n',...
    centers(2,:),norm(error),report.frame_shift_mm,direction,distance);
end

function [center,radius,rms]=rim(frame,origin,nominal)
% Identical sampling/fit to validate_disc_body_registration_20261114.m.
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
    med=median(err(keep));mad=median(abs(err(keep)-med));
    keep=abs(err-med)<max(1.5,3*1.4826*mad);
end
rms=sqrt(mean(err(keep).^2));
end
