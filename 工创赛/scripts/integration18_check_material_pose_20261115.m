function integration18_check_material_pose_20261115
% Refit current contacts with the existing contact-arc and geometry model.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-15','integration18');
session=fullfile(out,'after-forward4-h');
d=jsondecode(fileread(fullfile(session,'measurement-input.json')));
profile=jsondecode(fileread(fullfile(out,'source_body_registration_current-v2.json')));
rim=jsondecode(fileread(fullfile(session,'rim-matlab.json')));
scale=profile.scale_mm_per_px;c=zeros(3,2);radii=zeros(1,3);rms=zeros(1,3);
for k=1:3
    p=d.arcs(k).boundary_px;assert(size(p,1)>150);keep=true(size(p,1),1);
    for j=1:5
        o=mean(p(keep,:),1);q=p(keep,:)-o;
        v=[2*q ones(size(q,1),1)]\sum(q.^2,2);
        center=v(1:2)'+o;r=sqrt(v(3)+sum(v(1:2).^2));
        err=vecnorm(p-center,2,2)-r;
        med=median(err(keep));mad=median(abs(err(keep)-med));
        keep=abs(err-med)<max(1.5,3*1.4826*mad);
    end
    c(k,:)=center;radii(k)=r*scale;rms(k)=sqrt(mean(err(keep).^2))*scale;
    assert(radii(k)>18 && radii(k)<32 && rms(k)<1.5);
end
target=[960,540+65/scale];error=(c(3,:)-target)*scale;
positions=[226-(c(:,2)-540)*scale,-(c(:,1)-960)*scale];
yaw=180+atan2d(positions(:,2),positions(:,1));
extension=vecnorm(positions,2,2)-161;
assert(all(yaw>=0 & yaw<=270));
assert(all(extension(1:2)>=0 & extension(1:2)<=170));
nearWithin=norm(error)<=profile.position_tolerance_mm;
report=struct('passed',true,'local_matlab',true,'source_session',session,...
    'image_sha256',d.image_sha256,'colors_by_region',[d.arcs.color_id],...
    'contact_centers_px',c,'radii_mm',radii,'fit_rms_mm',rms,...
    'current_arm_angles_deg',yaw','current_extensions_mm',extension',...
    'target_near_px',target,'current_near_error_mm',error,...
    'current_near_error_norm_mm',norm(error),'near_within_existing_tolerance',nearWithin,...
    'physical_accuracy_verified',false,'motion_allowed',false);
f=fopen(fullfile(out,'parking-current-matlab.json'),'w','n','UTF-8');
fprintf(f,'%s\n',jsonencode(report));fclose(f);
% Fix the drift reference to this verified current observation, not an old image.
if nearWithin
    profile.reference_center_px=rim.centers_px(2,:);
    profile.reference_radius_px=rim.radii_px(2);
    profile.reference_image_sha256=d.image_sha256;
    profile.profile_id='source-rim-top180-ext65-z600-20261115-accepted-view';
    profile.calibration_evidence='parking-current-matlab.json + current-response-matlab-v2.json';
    f=fopen(fullfile(out,'source_body_registration_field.json'),'w','n','UTF-8');
    fprintf(f,'%s\n',jsonencode(profile));fclose(f);
end
fprintf('CURRENT MATERIAL: near error %.3fmm; within=%d; yaw %.3f %.3f %.3f; extension %.3f %.3f %.3f\n',...
    norm(error),nearWithin,yaw,extension);
end
