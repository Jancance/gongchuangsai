function check_pb9_current_parking_20261114
% Vehicle/camera mount unchanged by user report; refresh near alignment only.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-14','top-pick-pb9');
session=fullfile(out,'source-top-pb9-20261114-ready');
d=jsondecode(fileread(fullfile(session,'measurement-input.json')));
prior=jsondecode(fileread(fullfile(root,'evidence','integration','2026-11-14','source-top-repark-20261114-initial','matlab-result.json')));
p=d.arcs(3).boundary_px;assert(size(p,1)>150);keep=true(size(p,1),1);
for j=1:5
 o=mean(p(keep,:),1);q=p(keep,:)-o;v=[2*q ones(size(q,1),1)]\sum(q.^2,2);
 center=v(1:2)'+o;r=sqrt(v(3)+sum(v(1:2).^2));err=vecnorm(p-center,2,2)-r;
 med=median(err(keep));mad=median(abs(err(keep)-med));keep=abs(err-med)<max(1.5,3*1.4826*mad);
end
radius=r*prior.scale_mm_per_px;rms=sqrt(mean(err(keep).^2))*prior.scale_mm_per_px;
error=(center-prior.target_near_px(:)')*prior.scale_mm_per_px;
assert(prior.passed && radius>18 && radius<32 && rms<1.5 && norm(error)<=2);
report=prior;
report.image_sha256=d.image_sha256;report.source_session=d.source_session;
report.current_near_error_mm=error;report.current_near_error_norm_mm=norm(error);
report.current_near_center_px=center;report.current_near_radius_mm=radius;report.current_near_fit_rms_mm=rms;
report.far_poses_refitted_this_image=false;
report.far_poses_source='source-top-repark-20261114-initial: unchanged vehicle/camera; fresh near alignment confirmed';
report.current_full_three_contact_fit_passed=false;
report.note='Right yellow contact had only125 valid arc samples: full3-fit rejected; near-only refresh does not claim far refit';
f=fopen(fullfile(out,'parking-current.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('MATLAB near-only refresh: error %.3fmm; radius %.3fmm; RMS %.3fmm; far poses retained explicitly\n',norm(error),radius,rms);
end
